/******************************************************************************
 * @file   libra2_motion_controller.cpp
 * @brief  LIBRA-II pitch motion controller; implementation file.
 *
 * @author Christian Brice
 ******************************************************************************/

// Related Header
#include "libra2_motion_controller.h"

// C++ Standard Library Headers
#include <algorithm>
#include <cmath>
#include <string>

// Project Headers
#include "joint_defs.h"

/* --- TABLE OF CONTENTS ---
 * !Local Helpers
 * !Class Management
 * !Settings & Arm Model
 * !Calibration Commands
 * !Calibration Record
 * !Control Cycle
 * !Hooks
 * !Calibration Results
 */

// Mechanical Constants (heights above the arm/CW centerline)

constexpr double kPitchJointHeight = 0.1005;  // pitch joint O (m)
constexpr double kCwBracketHeight = 0.0427;   // CW hang points (m)

// Model Parameters

constexpr double kCwPivotDist = 0.7777;      // r_P at 0 liters, A (m)
constexpr double kCwPivotSlope = -0.001155;  // r_P change per liter, B (m/L)
constexpr double kCwMeanSqDist = 0.61088;  // mean r_bag² at 0 liters, C (m²)
constexpr double kCwMeanSqDistSlope =
    -0.0017897;  // mean r_bag² change per liter, D (m²/L)
constexpr double kCwHangOffset = kCwBracketHeight
                                 - kPitchJointHeight;  // q_P (m)
constexpr double kArmInertia =
    47.6;  // I_CAD, from Inertial Properties -> Global -> Izz (kg·m²)

// Experimentally, friction was so low it fell within the calculated uncertainty,
// so both default to zero (any artifacts will barely affect the controller)
constexpr double kDefaultFrictionCoulomb = 0.0;  // τ_c (Nm)
constexpr double kDefaultFrictionViscous = 0.0;  // b (Nm·s/rad)

// Predictive CW Command

// TODO: arbitrary; tune via experiments
constexpr double kCwVolumeTolerance = 0.01;  // "target reached" band (L)

//------------------------------------------------------------------------------
// !Local Helpers
//------------------------------------------------------------------------------

namespace {

constexpr std::size_t kPitchIdx = Joint::Name::kPitch;

/**
 * @brief Checks whether two counterweight (CW) models are the same (within
 *        a tolerance).
 *
 * @param a First CW model
 * @param b Second CW model
 *
 * @note Only compares gravity terms, which the arm model depends on; inertia
 *       terms (e.g., mean r_bag²) don't affect it.
 */
bool IsSameCwModel(const Motion::CwModelParams& a,
                   const Motion::CwModelParams& b) {
    constexpr double kCwParamTolerance = 1e-9;
    return std::abs(a.pivot_dist - b.pivot_dist) <= kCwParamTolerance
           && std::abs(a.pivot_slope - b.pivot_slope) <= kCwParamTolerance
           && std::abs(a.hang_offset - b.hang_offset) <= kCwParamTolerance
           && std::abs(a.capacity - b.capacity) <= kCwParamTolerance;
}

}  // namespace

//------------------------------------------------------------------------------
// !Class Management
//------------------------------------------------------------------------------

/**
 * @brief Standard constructor.
 *
 * @param cw_capacity CW capacity (L)
 * @param reactive_cw_lower_bound Reactive CW "soft" limit (Nm)
 * @param reactive_cw_upper_bound Reactive CW "hard" limit (Nm)
 * @param reactive_cw_slow_rate Trajectory rate between reactive CW limits
 * @param debug_mode Whether to output verbose debug text
 */
Libra2MotionController::Libra2MotionController(
    const double& cw_capacity, const double& reactive_cw_lower_bound,
    const double& reactive_cw_upper_bound, const double& reactive_cw_slow_rate,
    const bool& debug_mode)
    : AbstractMotionController({Joint::Name::kPitch}, debug_mode),
      cw_model_(Motion::CwModelParams{
          kCwPivotDist,
          kCwPivotSlope,
          kCwMeanSqDist,
          kCwMeanSqDistSlope,
          kCwHangOffset,
          cw_capacity,
      }),
      friction_coulomb_(kDefaultFrictionCoulomb),
      friction_viscous_(kDefaultFrictionViscous),
      arm_inertia_(kArmInertia),
      reactive_cw_lower_bound_(reactive_cw_lower_bound),
      reactive_cw_upper_bound_(reactive_cw_upper_bound),
      reactive_cw_slow_rate_(reactive_cw_slow_rate),
      calibrator_(*logger_) {
    if (!cw_model_.IsValid()) {
        logger_->Warn("CW model invalid: " + cw_model_.GetError()
                      + " (calibration and feedforward are unavailable)");
    }
}

//------------------------------------------------------------------------------
// !Settings & Arm Model
//------------------------------------------------------------------------------

/**
 * @brief Stores latest CW state (from WaterController).
 *
 * @param volume CW volume, V (L)
 * @param flow Net CW flow rate, V' (L/s); positive = filling
 */
void Libra2MotionController::SetCwState(const double& volume,
                                        const double& flow) {
    std::lock_guard<std::mutex> lock(mutex_);

    if (!std::isfinite(volume) || volume < 0.0 || !std::isfinite(flow)) {
        logger_->Warn("Ignoring invalid CW state: " + Motion::FormatNum(volume)
                      + " L, " + Motion::FormatNum(flow) + " L/s");
        return;
    }

    cw_volume_ = volume;
    cw_flow_ = flow;
}

/**
 * @brief Enables or disables reactive counterweight (CW).
 *
 * @param enabled Whether to enable reactive CW
 */
void Libra2MotionController::EnableReactiveCw(const bool& enabled) {
    std::lock_guard<std::mutex> lock(mutex_);

    reactive_cw_en_ = enabled;
    if (!reactive_cw_en_) {
        movement_en_ = true;  // "manual" mode: movement is always allowed
    }

    logger_->Debug(std::string("Torque-based movement limits and reactive CW ")
                   + (enabled ? "enabled" : "disabled"));
}

/**
 * @brief Enables or disables the inertia terms of the torque feedforward.
 *
 * @param enabled Whether to include inertia in the feedforward torque
 */
void Libra2MotionController::EnableInertiaFeedforward(const bool& enabled) {
    std::lock_guard<std::mutex> lock(mutex_);
    inertia_ff_en_ = enabled;
    logger_->Info("Inertia feedforward "
                  + std::string(enabled ? "enabled" : "disabled"));
}

/**
 * @brief Sets the reactive counterweight's soft and hard torque limits.
 *
 * @param lower_bound Soft limit (Nm); movement slows at or above it
 * @param upper_bound Hard limit (Nm); movement pauses above it
 */
bool Libra2MotionController::SetReactiveCwBounds(const double& lower_bound,
                                                 const double& upper_bound) {
    std::lock_guard<std::mutex> lock(mutex_);

    if (!(lower_bound >= 0.0) || !(lower_bound < upper_bound)) {
        logger_->Warn("Ignoring invalid reactive CW bounds: lower="
                      + Motion::FormatNum(lower_bound)
                      + " Nm, upper=" + Motion::FormatNum(upper_bound) + " Nm");
        return false;
    }

    reactive_cw_lower_bound_ = lower_bound;
    reactive_cw_upper_bound_ = upper_bound;
    logger_->Debug("Reactive CW bounds updated: lower="
                   + Motion::FormatNum(lower_bound)
                   + " Nm, upper=" + Motion::FormatNum(upper_bound) + " Nm");

    return true;
}

/**
 * @brief Sets the CW Primary mode's hand-off band.
 *
 * @param band Range around target angle (rad); inside it, CW stops and
 *             actuator takes over
 */
bool Libra2MotionController::SetHandoffBand(const double& band) {
    std::lock_guard<std::mutex> lock(mutex_);

    if (!(band > 0.0) || !std::isfinite(band)) {
        logger_->Warn("Ignoring invalid hand-off band: "
                      + Motion::FormatNum(band) + " rad");
        return false;
    }

    handoff_band_ = band;
    logger_->Info("Hand-off band updated: " + Motion::FormatNum(band) + " rad");

    return true;
}

/**
 * @brief Gets the CW Primary mode's hand-off band (rad).
 */
double Libra2MotionController::GetHandoffBand() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return handoff_band_;
}

/**
 * @brief Sets the ETA's extension.
 *
 * @param extension Extension from minimum (m)
 *
 * @note Shifts τ_g1,0 for the torque feedforward and predictive CW, and
 *       converts new calibrations to min. extension (Eq. 44).
 *
 * @todo As of 2026/11/05, the ETA's actuator does not have a controller so this
 *       function must be invoked by the user.
 *
 * @see Libra2MotionController::LevelTorqueAtExtension
 */
void Libra2MotionController::SetEtaExtension(const double& extension) {
    std::lock_guard<std::mutex> lock(mutex_);

    if (!(extension >= 0.0) || !std::isfinite(extension)) {
        logger_->Warn("Ignoring invalid ETA extension: "
                      + Motion::FormatNum(extension) + " m");
        return;
    }

    eta_extension_ = extension;

    logger_->Info("ETA extension set to "
                  + Motion::FormatNum(extension * 1000.0) + " mm");
}

/**
 * @brief Gets the ETA's extension (m).
 */
double Libra2MotionController::GetEtaExtension() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return eta_extension_;
}

/**
 * @brief Resets the arm and friction models. Fails if the arm is currently
 *        calibrating or in motion.
 *
 * @return Whether the models were cleared
 */
bool Libra2MotionController::ClearModel() {
    std::lock_guard<std::mutex> lock(mutex_);

    if (IsMovingLocked()) {
        logger_->Warn("Can't clear motion controller model while arm is in "
                      "motion or calibrating");
        return false;
    }

    arm_model_.reset();
    cw_phase_ = CwPhase::kIdle;

    // Reset to defaults
    friction_coulomb_ = kDefaultFrictionCoulomb;
    friction_viscous_ = kDefaultFrictionViscous;
    friction_model_.reset();

    logger_->Info("Arm and friction models cleared; feedforward and predictive "
                  "CW disabled");

    return true;
}

/**
 * @brief Checks whether an arm model is available (calibrated or loaded).
 */
bool Libra2MotionController::IsArmCalibrated() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return arm_model_.has_value();
}

/**
 * @brief Gets the current calibration level.
 *
 * @note An arm model only counts with a valid CW model.
 */
Motion::Calibration Libra2MotionController::GetCalibration() const {
    std::lock_guard<std::mutex> lock(mutex_);

    const bool arm_ok = arm_model_ && cw_model_.IsValid();
    if (arm_ok && friction_model_) {
        return Motion::Calibration::kFull;
    }
    if (arm_ok) {
        return Motion::Calibration::kArmOnly;
    }
    if (friction_model_) {
        return Motion::Calibration::kFrictionOnly;
    }
    return Motion::Calibration::kNone;
}

/**
 * @brief Computes the net gravitational torque (Nm) about pitch joint O for a
 *        given angle and CW mass.
 *
 * @param angle Pitch angle, θ (rad)
 * @param cw_mass CW mass, m_cw (kg)
 *
 * @note Needs an arm model. Mutex lock not required since callers hold it.
 *
 * @note From Eq. 36':
 *         τ_ext,g(θ) = (τ_g1,0 + m_cw·g·r_P)·cosθ
 *                      + (m_arm·h_1·g + m_cw·g·q_P)·sinθ
 *
 * @see Libra2MotionController::LevelTorqueAtExtension
 */
double Libra2MotionController::ComputeNetGravTorque(
    const double& angle, const double& cw_mass) const {
    return ((LevelTorqueAtExtension()
             + (Motion::kGrav * cw_model_.Moment(cw_mass)))
            * std::cos(angle))
           + ((arm_model_->hang_torque
               + (Motion::kGrav * cw_model_.HangMoment(cw_mass)))
              * std::sin(angle));
}

/**
 * @brief Gets τ_g1,0 (Nm) at the current ETA extension.
 *
 * @note From Eq. 5 at θ = 0 (i.e., τ_g1,0 = -m_arm·g·r_1) and Eq. 44:
 *         τ_g1,0 = τ_g1,0,min - g·Δ(m_arm·r_1)
 *
 *       Only τ_g1,0 shifts; m_arm·h_1·g stays at its calibrated value, since
 *       sag isn't modeled.
 *
 * @note Needs an arm model. Mutex lock not required since callers hold it.
 *
 * @see Libra2Model::EtaMomentChange
 */
double Libra2MotionController::LevelTorqueAtExtension() const {
    return arm_model_->level_torque
           - (Motion::kGrav * Libra2Model::EtaMomentChange(eta_extension_));
}

//------------------------------------------------------------------------------
// !Calibration Commands
//------------------------------------------------------------------------------

/**
 * @brief Starts the level calibration (arm must already be level and at rest).
 *
 * @note Uses Eq. 34 at the measured angle (near 0), which reduces to Eq. 31:
 *         τ_g1,0 = τ_meas(0) - m_cw·g·r_P
 *
 * @see Libra2MotionController::FinishLevelCalibration
 */
bool Libra2MotionController::StartLevelCalibration() {
    std::lock_guard<std::mutex> lock(mutex_);

    if (!cw_model_.IsValid()) {
        logger_->Error("Can't calibrate: invalid CW model ("
                       + cw_model_.GetError() + ")");
        return false;
    }
    if (!arm_model_) {
        logger_->Warn("Can't calibrate: missing required arm hang torque "
                      "(m_arm*h_1*g) from a two-angle (or loaded) calibration");
        return false;
    }

    return calibrator_.StartLevelCalibration();
}

/**
 * @brief Starts the two-angle calibration.
 *
 * @param angle_a First angle, θ_a (rad)
 * @param angle_b Second angle, θ_b (rad)
 *
 * @note From Eq. 35c:
 *         τ_g1,0 = (τ_g1(θ_a)·sinθ_b - τ_g1(θ_b)·sinθ_a) / sin(θ_b - θ_a)
 *
 * @see Libra2MotionController::FinishTwoAngleCalibration
 */
bool Libra2MotionController::StartTwoAngleCalibration(const double& angle_a,
                                                      const double& angle_b) {
    std::lock_guard<std::mutex> lock(mutex_);

    if (!cw_model_.IsValid()) {
        logger_->Error("Can't calibrate: invalid CW model ("
                       + cw_model_.GetError() + ")");
        return false;
    }

    return calibrator_.StartTwoAngleCalibration(angle_a, angle_b);
}

/**
 * @brief Starts the friction calibration.
 *
 * @param angle_a One end of the window (rad)
 * @param angle_b Other end of the window (rad)
 * @param max_vel Max. actuator speed (rad/s); sweep speeds are fractions of it
 *
 * @note Doesn't need an arm or CW model; gravity cancels between the up and
 *       down sweeps.
 *
 * @note No feedforward during the sweeps; the PID loop alone drives the arm.
 *
 * @note Isolates the friction component by averaging the up and down sweeps,
 *       which cancels the gravitational torque:
 *         (τ_up - τ_down) / 2 = b·v + τ_c
 *
 * @see Libra2MotionController::FinishFrictionCalibration
 */
bool Libra2MotionController::StartFrictionCalibration(const double& angle_a,
                                                      const double& angle_b,
                                                      const double& max_vel) {
    std::lock_guard<std::mutex> lock(mutex_);

    if (!cw_volume_) {
        logger_->Warn("Friction calibration rejected: CW volume unknown");
        return false;
    }

    // Arm + CW, for the sweeps' ramp lengths
    const double inertia = arm_inertia_
                           + (cw_model_.IsValid() ? cw_model_.Inertia(
                                  Libra2Model::CwMass(*cw_volume_))
                                                  : 0.0);

    return calibrator_.StartFrictionCalibration(angle_a, angle_b, max_vel,
                                                *cw_volume_, inertia);
}

/**
 * @brief Cancels a running calibration.
 */
void Libra2MotionController::CancelCalibration() {
    std::lock_guard<std::mutex> lock(mutex_);
    calibrator_.CancelCalibration();
}

/**
 * @brief Gets calibration progress.
 */
Motion::CalibrationStatus Libra2MotionController::GetCalibrationStatus() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return calibrator_.GetCalibrationStatus();
}

/**
 * @brief Gets a description of the last calibration, if any.
 */
std::string Libra2MotionController::GetCalibrationMessage() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return calibrator_.GetCalibrationMessage();
}

/**
 * @brief Gets the running calibration's current step (e.g., "Friction
 *        calibration [3/9]: ..."); empty if none is running.
 */
std::string Libra2MotionController::GetCalibrationProgress() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return calibrator_.GetCalibrationProgress();
}

//------------------------------------------------------------------------------
// !Calibration Record
//------------------------------------------------------------------------------

/**
 * @brief Gets the current arm and friction models with relevant raw data.
 *
 * @note Record metadata is left empty.
 */
Motion::CalibrationRecord Libra2MotionController::GetCalibrationRecord() const {
    std::lock_guard<std::mutex> lock(mutex_);

    Motion::CalibrationRecord record;
    record.cw_model = cw_model_.GetParams();
    record.arm_inertia = arm_inertia_;

    record.arm_model = arm_model_;
    record.friction = friction_model_;

    return record;
}

/**
 * @brief Applies the given arm and/or friction models.
 *
 * @param record A full or partial calibration
 *
 * @note If the CW model changed since the arm model was made, the saved arm
 *       model is used as-is (with a warning).
 */
bool Libra2MotionController::LoadCalibrationRecord(
    const Motion::CalibrationRecord& record) {
    std::lock_guard<std::mutex> lock(mutex_);

    if (IsMovingLocked()) {
        logger_->Warn("Can't load a calibration while the arm is moving or "
                      "calibrating");
        return false;
    }
    if (!record.arm_model && !record.friction) {
        logger_->Warn("Calibration record has no arm model or friction "
                      "result; nothing loaded");
        return false;
    }

    // Apply the arm model
    if (record.arm_model) {
        // Sanity check CW model first
        const Motion::ArmModel& arm = *record.arm_model;
        if (!IsSameCwModel(arm.cw_model, cw_model_.GetParams())) {
            logger_->Warn("CW model has changed since this calibration was "
                          "saved; using its arm model as-is");
        }

        const std::string old_level = arm_model_ ? Motion::FormatNum(
                                          arm_model_->level_torque)
                                                 : "none";
        const std::string old_hang = arm_model_ ? Motion::FormatNum(
                                         arm_model_->hang_torque)
                                                : "none";
        logger_->Info("Arm model loaded: τ_g1,0 " + old_level + " -> "
                      + Motion::FormatNum(arm.level_torque) + " Nm, m_arm*h_1*g "
                      + old_hang + " -> " + Motion::FormatNum(arm.hang_torque)
                      + " Nm (" + arm.source + ")");

        // Apply and validate
        arm_model_ = arm;
        CheckStability(arm.hang_torque);
    }

    // Apply the friction model
    if (record.friction) {
        logger_->Info(
            "Friction loaded: tau_c " + Motion::FormatNum(friction_coulomb_)
            + " -> " + Motion::FormatNum(record.friction->coulomb) + " Nm, b "
            + Motion::FormatNum(friction_viscous_) + " -> "
            + Motion::FormatNum(record.friction->viscous) + " Nm*s/rad");

        friction_coulomb_ = record.friction->coulomb;
        friction_viscous_ = record.friction->viscous;
        friction_model_ = record.friction;
    }
    return true;
}

//------------------------------------------------------------------------------
// !Control Cycle
//------------------------------------------------------------------------------

/**
 * @brief Applies three-step hysteresis to throttle trajectory-following speed
 *        based on sensed torque, using the counterweight (CW).
 *
 * @param torque_magnitude Absolute value of torque feedback (Nm)
 * @param direction CW adjustment direction, opposite to torque
 * @param braking Whether actuator torque opposes travel (CW leads)
 *
 * @note Slowing the trajectory only helps while the actuator lifts the arm.
 *       While it brakes, slowing would raise the torque, so the soft limit
 *       keeps full speed and only the CW reacts. The hard limit always stops.
 *
 * @see HebiThread::UpdateTrajectoryRate
 */
Libra2MotionController::ReactiveCwResult Libra2MotionController::UpdateReactiveCw(
    const double& torque_magnitude, const Motion::CwCommand& direction,
    const bool& braking) {
    ReactiveCwResult result;

    if (!reactive_cw_en_) {
        movement_en_ = true;
        return result;  // default: full speed, no CW command
    }

    bool in_slow_band = false;

    if (torque_magnitude > reactive_cw_upper_bound_) {
        // Hard limit: stop movement, then report direction (same as LIBRA-I)
        if (movement_en_) {
            movement_en_ = false;

            logger_->Info("Movement disabled due to high torque ("
                          + Motion::FormatNum(torque_magnitude) + " Nm > "
                          + Motion::FormatNum(reactive_cw_upper_bound_)
                          + " Nm)");
        } else {
            result.cw_command = direction;
        }

    } else if (torque_magnitude >= reactive_cw_lower_bound_) {
        // Soft limit: keep reporting direction, slow trajectory (unless braking)
        result.cw_command = direction;
        in_slow_band = !braking;

    } else {
        // Acceptable torque: stop balancing and re-enable movement
        result.cw_command = Motion::CwCommand::kIdle;
        if (!movement_en_) {
            movement_en_ = true;

            logger_->Info("Movement re-enabled ("
                          + Motion::FormatNum(torque_magnitude) + " Nm < "
                          + Motion::FormatNum(reactive_cw_lower_bound_)
                          + " Nm)");
        }
    }

    if (!movement_en_) {
        result.trajectory_rate = 0.0;
    } else {
        result.trajectory_rate = in_slow_band ? reactive_cw_slow_rate_ : 1.0;
    }

    return result;
}

/**
 * @brief Computes the CW mass (kg) that balances the arm at a target angle,
 *        m_cw,bal(θ_tgt), clamped to CW capacity.
 *
 * @param angle_tgt Target angle, θ_tgt (rad)
 *
 * @note Returns none if balance is undefined at this angle.
 *
 * @see Libra2Model::BalanceCwMoment CwModel::MassForMoment
 *      Libra2MotionController::LevelTorqueAtExtension
 */
std::optional<double> Libra2MotionController::TargetCwMass(
    const double& angle_tgt) const {
    const auto moment_bal =
        Libra2Model::BalanceCwMoment(LevelTorqueAtExtension(),
                                     arm_model_->hang_torque, angle_tgt);
    if (!moment_bal) {
        return std::nullopt;
    }
    return cw_model_.MassForMoment(*moment_bal, angle_tgt);
}

/**
 * @brief Determines the predictive CW command that balances the arm at the
 *        target angle.
 *
 * @param cw_mass_tgt Balancing CW mass, m_cw,bal(θ_tgt) (kg), from
 *                    `TargetCwMass()`
 * @param cw_mass Current CW mass, m_cw (kg)
 *
 * @note From Eq. 40a, with Eq. 28 to convert the tolerance to mass:
 *         Δm_tgt = m_cw,bal(θ_tgt) - m_cw
 */
Motion::CwCommand Libra2MotionController::PredictiveCwCommand(
    const double& cw_mass_tgt, const double& cw_mass) const {
    const double diff = cw_mass_tgt - cw_mass;
    if (std::abs(diff) <= kCwVolumeTolerance * Motion::kWaterDensity) {
        return Motion::CwCommand::kIdle;
    }

    // TODO: if valve latency causes CW overshoot, use this instead
    /*
    // Valves close late, so this much water still flows after the command (kg)
    const double overshoot_mass = kValveLatency * std::abs(cw_flow_)
                                  * Motion::kWaterDensity;
    if (std::abs(diff) <= overshoot_mass) {
        return Motion::CwCommand::kIdle;
    }
    */
    return (diff > 0.0) ? Motion::CwCommand::kFill : Motion::CwCommand::kDrain;
}

/**
 * @brief Runs one CW Primary cycle: pitch joint allowed to spin freely so the
 *        CW moves it.
 *
 * @param angle Actual angle, θ (rad)
 * @param angle_tgt Target angle, θ_tgt (rad)
 * @param[in,out] command Command with CW adjustment and free-spinning pitch
 *
 * @note Returns true while CW controls the free-spinning arm, and false at
 *       hand-off (where actuator takes over and a proper move is requested).
 */
bool Libra2MotionController::RunCwPrimary(const double& angle,
                                          const double& angle_tgt,
                                          Motion::Libra2Command& command) {
    const auto cw_mass_tgt = TargetCwMass(angle_tgt);
    if (!cw_mass_tgt) {
        logger_->Warn("CW Primary: target mass undefined at this angle; "
                      "handing off to actuator");
        HandOffToActuator();
        return false;
    }
    const double mass_tgt = *cw_mass_tgt;
    const Motion::CwCommand cw = PredictiveCwCommand(mass_tgt, *cw_mass_);

    const bool volume_reached = (cw == Motion::CwCommand::kIdle);
    const bool angle_reached = std::abs(angle - angle_tgt) <= handoff_band_;

    if (volume_reached || angle_reached) {
        // Log hand-off and set up for actuator takeover
        const std::string reached = std::string(volume_reached ? "volume" : "")
                                    + std::string(
                                        volume_reached && angle_reached ? "+"
                                                                        : "")
                                    + std::string(angle_reached ? "angle" : "");
        logger_->Info(
            "CW handing control to actuator (" + reached
            + " reached): angle error=" + Motion::FormatNum(angle - angle_tgt)
            + " rad, volume error="
            + Motion::FormatNum((*cw_mass_ - mass_tgt) / Motion::kWaterDensity)
            + " L");

        predictive_cw_mass_ = mass_tgt;
        HandOffToActuator();

        return false;
    }

    // Arm spins freely; CW does the moving
    command.mode[kPitchIdx] = Motion::CommandMode::kFree;
    command.stop_trajectory = trajectory_active_;
    command.cw_command = cw;
    command.trajectory_rate = 1.0;
    movement_en_ = true;

    return true;
}

/**
 * @brief Ends CW Primary's free-arm phase: the actuator takes over and moves to
 *        the target (if any). Predictive CW stays off until the next target.
 */
void Libra2MotionController::HandOffToActuator() {
    cw_phase_ = CwPhase::kIdle;
    cw_assist_done_ = true;
    reactive_cw_used_ = false;

    if (target_pos_) {
        pending_move_ = (*target_pos_)[kPitchIdx];
    }
}

/**
 * @brief Logs the error between the predictive counterweight (CW) estimate of
 *        the balancing mass and the mass the reactive CW actually settled at,
 *        once the system is at rest.
 *
 * @param reactive Reactive CW command this cycle
 * @param angle Actual angle, θ (rad)
 * @param torque Actual actuator torque (Nm)
 *
 * @note Only logs if reactive CW moved after predictive CW finished; otherwise
 *       the predicted mass held the torque below reactive CW's lower bound.
 */
void Libra2MotionController::LogPredictiveCwError(
    const std::optional<Motion::CwCommand>& reactive, const double& angle,
    const double& torque) {
    if (reactive && *reactive != Motion::CwCommand::kIdle) {
        reactive_cw_used_ = true;
        return;
    }

    // Settled: torque below reactive CW's lower bound, arm and water stopped
    if (!reactive_cw_used_ || reactive != Motion::CwCommand::kIdle
        || trajectory_active_ || cw_flow_ != 0.0) {
        return;
    }

    const double predicted = *predictive_cw_mass_ / Motion::kWaterDensity;
    const double settled = *cw_mass_ / Motion::kWaterDensity;
    logger_->Info("Reactive CW settled at " + Motion::FormatNum(settled)
                  + " L vs. predicted " + Motion::FormatNum(predicted)
                  + " L (error " + Motion::FormatNum(settled - predicted)
                  + " L) at angle=" + Motion::FormatNum(angle)
                  + " rad, torque=" + Motion::FormatNum(torque) + " Nm");

    predictive_cw_mass_.reset();
}

/**
 * @brief Runs one control cycle: decides what the actuator and CW do.
 *
 * @param actual Actual joint values
 * @param commanded Commanded joint values from previous cycle
 * @param dt Time since last cycle (s)
 * @param trajectory_active Whether a trajectory is running
 * @param mode Requested motion logic
 *
 * @note Modes other than CW Primary and Tandem only use the motion controller
 *       to calibrate.
 *
 * @note If anything CW Primary needs is lost while the arm is free (model, CW
 *       volume, target, or reactive CW control), the actuator takes over
 *       immediately.
 */
Motion::Libra2Command Libra2MotionController::Update(
    const Motion::ActualState& actual, const Motion::CommandedState& commanded,
    const double& dt, const bool& trajectory_active,
    const Motion::MotionMode& mode) {
    std::lock_guard<std::mutex> lock(mutex_);

    if (!IsValidInput(actual, commanded)) {
        if (!state_warned_) {
            logger_->Error("Ignoring control cycle: input arrays need "
                           + std::to_string(Motion::kNumJoints)
                           + " entries each, and pitch must be finite");
            state_warned_ = true;
        }
        return Motion::Libra2Command();
    }

    // Update state
    const double angle = actual.pos[kPitchIdx];
    const double actual_vel = actual.vel[kPitchIdx];
    const double actual_torque = actual.torque[kPitchIdx];
    const double ref_vel = commanded.vel[kPitchIdx];
    const double ref_acc = commanded.acc[kPitchIdx];
    trajectory_active_ = trajectory_active;

    // Eq. 28: CW mass from latest volume
    cw_mass_ = cw_volume_
                   ? std::optional<double>(Libra2Model::CwMass(*cw_volume_))
                   : std::nullopt;

    const Libra2Calibrator::PitchCycle cycle{
        dt, angle, actual_vel, actual_torque, trajectory_active, cw_volume_,
    };
    const bool calibrating = calibrator_.IsRunning();
    const bool model_ready = arm_model_ && cw_model_.IsValid()
                             && cw_mass_.has_value();

    std::optional<double> target;
    if (target_pos_) {
        target = (*target_pos_)[kPitchIdx];
    }

    // Determine how a new target is handled
    const bool cw_primary = (mode == Motion::MotionMode::kCwPrimary);
    if (new_target_) {
        new_target_ = false;
        if (cw_primary) {
            // Let CW lead
            cw_phase_ = CwPhase::kCwMoving;
        } else if (target) {
            // Move actuator
            pending_move_ = *target;
        }
    }

    // Left CW Primary while the CW was moving the arm
    if (!cw_primary && cw_phase_ == CwPhase::kCwMoving) {
        logger_->Warn("Left CW Primary while CW was moving the arm; handing "
                      "off to actuator");
        HandOffToActuator();
    }

    // Determine movement and CW commands
    Motion::Libra2Command command(Motion::kNumJoints);
    bool arm_free = false;

    if (cw_primary && cw_phase_ == CwPhase::kCwMoving) {
        // (A) CW Primary: arm spins freely while CW moves it
        if (!model_ready || !target || calibrating || !reactive_cw_en_) {
            logger_->Warn(
                "CW Primary: model, CW volume, target, or reactive CW "
                "unavailable; handing control to the actuator");
            HandOffToActuator();
        } else {
            arm_free = RunCwPrimary(angle, *target, command);
        }
    }

    if (!arm_free && calibrator_.IsFrictionRunning()) {
        // Friction sweep: no throttle or CW command necessary
        command.trajectory_rate = 1.0;
        command.cw_command = Motion::CwCommand::kIdle;
        calibrator_.Update(cycle);

    } else if (!arm_free) {
        // (B) Tandem/Holding: reactive CW throttle applies,
        const Motion::CwCommand direction = (actual_torque > 0.0)
                                                ? Motion::CwCommand::kFill
                                                : Motion::CwCommand::kDrain;
        const bool braking = target
                             && (actual_torque * (*target - angle) < 0.0);
        const ReactiveCwResult reactive_cw =
            UpdateReactiveCw(std::abs(actual_torque), direction, braking);

        command.trajectory_rate = reactive_cw.trajectory_rate;
        command.cw_command = reactive_cw.cw_command;  // reactive (LIBRA-I style)

        if (calibrating) {
            // CW reactive while calibrating
            calibrator_.Update(cycle);
        } else {
            // Predictive CW replaces reactive until it reaches its target
            // mass; reactive then handles any torque left above its bounds
            if (model_ready && target && !cw_assist_done_) {
                const auto cw_mass_tgt = TargetCwMass(*target);
                if (cw_mass_tgt) {
                    const Motion::CwCommand cw =
                        PredictiveCwCommand(*cw_mass_tgt, *cw_mass_);
                    if (cw == Motion::CwCommand::kIdle) {
                        cw_assist_done_ = true;
                        predictive_cw_mass_ = *cw_mass_tgt;
                        reactive_cw_used_ = false;
                    } else {
                        command.cw_command = cw;
                    }
                }
            } else if (predictive_cw_mass_ && cw_mass_) {
                LogPredictiveCwError(reactive_cw.cw_command, angle,
                                     actual_torque);
            }
        }

        // Also on while calibrating, since toggling it jolts the arm;
        // calibration samples are unaffected (only total torque is measured)
        command.ff_torque[kPitchIdx] = ModelFeedforward(angle, ref_vel, ref_acc,
                                                        model_ready);
    }

    // Apply calibration results
    const Libra2Calibrator::Output calibration = calibrator_.TakeOutput();
    if (calibration.level_sample) {
        FinishLevelCalibration(*calibration.level_sample);
    }
    if (calibration.two_angle_samples) {
        FinishTwoAngleCalibration(calibration.two_angle_samples->first,
                                  calibration.two_angle_samples->second);
    }
    if (calibration.friction) {
        FinishFrictionCalibration(*calibration.friction);
    }

    // Deal with requests
    if (calibration.stop_trajectory) {
        command.stop_trajectory = true;
    }
    if (calibration.move_to) {
        pending_move_ = calibration.move_to;
    }
    if (pending_move_) {
        std::vector<double> move = actual.pos;
        move[kPitchIdx] = *pending_move_;
        command.move_to = move;
        pending_move_.reset();
    }
    command.sweep = calibration.sweep;

    command.diagnostics = ComputeDiagnostics(angle, target);
    command.moving = IsMovingLocked();

    return command;
}

/**
 * @brief Computes the actuator torque feedforward (Nm) for the commanded motion
 *        (Eq. 37).
 *
 * @param angle Actual angle, θ (rad)
 * @param ref_vel Reference velocity, θ'_ref (rad/s)
 * @param ref_acc Reference acceleration, θ''_ref (rad/s²)
 * @param model_ready Whether the arm model and CW mass are known; if not, the
 *                    gravity term is left out (PID loop and CW handle it)
 *
 * @note Friction (and inertia, if enabled) terms always apply, so a move gets
 *       the torque required to overcome friction even before the arm model is
 *       calibrated.
 *
 * @note Result is not clamped.
 *
 * @note From Eq. 37:
 *         τ_actuator = I·θ''_ref + (b + I')·θ'_ref + τ_c·sign(θ'_ref)
 *                      - τ_ext,g(θ)
 */
double Libra2MotionController::ModelFeedforward(const double& angle,
                                                const double& ref_vel,
                                                const double& ref_acc,
                                                const bool& model_ready) const {
    const double torque_ext_g = model_ready
                                    ? ComputeNetGravTorque(angle, *cw_mass_)
                                    : 0.0;
    // Eq. 15: inertia and its rate, with the CW's share (m'_cw = ρ_water·V');
    // CW terms are code-specific (see `CwModel::Inertia()`/`InertiaSlope()`)
    double inertia = 0.0;
    double inertia_rate = 0.0;
    if (inertia_ff_en_) {
        inertia = arm_inertia_;
        if (cw_mass_ && cw_model_.IsValid()) {
            inertia += cw_model_.Inertia(*cw_mass_);
            inertia_rate = cw_model_.InertiaSlope(*cw_mass_)
                           * Motion::kWaterDensity * cw_flow_;
        }
    }

    return (inertia * ref_acc) + ((friction_viscous_ + inertia_rate) * ref_vel)
           + (friction_coulomb_ * Motion::Sign(ref_vel)) - torque_ext_g;
}

/**
 * @brief Computes diagnostic data about the active model.
 *
 * @param angle Actual angle, θ (rad)
 * @param angle_tgt Target angle, θ_tgt (rad), if any
 *
 * @note Entries that can't be computed (i.e., no model, CW volume, or target)
 *       are left empty.
 *
 * @see Libra2Model::BalanceCwMoment
 *      Libra2MotionController::LevelTorqueAtExtension
 */
Motion::Libra2Diagnostics Libra2MotionController::ComputeDiagnostics(
    const double& angle, const std::optional<double>& angle_tgt) const {
    Motion::Libra2Diagnostics diagnostics;
    diagnostics.cw_mass = cw_mass_;
    if (!arm_model_ || !cw_model_.IsValid()) {
        return diagnostics;
    }

    if (cw_mass_) {
        diagnostics.torque_ext_g = ComputeNetGravTorque(angle, *cw_mass_);
    }

    // Eqs. 38'-40
    if (angle_tgt) {
        const auto moment_bal =
            Libra2Model::BalanceCwMoment(LevelTorqueAtExtension(),
                                         arm_model_->hang_torque, *angle_tgt);
        const auto mass_tgt = moment_bal ? cw_model_.MassForMoment(*moment_bal,
                                                                   *angle_tgt)
                                         : std::nullopt;
        if (mass_tgt) {
            diagnostics.cw_mass_tgt = *mass_tgt;
            if (cw_mass_) {
                diagnostics.cw_mass_change = *mass_tgt - *cw_mass_;
                // Code-specific extension of Eq. 39' for r_P that varies with
                // volume, using effective moments (see `BalanceCwMoment()`):
                //   τ_ext,g(θ_tgt) = g·cosθ_tgt·(M_eff(m_cw) - M_eff(m_cw,bal))
                diagnostics.torque_ext_g_tgt =
                    Motion::kGrav * std::cos(*angle_tgt)
                    * (cw_model_.EffectiveMoment(*cw_mass_, *angle_tgt)
                       - *moment_bal);
            }
        }
    }

    return diagnostics;
}

//------------------------------------------------------------------------------
// !Hooks
//------------------------------------------------------------------------------

/**
 * @brief Handles a new target: cancels any calibration and CW assistance, and
 *        marks the target as new.
 *
 * @note How to reach it depends on the motion mode, so the next `Update()`
 *       decides (HebiThread starts every LIBRA-II trajectory from `move_to`).
 */
void Libra2MotionController::OnTargetChanged() {
    calibrator_.CancelCalibration();

    cw_phase_ = CwPhase::kIdle;
    cw_assist_done_ = false;
    predictive_cw_mass_.reset();
    pending_move_.reset();

    new_target_ = true;
}

/**
 * @brief Handles a cleared target: cancels any calibration and CW assistance.
 */
void Libra2MotionController::OnStop() {
    calibrator_.CancelCalibration();

    cw_phase_ = CwPhase::kIdle;
    cw_assist_done_ = false;
    predictive_cw_mass_.reset();

    pending_move_.reset();
    new_target_ = false;
}

/**
 * @brief Handles a reset: clears runtime state. Keeps settings, arm model, and
 *        friction.
 */
void Libra2MotionController::OnReset() {
    calibrator_.Reset();

    cw_phase_ = CwPhase::kIdle;
    cw_assist_done_ = false;
    predictive_cw_mass_.reset();

    pending_move_.reset();
    new_target_ = false;
    trajectory_active_ = false;
    movement_en_ = true;

    state_warned_ = false;
}

/**
 * @brief Checks whether the arm is moving or about to move: trajectory, CW-led
 *        move, new or queued move, or calibration (`mutex_` must already be
 *        held).
 *
 * @note Trajectory state is as of the last `Update()`.
 */
bool Libra2MotionController::IsMovingLocked() const {
    return trajectory_active_ || new_target_
           || (cw_phase_ == CwPhase::kCwMoving) || pending_move_.has_value()
           || calibrator_.IsRunning();
}

//------------------------------------------------------------------------------
// !Calibration Results
//------------------------------------------------------------------------------

/**
 * @brief Converts an averaged rest sample into the arm's own gravitational
 *        torque (Nm).
 *
 * @param sample Averaged measurements
 *
 * @note At rest, measured torque is the opposite of actual actuator torque, per
 *       Eq. 19b. Using Eq. 29a removes the CW torque (Eq. 29b'), and with
 *       Eq. 28 for CW mass:
 *         τ_meas(θ) = -τ_actuator
 *         τ_g2(θ) = m_cw·g·(r_P·cosθ + q_P·sinθ)
 *         τ_g1(θ) = τ_meas(θ) - τ_g2(θ)
 */
double Libra2MotionController::SampleToArmTorque(
    const Motion::RestSample& sample) const {
    const double cw_mass = Libra2Model::CwMass(sample.volume);
    const double cw_torque = Motion::kGrav
                             * ((cw_model_.Moment(cw_mass)
                                 * std::cos(sample.angle))
                                + (cw_model_.HangMoment(cw_mass)
                                   * std::sin(sample.angle)));
    const double arm_torque = -sample.torque - cw_torque;

    // Logging a sample's terms could help with troubleshooting where a
    // calibration error comes from
    logger_->Debug("Calibration sample: θ="
                   + Motion::FormatRadAsDeg(sample.angle)
                   + " deg, τ_actuator=" + Motion::FormatNum(sample.torque)
                   + " Nm, V=" + Motion::FormatNum(sample.volume)
                   + " L, τ_g2=" + Motion::FormatNum(cw_torque)
                   + " Nm, τ_g1=" + Motion::FormatNum(arm_torque) + " Nm");

    return arm_torque;
}

/**
 * @brief Checks whether the arm is stable at every possible CW volume.
 *
 * @param arm_hang_torque Arm hang torque, m_arm·h_1·g (Nm)
 *
 * @note Stable while the numerator g·S of Eq. 26c' is negative (Eq. 27'):
 *         g·S = m_arm·h_1·g + m_cw·g·q_P
 *
 *       S is linear in m_cw, so checking only empty and full covers every
 *       volume.
 */
bool Libra2MotionController::CheckStability(const double& arm_hang_torque) {
    const auto stability_torque = [&](const double& cw_mass) {
        return arm_hang_torque
               + (Motion::kGrav * cw_model_.HangMoment(cw_mass));
    };

    const double cap = cw_model_.GetCapacityMass();
    const double empty = stability_torque(0.0);
    const double full = stability_torque(cap);

    const std::string values = Motion::FormatNum(empty) + " Nm empty, "
                               + Motion::FormatNum(full) + " Nm full";
    if (std::max(empty, full) < 0.0) {
        logger_->Info("Arm model is stable at all CW volumes (g*S < 0): "
                      + values);
        return true;
    }

    // NOTE: Code-specific equation. S is linear in m_cw, so a sign change
    //       between empty and full has one crossover:
    //         V_cross = m_cap·g·S_empty / (g·S_empty - g·S_full) / ρ_water
    std::string unstable_range_str = "at every CW volume";
    if ((empty < 0.0) != (full < 0.0)) {
        const double crossover = cap * empty / (empty - full)
                                 / Motion::kWaterDensity;
        unstable_range_str = std::string(empty < 0.0 ? "above " : "below ")
                             + Motion::FormatNum(crossover) + " L";
    }
    logger_->Warn("Arm model is unstable " + unstable_range_str
                  + " (g*S >= 0): " + values + "; check the calibration");

    return false;
}

/**
 * @brief Solves the arm model from a level sample and applies it.
 *
 * @param sample Averaged sample with the arm level
 *
 * @note Uses Eq. 34 with the current arm hang torque (from a two-angle or
 *       loaded calibration), since one sample can't separate the two arm
 *       terms. τ_g1,0 is converted to min. ETA extension (see
 *       `LevelTorqueAtExtension()`).
 *
 * @see Libra2Model::ArmLevelTorque Libra2Model::EtaMomentChange
 */
void Libra2MotionController::FinishLevelCalibration(
    const Motion::RestSample& sample) {
    if (std::abs(sample.angle) > Motion::kMaxLevelAngle) {
        calibrator_.FinishCalibration(false,
                                      "arm is not level (theta="
                                          + Motion::FormatNum(sample.angle)
                                          + " rad)");
        return;
    }

    // Eq. 34 with current m_arm·h_1·g
    const double hang_torque = arm_model_->hang_torque;
    const auto level = Libra2Model::ArmLevelTorque(SampleToArmTorque(sample),
                                                   sample.angle, hang_torque);
    if (!level) {
        calibrator_.FinishCalibration(false, "cos(theta) too small");
        return;
    }

    Motion::ArmModel arm;
    arm.source = "level";
    arm.cw_model = cw_model_.GetParams();
    // Convert to min. ETA extension (Eq. 44)
    arm.level_torque = *level
                       + (Motion::kGrav
                          * Libra2Model::EtaMomentChange(eta_extension_));
    arm.hang_torque = hang_torque;
    arm_model_ = arm;

    calibrator_.FinishCalibration(true, "Level calibration succeeded: τ_g1,0="
                                            + Motion::FormatNum(arm.level_torque)
                                            + " Nm"
                                            + (eta_extension_ > 0.0
                                                   ? " (at min. extension)"
                                                   : "")
                                            + ", m_arm*h_1*g="
                                            + Motion::FormatNum(arm.hang_torque)
                                            + " Nm");
}

/**
 * @brief Solves the arm model from the samples at both angles and applies it.
 *
 * @param sample_a Averaged sample at the first angle
 * @param sample_b Averaged sample at the second angle
 *
 * @note τ_g1,0 is converted to min. ETA extension (see
 *       `LevelTorqueAtExtension()`).
 *
 * @see Libra2Model::SolveTwoAngle Libra2Model::EtaMomentChange
 */
void Libra2MotionController::FinishTwoAngleCalibration(
    const Motion::RestSample& sample_a, const Motion::RestSample& sample_b) {
    // Separate statements so samples log in order (a, b)
    const double arm_torque_a = SampleToArmTorque(sample_a);
    const double arm_torque_b = SampleToArmTorque(sample_b);

    // Eq. 35 with measured angles
    auto arm = Libra2Model::SolveTwoAngle(arm_torque_a, sample_a.angle,
                                          arm_torque_b, sample_b.angle);
    if (!arm) {
        calibrator_.FinishCalibration(false,
                                      "measured angles too close together");
        return;
    }

    arm->source = "two-angle";
    arm->cw_model = cw_model_.GetParams();
    // Convert to min. ETA extension (Eq. 44)
    arm->level_torque += Motion::kGrav
                         * Libra2Model::EtaMomentChange(eta_extension_);
    arm_model_ = arm;
    const bool stable = CheckStability(arm->hang_torque);

    calibrator_.FinishCalibration(
        true, "Two-angle calibration succeeded: τ_g1,0="
                  + Motion::FormatNum(arm->level_torque) + " Nm"
                  + (eta_extension_ > 0.0 ? " (at min. extension)" : "")
                  + ", m_arm*h_1*g=" + Motion::FormatNum(arm->hang_torque)
                  + " Nm" + (stable ? "" : " (UNSTABLE; see log)"));
}

/**
 * @brief Applies a fitted friction model, and checks the arm model against the
 *        sweeps' holding torques.
 *
 * @param model Friction model from the calibrator
 *
 * @note Each bin's holding torque should equal -τ_ext,g(θ) at its center, so
 *       the model error per speed is the bins' mean of:
 *         holding + τ_ext,g(θ_bin)
 */
void Libra2MotionController::FinishFrictionCalibration(
    const Motion::FrictionModel& model) {
    friction_coulomb_ = model.coulomb;
    friction_viscous_ = model.viscous;
    friction_model_ = model;

    if (arm_model_ && cw_model_.IsValid()) {
        const double cw_mass = Libra2Model::CwMass(model.volume);

        std::string msg = "Arm model error vs. friction sweeps (mean):";
        for (const auto& point : model.points) {
            double err_sum = 0.0;
            for (const auto& bin : point.bins) {
                err_sum += bin.holding
                           + ComputeNetGravTorque(bin.angle, cw_mass);
            }
            msg += "\n  " + Motion::FormatRadAsDeg(point.vel) + " deg/s: "
                   + Motion::FormatNum(err_sum
                                       / static_cast<double>(point.bins.size()))
                   + " Nm";
        }
        logger_->Debug(msg);
    }

    calibrator_.FinishCalibration(true,
                                  "Friction calibration succeeded: tau_c="
                                      + Motion::FormatNum(model.coulomb)
                                      + " Nm, b="
                                      + Motion::FormatNum(model.viscous)
                                      + " Nm*s/rad (fit RMS "
                                      + Motion::FormatNum(model.residual_rms)
                                      + " Nm)");
}
