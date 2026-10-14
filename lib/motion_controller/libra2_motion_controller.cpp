/******************************************************************************
 * @file   libra2_motion_controller.cpp
 * @brief  LIBRA-II pitch motion controller implementation file.
 *
 * @author Christian Brice
 ******************************************************************************/

// Related Header
#include "libra2_motion_controller.h"

// C++ Standard Library Headers
#include <algorithm>
#include <cmath>
#include <string>
#include <utility>

// Project Headers
#include "joint_defs.h"

/* --- TABLE OF CONTENTS ---
 * !Local Helpers
 * !CW Model
 * !Pure Equations
 * !Rest Sampler
 * !Class Management
 * !Settings & Arm Model
 * !Calibration (public)
 * !Control Cycle
 * !Hooks
 * !Calibration (private)
 */

namespace Motion {

//------------------------------------------------------------------------------
// !Local Helpers
//------------------------------------------------------------------------------

namespace {

constexpr std::size_t kPitchIdx = Joint::Name::kPitch;

/**
 * @brief Sign of a number.
 *
 * @param x Number to check
 * @return double +1, -1, or 0 (for exactly 0)
 */
double Sign(double x) {
    return (x > 0.0) - (x < 0.0);
}

}  // namespace

/**
 * @brief Tells whether a calibration is in progress.
 *
 * @param status Calibration status to check
 * @return true Sampling or moving stage
 * @return false Idle, succeeded, or failed
 */
bool IsCalibrationRunning(CalibrationStatus status) {
    return status != CalibrationStatus::kIdle
           && status != CalibrationStatus::kSucceeded
           && status != CalibrationStatus::kFailed;
}

//------------------------------------------------------------------------------
// !CW Model
//------------------------------------------------------------------------------

/**
 * @brief Builds the CW model r_P(V) = A + B*V and checks it.
 *
 * @param cw_pivot_dist_m A: r_P at 0 L (m); the sign is ignored. 0 means "not
 *                        measured yet" and leaves the model invalid.
 * @param cw_pivot_slope_m_per_l B: change in r_P per liter (m/L); may be
 *                               negative, so the sign is kept
 * @param capacity_l Largest volume the CW can hold (L)
 */
CwModel::CwModel(double cw_pivot_dist_m, double cw_pivot_slope_m_per_l,
                 double capacity_l)
    : capacity_l_(capacity_l) {
    if (!std::isfinite(capacity_l) || capacity_l <= 0.0) {
        error_ = "CW capacity must be a positive number of liters";
        return;
    }
    if (!std::isfinite(cw_pivot_dist_m)
        || !std::isfinite(cw_pivot_slope_m_per_l)) {
        error_ = "r_P intercept or slope is not a number";
        return;
    }
    if (cw_pivot_dist_m == 0.0) {
        error_ = "r_P is not set (intercept = 0)";
        return;
    }

    r_p_m_ = std::abs(cw_pivot_dist_m);
    r_p_slope_m_per_l_ = cw_pivot_slope_m_per_l;

    // The moment A*m + (B/rho)*m^2 must keep increasing up to full, otherwise
    // two masses give the same moment and MassForMoment() is ambiguous. The
    // slope there is A + 2*B*V; it is linear in V, so checking the end is
    // enough. (This also guarantees r_P(V) > 0 on [0, capacity].)
    if (!(r_p_m_ + 2.0 * r_p_slope_m_per_l_ * capacity_l_ > 0.0)) {
        error_ = "r_P line is too steep: the CW moment must increase with "
                 "volume up to full (need A + 2*B*capacity > 0)";
    }
}

/**
 * @brief Mass of a full CW.
 *
 * @return double Mass at capacity (kg)
 */
double CwModel::CapacityMass() const {
    return capacity_l_ * kWaterDensity;
}

/**
 * @brief Distance from the pivot to the CW center of mass at a given volume.
 *
 * @param volume_l CW volume (L)
 * @return double r_P(V) = A + B*V (m)
 */
double CwModel::PivotDist(double volume_l) const {
    return r_p_m_ + r_p_slope_m_per_l_ * volume_l;
}

/**
 * @brief Gravity moment of the CW, m_cw r_P (Eq. 28 gives V from m_cw).
 *
 * @param mass_kg CW mass (kg)
 * @return double m_cw * r_P(V) (kg*m)
 */
double CwModel::Moment(double mass_kg) const {
    return mass_kg * PivotDist(mass_kg / kWaterDensity);
}

/**
 * @brief Inverts `Moment()`: the CW mass that gives a requested moment.
 *
 * @param moment Requested m_cw r_P (kg*m)
 * @param clamped If given, set to true when the request is outside what an
 *                empty-to-full CW can produce
 * @return double CW mass (kg), limited to [0, capacity]; 0 if the model is
 *                invalid
 */
double CwModel::MassForMoment(double moment, bool* clamped) const {
    if (clamped != nullptr) {
        *clamped = false;
    }
    if (!IsValid()) {
        return 0.0;
    }

    const double cap = CapacityMass();
    if (!(moment > 0.0)) {
        if (clamped != nullptr) {
            *clamped = (moment < 0.0);
        }
        return 0.0;
    }
    const double max_moment = Moment(cap);
    if (moment >= max_moment) {
        if (clamped != nullptr) {
            *clamped = (moment > max_moment);
        }
        return cap;
    }

    // Moment = A*m + c*m^2 with c = B/rho. The root written as 2M/(A+sqrt(..))
    // stays accurate when c is 0 or tiny (it reduces to M/A).
    const double c = r_p_slope_m_per_l_ / kWaterDensity;
    const double disc = std::max(0.0, r_p_m_ * r_p_m_ + 4.0 * c * moment);
    return 2.0 * moment / (r_p_m_ + std::sqrt(disc));
}

//------------------------------------------------------------------------------
// !Pure Equations
//------------------------------------------------------------------------------

namespace Libra2Model {

/**
 * @brief Eq. 28: CW mass from its volume.
 *
 * @param volume_l CW volume (L)
 * @return double m_cw (kg)
 */
double CwMass(double volume_l) {
    return kWaterDensity * volume_l;  // Eq. 28
}

/**
 * @brief Eq. 29: gravity torque of the arm alone (CW removed from the
 *        measured torque).
 *
 * @param torque_meas Measured torque on the pivot (Nm)
 * @param cw_moment m_cw r_P (kg*m)
 * @param grav g (m/s^2)
 * @param theta Pitch angle (rad)
 * @return double tau_g1(theta) (Nm)
 */
double ArmGravTorque(double torque_meas, double cw_moment, double grav,
                     double theta) {
    // Eq. 29
    return torque_meas - grav * cw_moment * std::cos(theta);
}

/**
 * @brief Eq. 34: arm torque at level (theta = 0), from one measurement.
 *
 * @param arm_torque tau_g1(theta) (Nm)
 * @param theta Pitch angle (rad)
 * @param arm_sag m_arm h1 g (Nm)
 * @return std::optional<double> tau_g1,0 (Nm); none if cos(theta) is too small
 */
std::optional<double> ArmLevelTorque(double arm_torque, double theta,
                                     double arm_sag) {
    // Eq. 34 (Eq. 31 when theta = 0)
    const double c = std::cos(theta);
    if (std::abs(c) < kMinCosForBalance) {
        return std::nullopt;
    }
    return (arm_torque - arm_sag * std::sin(theta)) / c;
}

/**
 * @brief Eq. 35: solves tau_g1,0 and m_arm h1 g from two angles.
 *
 * @param arm_torque_a tau_g1 at angle a (Nm)
 * @param theta_a Angle a (rad)
 * @param arm_torque_b tau_g1 at angle b (Nm)
 * @param theta_b Angle b (rad)
 * @param[out] arm_level_torque tau_g1,0 (Nm)
 * @param[out] arm_sag m_arm h1 g (Nm)
 * @return true Solved
 * @return false The angles are too close together; outputs are untouched
 */
bool SolveTwoAngle(double arm_torque_a, double theta_a, double arm_torque_b,
                   double theta_b, double& arm_level_torque, double& arm_sag) {
    // Eq. 35
    const double s = std::sin(theta_b - theta_a);
    if (std::abs(s) < std::sin(0.5 * kMinAngleSeparation)) {
        return false;
    }
    arm_level_torque = (arm_torque_a * std::sin(theta_b)
                        - arm_torque_b * std::sin(theta_a))
                       / s;
    arm_sag = (arm_torque_b * std::cos(theta_a)
               - arm_torque_a * std::cos(theta_b))
              / s;
    return true;
}

/**
 * @brief Eq. 36: net gravity torque (arm + CW) about the pivot.
 *
 * @param arm_level_torque tau_g1,0 (Nm)
 * @param arm_sag m_arm h1 g (Nm)
 * @param cw_moment m_cw r_P (kg*m)
 * @param grav g (m/s^2)
 * @param theta Pitch angle (rad)
 * @return double tau_ext,g(theta) (Nm)
 */
double NetGravTorque(double arm_level_torque, double arm_sag, double cw_moment,
                     double grav, double theta) {
    // Eq. 36
    return (arm_level_torque + grav * cw_moment) * std::cos(theta)
           + arm_sag * std::sin(theta);
}

/**
 * @brief Eq. 37: actuator torque to follow a reference with gravity canceled.
 *
 * @param inertia I (kg*m^2)
 * @param inertia_rate dI/dt (kg*m^2/s)
 * @param friction_viscous b (Nm*s/rad)
 * @param friction_coulomb tau_c (Nm)
 * @param ref_acc Reference acceleration (rad/s^2)
 * @param ref_vel Reference velocity (rad/s)
 * @param net_grav_torque tau_ext,g(theta) at the MEASURED angle (Nm)
 * @return double tau_actuator (Nm); not clamped
 */
double TorqueCommand(double inertia, double inertia_rate,
                     double friction_viscous, double friction_coulomb,
                     double ref_acc, double ref_vel, double net_grav_torque) {
    // Eq. 37
    return inertia * ref_acc + (friction_viscous + inertia_rate) * ref_vel
           + friction_coulomb * Sign(ref_vel) - net_grav_torque;
}

/**
 * @brief Eq. 38: CW moment that balances the arm at an angle.
 *
 * @param arm_level_torque tau_g1,0 (Nm)
 * @param arm_sag m_arm h1 g (Nm)
 * @param grav g (m/s^2)
 * @param theta Angle to balance at (rad)
 * @return std::optional<double> m_cw,bal r_P (kg*m); none if cos(theta) is
 *                               too small
 */
std::optional<double> BalanceCwMoment(double arm_level_torque, double arm_sag,
                                      double grav, double theta) {
    // Eq. 38
    if (std::abs(std::cos(theta)) < kMinCosForBalance) {
        return std::nullopt;
    }
    return -(arm_level_torque + arm_sag * std::tan(theta)) / grav;
}

/**
 * @brief Eq. 39: net gravity torque at the target for the current CW.
 *
 * @param grav g (m/s^2)
 * @param theta_tgt Target angle (rad)
 * @param cw_moment m_cw r_P (kg*m)
 * @param cw_moment_bal m_cw,bal r_P (kg*m), from `BalanceCwMoment()`
 * @return double tau_ext,g(theta_tgt) (Nm)
 */
double TargetNetGravTorque(double grav, double theta_tgt, double cw_moment,
                           double cw_moment_bal) {
    // Eq. 39
    return grav * std::cos(theta_tgt) * (cw_moment - cw_moment_bal);
}

}  // namespace Libra2Model

//------------------------------------------------------------------------------
// !Rest Sampler
//------------------------------------------------------------------------------

/**
 * @brief Discards the running window.
 */
void RestSampler::Reset() {
    count_ = 0;
    elapsed_ = 0.0;
    sum_theta_ = 0.0;
    sum_torque_ = 0.0;
    sum_volume_ = 0.0;
}

/**
 * @brief Adds one cycle of measurements to the window.
 *
 * @param dt Time since the last cycle (s)
 * @param pos Measured angle (rad)
 * @param vel Measured velocity (rad/s)
 * @param torque_actuator Measured actuator torque (Nm)
 * @param trajectory_active A trajectory is being followed
 * @param volume_l Latest CW volume (L), if known
 * @return std::optional<RestSample> The averages once the window is complete;
 *                                   none otherwise
 */
std::optional<RestSample> RestSampler::Add(
    double dt, double pos, double vel, double torque_actuator,
    bool trajectory_active, const std::optional<double>& volume_l) {
    const bool still = !trajectory_active && std::abs(vel) <= kRestVelTolerance
                       && volume_l.has_value();
    if (!still) {
        Reset();
        return std::nullopt;
    }

    // Restart the window if pose or CW volume drifted from where it began
    if (count_ > 0
        && (std::abs(pos - anchor_theta_) > kRestPosTolerance
            || std::abs(*volume_l - anchor_volume_) > kRestVolTolerance)) {
        Reset();
    }

    if (count_ == 0) {
        anchor_theta_ = pos;
        anchor_volume_ = *volume_l;
    } else {
        elapsed_ += dt;
    }

    sum_theta_ += pos;
    sum_torque_ += torque_actuator;
    sum_volume_ += *volume_l;
    ++count_;

    if (elapsed_ >= kRestWindowTime && count_ >= 2) {
        RestSample sample;
        sample.theta = sum_theta_ / count_;
        sample.torque_actuator = sum_torque_ / count_;
        sample.volume_l = sum_volume_ / count_;
        Reset();
        return sample;
    }
    return std::nullopt;
}

//------------------------------------------------------------------------------
// !Class Management
//------------------------------------------------------------------------------

/**
 * @brief Convenience constructor: every parameter comes from the kDefault*
 *        constants in the header. This is the one HebiThread uses.
 *
 * @param logger Logger to use (NOT owned; must outlive this object)
 */
Libra2MotionController::Libra2MotionController(Logger& logger)
    : Libra2MotionController(logger, kDefaultCwPivotDist, kDefaultCwPivotSlope,
                             kDefaultCwCapacity, kDefaultGrav,
                             kDefaultFrictionCoulomb, kDefaultFrictionViscous,
                             kDefaultInertia, kDefaultInertiaRate) {}

/**
 * @brief Full constructor.
 *
 * @param logger Logger to use (NOT owned; must outlive this object)
 * @param cw_pivot_dist_m A: r_P at 0 L (m); 0 = not measured (model invalid)
 * @param cw_pivot_slope_m_per_l B: change in r_P per liter (m/L)
 * @param cw_capacity_l CW capacity (L)
 * @param grav g (m/s^2)
 * @param friction_coulomb tau_c (Nm)
 * @param friction_viscous b (Nm*s/rad)
 * @param inertia I (kg*m^2)
 * @param inertia_rate dI/dt (kg*m^2/s)
 */
Libra2MotionController::Libra2MotionController(
    Logger& logger, double cw_pivot_dist_m, double cw_pivot_slope_m_per_l,
    double cw_capacity_l, double grav, double friction_coulomb,
    double friction_viscous, double inertia, double inertia_rate)
    : AbstractMotionController(logger, Joint::GetHebiStrings().size()),
      cw_model_(cw_pivot_dist_m, cw_pivot_slope_m_per_l, cw_capacity_l),
      grav_(grav),
      friction_coulomb_(friction_coulomb),
      friction_viscous_(friction_viscous),
      inertia_(inertia),
      inertia_rate_(inertia_rate) {
    if (!cw_model_.IsValid()) {
        logger_.Error("CW model invalid: " + cw_model_.Error()
                      + " (calibration and feedforward are unavailable)");
    }
}

//------------------------------------------------------------------------------
// !Settings & Arm Model
//------------------------------------------------------------------------------

/**
 * @brief Selects tandem or CW-primary motion.
 *
 * @param mode Mode to use from the next target on
 * @return true Mode changed
 * @return false CW-primary refused (no usable arm model and CW model)
 */
bool Libra2MotionController::SetMotionMode(MotionMode mode) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (mode == MotionMode::kCwPrimary
        && !(calibrated_ && cw_model_.IsValid())) {
        logger_.Warn("CW-primary mode needs a calibrated arm model and a valid "
                     "CW model; staying in the current mode");
        return false;
    }

    // TODO: DECISION (not asked): a mode change only takes effect from the NEXT
    //       target; it resets the CW phase and the "CW done" flag so a move in
    //       progress is not switched mid-way. The UI is meant to disable the
    //       switch while moving (as requested), so this only matters if
    //       something else calls it during a move. Revisit if mid-move
    //       switching is ever wanted.
    mode_ = mode;
    cw_phase_ = CwPhase::kIdle;
    cw_assist_done_ = false;
    logger_.Info(std::string("Motion mode: ")
                 + (mode == MotionMode::kTandem ? "tandem" : "CW-primary"));
    return true;
}

/**
 * @brief Gets the selected motion mode.
 *
 * @return MotionMode Current mode
 */
MotionMode Libra2MotionController::GetMotionMode() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return mode_;
}

/**
 * @brief Stores the latest CW volume (from the TankWidget).
 *
 * @param liters CW volume (L)
 */
void Libra2MotionController::SetCwVolume(double liters) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!std::isfinite(liters) || liters < 0.0) {
        logger_.Warn("Ignoring invalid CW volume: " + MotionFormat(liters)
                     + " L");
        return;
    }
    cw_volume_l_ = liters;
}

/**
 * @brief Enables or disables automatic torque compensation (the torque
 *        throttle and, when uncalibrated, the reactive CW command).
 *
 * @param enabled true to enable
 */
void Libra2MotionController::EnableTorqueComp(const bool& enabled) {
    std::lock_guard<std::mutex> lock(mutex_);
    atc_en_ = enabled;
    if (!atc_en_) {
        movement_en_ = true;  // "manual" mode: movement is always allowed
    }
    logger_.Debug("ATC - Automatic torque compensation "
                  + std::string(enabled ? "enabled" : "disabled"));
}

/**
 * @brief Sets the soft and hard torque limits of the throttle.
 *
 * @param lower_bound Soft limit (Nm); playback slows at or above it
 * @param upper_bound Hard limit (Nm); playback pauses above it
 * @return true Bounds applied
 * @return false Invalid (need 0 <= lower < upper); nothing changed
 */
bool Libra2MotionController::SetTorqueCompBounds(const double& lower_bound,
                                                 const double& upper_bound) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!(lower_bound >= 0.0) || !(lower_bound < upper_bound)) {
        logger_.Warn("ATC - Ignoring invalid torque compensation bounds: lower="
                     + MotionFormat(lower_bound)
                     + " Nm, upper=" + MotionFormat(upper_bound) + " Nm");
        return false;
    }
    torque_comp_lower_bound_ = lower_bound;
    torque_comp_upper_bound_ = upper_bound;
    logger_.Debug("ATC - Torque compensation bounds updated: lower="
                  + MotionFormat(lower_bound)
                  + " Nm, upper=" + MotionFormat(upper_bound) + " Nm");
    return true;
}

/**
 * @brief Sets the CW-primary hand-off band.
 *
 * @param band_rad Angle band around the target (rad); the CW stops and the
 *                 actuator takes over once the arm is inside it
 * @return true Band applied
 * @return false Not a positive number; nothing changed
 */
bool Libra2MotionController::SetHandoffBand(const double& band_rad) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!(band_rad > 0.0) || !std::isfinite(band_rad)) {
        logger_.Warn("Ignoring invalid hand-off band: " + MotionFormat(band_rad)
                     + " rad");
        return false;
    }
    handoff_band_ = band_rad;
    logger_.Debug("Hand-off band updated: " + MotionFormat(band_rad) + " rad");
    return true;
}

/**
 * @brief Sets the CW volume tolerance (the "volume reached" deadband).
 *
 * @param tolerance_l Tolerance (L); prevents fill/drain chatter near the
 *                    target volume
 * @return true Tolerance applied
 * @return false Negative or not finite; nothing changed
 */
bool Libra2MotionController::SetCwVolumeTolerance(const double& tolerance_l) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!(tolerance_l >= 0.0) || !std::isfinite(tolerance_l)) {
        logger_.Warn("Ignoring invalid CW volume tolerance: "
                     + MotionFormat(tolerance_l) + " L");
        return false;
    }
    cw_volume_tolerance_ = tolerance_l;
    return true;
}

/**
 * @brief Sets the arm model directly (e.g., from a saved calibration).
 *
 * @param arm_level_torque tau_g1,0 (Nm)
 * @param arm_sag m_arm h1 g (Nm)
 */
void Libra2MotionController::SetArmModel(double arm_level_torque,
                                         double arm_sag) {
    std::lock_guard<std::mutex> lock(mutex_);
    arm_level_torque_ = arm_level_torque;
    arm_sag_ = arm_sag;
    calibrated_ = true;
    logger_.Info("Arm model set: arm_level_torque="
                 + MotionFormat(arm_level_torque)
                 + " Nm, arm_sag=" + MotionFormat(arm_sag) + " Nm");
}

/**
 * @brief Forgets the arm model; feedforward, the predictive CW command, and
 *        CW-primary mode become unavailable.
 */
void Libra2MotionController::ClearArmModel() {
    std::lock_guard<std::mutex> lock(mutex_);
    arm_level_torque_ = 0.0;
    arm_sag_ = 0.0;
    calibrated_ = false;
    mode_ = MotionMode::kTandem;  // CW-primary needs the model
    cw_phase_ = CwPhase::kIdle;
    logger_.Info("Arm model cleared; feedforward and predictive CW disabled");
}

/**
 * @brief Tells whether an arm model is available.
 *
 * @return true Calibrated or set with `SetArmModel()`
 * @return false No arm model
 */
bool Libra2MotionController::IsCalibrated() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return calibrated_;
}

/**
 * @brief Gets tau_g1,0.
 *
 * @return double Arm torque at level (Nm)
 */
double Libra2MotionController::GetArmLevelTorque() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return arm_level_torque_;
}

/**
 * @brief Gets m_arm h1 g.
 *
 * @return double Arm sag term (Nm)
 */
double Libra2MotionController::GetArmSag() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return arm_sag_;
}

/**
 * @brief Eq. 36 for a given pose and CW mass.
 *
 * @note Does NOT lock, because `Update()` calls it while already holding the
 *       lock. From outside, call it only while the controller is not running.
 *
 * @param theta Pitch angle (rad)
 * @param cw_mass_kg CW mass (kg)
 * @return double tau_ext,g(theta) (Nm)
 */
double Libra2MotionController::ComputeNetGravTorque(double theta,
                                                    double cw_mass_kg) const {
    return Libra2Model::NetGravTorque(arm_level_torque_, arm_sag_,
                                      cw_model_.Moment(cw_mass_kg), grav_,
                                      theta);
}

//------------------------------------------------------------------------------
// !Calibration (public)
//------------------------------------------------------------------------------

/**
 * @brief Starts the level calibration (Eq. 34 at theta ~ 0). The arm must
 *        already be level and at rest.
 *
 * @return true Started
 * @return false Another calibration is running, or the CW model is invalid
 */
bool Libra2MotionController::StartLevelCalibration() {
    std::lock_guard<std::mutex> lock(mutex_);
    if (IsCalibrationRunning(status_)) {
        logger_.Warn("Calibration already running; ignoring level request");
        return false;
    }
    if (!cw_model_.IsValid()) {
        logger_.Error("Can't calibrate: CW model invalid (" + cw_model_.Error()
                      + ")");
        return false;
    }

    two_angle_ = false;
    BeginStage(CalibrationStatus::kSamplingLevel);
    logger_.Info("Level calibration started (arm must be level and at rest)");
    return true;
}

/**
 * @brief Starts the two-angle calibration (Eq. 35). The controller requests
 *        the moves through `Libra2Command::move_to`.
 *
 * @param angle_a First angle (rad, joint space)
 * @param angle_b Second angle (rad, joint space)
 * @return true Started
 * @return false Another calibration is running, the CW model is invalid, or
 *               the angles are out of range / too close
 */
bool Libra2MotionController::StartTwoAngleCalibration(double angle_a,
                                                      double angle_b) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (IsCalibrationRunning(status_)) {
        logger_.Warn("Calibration already running; ignoring two-angle request");
        return false;
    }
    if (!cw_model_.IsValid()) {
        logger_.Error("Can't calibrate: CW model invalid (" + cw_model_.Error()
                      + ")");
        return false;
    }
    if (!std::isfinite(angle_a) || !std::isfinite(angle_b)
        || std::abs(angle_a) > kMaxCalibrationAngle
        || std::abs(angle_b) > kMaxCalibrationAngle
        || std::abs(angle_b - angle_a) < kMinAngleSeparation) {
        logger_.Error("Two-angle calibration rejected: need |angle| <= "
                      + MotionFormat(kMaxCalibrationAngle) + " rad and "
                      + "|b - a| >= " + MotionFormat(kMinAngleSeparation)
                      + " rad (got a=" + MotionFormat(angle_a)
                      + ", b=" + MotionFormat(angle_b) + ")");
        return false;
    }

    two_angle_ = true;
    angle_a_ = angle_a;
    angle_b_ = angle_b;
    BeginStage(CalibrationStatus::kMovingToA);
    logger_.Info("Two-angle calibration started: a=" + MotionFormat(angle_a)
                 + " rad, b=" + MotionFormat(angle_b) + " rad");
    return true;
}

/**
 * @brief Cancels a running calibration (no effect otherwise).
 */
void Libra2MotionController::CancelCalibration() {
    std::lock_guard<std::mutex> lock(mutex_);
    CancelCalibrationLocked();
}

/**
 * @brief Gets the calibration progress.
 *
 * @return CalibrationStatus Current status
 */
CalibrationStatus Libra2MotionController::GetCalibrationStatus() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return status_;
}

//------------------------------------------------------------------------------
// !Control Cycle
//------------------------------------------------------------------------------

/**
 * @brief Torque throttle with hysteresis (same behavior as LIBRA-I's
 *        `CheckTorqueControl`): slows or pauses the trajectory and says which
 *        way the CW should go.
 *
 * @param torque_magnitude |measured actuator torque| (Nm)
 * @param direction CW direction that relieves the actuator
 * @return AtcResult Playback rate and the CW command (none = nothing to report)
 */
Libra2MotionController::AtcResult Libra2MotionController::UpdateTorqueComp(
    double torque_magnitude, CwCommand direction) {
    AtcResult result;

    if (!atc_en_) {
        movement_en_ = true;
        return result;  // defaults: full speed, no report
    }

    bool in_slow_band = false;

    if (torque_magnitude > torque_comp_upper_bound_) {
        // Hard limit: stop movement first, then report the direction on the
        // following cycles (same order as LIBRA-I)
        if (movement_en_) {
            movement_en_ = false;

            logger_.Debug("ATC - Movement disabled due to high torque ("
                          + MotionFormat(torque_magnitude) + " Nm > "
                          + MotionFormat(torque_comp_upper_bound_) + " Nm)");
        } else {
            result.cw_command = direction;
        }

    } else if (torque_magnitude >= torque_comp_lower_bound_) {
        // Soft limit: keep reporting the direction and slow the trajectory
        result.cw_command = direction;
        in_slow_band = true;

    } else {
        // Acceptable torque: stop balancing and re-enable movement
        result.cw_command = CwCommand::kIdle;
        if (!movement_en_) {
            movement_en_ = true;

            logger_.Debug("ATC - Movement re-enabled ("
                          + MotionFormat(torque_magnitude) + " Nm < "
                          + MotionFormat(torque_comp_lower_bound_) + " Nm)");
        }
    }

    // TODO: DECISION (not asked): the throttle looks at |torque| only. That is
    //       right while the actuator LIFTS the arm and the CW is catching up.
    //       If the CW ever leads the trajectory (the actuator brakes instead of
    //       lifts), slowing the trajectory is the WRONG response and the
    //       torque would stay high. Check once the real CW speed is known; a
    //       signed check (torque vs. direction of travel) would fix it.
    if (!movement_en_) {
        result.playback_rate = 0.0;
    } else {
        result.playback_rate = in_slow_band ? kAtcSlowPlaybackRate : 1.0;
    }
    return result;
}

/**
 * @brief Predictive CW command: drive the CW toward the mass that balances the
 *        arm at the target (Eq. 38 -> inverse of Eq. 28/40a).
 *
 * @param theta_tgt Target angle (rad)
 * @param cw_mass_kg Current CW mass (kg)
 * @param[out] cw_mass_tgt_kg If given, set to the target mass (kg)
 * @return std::optional<CwCommand> Fill/drain/idle; none if the balance mass is
 *                                  undefined at this angle
 */
std::optional<CwCommand> Libra2MotionController::PredictiveCwCommand(
    double theta_tgt, double cw_mass_kg, double* cw_mass_tgt_kg) const {
    // Eq. 38 -> target mass (clamped to what the CW can hold)
    const auto moment_bal = Libra2Model::BalanceCwMoment(arm_level_torque_,
                                                         arm_sag_, grav_,
                                                         theta_tgt);
    if (!moment_bal) {
        return std::nullopt;
    }
    const double mass_tgt = cw_model_.MassForMoment(*moment_bal);
    if (cw_mass_tgt_kg != nullptr) {
        *cw_mass_tgt_kg = mass_tgt;
    }

    // Eq. 40a
    const double diff = mass_tgt - cw_mass_kg;
    if (std::abs(diff) <= cw_volume_tolerance_ * kWaterDensity) {
        return CwCommand::kIdle;
    }

    // TODO: DECISION (not asked): the command flips to idle the moment the
    //       volume is within tolerance. Valves have latency, so water keeps
    //       flowing for a moment after kIdle and the CW can overshoot the
    //       tolerance, then reverse (a small fill/drain limit cycle). If that
    //       is observed, add an early-stop margin (stop `latency * flow rate`
    //       before the target).
    return (diff > 0.0) ? CwCommand::kFill : CwCommand::kDrain;
}

/**
 * @brief One cycle of CW-primary motion: leave the arm free and let the CW move
 *        it until the volume OR the angle criterion is met.
 *
 * @param theta Measured angle (rad)
 * @param theta_tgt Target angle (rad)
 * @param[in,out] command Command to fill when the arm stays free
 * @return true The arm stays free this cycle
 * @return false Hand-off: the actuator takes over (a move is requested)
 */
bool Libra2MotionController::RunCwPrimary(double theta, double theta_tgt,
                                          Libra2Command& command) {
    double mass_tgt = 0.0;
    const auto cw = PredictiveCwCommand(theta_tgt, *cw_mass_kg_, &mass_tgt);
    if (!cw) {
        logger_.Warn("CW-primary: target mass undefined at this angle; handing "
                     "off to the actuator");
        cw_phase_ = CwPhase::kIdle;
        cw_assist_done_ = true;
        pending_move_ = theta_tgt;
        return false;
    }

    const bool volume_reached = (*cw == CwCommand::kIdle);
    const bool angle_reached = std::abs(theta - theta_tgt) <= handoff_band_;

    if (volume_reached || angle_reached) {
        // Log how far the model was from reality; the criterion that did NOT
        // fire shows the difference
        logger_.Info(
            "CW-primary hand-off (" + std::string(volume_reached ? "volume" : "")
            + std::string(volume_reached && angle_reached ? "+" : "")
            + std::string(angle_reached ? "angle" : "") + " reached): theta="
            + MotionFormat(theta) + " rad, target=" + MotionFormat(theta_tgt)
            + " rad, angle error=" + MotionFormat(theta - theta_tgt)
            + " rad; volume=" + MotionFormat(*cw_mass_kg_ / kWaterDensity)
            + " L, target volume=" + MotionFormat(mass_tgt / kWaterDensity)
            + " L, volume error="
            + MotionFormat((*cw_mass_kg_ - mass_tgt) / kWaterDensity) + " L");

        // TODO: DECISION (not asked): after the hand-off the CW goes (and
        //       stays) idle until the next target, so the predictive command
        //       cannot refill the CW while the actuator finishes the move. The
        //       cost: the CW will not trim a residual error left by an
        //       inaccurate model, and the actuator may end up holding a large
        //       steady torque. If that shows up in the hand-off log, let the
        //       CW resume (e.g., with the reactive torque command).
        cw_phase_ = CwPhase::kIdle;
        cw_assist_done_ = true;
        pending_move_ = theta_tgt;
        return false;
    }

    // Arm stays free (effort only, no position command); the CW does the moving
    command.mode[kPitchIdx] = CommandMode::kEffortOnly;
    command.cw_command = cw;
    command.playback_rate = 1.0;
    movement_en_ = true;
    return true;
}

/**
 * @brief Runs one control cycle.
 *
 * @param state Measurements and the previous cycle's reference
 * @return Libra2Command What to do with the actuator and the CW this cycle
 */
Libra2Command Libra2MotionController::Update(const MotionState& state) {
    std::lock_guard<std::mutex> lock(mutex_);
    Libra2Command command(n_joints_);

    if (!IsValid(state) || n_joints_ <= kPitchIdx) {
        if (!state_warned_) {
            logger_.Error("MotionState arrays must each have "
                          + std::to_string(n_joints_) + " entries");
            state_warned_ = true;
        }
        return command;
    }

    const double theta = state.pos[kPitchIdx];
    const double vel = state.vel[kPitchIdx];
    const double torque = state.torque_actuator[kPitchIdx];

    // Eq. 28: CW mass from the latest volume (none until one is received)
    cw_mass_kg_ = cw_volume_l_
                      ? std::optional<double>(Libra2Model::CwMass(*cw_volume_l_))
                      : std::nullopt;

    const bool calibrating = IsCalibrationRunning(status_);
    const bool model_ready = calibrated_ && cw_model_.IsValid()
                             && cw_mass_kg_.has_value();
    std::optional<double> target;
    if (target_pos_) {
        target = (*target_pos_)[kPitchIdx];
    }

    // CW-primary: the arm is left free while the CW moves it
    bool arm_free = false;
    if (mode_ == MotionMode::kCwPrimary && cw_phase_ == CwPhase::kCwMoving) {
        if (!model_ready || !target || calibrating) {
            // TODO: DECISION (not asked): if the model, CW volume, or target is
            //       lost (or a calibration starts) while the arm is free, hand
            //       off to the actuator right away instead of leaving the arm
            //       uncontrolled. The actuator then moves to the target if
            //       there still is one. A free arm falling under gravity is the
            //       thing to avoid, so this errs on the side of grabbing it.
            logger_.Warn("CW-primary: model, CW volume, or target unavailable; "
                         "handing off to the actuator");
            cw_phase_ = CwPhase::kIdle;
            cw_assist_done_ = true;
            if (target) {
                pending_move_ = *target;
            }
        } else {
            arm_free = RunCwPrimary(theta, *target, command);
        }
    }

    // Tandem / holding: actuator follows the trajectory (or holds) with
    // gravity feedforward while the CW works toward its target
    if (!arm_free) {
        // Throttle the trajectory from the measured torque (1 / 0.5 / 0).
        // Filling relieves a positive (joint-space) actuator torque.
        const CwCommand direction = (torque > 0.0) ? CwCommand::kFill
                                                   : CwCommand::kDrain;
        const AtcResult atc = UpdateTorqueComp(std::abs(torque), direction);
        command.playback_rate = atc.playback_rate;
        command.cw_command = atc.cw_command;  // reactive (LIBRA-I style)

        if (calibrating) {
            // Feedforward stays off and the CW stays reactive while calibrating
            RunCalibration(state, theta, vel, torque);
        } else {
            // Once calibrated, the predictive target replaces the reactive
            // command. After a CW-primary hand-off the CW stays idle.
            if (model_ready && target) {
                const auto predicted = cw_assist_done_
                                           ? std::optional<CwCommand>(
                                               CwCommand::kIdle)
                                           : PredictiveCwCommand(*target,
                                                                 *cw_mass_kg_,
                                                                 nullptr);
                if (predicted) {
                    command.cw_command = predicted;
                }
            }

            // Eq. 36-37 (no clamping)
            if (model_ready) {
                const double torque_ext_g = ComputeNetGravTorque(theta,
                                                                 *cw_mass_kg_);
                command.torque_actuator_cmd[kPitchIdx] =
                    Libra2Model::TorqueCommand(inertia_, inertia_rate_,
                                               friction_viscous_,
                                               friction_coulomb_,
                                               state.ref_acc[kPitchIdx],
                                               state.ref_vel[kPitchIdx],
                                               torque_ext_g);
            }
        }
    }

    // One-shot request to move (calibration moves, CW-primary hand-off)
    if (pending_move_) {
        JointValues move = state.pos;
        move[kPitchIdx] = *pending_move_;
        command.move_to = move;
        pending_move_.reset();
    }

    ComputeDiagnostics(theta, target, command.diagnostics);
    return command;
}

/**
 * @brief Fills the log-only values (Eq. 28, 36, 38-40).
 *
 * @param theta Measured angle (rad)
 * @param theta_tgt Target angle (rad), if any
 * @param[out] diagnostics Values to fill; entries that can't be computed stay
 *                         empty
 */
void Libra2MotionController::ComputeDiagnostics(
    double theta, const std::optional<double>& theta_tgt,
    Libra2Diagnostics& diagnostics) const {
    diagnostics.cw_mass_kg = cw_mass_kg_;
    if (!calibrated_ || !cw_model_.IsValid()) {
        return;
    }

    std::optional<double> moment;
    if (cw_mass_kg_) {
        moment = cw_model_.Moment(*cw_mass_kg_);
        diagnostics.torque_ext_g = Libra2Model::NetGravTorque(arm_level_torque_,
                                                              arm_sag_, *moment,
                                                              grav_, theta);
    }

    // Eq. 38-40
    if (theta_tgt) {
        const auto moment_bal = Libra2Model::BalanceCwMoment(arm_level_torque_,
                                                             arm_sag_, grav_,
                                                             *theta_tgt);
        if (moment_bal) {
            const double mass_tgt = cw_model_.MassForMoment(*moment_bal);
            diagnostics.cw_mass_tgt_kg = mass_tgt;
            if (cw_mass_kg_) {
                diagnostics.cw_mass_change_kg = mass_tgt - *cw_mass_kg_;
                diagnostics.torque_ext_g_tgt =
                    Libra2Model::TargetNetGravTorque(grav_, *theta_tgt, *moment,
                                                     *moment_bal);
            }
        }
    }
}

//------------------------------------------------------------------------------
// !Hooks
//------------------------------------------------------------------------------

/**
 * @brief A new target arrived: cancel any calibration move and, in CW-primary
 *        mode, let the CW start moving the arm.
 */
void Libra2MotionController::OnTargetChanged() {
    // A manual target overrides any calibration move
    CancelCalibrationLocked();

    // The mode is read here, so a mode change applies from the next target
    // (see the DECISION note in SetMotionMode)
    cw_assist_done_ = false;
    cw_phase_ = (mode_ == MotionMode::kCwPrimary) ? CwPhase::kCwMoving
                                                  : CwPhase::kIdle;
}

/**
 * @brief The target was dropped: cancel calibration and CW assistance.
 */
void Libra2MotionController::OnStop() {
    CancelCalibrationLocked();
    cw_phase_ = CwPhase::kIdle;
    cw_assist_done_ = false;
    pending_move_.reset();
}

/**
 * @brief Full runtime reset (disconnect): like `OnStop()` plus the ATC and
 *        warning latches. Settings and the arm model are kept.
 */
void Libra2MotionController::OnReset() {
    CancelCalibrationLocked();
    cw_phase_ = CwPhase::kIdle;
    cw_assist_done_ = false;
    pending_move_.reset();
    movement_en_ = true;
    state_warned_ = false;
}

//------------------------------------------------------------------------------
// !Calibration (private)
//------------------------------------------------------------------------------

/**
 * @brief Cancels a running calibration; `mutex_` must already be held.
 */
void Libra2MotionController::CancelCalibrationLocked() {
    if (IsCalibrationRunning(status_)) {
        status_ = CalibrationStatus::kIdle;
        pending_move_.reset();
        sampler_.Reset();
        logger_.Info("Calibration cancelled");
    }
}

/**
 * @brief Enters a calibration stage and, for moving stages, requests the move.
 *
 * @param status Stage to enter
 */
void Libra2MotionController::BeginStage(CalibrationStatus status) {
    status_ = status;
    stage_time_ = 0.0;
    sampler_.Reset();

    if (status == CalibrationStatus::kMovingToA) {
        pending_move_ = angle_a_;
    } else if (status == CalibrationStatus::kMovingToB) {
        pending_move_ = angle_b_;
    }
}

/**
 * @brief Ends the calibration and logs the outcome.
 *
 * @param success true if the model was updated
 * @param message Outcome (success: the result; failure: the reason)
 */
void Libra2MotionController::FinishCalibration(bool success,
                                               const std::string& message) {
    status_ = success ? CalibrationStatus::kSucceeded
                      : CalibrationStatus::kFailed;
    pending_move_.reset();
    sampler_.Reset();
    if (success) {
        logger_.Info(message);
    } else {
        logger_.Error("Calibration failed: " + message);
    }
}

/**
 * @brief Converts an averaged rest sample into the arm's own gravity torque.
 *
 * @param sample Averaged measurements
 * @return double tau_g1(theta) (Nm), Eq. 29
 */
double Libra2MotionController::SampleToArmTorque(const RestSample& sample) const {
    // tau_meas = -tau_actuator (Eq. 19); then Eq. 29
    return Libra2Model::ArmGravTorque(-sample.torque_actuator,
                                      cw_model_.Moment(
                                          Libra2Model::CwMass(sample.volume_l)),
                                      grav_, sample.theta);
}

/**
 * @brief Advances the running calibration by one cycle.
 *
 * @param state Current measurements
 * @param theta Measured angle (rad)
 * @param vel Measured velocity (rad/s)
 * @param torque Measured actuator torque (Nm)
 */
void Libra2MotionController::RunCalibration(const MotionState& state,
                                            double theta, double vel,
                                            double torque) {
    stage_time_ += state.dt;

    const bool moving_stage = (status_ == CalibrationStatus::kMovingToA
                               || status_ == CalibrationStatus::kMovingToB);
    const bool at_a = (status_ == CalibrationStatus::kMovingToA
                       || status_ == CalibrationStatus::kSamplingA);
    const double target = at_a ? angle_a_ : angle_b_;

    if (stage_time_ > (moving_stage ? kMoveTimeout : kSampleTimeout)) {
        FinishCalibration(false, moving_stage ? "timed out moving to target"
                                              : "timed out waiting for steady "
                                                "pose, torque, and CW volume");
        return;
    }

    if (moving_stage) {
        // A zero-length move never starts a trajectory, so "already there"
        // counts as arrived
        if (!state.trajectory_active
            && std::abs(theta - target) <= kReachTolerance
            && std::abs(vel) <= kRestVelTolerance) {
            BeginStage(at_a ? CalibrationStatus::kSamplingA
                            : CalibrationStatus::kSamplingB);
        }
        return;
    }

    // Sampling: the pose must stay near the target (except for level cal)
    if (status_ != CalibrationStatus::kSamplingLevel
        && std::abs(theta - target) > 2.0 * kReachTolerance) {
        sampler_.Reset();
        return;
    }

    const auto sample = sampler_.Add(state.dt, theta, vel, torque,
                                     state.trajectory_active, cw_volume_l_);
    if (!sample) {
        return;
    }
    const double arm_torque = SampleToArmTorque(*sample);

    if (status_ == CalibrationStatus::kSamplingLevel) {
        if (std::abs(sample->theta) > kLevelTolerance) {
            FinishCalibration(false, "arm is not level (theta="
                                         + MotionFormat(sample->theta)
                                         + " rad)");
            return;
        }
        // Eq. 34 with the current arm_sag_ (0 if never calibrated)
        const auto level = Libra2Model::ArmLevelTorque(arm_torque,
                                                       sample->theta, arm_sag_);
        if (!level) {
            FinishCalibration(false, "cos(theta) too small");
            return;
        }
        if (!calibrated_) {
            logger_.Warn("arm_sag not calibrated: using arm_sag = 0 (run "
                         "two-angle calibration to measure it)");
        }
        arm_level_torque_ = *level;
        calibrated_ = true;
        FinishCalibration(true, "Level calibration succeeded: arm_level_torque="
                                    + MotionFormat(arm_level_torque_)
                                    + " Nm, arm_sag=" + MotionFormat(arm_sag_)
                                    + " Nm");
        return;
    }

    if (status_ == CalibrationStatus::kSamplingA) {
        arm_torque_a_ = arm_torque;
        theta_a_ = sample->theta;
        BeginStage(CalibrationStatus::kMovingToB);
        return;
    }

    // kSamplingB: Eq. 35 with the MEASURED angles
    double level = 0.0;
    double sag = 0.0;
    if (!Libra2Model::SolveTwoAngle(arm_torque_a_, theta_a_, arm_torque,
                                    sample->theta, level, sag)) {
        FinishCalibration(false, "measured angles too close together");
        return;
    }
    arm_level_torque_ = level;
    arm_sag_ = sag;
    calibrated_ = true;
    if (sag > 0.0) {
        logger_.Warn("arm_sag is positive (h1 > 0 means unstable, Eq. 27); "
                     "check the calibration");
    }
    FinishCalibration(true, "Two-angle calibration succeeded: arm_level_torque="
                                + MotionFormat(arm_level_torque_)
                                + " Nm, arm_sag=" + MotionFormat(arm_sag_)
                                + " Nm");
}

}  // namespace Motion
