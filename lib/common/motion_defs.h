/******************************************************************************
 * @file   motion_defs.h
 * @brief  Common definitions for motion controllers; header-only.
 *
 * @note   Equation numbers (Eq. ##) refer to the notebook scans in
 *         `LIBRA Archive/2026-10-02 LIBRA-II Controller Equations/`.
 *         All values are in joint space (positive pitch = arm up).
 *
 * @author Christian Brice
 ******************************************************************************/

#pragma once

// C++ Standard Library Headers
#include <cstddef>
#include <cstdint>
#include <optional>
#include <string>
#include <vector>

// Other Library Headers
// (none)

// Project Headers
#include "joint_defs.h"

/* --- TABLE OF CONTENTS ---
 * !Shared Constants
 * !Shared Types
 * !LIBRA-II Constants
 * !LIBRA-II Types
 * !LIBRA-II Calibration Record
 */

/**
 * @brief Common definitions for interacting with motion controllers.
 *
 * @note Per-joint arrays are indexed by `Joint::Name`.
 */
namespace Motion {

//------------------------------------------------------------------------------
// !Shared Constants
//------------------------------------------------------------------------------

// `kUndefined` is last in our enums, so it conveniently gives the number of
// joints in the system
constexpr std::size_t kNumJoints = Joint::Name::kUndefined;  // array size

//------------------------------------------------------------------------------
// !Shared Types
//------------------------------------------------------------------------------

/**
 * @brief Movement mode for a given joint.
 */
enum class CommandMode : uint8_t {
    kPositionEnhanced,  // position/velocity command + torque feedforward
    kTorqueOnly         // torque feedforward only
};

/**
 * @brief Actual joint values (i.e., measured feedback); NaN for joints the
 *        motion controller doesn't drive.
 */
struct ActualState {
    std::vector<double> pos;     // θ (rad)
    std::vector<double> vel;     // θ̇ (rad/s)
    std::vector<double> torque;  // τ_actuator, -τ_meas at rest (Nm)
};

/**
 * @brief Commanded joint values (i.e., from the previous cycle); NaN for joints
 *        the motion controller doesn't drive.
 */
struct CommandedState {
    std::vector<double> vel;  // θ̇_ref (rad/s)
    std::vector<double> acc;  // θ̈_ref (rad/s²)
};

/**
 * @brief The ultimate output of a motion controller; instructions for the
 *        actuator thread on augmenting its current trajectory.
 *
 * @note Entries for joints the motion controller doesn't drive are ignored.
 */
struct MotionCommand {
    explicit MotionCommand(const std::size_t& n_joints = 0)
        : mode(n_joints, CommandMode::kPositionEnhanced), ff_torque(n_joints) {}

    std::vector<CommandMode> mode;
    std::vector<std::optional<double>> ff_torque;  // effort ff (Nm)

    // Trajectory speed, [0,1] (1 = normal, 0 = paused)
    double playback_rate{1.0};
};

#if LIBRA_VERSION == 2
//------------------------------------------------------------------------------
// !LIBRA-II Constants
//------------------------------------------------------------------------------

// CW-primary Mode

// TODO: guess; tune via experiments
constexpr double kDefaultHandoffBand = 0.0175;  // CW stop point, ~1 deg (rad)

// Calibration Limits

// TODO: guesses; tune via experiments
constexpr double kMaxCalibrationAngle = 1.4;  // |θ| limit, ~80 deg (rad)
constexpr double kMinAngleSeparation = 0.2;   // |θ_b - θ_a|, ~11 deg (rad)
constexpr double kFrictionMinWindow = 0.105;  // incl. unsampled ends (rad)

//------------------------------------------------------------------------------
// !LIBRA-II Types
//------------------------------------------------------------------------------

/**
 * @brief Fluid system command.
 */
enum class CwCommand : uint8_t { kIdle, kFill, kDrain };

/**
 * @brief How the actuators and counterweight (CW) should work together to
 *        achieve the desired motion.
 */
enum class MotionMode : uint8_t {
    kTandem,    // actuator follows trajectory while CW trims toward target mass
    kCwPrimary  // CW moves free arm; actuator takes over near target
};

/**
 * @brief State of the arm or friction calibration.
 */
enum class CalibrationStatus : uint8_t {
    kIdle,                   // never started -or- cancelled
    kArmSamplingLevel,       // level calibration: averaging torque at rest
    kArmMovingToA,           // two-angle calibration: moving to first angle
    kArmSamplingA,           // ... averaging at first angle
    kArmMovingToB,           // ... moving to second angle
    kArmSamplingB,           // ... averaging at second angle
    kFrictionMovingToStart,  // friction calibration: moving to lower angle
    kFrictionSweepUp,        // ... sweeping up
    kFrictionSweepDown,      // ... sweeping down
    kSucceeded,
    kFailed
};

/**
 * @brief Whether a calibration is running.
 *
 * @param status Status to check
 */
inline bool IsCalibrationRunning(const CalibrationStatus& status) {
    return status != CalibrationStatus::kIdle
           && status != CalibrationStatus::kSucceeded
           && status != CalibrationStatus::kFailed;
}

/**
 * @brief Model predictions at the current and target angles, for comparing the
 *        model against actual measurements.
 */
struct Libra2Diagnostics {
    std::optional<double> cw_mass;           // m_cw (kg) -- Eq. 28
    std::optional<double> torque_ext_g;      // τ_ext,g(θ) (Nm) -- Eq. 36'
    std::optional<double> cw_mass_tgt;       // m_cw,bal(θ_tgt) (kg) -- Eq. 38'
    std::optional<double> cw_mass_change;    // Δm_tgt (kg) -- Eq. 40a
    std::optional<double> torque_ext_g_tgt;  // τ_ext,g(θ_tgt) (Nm) -- Eq. 39'
};

/**
 * @brief Constant-speed pitch move for friction calibration.
 *
 * @note At constant speed, actuator torque differs from holding torque only by
 *       friction (Eq. 18).
 */
struct SweepRequest {
    double from{0.0};       // start angle (rad)
    double to{0.0};         // end angle (rad)
    double speed{0.0};      // speed between ramps, > 0 (rad/s)
    double ramp_dist{0.0};  // length of each ramp (rad)
};

/**
 * @brief Motion command extended for LIBRA-II.
 */
struct Libra2Command : MotionCommand {
    explicit Libra2Command(const std::size_t& n_joints = 0)
        : MotionCommand(n_joints) {}

    std::optional<CwCommand> cw_command;  // none = no change
    std::optional<std::vector<double>> move_to;
    std::optional<SweepRequest> sweep;

    bool stop_trajectory{false};

    Libra2Diagnostics diagnostics;
};

//------------------------------------------------------------------------------
// !LIBRA-II Calibration Record
//------------------------------------------------------------------------------

/**
 * @brief Counterweight (CW) geometry informing the `CwModel`.
 */
struct CwModelParams {
    double pivot_dist{0.0};   // r_P at 0 liters, A (m)
    double pivot_slope{0.0};  // r_P change per liter, B (m/L)
    double hang_offset{0.0};  // hang points (normal to arm) from O, q_P (m)
    double capacity{0.0};     // CW capacity (L)
};

/**
 * @brief One static-equilibrium point from `RestSampler`. The arm model is
 *        solved from one (level) or two (two-angle) of these.
 */
struct RestSample {
    double theta{0.0};   // mean actual angle (rad)
    double torque{0.0};  // mean actual actuator torque, τ_actuator (Nm)
    double volume{0.0};  // mean CW volume (L)
};

/**
 * @brief Arm's own gravity torque model (Eq. 30).
 */
struct ArmModel {
    std::string source;  // "level", "two-angle", or "set"

    double level_torque{0.0};  // τ_g1,0 (Nm)
    double hang_torque{0.0};   // m_arm·h_1·g (Nm)

    CwModelParams cw_model;           // CW model used to compute it
    std::vector<RestSample> samples;  // raw; level: 1, two-angle: 2 (a, b)
};

/**
 * @brief One point of the friction line  (i.e., measurement at one sweep speed).
 */
struct FrictionPoint {
    double speed{0.0};  // v (rad/s)
    double drag{0.0};   // (τ_up - τ_down) / 2, averaged over bins (Nm)

    double spread{0.0};  // (max - min) of per-bin drag (Nm)
    int n_bins{0};       // bins sampled in both directions
};

/**
 * @brief Pitch joint friction model (Eq. 17): viscous and Coulomb terms fitted
 *        through drag at several sweep speeds.
 */
struct FrictionModel {
    double coulomb{0.0};       // τ_c (Nm)
    double viscous{0.0};       // b (Nm·s/rad)
    double residual_rms{0.0};  // line fit residual (Nm)

    double angle_lo{0.0};  // sweep window start (rad)
    double angle_hi{0.0};  // sweep window end (rad)
    double volume{0.0};    // CW volume during sweeps (L)

    std::vector<FrictionPoint> points;
};

/**
 * @brief Everything needed to define a calibrated controller, plus the inputs
 *        each result came from.
 */
struct CalibrationRecord {
    // File metadata
    std::string timestamp;            // ISO 8601
    std::optional<double> extension;  // arm extension, entered manually (mm)
    std::string note;

    // Controller constants when the record was made
    CwModelParams cw_model;
    double arm_inertia{0.0};  // (kg·m²)

    std::optional<ArmModel> arm_model;      // none = not calibrated
    std::optional<FrictionModel> friction;  // none = not measured
};
#endif

}  // namespace Motion
