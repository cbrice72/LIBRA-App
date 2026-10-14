/******************************************************************************
 * @file   libra2_motion_controller.h
 * @brief  LIBRA-II pitch motion controller (holding + active); plain C++.
 *
 * @note   Equation numbers (Eq. NN) refer to the user's notebook (Sets A-I).
 *         All values are JOINT space: positive pitch = arm up (CCW), and
 *         `torque_actuator` is the torque the actuator applies to the joint.
 *
 * @note   Two ways to move the pitch joint (`MotionMode`):
 *           kTandem    - the actuator follows the trajectory with a gravity
 *                        feedforward while the CW fills/drains toward the
 *                        target mass; torque limits throttle the trajectory.
 *           kCwPrimary - the actuator is left free and the CW alone moves the
 *                        arm until the volume reaches the target mass OR the
 *                        angle is within a band of the target, whichever comes
 *                        first; then the actuator takes over and holds.
 *
 * @author Christian Brice
 ******************************************************************************/

#pragma once

// C++ Standard Library Headers
#include <cstdint>
#include <optional>
#include <string>
#include <vector>

// Project Headers
#include "abstract_motion_controller.h"

/* --- TABLE OF CONTENTS ---
 * !Constants
 * !Types
 * !CW Model
 * !Pure Equations
 * !Rest Sampler
 * !Class Declaration
 */

namespace Motion {

//------------------------------------------------------------------------------
// !Constants
//------------------------------------------------------------------------------

// Eq. 28: m_cw = rho_water * V
constexpr double kWaterDensity = 1.0;  // rho_water (kg/L)

// Defaults used by the convenience constructor (HebiThread uses only that one)
// TODO: r_P is not measured yet. 0 leaves the CW model INVALID on purpose,
//       which refuses calibration and disables feedforward and the predictive
//       CW command until real values are set here.
constexpr double kDefaultCwPivotDist = 0.0;   // r_P at 0 L (m)
constexpr double kDefaultCwPivotSlope = 0.0;  // change in r_P per liter (m/L)
// TODO: keep in sync with TankWidget's kCapacity (19 L for LIBRA-II)
constexpr double kDefaultCwCapacity = 19.0;  // L
constexpr double kDefaultGrav = 9.80665;     // g (m/s^2)
// TODO: friction has not been measured yet
constexpr double kDefaultFrictionCoulomb = 0.0;  // tau_c (Nm)
constexpr double kDefaultFrictionViscous = 0.0;  // b (Nm*s/rad)
// TODO: take I from CAD; dI/dt stays 0 until the ETA extension is estimated
//       (Set J)
constexpr double kDefaultInertia = 0.0;      // I (kg*m^2)
constexpr double kDefaultInertiaRate = 0.0;  // dI/dt (kg*m^2/s)

// Automatic torque compensation (ATC) hysteresis (same values as LIBRA-I)
constexpr double kDefaultTorqueCompLowerBound = 3.0;  // Nm ("soft" limit)
constexpr double kDefaultTorqueCompUpperBound = 6.0;  // Nm ("hard" limit)
constexpr double kAtcSlowPlaybackRate = 0.5;  // between the soft and hard limits

// CW-primary / predictive CW defaults (both changeable at runtime)
// TODO: guess; tune against the logged model error at each CW-primary hand-off
constexpr double kDefaultHandoffBand = 0.0175;  // rad (~1 deg)
// TODO: guess (~ one TankWidget animation step); only prevents valve chatter
constexpr double kDefaultCwVolumeTolerance = 0.01;  // L

// Calibration limits
// TODO: all of these are guesses; tune on hardware
constexpr double kLevelTolerance = 0.0175;    // max |theta| for level cal (rad)
constexpr double kMaxCalibrationAngle = 1.4;  // max |angle| for 2-angle (rad)
constexpr double kMinAngleSeparation = 0.2;   // min |b - a| (rad)
constexpr double kReachTolerance = 0.01;      // "arrived at target" (rad)
constexpr double kRestVelTolerance = 0.005;   // "at rest" speed (rad/s)
constexpr double kRestPosTolerance = 0.005;  // max drift in sample window (rad)
constexpr double kRestVolTolerance = 0.01;   // max CW change in window (L)
constexpr double kRestWindowTime = 3.0;      // averaging window (s)
constexpr double kMoveTimeout = 180.0;       // per move stage (s)
constexpr double kSampleTimeout = 60.0;      // per sampling stage (s)
constexpr double kMinCosForBalance = 1e-3;   // |cos(theta)| floor, Eq. 34/38

//------------------------------------------------------------------------------
// !Types
//------------------------------------------------------------------------------

/**
 * @brief What the counterweight (CW) should do.
 */
enum class CwCommand {
    kIdle,  // stop adjusting
    kFill,  // add water
    kDrain  // remove water
};

enum class MotionMode : uint8_t { kTandem, kCwPrimary };

/**
 * @brief Progress of an automatic calibration.
 */
enum class CalibrationStatus {
    kIdle,           // never started or cancelled
    kSamplingLevel,  // level calibration: averaging torque at rest
    kMovingToA,      // two-angle calibration: moving to first angle
    kSamplingA,      // ... averaging at first angle
    kMovingToB,      // ... moving to second angle
    kSamplingB,      // ... averaging at second angle
    kSucceeded,
    kFailed
};

bool IsCalibrationRunning(CalibrationStatus status);

/**
 * @brief Values computed for logging only (Eq. 28, 36, 38-40).
 */
struct Libra2Diagnostics {
    std::optional<double> cw_mass_kg;         // m_cw
    std::optional<double> torque_ext_g;       // tau_ext,g(theta)
    std::optional<double> cw_mass_tgt_kg;     // m_cw,bal(theta_tgt)
    std::optional<double> cw_mass_change_kg;  // delta_m_tgt (Eq. 40a)
    std::optional<double> torque_ext_g_tgt;   // tau_ext,g(theta_tgt) (Eq. 39)
};

/**
 * @brief The generic command plus what is specific to LIBRA-II.
 */
struct Libra2Command : MotionCommand {
    explicit Libra2Command(std::size_t n_joints = 0)
        : MotionCommand(n_joints) {}

    // CW command; none = nothing to report this cycle (kIdle clears it)
    std::optional<CwCommand> cw_command;

    // Pose (joint space, rad) the caller must start a trajectory to. Emitted
    // ONCE per request (calibration moves, CW-primary hand-off).
    std::optional<JointValues> move_to;

    Libra2Diagnostics diagnostics;
};

//------------------------------------------------------------------------------
// !CW Model
//------------------------------------------------------------------------------

/**
 * @brief Gravity moment of the CW, m_cw r_P (kg*m), as a function of m_cw.
 *
 * @note r_P is a straight line in volume: r_P(V) = r_P at 0 L + slope * V.
 *       A slope of 0 is a constant r_P. The moment is then quadratic in m_cw,
 *       so it can be inverted exactly (no lookup table).
 */
class CwModel {
  public:
    CwModel(double cw_pivot_dist_m, double cw_pivot_slope_m_per_l,
            double capacity_l);

    [[nodiscard]] bool IsValid() const {
        return error_.empty();
    }

    [[nodiscard]] const std::string& Error() const {
        return error_;
    }

    [[nodiscard]] double CapacityMass() const;

    [[nodiscard]] double Moment(double mass_kg) const;
    [[nodiscard]] double MassForMoment(double moment,
                                       bool* clamped = nullptr) const;

  private:
    [[nodiscard]] double PivotDist(double volume_l) const;

    double r_p_m_{0.0};              // r_P at 0 L (m)
    double r_p_slope_m_per_l_{0.0};  // change in r_P per liter (m/L)
    double capacity_l_;
    std::string error_;
};

//------------------------------------------------------------------------------
// !Pure Equations
//------------------------------------------------------------------------------

/**
 * @brief Stateless implementations of the notebook equations. `cw_moment` is
 *        m_cw r_P (kg*m), see `CwModel::Moment()`.
 */
namespace Libra2Model {

double CwMass(double volume_l);
double ArmGravTorque(double torque_meas, double cw_moment, double grav,
                     double theta);
std::optional<double> ArmLevelTorque(double arm_torque, double theta,
                                     double arm_sag);
bool SolveTwoAngle(double arm_torque_a, double theta_a, double arm_torque_b,
                   double theta_b, double& arm_level_torque, double& arm_sag);
double NetGravTorque(double arm_level_torque, double arm_sag, double cw_moment,
                     double grav, double theta);
double TorqueCommand(double inertia, double inertia_rate,
                     double friction_viscous, double friction_coulomb,
                     double ref_acc, double ref_vel, double net_grav_torque);
std::optional<double> BalanceCwMoment(double arm_level_torque, double arm_sag,
                                      double grav, double theta);
double TargetNetGravTorque(double grav, double theta_tgt, double cw_moment,
                           double cw_moment_bal);

}  // namespace Libra2Model

//------------------------------------------------------------------------------
// !Rest Sampler
//------------------------------------------------------------------------------

/**
 * @brief Averaged measurements over one steady window.
 */
struct RestSample {
    double theta{0.0};            // mean measured angle (rad)
    double torque_actuator{0.0};  // mean measured actuator torque (Nm)
    double volume_l{0.0};         // mean CW volume (L)
};

/**
 * @brief Averages pose, torque, and CW volume once all three have been steady
 *        for `kRestWindowTime`. Any disturbance restarts the window.
 */
class RestSampler {
  public:
    void Reset();
    std::optional<RestSample> Add(double dt, double pos, double vel,
                                  double torque_actuator,
                                  bool trajectory_active,
                                  const std::optional<double>& volume_l);

  private:
    int count_{0};
    double elapsed_{0.0};
    double anchor_theta_{0.0};
    double anchor_volume_{0.0};
    double sum_theta_{0.0};
    double sum_torque_{0.0};
    double sum_volume_{0.0};
};

//------------------------------------------------------------------------------
// !Class Declaration
//------------------------------------------------------------------------------

class Libra2MotionController : public AbstractMotionController {
  public:
    Libra2MotionController(Logger& logger);  // uses the kDefault* values
    Libra2MotionController(Logger& logger, double cw_pivot_dist_m,
                           double cw_pivot_slope_m_per_l, double cw_capacity_l,
                           double grav, double friction_coulomb,
                           double friction_viscous, double inertia,
                           double inertia_rate);

    // --- Control Cycle ---

    Libra2Command Update(const MotionState& state);

    // --- Settings (all lock) ---

    bool SetMotionMode(MotionMode mode);
    [[nodiscard]] MotionMode GetMotionMode() const;
    void SetCwVolume(double liters);
    void EnableTorqueComp(const bool& enabled);
    bool SetTorqueCompBounds(const double& lower_bound,
                             const double& upper_bound);
    bool SetHandoffBand(const double& band_rad);
    bool SetCwVolumeTolerance(const double& tolerance_l);

    // --- Arm Model ---

    void SetArmModel(double arm_level_torque, double arm_sag);
    void ClearArmModel();
    [[nodiscard]] bool IsCalibrated() const;
    [[nodiscard]] double GetArmLevelTorque() const;
    [[nodiscard]] double GetArmSag() const;
    [[nodiscard]] double ComputeNetGravTorque(double theta,
                                              double cw_mass_kg) const;

    // --- Calibration ---

    bool StartLevelCalibration();
    bool StartTwoAngleCalibration(double angle_a, double angle_b);
    void CancelCalibration();
    [[nodiscard]] CalibrationStatus GetCalibrationStatus() const;

  protected:
    void OnTargetChanged() override;
    void OnStop() override;
    void OnReset() override;

  private:
    enum class CwPhase { kIdle, kCwMoving };

    struct AtcResult {
        double playback_rate{1.0};
        std::optional<CwCommand> cw_command;
    };

    // --- Helper Functions ---

    AtcResult UpdateTorqueComp(double torque_magnitude, CwCommand direction);
    std::optional<CwCommand> PredictiveCwCommand(double theta_tgt,
                                                 double cw_mass_kg,
                                                 double* cw_mass_tgt_kg) const;
    bool RunCwPrimary(double theta, double theta_tgt, Libra2Command& command);
    void ComputeDiagnostics(double theta,
                            const std::optional<double>& theta_tgt,
                            Libra2Diagnostics& diagnostics) const;

    void RunCalibration(const MotionState& state, double theta, double vel,
                        double torque);
    void BeginStage(CalibrationStatus status);
    void FinishCalibration(bool success, const std::string& message);
    [[nodiscard]] double SampleToArmTorque(const RestSample& sample) const;
    void CancelCalibrationLocked();

    // --- Data Members ---

    // Parameters (Set I* names)
    CwModel cw_model_;
    double grav_;
    double friction_coulomb_;
    double friction_viscous_;
    double inertia_;
    double inertia_rate_;

    // Arm model
    double arm_level_torque_{0.0};  // tau_g1,0
    double arm_sag_{0.0};           // m_arm h1 g
    bool calibrated_{false};

    // Inputs from slots
    std::optional<double> cw_volume_l_;
    std::optional<double> cw_mass_kg_;  // Eq. 28, refreshed each cycle

    // Mode and CW-primary state
    MotionMode mode_{MotionMode::kTandem};
    CwPhase cw_phase_{CwPhase::kIdle};
    // TODO: after a CW-primary hand-off the CW stays idle until the next
    //       target. This keeps the predictive command from refilling the CW
    //       while the actuator finishes the move, but it also means the CW
    //       will not trim a residual error left by an inaccurate model. If
    //       the actuator ends up holding a large steady torque, consider
    //       letting the CW resume (e.g., via the reactive torque command).
    bool cw_assist_done_{false};
    double handoff_band_{kDefaultHandoffBand};
    double cw_volume_tolerance_{kDefaultCwVolumeTolerance};

    // ATC
    bool atc_en_{false};
    bool movement_en_{true};
    double torque_comp_lower_bound_{kDefaultTorqueCompLowerBound};
    double torque_comp_upper_bound_{kDefaultTorqueCompUpperBound};

    // Calibration
    CalibrationStatus status_{CalibrationStatus::kIdle};
    bool two_angle_{false};
    double angle_a_{0.0};
    double angle_b_{0.0};
    double stage_time_{0.0};
    double arm_torque_a_{0.0};
    double theta_a_{0.0};
    std::optional<double> pending_move_;  // pitch angle to request next cycle
    RestSampler sampler_;

    bool state_warned_{false};
};

}  // namespace Motion
