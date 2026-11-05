/******************************************************************************
 * @file   libra2_motion_controller.h
 * @brief  LIBRA-II pitch motion controller (holding + active); header file.
 *
 * @note   Equation numbers (Eq. ##) refer to the notebook scans in
 *         `LIBRA Archive/2026-10-02 LIBRA-II Controller Equations/`.
 *         All values are in joint space (positive pitch = arm up).
 *
 * @author Christian Brice
 ******************************************************************************/

#pragma once

// C++ Standard Library Headers
#include <cstdint>
#include <deque>
#include <optional>
#include <utility>
#include <vector>

// Other Library Headers
// (none)

// Project Headers
#include "abstract_motion_controller.h"
#include "libra2_model.h"
#include "motion_defs.h"

/**
 * @brief Time-weighted moving average of velocity over `kVelAvgWindow`, since
 *        raw values are too noisy to compare against small thresholds.
 */
class VelAverager {
  public:
    void Reset();
    double Add(const double& dt, const double& vel);

  private:
    std::deque<std::pair<double, double>> history_;  // (dt, θ̇) pairs
    double time_{0.0};                               // Σdt (s)
    double sum_{0.0};                                // Σθ̇·dt (rad)
};

/**
 * @brief Static-equilibrium measurement for arm model calibration.
 *
 * @note At rest, actuator torque equals holding torque (Eq. 19a), so averages
 *       taken while the arm and counterweight (CW) volume stay steady give one
 *       point of the gravity model.
 */
class RestSampler {
  public:
    void Reset();

    std::optional<Motion::RestSample> Add(const double& dt, const double& pos,
                                          const double& vel,
                                          const double& torque,
                                          const bool& trajectory_active,
                                          const std::optional<double>& volume);

  private:
    void RestartWindow();

    VelAverager vel_avg_;  // kept across window restarts

    int count_{0};
    double t_elapsed_s_{0.0};  // seconds

    double anchor_angle_{0.0};
    double anchor_volume_{0.0};

    double sum_angle_{0.0};
    double sum_torque_{0.0};
    double sum_volume_{0.0};
};

/**
 * @brief Model-based pitch control for LIBRA-II. Augments the HEBI API's
 *        trajectory planner with feedforward torque that cancels gravity,
 *        friction, and inertia (Eq. 37), and (2) predictive counterweight
 *        adjustment based on target angle (Eqs. 38'-40a).
 */
class Libra2MotionController : public AbstractMotionController {
  public:
    explicit Libra2MotionController(const double& cw_capacity,
                                    const double& torque_comp_lower_bound,
                                    const double& torque_comp_upper_bound,
                                    const double& torque_comp_slow_rate,
                                    const bool& debug_mode = false);
    explicit Libra2MotionController(
        const double& cw_pivot_dist, const double& cw_pivot_slope,
        const double& cw_mean_sq_dist, const double& cw_mean_sq_dist_slope,
        const double& cw_hang_offset, const double& cw_capacity,
        const double& friction_coulomb, const double& friction_viscous,
        const double& arm_inertia, const double& torque_comp_lower_bound,
        const double& torque_comp_upper_bound,
        const double& torque_comp_slow_rate, const bool& debug_mode = false);

    // --- Control Cycle ---

    Motion::Libra2Command Update(const double& dt,
                                 const Motion::ActualState& actual,
                                 const Motion::CommandedState& commanded,
                                 const bool& trajectory_active);

    // --- Settings ---

    bool SetMotionMode(const Motion::MotionMode& mode);
    [[nodiscard]] Motion::MotionMode GetMotionMode() const;

    [[nodiscard]] bool IsMoving() const;

    void SetCwState(const double& volume, const double& flow);

    void EnableTorqueComp(const bool& enabled);
    void EnableInertiaFeedforward(const bool& enabled);
    bool SetTorqueCompBounds(const double& lower_bound,
                             const double& upper_bound);
    bool SetHandoffBand(const double& band);
    void SetEtaExtension(const double& extension);
    [[nodiscard]] double GetEtaExtension() const;

    // --- Arm Model ---

    void SetArmModel(const double& arm_level_torque,
                     const double& arm_hang_torque);
    bool ClearModel();

    [[nodiscard]] bool IsArmCalibrated() const;

    [[nodiscard]] double GetArmLevelTorque() const;
    [[nodiscard]] double GetArmHangTorque() const;
    [[nodiscard]] double ComputeNetGravTorque(const double& angle,
                                              const double& cw_mass) const;

    // --- Calibration ---

    bool StartLevelCalibration();
    bool StartTwoAngleCalibration(const double& angle_a, const double& angle_b);
    bool StartFrictionCalibration(const double& angle_a, const double& angle_b,
                                  const double& max_vel);
    void CancelCalibration();

    [[nodiscard]] Motion::CalibrationStatus GetCalibrationStatus() const;
    [[nodiscard]] std::string GetCalibrationMessage() const;
    [[nodiscard]] std::string GetCalibrationProgress() const;

    // --- Calibration Record ---

    [[nodiscard]] Motion::CalibrationRecord GetCalibrationRecord() const;
    bool LoadCalibrationRecord(const Motion::CalibrationRecord& record);

  protected:
    // --- Overrides ---

    void OnTargetChanged() override;
    void OnStop() override;
    void OnReset() override;

  private:
    /**
     * @brief Counterweight (CW) state machine for predictive CW control.
     */
    enum class CwPhase : uint8_t { kIdle, kCwMoving };

    struct AtcResult {
        double playback_rate{1.0};
        std::optional<Motion::CwCommand> cw_command;
    };

    /**
     * @brief Friction calibration state.
     */
    struct FrictionRun {
        double angle_lo{0.0};  // window start (rad)
        double angle_hi{0.0};  // window end (rad)
        double max_vel{0.0};   // (rad/s)

        double volume{0.0};   // initial CW volume (L)
        double inertia{0.0};  // arm + CW, used to set ramp lengths (kg·m²)
        double margin{0.0};   // unsampled area at each window end (rad)

        std::size_t vel_idx{0};

        bool trajectory_seen{false};  // whether the planned sweep has started
        double t_over_max_s{0.0};     // time over the |τ| limit (s)

        VelAverager vel_avg;  // current sweep's θ̇, for the steady check

        // Per-bin torque sums/counts for the current speed
        std::vector<double> up_sum;
        std::vector<double> down_sum;
        std::vector<int> up_count;
        std::vector<int> down_count;
        int range_samples{0};   // valid samples inside sampled range
        int steady_samples{0};  // ... of those, at steady speed

        std::vector<Motion::FrictionPoint> points;
    };

    // --- Helper Functions ---

    AtcResult UpdateTorqueComp(const double& torque_magnitude,
                               const Motion::CwCommand& direction,
                               const bool& braking);
    [[nodiscard]] std::optional<double> TargetCwMass(
        const double& angle_tgt) const;
    [[nodiscard]] Motion::CwCommand PredictiveCwCommand(
        const double& cw_mass_tgt, const double& cw_mass) const;
    bool RunCwPrimary(const double& angle, const double& angle_tgt,
                      Motion::Libra2Command& command);

    [[nodiscard]] double ModelFeedforward(const double& angle,
                                          const double& ref_vel,
                                          const double& ref_acc,
                                          const bool& model_ready) const;

    [[nodiscard]] Motion::Libra2Diagnostics ComputeDiagnostics(
        const double& angle, const std::optional<double>& angle_tgt) const;

    void RunArmCalibration(const double& dt, const bool& trajectory_active,
                           const double& angle, const double& actual_vel,
                           const double& actual_torque);
    void FinishLevelCalibration(const Motion::RestSample& sample);
    void FinishTwoAngleCalibration(const Motion::RestSample& sample_b);

    void RunFrictionCalibration(const double& dt, const bool& trajectory_active,
                                const double& angle, const double& actual_vel,
                                const double& actual_torque);
    void AddFrictionSample(const double& dt, const double& angle,
                           const double& actual_vel,
                           const double& actual_torque, const bool& going_up,
                           const double& vel);
    bool FinishFrictionVel(const double& vel, std::string& error);
    void FinishFrictionCalibration();

    void BeginCalibrationStage(const Motion::CalibrationStatus& status);
    void FinishCalibration(const bool& success, const std::string& message);

    [[nodiscard]] std::string DescribeCalibrationStage() const;
    void LogCalibrationProgress(const double& dt, const double& timeout,
                                const double& angle, const double& actual_vel,
                                const bool& trajectory_active);

    [[nodiscard]] double SampleToArmTorque(
        const Motion::RestSample& sample) const;
    bool CheckStability(const double& arm_hang_torque);
    void CancelCalibrationLocked();
    [[nodiscard]] double LevelTorqueAtExtension() const;

    [[nodiscard]] bool IsMovingLocked() const;

    // --- Data Members ---

    // Parameters (see `I_Controller-Code-Equivalencies.jpg`)

    CwModel cw_model_;
    double friction_coulomb_;  // τ_c (Nm)
    double friction_viscous_;  // b (Nm·s/rad)
    double arm_inertia_;       // arm only, about pitch joint O (kg·m²)

    // Arm Model

    double arm_level_torque_{0.0};  // τ_g1,0 at min. ETA extension (Nm)
    double arm_hang_torque_{0.0};   // m_arm·h_1·g (Nm)
    bool arm_calibrated_{false};

    std::string arm_source_;               // see `ArmModel`
    Motion::CwModelParams arm_cw_params_;  // reference CW model

    std::optional<Motion::FrictionModel> friction_model_;

    // CW State (from WaterController)

    std::optional<double> cw_volume_;  // V (L)
    std::optional<double> cw_mass_;    // m_cw (kg) -- Eq. 28
    double cw_flow_{0.0};              // V̇ (L/s)

    // Dynamics (Eq. 15)

    double inertia_{0.0};       // arm + CW (kg·m²)
    double inertia_rate_{0.0};  // from CW flow (kg·m²/s)
    bool inertia_ff_en_{false};

    // Movement Mode & State

    Motion::MotionMode mode_{Motion::MotionMode::kTandem};
    CwPhase cw_phase_{CwPhase::kIdle};

    bool cw_assist_done_{false};  // hand-off completed; predictive CW off
    bool trajectory_active_{false};

    double handoff_band_{Motion::kDefaultHandoffBand};  // (rad)
    double eta_extension_{0.0};  // from min. extension (m)

    // ATC (Automatic Torque Compensation)

    bool atc_en_{false};
    bool movement_en_{true};

    double torque_comp_lower_bound_;      // "soft" limit (Nm)
    double torque_comp_upper_bound_;      // "hard" limit (Nm)
    const double torque_comp_slow_rate_;  // movement rate between limits

    // Calibration Metadata

    Motion::CalibrationStatus status_{Motion::CalibrationStatus::kIdle};
    double t_stage_s_{0.0};  // seconds

    std::string calibration_msg_;  // how the previous calibration ended

    std::string calibration_progress_;  // current step; empty when not running
    double t_progress_log_s_{0.0};      // seconds since last progress log

    // Arm Calibration

    bool two_angle_{false};
    double angle_a_{0.0};          // (rad)
    double angle_b_{0.0};          // (rad)
    Motion::RestSample sample_a_;  // two-angle: first angle's sample
    RestSampler sampler_;

    // Friction Calibration

    FrictionRun friction_run_;

    std::optional<double> pending_move_;  // pitch angle (rad)
    std::optional<Motion::SweepRequest> pending_sweep_;
    bool pending_stop_{false};

    bool state_warned_{false};  // avoid spamming the log with repeated warnings
};
