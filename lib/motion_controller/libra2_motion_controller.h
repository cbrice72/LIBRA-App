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
#include <optional>
#include <vector>

// Other Library Headers
// (none)

// Project Headers
#include "abstract_motion_controller.h"
#include "motion_defs.h"

/**
 * @brief Hanging counterweight (CW) modeled as one mass at pivot P, whose
 *        distance along the arm (r_P) shifts with water volume.
 *
 * @note Defines the CW's share of the gravitational torque at the pitch joint
 *       (Eq. 12'), the arm's inertia (Eq. 15), and the CW mass that balances
 *       the arm at a given angle (Eq. 38').
 */
class CwModel {
  public:
    explicit CwModel(const double& cw_pivot_dist, const double& cw_pivot_slope,
                     const double& cw_hang_offset, const double& capacity);

    [[nodiscard]] bool IsValid() const {
        return error_.empty();
    }

    [[nodiscard]] const std::string& GetError() const {
        return error_;
    }

    [[nodiscard]] Motion::CwModelParams GetParams() const;
    [[nodiscard]] double GetCapacityMass() const;

    [[nodiscard]] double Moment(const double& mass) const;
    [[nodiscard]] double HangMoment(const double& mass) const;
    [[nodiscard]] double EffectiveMoment(const double& mass,
                                         const double& theta) const;
    [[nodiscard]] std::optional<double> MassForMoment(const double& moment,
                                                      const double& theta) const;

    [[nodiscard]] double Inertia(const double& mass) const;
    [[nodiscard]] double InertiaSlope(const double& mass) const;

  private:
    [[nodiscard]] double PivotDist(const double& volume) const;

    double r_P_0_{0.0};    // r_P at 0 liters, A (m)
    double dr_P_dV_{0.0};  // r_P change per liter, B (m/L)
    double q_P_{0.0};      // hang-point offset normal to arm, from O (m)

    double capacity_;  // CW capacity (L)

    std::string error_;
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
    int count_{0};
    double t_elapsed_s_{0.0};  // seconds

    double anchor_theta_{0.0};
    double anchor_volume_{0.0};

    double sum_theta_{0.0};
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

    // --- Arm Model ---

    void SetArmModel(const double& arm_level_torque,
                     const double& arm_hang_torque);
    void ClearArmModel();

    [[nodiscard]] bool IsArmCalibrated() const;

    [[nodiscard]] double GetArmLevelTorque() const;
    [[nodiscard]] double GetArmHangTorque() const;
    [[nodiscard]] double ComputeNetGravTorque(const double& theta,
                                              const double& cw_mass) const;

    // --- Calibration ---

    bool StartLevelCalibration();
    bool StartTwoAngleCalibration(const double& angle_a, const double& angle_b);
    bool StartFrictionCalibration(const double& angle_a, const double& angle_b,
                                  const double& max_speed);
    void CancelCalibration();

    [[nodiscard]] Motion::CalibrationStatus GetCalibrationStatus() const;
    [[nodiscard]] std::string GetCalibrationMessage() const;

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
        double angle_lo{0.0};   // window start (rad)
        double angle_hi{0.0};   // window end (rad)
        double max_speed{0.0};  // (rad/s)

        double volume{0.0};  // initial CW volume (L)

        std::size_t speed_idx{0};

        bool trajectory_seen{false};  // current sweep has started

        // Per-bin torque sums/counts for the current speed
        std::vector<double> up_sum;
        std::vector<double> down_sum;
        std::vector<int> up_count;
        std::vector<int> down_count;

        std::vector<Motion::FrictionPoint> points;
    };

    // --- Helper Functions ---

    AtcResult UpdateTorqueComp(const double& torque_magnitude,
                               const Motion::CwCommand& direction,
                               const bool& braking);
    [[nodiscard]] std::optional<double> TargetCwMass(
        const double& theta_tgt) const;
    [[nodiscard]] Motion::CwCommand PredictiveCwCommand(
        const double& cw_mass_tgt, const double& cw_mass) const;
    bool RunCwPrimary(const double& theta, const double& theta_tgt,
                      Motion::Libra2Command& command);

    [[nodiscard]] Motion::Libra2Diagnostics ComputeDiagnostics(
        const double& theta, const std::optional<double>& theta_tgt) const;

    void RunArmCalibration(const double& dt, const bool& trajectory_active,
                           const double& theta, const double& actual_vel,
                           const double& actual_torque);
    void FinishLevelCalibration(const Motion::RestSample& sample);
    void FinishTwoAngleCalibration(const Motion::RestSample& sample_b);

    void RunFrictionCalibration(const double& dt, const bool& trajectory_active,
                                const double& theta, const double& actual_vel,
                                const double& actual_torque);
    void AddFrictionSample(const double& theta, const double& actual_vel,
                           const double& actual_torque, const bool& going_up,
                           const double& speed);
    bool FinishFrictionSpeed(const double& speed, std::string& error);
    void FinishFrictionCalibration();

    void BeginCalibrationStage(const Motion::CalibrationStatus& status);
    void FinishCalibration(const bool& success, const std::string& message);

    [[nodiscard]] double SampleToArmTorque(
        const Motion::RestSample& sample) const;
    [[nodiscard]] std::optional<Motion::ArmModel> RecomputeArmModel(
        const Motion::ArmModel& saved) const;
    bool CheckStability(const double& arm_hang_torque);
    void CancelCalibrationLocked();

    [[nodiscard]] bool IsMovingLocked() const;

    // --- Data Members ---

    // Parameters (see `I_Controller-Code-Equivalencies.jpg`)

    CwModel cw_model_;
    double friction_coulomb_;  // τ_c (Nm)
    double friction_viscous_;  // b (Nm·s/rad)
    double arm_inertia_;       // arm only, about pitch joint O (kg·m²)

    // Arm Model

    double arm_level_torque_{0.0};  // τ_g1,0 (Nm)
    double arm_hang_torque_{0.0};   // m_arm·h_1·g (Nm)
    bool arm_calibrated_{false};

    std::string arm_source_;                       // see `ArmModel`
    Motion::CwModelParams arm_cw_params_;          // reference CW model
    std::vector<Motion::RestSample> arm_samples_;  // source calibration samples

    std::optional<Motion::FrictionModel> friction_model_;

    // CW State (from WaterController)

    std::optional<double> cw_vol_;   // V (L)
    std::optional<double> cw_mass_;  // m_cw (kg) -- Eq. 28
    double cw_flow_{0.0};            // V̇ (L/s)

    // Dynamics (Eq. 15)

    double inertia_{0.0};       // arm + CW (kg·m²)
    double inertia_rate_{0.0};  // from CW flow (kg·m²/s)
    bool inertia_ff_en_{true};  // affects Eq. 37

    // Movement Mode & State

    Motion::MotionMode mode_{Motion::MotionMode::kTandem};
    CwPhase cw_phase_{CwPhase::kIdle};

    bool cw_assist_done_{false};  // hand-off completed; predictive CW off
    bool trajectory_active_{false};

    double handoff_band_{Motion::kDefaultHandoffBand};  // (rad)

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
