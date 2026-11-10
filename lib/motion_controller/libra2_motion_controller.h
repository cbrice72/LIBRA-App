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
#include "libra2_calibrator.h"
#include "libra2_model.h"
#include "motion_defs.h"

/**
 * @brief Model-based pitch control for LIBRA-II. Augments the HEBI API's
 *        trajectory planner with (1) feedforward torque that cancels gravity,
 *        friction, and inertia (Eq. 37), and (2) predictive counterweight
 *        adjustment based on target angle (Eqs. 38'-40a).
 */
class Libra2MotionController : public AbstractMotionController {
  public:
    explicit Libra2MotionController(const double& cw_capacity,
                                    const double& reactive_cw_lower_bound,
                                    const double& reactive_cw_upper_bound,
                                    const double& reactive_cw_slow_rate,
                                    const double& reactive_cw_off_margin,
                                    const bool& debug_mode = false);

    // --- Control Cycle ---

    Motion::Libra2Command Update(const Motion::ActualState& actual,
                                 const Motion::CommandedState& commanded,
                                 const double& dt,
                                 const bool& trajectory_active,
                                 const Motion::MotionMode& mode);

    // --- Settings ---

    void SetCwState(const double& volume, const double& flow);

    void EnableReactiveCw(const bool& enabled);
    void EnableInertiaFeedforward(const bool& enabled);
    void SetReactiveCwBounds(const double& lower_bound,
                             const double& upper_bound);
    bool SetHandoffBand(const double& band);
    [[nodiscard]] double GetHandoffBand() const;
    void SetEtaExtension(const double& extension);
    [[nodiscard]] double GetEtaExtension() const;

    // --- Arm Model ---

    bool ClearModel();

    [[nodiscard]] bool IsArmCalibrated() const;
    [[nodiscard]] Motion::Calibration GetCalibration() const;

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

    struct ReactiveCwResult {
        double trajectory_rate{1.0};
        std::optional<Motion::CwCommand> cw_command;
    };

    // --- Helper Functions ---

    ReactiveCwResult UpdateReactiveCw(const double& torque_magnitude,
                                      const Motion::CwCommand& direction,
                                      const bool& braking);
    [[nodiscard]] std::optional<double> TargetCwMass(
        const double& angle_tgt) const;
    [[nodiscard]] Motion::CwCommand PredictiveCwCommand(
        const double& cw_mass_tgt, const double& cw_mass) const;
    bool RunCwPrimary(const double& angle, const double& angle_tgt,
                      Motion::Libra2Command& command);
    void HandOffToActuator();
    void LogMoveStart(const Motion::MotionMode& mode, const double& angle,
                      const double& angle_tgt) const;
    void LogPredictiveCwError(const std::optional<Motion::CwCommand>& reactive,
                              const double& angle, const double& torque);

    [[nodiscard]] double ComputeNetGravTorque(const double& angle,
                                              const double& cw_mass) const;
    [[nodiscard]] double ModelFeedforward(const double& angle,
                                          const double& ref_vel,
                                          const double& ref_acc,
                                          const bool& model_ready) const;

    [[nodiscard]] Motion::Libra2Diagnostics ComputeDiagnostics(
        const double& angle, const std::optional<double>& angle_tgt) const;

    void FinishLevelCalibration(const Motion::RestSample& sample);
    void FinishTwoAngleCalibration(const Motion::RestSample& sample_a,
                                   const Motion::RestSample& sample_b);
    void FinishFrictionCalibration(const Motion::FrictionModel& model);

    [[nodiscard]] double SampleToArmTorque(
        const Motion::RestSample& sample) const;
    bool CheckStability(const double& arm_hang_torque);
    [[nodiscard]] double LevelTorqueAtExtension() const;

    [[nodiscard]] bool IsMovingLocked() const;

    // --- Data Members ---

    // Parameters (see `I_Controller-Code-Equivalencies.jpg`)

    CwModel cw_model_;
    double friction_coulomb_;  // τ_c (Nm)
    double friction_viscous_;  // b (Nm·s/rad)
    double arm_inertia_;       // arm only, about pitch joint O (kg·m²)

    // Arm Model

    std::optional<Motion::ArmModel> arm_model_;  // none = not calibrated
    std::optional<Motion::FrictionModel> friction_model_;

    // CW State (from WaterController)

    std::optional<double> cw_volume_;  // V (L)
    std::optional<double> cw_mass_;    // m_cw (kg) -- Eq. 28
    double cw_flow_{0.0};              // V' (L/s)

    // Dynamics (Eq. 15)

    bool inertia_ff_en_{false};

    // Movement State

    bool new_target_{false};  // set since the last `Update()`
    CwPhase cw_phase_{CwPhase::kIdle};

    bool cw_assist_done_{false};  // predictive CW done; reactive CW allowed
    std::optional<double>
        predictive_cw_mass_;  // m_cw,bal(θ_tgt) when predictive CW finished (kg)
    bool reactive_cw_used_{false};  // reactive CW moved since hand-off
    std::optional<Motion::CwCommand> last_reactive_cw_;  // for change logs
    bool trajectory_active_{false};

    double handoff_band_{Motion::kDefaultHandoffBand};  // (rad)
    double eta_extension_{0.0};  // from min. extension (m)

    // Reactive CW (torque feedback)

    bool reactive_cw_en_{false};
    bool torque_high_{false};  // latched until below the off bound
    bool movement_en_{true};

    double reactive_cw_lower_bound_;  // "soft" limit (Nm)
    double reactive_cw_upper_bound_;  // "hard" limit (Nm)
    double reactive_cw_off_margin_;  // "turn-off" margin below lower bound (Nm)
    double reactive_cw_slow_rate_;   // movement rate between limits

    // Calibration

    Libra2Calibrator calibrator_;

    std::optional<double> pending_move_;  // pitch angle (rad)

    bool state_warned_{false};  // avoid spamming the log with repeated warnings
};
