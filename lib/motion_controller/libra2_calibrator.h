/******************************************************************************
 * @file   libra2_calibrator.h
 * @brief  LIBRA-II calibration measurements; header file.
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
#include <deque>
#include <optional>
#include <string>
#include <utility>
#include <vector>

// Other Library Headers
// (none)

// Project Headers
#include "logger.h"
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
    std::deque<std::pair<double, double>> history_;  // (dt, θ') pairs
    double time_{0.0};                               // Σdt (s)
    double sum_{0.0};                                // Σθ'·dt (rad)
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
 * @brief Runs the LIBRA-II calibrations: requests their moves and sweeps,
 *        averages rest samples, and fits friction. The motion controller turns
 *        the results into its models and must call `FinishCalibration()` the
 *        same cycle it takes them.
 *
 * @note Not thread-safe; the motion controller calls it under its own lock.
 */
class Libra2Calibrator {
  public:
    /**
     * @brief One control cycle's pitch measurements.
     */
    struct PitchCycle {
        double dt{0.0};      // time since last cycle (s)
        double angle{0.0};   // actual angle, θ (rad)
        double vel{0.0};     // actual velocity, θ' (rad/s)
        double torque{0.0};  // actual actuator torque, τ_actuator (Nm)
        bool trajectory_active{false};
        std::optional<double> cw_volume;  // V (L), if known
    };

    /**
     * @brief Requests and results for the motion controller, taken each cycle.
     */
    struct Output {
        std::optional<double> move_to;  // pitch angle (rad)
        std::optional<Motion::SweepRequest> sweep;
        bool stop_trajectory{false};

        std::optional<Motion::RestSample> level_sample;
        std::optional<std::pair<Motion::RestSample, Motion::RestSample>>
            two_angle_samples;  // (a, b)
        std::optional<Motion::FrictionModel> friction;
    };

    explicit Libra2Calibrator(Logger& logger);

    // --- Commands ---

    bool StartLevelCalibration();
    bool StartTwoAngleCalibration(const double& angle_a, const double& angle_b);
    bool StartFrictionCalibration(const double& angle_a, const double& angle_b,
                                  const double& max_vel,
                                  const double& cw_volume,
                                  const double& inertia);
    void CancelCalibration();
    void FinishCalibration(const bool& success, const std::string& message);
    void Reset();

    // --- Control Cycle ---

    void Update(const PitchCycle& cycle);
    [[nodiscard]] Output TakeOutput();

    // --- Status ---

    [[nodiscard]] bool IsRunning() const;
    [[nodiscard]] bool IsFrictionRunning() const;
    [[nodiscard]] Motion::CalibrationStatus GetCalibrationStatus() const;
    [[nodiscard]] const std::string& GetCalibrationMessage() const;
    [[nodiscard]] const std::string& GetCalibrationProgress() const;

  private:
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

        VelAverager vel_avg;  // current sweep's θ', for the steady check

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

    void BeginCalibrationStage(const Motion::CalibrationStatus& status);
    [[nodiscard]] std::string DescribeCalibrationStage() const;
    void LogCalibrationProgress(const PitchCycle& cycle, const double& timeout);

    void RunArmCalibration(const PitchCycle& cycle);

    void RunFrictionCalibration(const PitchCycle& cycle);
    void AddFrictionSample(const PitchCycle& cycle, const bool& going_up,
                           const double& vel);
    [[nodiscard]] std::optional<std::string> FinishFrictionVel(const double& vel);
    void FinishFrictionCalibration();

    // --- Data Members ---

    Logger* logger_;  // motion controller's (not owned)

    // Calibration Metadata

    Motion::CalibrationStatus status_{Motion::CalibrationStatus::kIdle};
    double t_stage_s_{0.0};  // seconds

    std::string calibration_msg_;  // how the previous calibration ended

    std::string calibration_progress_;  // current step; empty when not running
    double t_progress_log_s_{0.0};      // seconds since last progress log

    // Arm Calibration

    double angle_a_{0.0};          // (rad)
    double angle_b_{0.0};          // (rad)
    Motion::RestSample sample_a_;  // two-angle: first angle's sample
    RestSampler sampler_;

    // Friction Calibration

    FrictionRun friction_run_;

    Output output_;  // pending until `TakeOutput()`
};
