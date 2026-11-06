/******************************************************************************
 * @file   libra2_calibrator.cpp
 * @brief  LIBRA-II calibration measurements; implementation file.
 *
 * @author Christian Brice
 ******************************************************************************/

// Related Header
#include "libra2_calibrator.h"

// C++ Standard Library Headers
#include <algorithm>
#include <array>
#include <cmath>

/* --- TABLE OF CONTENTS ---
 * !Local Helpers
 * !Velocity Averager
 * !Rest Sampler
 * !Class Management
 * !Commands
 * !Control Cycle
 * !Status
 * !Calibration Helpers
 * !Friction Calibration Helpers
 */

// Model Calibration

constexpr double kReachTolerance = 0.0175;  // "arrived at target", ~1 deg (rad)
constexpr double kRestPosTolerance = 0.005;    // allowed drift in window (rad)
constexpr double kRestVelTolerance = 0.005;    // ... and noise at rest (rad/s)
constexpr double kRestVolumeTolerance = 0.01;  // allowed CW change in window (L)
constexpr double kRestWindowTime = 3.0;        // averaging window (s)

constexpr double kProgressLogPeriod = 15.0;  // heartbeat message (s)
constexpr double kMoveTimeout = 180.0;       // (s)
constexpr double kSamplingTimeout = 60.0;    // (s)

// Velocity averaging window for rest and steady speed checks, arbitrary (s);
// at rest, raw vel noise is ~0.6 deg/s but its 0.75 s mean stays < 0.2 deg/s
constexpr double kVelAvgWindow = 0.75;

// Friction Calibration

// Sweep speeds as fractions of actuator max speed
constexpr std::array<double, 4> kFrictionVelFractions = {
    1.0 / 6.0,
    1.0 / 3.0,
    2.0 / 3.0,
    1.0,
};

// TODO: all of these are arbitrary; tune via experiments
constexpr double kMaxFrictionTorque = 14.0;     // abort if above this |τ| (Nm)
constexpr double kMaxFrictionTorqueTime = 0.5;  // ... for this long (s)

constexpr double kFrictionRampTorque = 4.0;      // inertia torque per ramp (Nm)
constexpr double kFrictionMinRampDist = 0.0175;  // shortest ramp, ~1 deg (rad)
constexpr double kFrictionSampleMargin = 0.0175;  // unsampled ends (rad)
constexpr double kFrictionVelTolerance = 0.1;     // allowed |θ' - v| / v

constexpr double kFrictionBinWidth = 0.0087;  // angle bin, ~0.5 deg (rad)
constexpr int kFrictionMinBinSamples = 3;     // per bin and direction
constexpr int kFrictionMinBins = 3;           // usable bins per speed

//------------------------------------------------------------------------------
// !Local Helpers
//------------------------------------------------------------------------------

namespace {

/**
 * @brief Gets the friction sweep ramp length (rad) that keeps the torque
 *        needed to accelerate the arm within `kFrictionRampTorque`.
 *
 * @param vel Sweep speed, v (rad/s)
 * @param inertia Arm + CW moment of inertia, I (kg·m²)
 *
 * @note Code-specific equation. HEBI's API uses a minimum-jerk ramp, which from
 *       rest to speed v over distance d takes T = 2·d/v seconds (see
 *       `HebiThread::StartSweep()`). With zero acceleration at both ends, its
 *       position is:
 *         d·(2τ³ - τ⁴) for τ = t/T,
 *
 *       Peaking at an acceleration of:
 *         3·d/T² = 0.75·v²/d
 *
 *       Which gives d = 0.75·I·v² / τ_ramp.
 */
double RampDist(const double& vel, const double& inertia) {
    return std::max(kFrictionMinRampDist,
                    0.75 * inertia * vel * vel / kFrictionRampTorque);
}

/**
 * @brief Estimates how long one friction sweep will take (s).
 *
 * @param lo Window start (rad)
 * @param hi Window end (rad)
 * @param vel Sweep speed (rad/s)
 * @param ramp_dist Length of each ramp (rad)
 *
 * @note Code-specific equation. Usually, each ramp is traversed at half speed,
 *       which gives:
 *         t = (θ_hi - θ_lo + 2·d) / v
 */
double SweepTime(const double& lo, const double& hi, const double& vel,
                 const double& ramp_dist) {
    return (hi - lo + (2.0 * ramp_dist)) / vel;
}

/**
 * @brief Checks whether a status belongs to the friction calibration.
 *
 * @param status Status to check
 */
bool IsFrictionStage(const Motion::CalibrationStatus& status) {
    return status == Motion::CalibrationStatus::kFrictionMovingToStart
           || status == Motion::CalibrationStatus::kFrictionSweepUp
           || status == Motion::CalibrationStatus::kFrictionSweepDown;
}

/**
 * @brief Fits a least-squares line through the friction sweep results (drag vs.
 *        speed).
 *
 * @param vels Sweep speeds, v (rad/s)
 * @param drags Drag at each speed, (τ_up - τ_down) / 2 (Nm)
 *
 * @note Returns the fit (intercept τ_c, slope b, and RMS of residuals; other
 *       fields empty), or none with fewer than two distinct speeds.
 *
 * @note Code-specific equation, based on Eq. 18 at constant speed (θ'' = 0,
 *       I' = 0). Isolates the friction component by averaging the up/down
 *       sweeps, which should cancel gravitational torque:
 *         (τ_up - τ_down) / 2 = b·v + τ_c
 */
std::optional<Motion::FrictionModel> FitFrictionLine(
    const std::vector<double>& vels, const std::vector<double>& drags) {
    const std::size_t n = vels.size();
    if (n < 2 || drags.size() != n) {
        return std::nullopt;
    }

    // Compute means
    double mean_v = 0.0;
    double mean_f = 0.0;
    for (std::size_t i = 0; i < n; ++i) {
        mean_v += vels[i];
        mean_f += drags[i];
    }
    mean_v /= static_cast<double>(n);
    mean_f /= static_cast<double>(n);

    // Compute slope and intercept
    double s_vv = 0.0;
    double s_vf = 0.0;
    for (std::size_t i = 0; i < n; ++i) {
        s_vv += (vels[i] - mean_v) * (vels[i] - mean_v);
        s_vf += (vels[i] - mean_v) * (drags[i] - mean_f);
    }
    if (!(s_vv > 0.0)) {
        return std::nullopt;
    }

    // Determine friction coefficients (+ residuals)
    Motion::FrictionModel model;
    model.viscous = s_vf / s_vv;
    model.coulomb = mean_f - (model.viscous * mean_v);

    double sum_sq = 0.0;
    for (std::size_t i = 0; i < n; ++i) {
        const double r = drags[i] - (model.coulomb + (model.viscous * vels[i]));
        sum_sq += r * r;
    }
    model.residual_rms = std::sqrt(sum_sq / static_cast<double>(n));

    return model;
}

}  // namespace

//------------------------------------------------------------------------------
// !Velocity Averager
//------------------------------------------------------------------------------

/**
 * @brief Clears averaging history.
 */
void VelAverager::Reset() {
    history_.clear();
    time_ = 0.0;
    sum_ = 0.0;
}

/**
 * @brief Adds one cycle's velocity and returns the time-weighted mean over the
 *        last `kVelAvgWindow`.
 *
 * @param dt Time since last cycle (s)
 * @param vel Actual velocity, θ' (rad/s)
 */
double VelAverager::Add(const double& dt, const double& vel) {
    history_.emplace_back(dt, vel);
    time_ += dt;
    sum_ += dt * vel;

    while (history_.size() > 1
           && time_ - history_.front().first >= kVelAvgWindow) {
        const auto& [old_dt, old_vel] = history_.front();
        time_ -= old_dt;
        sum_ -= old_dt * old_vel;
        history_.pop_front();
    }

    return (time_ > 0.0) ? sum_ / time_ : vel;
}

//------------------------------------------------------------------------------
// !Rest Sampler
//------------------------------------------------------------------------------

/**
 * @brief Clears active sampling window, including velocity history.
 */
void RestSampler::Reset() {
    vel_avg_.Reset();
    RestartWindow();
}

/**
 * @brief Clears active sampling window.
 */
void RestSampler::RestartWindow() {
    count_ = 0;
    t_elapsed_s_ = 0.0;
    sum_angle_ = 0.0;
    sum_torque_ = 0.0;
    sum_volume_ = 0.0;
}

/**
 * @brief Adds one cycle of measurements to active sampling window; returns
 *        averages once window is complete.
 *
 * @param dt Time since last cycle (s)
 * @param pos Actual angle, θ (rad)
 * @param vel Actual velocity, θ' (rad/s)
 * @param torque Actual actuator torque, τ_actuator (Nm)
 * @param trajectory_active Whether a trajectory is running
 * @param volume Latest CW volume (L), if known
 *
 * @note Stillness is judged on velocity averaged over `kVelAvgWindow`.
 */
std::optional<Motion::RestSample> RestSampler::Add(
    const double& dt, const double& pos, const double& vel,
    const double& torque, const bool& trajectory_active,
    const std::optional<double>& volume) {
    const double mean_vel = vel_avg_.Add(dt, vel);
    const bool still = !trajectory_active
                       && std::abs(mean_vel) <= kRestVelTolerance
                       && volume.has_value();
    if (!still) {
        RestartWindow();
        return std::nullopt;
    }

    // Restart window if pose or CW volume drifted since start
    if (count_ > 0
        && (std::abs(pos - anchor_angle_) > kRestPosTolerance
            || std::abs(*volume - anchor_volume_) > kRestVolumeTolerance)) {
        RestartWindow();
    }

    // Accumulate values
    if (count_ == 0) {
        anchor_angle_ = pos;
        anchor_volume_ = *volume;
    } else {
        t_elapsed_s_ += dt;
    }

    sum_angle_ += pos;
    sum_torque_ += torque;
    sum_volume_ += *volume;
    ++count_;

    if (t_elapsed_s_ >= kRestWindowTime && count_ >= 2) {
        // Window finished, return results
        Motion::RestSample sample;
        sample.angle = sum_angle_ / count_;
        sample.torque = sum_torque_ / count_;
        sample.volume = sum_volume_ / count_;
        RestartWindow();
        return sample;
    }

    return std::nullopt;
}

//------------------------------------------------------------------------------
// !Class Management
//------------------------------------------------------------------------------

/**
 * @brief Standard constructor.
 *
 * @param logger Logger to report through (must outlive the calibrator)
 */
Libra2Calibrator::Libra2Calibrator(Logger& logger) : logger_(&logger) {}

//------------------------------------------------------------------------------
// !Commands
//------------------------------------------------------------------------------

/**
 * @brief Starts the level calibration (arm must already be level and at rest).
 *
 * @note Hands the averaged rest sample over as `Output::level_sample`.
 */
bool Libra2Calibrator::StartLevelCalibration() {
    if (IsRunning()) {
        logger_->Warn("Calibration already running; ignoring level request");
        return false;
    }

    logger_->Info("Starting level calibration");
    BeginCalibrationStage(Motion::CalibrationStatus::kArmSamplingLevel);

    return true;
}

/**
 * @brief Starts the two-angle calibration.
 *
 * @param angle_a First angle, θ_a (rad)
 * @param angle_b Second angle, θ_b (rad)
 *
 * @note Hands the averaged rest samples over as `Output::two_angle_samples`.
 */
bool Libra2Calibrator::StartTwoAngleCalibration(const double& angle_a,
                                                const double& angle_b) {
    if (IsRunning()) {
        logger_->Warn(
            "Calibration already running; ignoring two-angle request");
        return false;
    }

    // Validate params
    if (!std::isfinite(angle_a) || !std::isfinite(angle_b)
        || angle_a == angle_b) {
        logger_->Warn("Two-angle calibration rejected: need two distinct "
                      "finite angles (got a="
                      + Motion::FormatNum(angle_a)
                      + ", b=" + Motion::FormatNum(angle_b) + ")");
        return false;
    } else if (std::abs(angle_a) > Motion::kMaxCalibrationAngle
               || std::abs(angle_b) > Motion::kMaxCalibrationAngle) {
        logger_->Warn("Two-angle calibration rejected: need |angle| <= "
                      + Motion::FormatNum(Motion::kMaxCalibrationAngle)
                      + " rad (got a=" + Motion::FormatNum(angle_a)
                      + ", b=" + Motion::FormatNum(angle_b) + ")");
        return false;
    } else if (std::abs(angle_b - angle_a) < Motion::kMinAngleSeparation) {
        logger_->Warn("Two-angle calibration rejected: need |b - a| >= "
                      + Motion::FormatNum(Motion::kMinAngleSeparation)
                      + " rad (got a=" + Motion::FormatNum(angle_a)
                      + ", b=" + Motion::FormatNum(angle_b) + ")");
        return false;
    }

    // Set up and start calibration
    angle_a_ = angle_a;
    angle_b_ = angle_b;

    logger_->Info("Starting two-angle calibration: a="
                  + Motion::FormatRadAsDeg(angle_a)
                  + " deg, b=" + Motion::FormatRadAsDeg(angle_b) + " deg");
    BeginCalibrationStage(Motion::CalibrationStatus::kArmMovingToA);

    return true;
}

/**
 * @brief Starts the friction calibration.
 *
 * @param angle_a One end of the window (rad)
 * @param angle_b Other end of the window (rad)
 * @param max_vel Max. actuator speed (rad/s); sweep speeds are fractions of it
 * @param cw_volume CW volume (L), which must stay put during the sweeps
 * @param inertia Arm + CW moment of inertia (kg·m²), for the ramp lengths
 *
 * @note Hands the fitted model over as `Output::friction`.
 *
 * @see kFrictionVelFractions FitFrictionLine
 */
bool Libra2Calibrator::StartFrictionCalibration(const double& angle_a,
                                                const double& angle_b,
                                                const double& max_vel,
                                                const double& cw_volume,
                                                const double& inertia) {
    if (IsRunning()) {
        logger_->Warn("Calibration already running; ignoring friction request");
        return false;
    }

    // Validate params
    if (!(max_vel > 0.0) || !std::isfinite(max_vel)) {
        logger_->Error("Friction calibration rejected: invalid max speed "
                       + Motion::FormatNum(max_vel) + " rad/s");
        return false;
    }

    const double lo = std::min(angle_a, angle_b);
    const double hi = std::max(angle_a, angle_b);
    if (!std::isfinite(lo) || !std::isfinite(hi) || lo == hi) {
        logger_->Warn("Friction calibration rejected: need two distinct "
                      "finite angles (got a="
                      + Motion::FormatNum(angle_a)
                      + ", b=" + Motion::FormatNum(angle_b) + ")");
        return false;
    } else if (std::abs(lo) > Motion::kMaxCalibrationAngle
               || std::abs(hi) > Motion::kMaxCalibrationAngle) {
        logger_->Warn("Friction calibration rejected: need |angle| <= "
                      + Motion::FormatNum(Motion::kMaxCalibrationAngle)
                      + " rad (got a=" + Motion::FormatNum(angle_a)
                      + ", b=" + Motion::FormatNum(angle_b) + ")");
        return false;
    } else if (hi - lo < Motion::kFrictionMinWindow) {
        logger_->Warn("Friction calibration rejected: need |b - a| >= "
                      + Motion::FormatNum(Motion::kFrictionMinWindow)
                      + " rad (got a=" + Motion::FormatNum(angle_a)
                      + ", b=" + Motion::FormatNum(angle_b) + ")");
        return false;
    }

    // Ramps widen with speed, so fastest sweep determines unsampled ends
    const double longest_ramp = RampDist(max_vel, inertia);
    const double margin = longest_ramp + kFrictionSampleMargin;

    const double min_window = (2.0 * margin)
                              + (kFrictionMinBins * kFrictionBinWidth);
    if (hi - lo < min_window) {
        logger_->Warn("Friction calibration rejected: window is "
                      + Motion::FormatRadAsDeg(hi - lo) + " deg, need at least "
                      + Motion::FormatRadAsDeg(min_window) + " deg (the "
                      + Motion::FormatRadAsDeg(max_vel) + " deg/s sweeps take "
                      + Motion::FormatRadAsDeg(longest_ramp)
                      + " deg at each end to speed up and slow down)");
        return false;
    }

    // Ensure slowest sweep fits in one move stage
    const double slowest = max_vel * kFrictionVelFractions.front();
    const double t_slowest_s = SweepTime(lo, hi, slowest,
                                         RampDist(slowest, inertia));
    if (t_slowest_s > kMoveTimeout) {
        logger_->Warn("Friction calibration rejected: slowest sweep takes "
                      + Motion::FormatNum(t_slowest_s) + " s (limit "
                      + Motion::FormatNum(kMoveTimeout)
                      + " s); use a smaller window");
        return false;
    }

    // Set up and start calibration
    friction_run_ = FrictionRun();
    friction_run_.angle_lo = lo;
    friction_run_.angle_hi = hi;
    friction_run_.max_vel = max_vel;
    friction_run_.volume = cw_volume;
    friction_run_.inertia = inertia;
    friction_run_.margin = margin;

    // Sweep speeds, for the start log
    std::string vels_deg;
    for (const double& fraction : kFrictionVelFractions) {
        vels_deg += (vels_deg.empty() ? "" : ", ")
                    + Motion::FormatRadAsDeg(max_vel * fraction);
    }

    logger_->Info("Starting friction calibration: " + Motion::FormatRadAsDeg(lo)
                  + " to " + Motion::FormatRadAsDeg(hi) + " deg at " + vels_deg
                  + " deg/s");
    logger_->Debug("Friction calibration samples "
                   + Motion::FormatRadAsDeg(lo + margin) + " to "
                   + Motion::FormatRadAsDeg(hi - margin) + " deg, CW idle at "
                   + Motion::FormatNum(cw_volume) + " L, aborts above "
                   + Motion::FormatNum(kMaxFrictionTorque) + " Nm");
    BeginCalibrationStage(Motion::CalibrationStatus::kFrictionMovingToStart);

    return true;
}

/**
 * @brief Cancels a running calibration, stopping any sweep in progress.
 */
void Libra2Calibrator::CancelCalibration() {
    if (!IsRunning()) {
        return;
    }

    // Stop in-progress moves (check BEFORE clearing status)
    if (IsFrictionRunning()) {
        output_.stop_trajectory = true;
    }

    status_ = Motion::CalibrationStatus::kIdle;

    output_.move_to.reset();
    output_.sweep.reset();

    sampler_.Reset();
    calibration_progress_.clear();

    calibration_msg_ = "Calibration cancelled";
    logger_->Info(calibration_msg_);
}

/**
 * @brief Ends a calibration and logs the outcome.
 *
 * @param success Whether the arm model or friction was updated
 * @param message Descriptive outcome (success: the result; failure: the reason)
 *
 * @note The motion controller calls it on success, once it has applied the
 *       results.
 */
void Libra2Calibrator::FinishCalibration(const bool& success,
                                         const std::string& message) {
    status_ = success ? Motion::CalibrationStatus::kSucceeded
                      : Motion::CalibrationStatus::kFailed;

    output_.move_to.reset();
    output_.sweep.reset();
    sampler_.Reset();
    calibration_progress_.clear();

    if (success) {
        calibration_msg_ = message;
        logger_->Info(calibration_msg_);
    } else {
        calibration_msg_ = "Calibration failed: " + message;
        logger_->Error(calibration_msg_);
    }
}

/**
 * @brief Cancels a running calibration and drops all pending requests.
 */
void Libra2Calibrator::Reset() {
    CancelCalibration();
    output_ = Output();
}

//------------------------------------------------------------------------------
// !Control Cycle
//------------------------------------------------------------------------------

/**
 * @brief Advances the running calibration by one cycle.
 *
 * @param cycle This cycle's pitch measurements
 */
void Libra2Calibrator::Update(const PitchCycle& cycle) {
    if (IsFrictionRunning()) {
        RunFrictionCalibration(cycle);
    } else if (IsRunning()) {
        RunArmCalibration(cycle);
    }
}

/**
 * @brief Takes the pending requests and results, leaving none behind.
 */
Libra2Calibrator::Output Libra2Calibrator::TakeOutput() {
    return std::exchange(output_, Output());
}

//------------------------------------------------------------------------------
// !Status
//------------------------------------------------------------------------------

/**
 * @brief Checks whether a calibration is running.
 */
bool Libra2Calibrator::IsRunning() const {
    return Motion::IsCalibrationRunning(status_);
}

/**
 * @brief Checks whether the friction calibration is running.
 */
bool Libra2Calibrator::IsFrictionRunning() const {
    return IsFrictionStage(status_);
}

/**
 * @brief Gets calibration progress.
 */
Motion::CalibrationStatus Libra2Calibrator::GetCalibrationStatus() const {
    return status_;
}

/**
 * @brief Gets a description of the last calibration, if any.
 */
const std::string& Libra2Calibrator::GetCalibrationMessage() const {
    return calibration_msg_;
}

/**
 * @brief Gets the running calibration's current step (e.g., "Friction
 *        calibration [3/9]: ..."); empty if none is running.
 */
const std::string& Libra2Calibrator::GetCalibrationProgress() const {
    return calibration_progress_;
}

//------------------------------------------------------------------------------
// !Calibration Helpers
//------------------------------------------------------------------------------

/**
 * @brief Enters a calibration stage and, if applicable, requests a move.
 *
 * @param status Calibration stage to transition to
 */
void Libra2Calibrator::BeginCalibrationStage(
    const Motion::CalibrationStatus& status) {
    status_ = status;
    t_stage_s_ = 0.0;
    sampler_.Reset();

    FrictionRun& fr = friction_run_;
    switch (status) {
        case Motion::CalibrationStatus::kArmMovingToA:
            output_.move_to = angle_a_;
            break;
        case Motion::CalibrationStatus::kArmMovingToB:
            output_.move_to = angle_b_;
            break;
        case Motion::CalibrationStatus::kFrictionMovingToStart:
            output_.move_to = fr.angle_lo;
            break;
        case Motion::CalibrationStatus::kFrictionSweepUp:
        case Motion::CalibrationStatus::kFrictionSweepDown:
            {
                const bool going_up =
                    (status == Motion::CalibrationStatus::kFrictionSweepUp);

                Motion::SweepRequest sweep;
                sweep.from = going_up ? fr.angle_lo : fr.angle_hi;
                sweep.to = going_up ? fr.angle_hi : fr.angle_lo;
                sweep.vel = fr.max_vel * kFrictionVelFractions.at(fr.vel_idx);
                sweep.ramp_dist = RampDist(sweep.vel, fr.inertia);

                output_.sweep = sweep;
                fr.trajectory_seen = false;

                // Each sweep averages vel from its own start
                fr.vel_avg.Reset();

                if (going_up) {
                    // Each run starts with an upwards sweep and fresh history
                    const auto n_bins = static_cast<std::size_t>(std::ceil(
                        (fr.angle_hi - fr.angle_lo - (2.0 * fr.margin))
                        / kFrictionBinWidth));
                    fr.up_sum.assign(n_bins, 0.0);
                    fr.down_sum.assign(n_bins, 0.0);
                    fr.up_count.assign(n_bins, 0);
                    fr.down_count.assign(n_bins, 0);
                    fr.range_samples = 0;
                    fr.steady_samples = 0;
                }
                break;
            }
        default:
            break;
    }

    calibration_progress_ = DescribeCalibrationStage();
    if (!calibration_progress_.empty()) {
        logger_->Debug(calibration_progress_);
    }
    t_progress_log_s_ = 0.0;
}

/**
 * @brief Describes the active calibration step (for progress messages).
 *
 * @note Returns empty string if no calibration is running.
 */
std::string Libra2Calibrator::DescribeCalibrationStage() const {
    const FrictionRun& fr = friction_run_;
    const auto n_vels = static_cast<int>(kFrictionVelFractions.size());
    const std::string at_rest = "averaging torque once system at rest for "
                                + Motion::FormatNum(kRestWindowTime) + " s";

    std::string name;
    int step = 0;
    int total = 0;
    std::string action;

    switch (status_) {
        case Motion::CalibrationStatus::kArmSamplingLevel:
            name = "Level";
            step = 1;
            total = 1;
            action = at_rest;
            break;
        case Motion::CalibrationStatus::kArmMovingToA:
        case Motion::CalibrationStatus::kArmSamplingA:
        case Motion::CalibrationStatus::kArmMovingToB:
        case Motion::CalibrationStatus::kArmSamplingB:
            {
                const bool at_a =
                    (status_ == Motion::CalibrationStatus::kArmMovingToA
                     || status_ == Motion::CalibrationStatus::kArmSamplingA);
                const bool moving =
                    (status_ == Motion::CalibrationStatus::kArmMovingToA
                     || status_ == Motion::CalibrationStatus::kArmMovingToB);
                const std::string label = at_a ? "a" : "b";

                name = "Two-angle";
                step = (at_a ? 1 : 3) + (moving ? 0 : 1);
                total = 4;
                action = moving ? "moving to " + label + "="
                                      + Motion::FormatRadAsDeg(at_a ? angle_a_
                                                                    : angle_b_)
                                      + " deg"
                                : "at " + label + ", " + at_rest;
                break;
            }
        case Motion::CalibrationStatus::kFrictionMovingToStart:
            name = "Friction";
            step = 1;
            total = 1 + (2 * n_vels);
            action = "moving to window start "
                     + Motion::FormatRadAsDeg(fr.angle_lo) + " deg";
            break;
        case Motion::CalibrationStatus::kFrictionSweepUp:
        case Motion::CalibrationStatus::kFrictionSweepDown:
            {
                const bool going_up =
                    (status_ == Motion::CalibrationStatus::kFrictionSweepUp);
                const auto idx = static_cast<int>(fr.vel_idx);
                const double vel = fr.max_vel
                                   * kFrictionVelFractions.at(fr.vel_idx);
                const double t_sweep_s = SweepTime(fr.angle_lo, fr.angle_hi,
                                                   vel,
                                                   RampDist(vel, fr.inertia));

                name = "Friction";
                step = 2 + (2 * idx) + (going_up ? 0 : 1);
                total = 1 + (2 * n_vels);
                action = "speed " + std::to_string(idx + 1) + "/"
                         + std::to_string(n_vels) + " ("
                         + Motion::FormatRadAsDeg(vel) + " deg/s), sweeping "
                         + (going_up ? "up" : "down") + " to "
                         + Motion::FormatRadAsDeg(going_up ? fr.angle_hi
                                                           : fr.angle_lo)
                         + " deg (~"
                         + std::to_string(static_cast<int>(std::ceil(t_sweep_s)))
                         + " s)";
                break;
            }
        default:
            return "";
    }

    return name + " calibration [" + std::to_string(step) + "/"
           + std::to_string(total) + "]: " + action;
}

/**
 * @brief Logs a "still running" line every `kProgressLogPeriod`, to
 *        give the user visual feedback during slow steps.
 *
 * @param cycle This cycle's pitch measurements
 * @param timeout Time limit of active step (s)
 */
void Libra2Calibrator::LogCalibrationProgress(const PitchCycle& cycle,
                                              const double& timeout) {
    t_progress_log_s_ += cycle.dt;
    if (t_progress_log_s_ < kProgressLogPeriod) {
        return;
    }
    t_progress_log_s_ = 0.0;

    logger_->Debug(calibration_progress_ + "\n  still running: "
                   + std::to_string(static_cast<int>(t_stage_s_)) + " of "
                   + std::to_string(static_cast<int>(timeout))
                   + " s before timeout"
                     "\n  angle:         "
                   + Motion::FormatRadAsDeg(cycle.angle)
                   + " deg"
                     "\n  speed:         "
                   + Motion::FormatRadAsDeg(cycle.vel)
                   + " deg/s"
                     "\n  trajectory:    "
                   + (cycle.trajectory_active ? "running" : "finished"));
}

/**
 * @brief Advances the active arm calibration (level or two-angle) by one cycle.
 *
 * @param cycle This cycle's pitch measurements
 */
void Libra2Calibrator::RunArmCalibration(const PitchCycle& cycle) {
    t_stage_s_ += cycle.dt;

    const bool moving_stage =
        (status_ == Motion::CalibrationStatus::kArmMovingToA
         || status_ == Motion::CalibrationStatus::kArmMovingToB);
    const bool at_a = (status_ == Motion::CalibrationStatus::kArmMovingToA
                       || status_ == Motion::CalibrationStatus::kArmSamplingA);
    const double target = at_a ? angle_a_ : angle_b_;

    const double timeout = moving_stage ? kMoveTimeout : kSamplingTimeout;
    if (t_stage_s_ > timeout) {
        FinishCalibration(false, moving_stage
                                     ? "timed out moving to target"
                                     : "timed out waiting for steady state");
        return;
    }
    LogCalibrationProgress(cycle, timeout);

    // Sampling stages: average once steady
    std::optional<Motion::RestSample> sample;
    if (!moving_stage) {
        // Sampling is invalid if pose drifts (except for level calibration)
        if (status_ != Motion::CalibrationStatus::kArmSamplingLevel
            && std::abs(cycle.angle - target) > 2.0 * kReachTolerance) {
            sampler_.Reset();
            return;
        }

        sample = sampler_.Add(cycle.dt, cycle.angle, cycle.vel, cycle.torque,
                              cycle.trajectory_active, cycle.cw_volume);
        if (!sample) {
            return;
        }
    }

    switch (status_) {
        case Motion::CalibrationStatus::kArmMovingToA:
        case Motion::CalibrationStatus::kArmMovingToB:
            // Start next step of an active calibration
            if (!cycle.trajectory_active
                && std::abs(cycle.angle - target) <= kReachTolerance
                && std::abs(cycle.vel) <= kRestVelTolerance) {
                BeginCalibrationStage(
                    at_a ? Motion::CalibrationStatus::kArmSamplingA
                         : Motion::CalibrationStatus::kArmSamplingB);
            }
            break;

        case Motion::CalibrationStatus::kArmSamplingLevel:
            output_.level_sample = *sample;
            break;

        case Motion::CalibrationStatus::kArmSamplingA:
            sample_a_ = *sample;
            BeginCalibrationStage(Motion::CalibrationStatus::kArmMovingToB);
            break;

        case Motion::CalibrationStatus::kArmSamplingB:
            output_.two_angle_samples = std::make_pair(sample_a_, *sample);
            break;

        default:
            break;
    }
}

//------------------------------------------------------------------------------
// !Friction Calibration Helpers
//------------------------------------------------------------------------------

/**
 * @brief Advances the running friction calibration by one cycle.
 *
 * @param cycle This cycle's pitch measurements
 *
 * @note Aborts (and stops the sweep) if the torque stays above
 *       `kMaxFrictionTorque` for `kMaxFrictionTorqueTime`, or if the CW volume
 *       changes.
 */
void Libra2Calibrator::RunFrictionCalibration(const PitchCycle& cycle) {
    FrictionRun& run = friction_run_;
    t_stage_s_ += cycle.dt;

    // Brief spikes shouldn't count against the limit
    if (std::abs(cycle.torque) > kMaxFrictionTorque) {
        run.t_over_max_s += cycle.dt;
    } else {
        run.t_over_max_s = 0.0;
    }

    // Abort guards
    std::string abort_reason;
    if (t_stage_s_ > kMoveTimeout) {
        abort_reason = "timed out moving or sweeping";
    } else if (run.t_over_max_s >= kMaxFrictionTorqueTime) {
        abort_reason = "torque (" + Motion::FormatNum(cycle.torque)
                       + " Nm) stayed above the calibration's torque limit ("
                       + Motion::FormatNum(kMaxFrictionTorque) + " Nm) for "
                       + Motion::FormatNum(kMaxFrictionTorqueTime)
                       + " s; balance the CW or narrow/move the window";
    } else if (!cycle.cw_volume
               || std::abs(*cycle.cw_volume - run.volume)
                      > kRestVolumeTolerance) {
        abort_reason = "CW volume changed during the sweeps";
    }

    if (!abort_reason.empty()) {
        output_.stop_trajectory = true;
        FinishCalibration(false, abort_reason);
        return;
    }
    LogCalibrationProgress(cycle, kMoveTimeout);

    const bool going_up = (status_
                           == Motion::CalibrationStatus::kFrictionSweepUp);
    const double end =
        (status_ == Motion::CalibrationStatus::kFrictionSweepDown
         || status_ == Motion::CalibrationStatus::kFrictionMovingToStart)
            ? run.angle_lo
            : run.angle_hi;
    const double vel = run.max_vel * kFrictionVelFractions.at(run.vel_idx);

    if (status_ != Motion::CalibrationStatus::kFrictionMovingToStart) {
        if (cycle.trajectory_active) {
            run.trajectory_seen = true;
            AddFrictionSample(cycle, going_up, vel);
            return;
        }
        if (!run.trajectory_seen) {
            return;  // sweep requested but not started yet
        }
    }

    // Wait until the move/sweep has ended at rest at its end
    const bool arrived = !cycle.trajectory_active
                         && std::abs(cycle.angle - end) <= kReachTolerance
                         && std::abs(cycle.vel) <= kRestVelTolerance;
    if (!arrived) {
        return;
    }

    switch (status_) {
        case Motion::CalibrationStatus::kFrictionMovingToStart:
            BeginCalibrationStage(Motion::CalibrationStatus::kFrictionSweepUp);
            break;

        case Motion::CalibrationStatus::kFrictionSweepUp:
            BeginCalibrationStage(Motion::CalibrationStatus::kFrictionSweepDown);
            break;

        case Motion::CalibrationStatus::kFrictionSweepDown:
            if (const auto error = FinishFrictionVel(vel)) {
                FinishCalibration(false, *error);
                return;
            }
            ++run.vel_idx;
            if (run.vel_idx < kFrictionVelFractions.size()) {
                BeginCalibrationStage(
                    Motion::CalibrationStatus::kFrictionSweepUp);
            } else {
                FinishFrictionCalibration();
            }
            break;

        default:
            break;
    }
}

/**
 * @brief Bins a friction sweep sample if it is inside the sampled range and at
 *        steady speed.
 *
 * @param cycle This cycle's pitch measurements
 * @param going_up Whether this is the up sweep
 * @param vel Sweep speed, v (rad/s)
 *
 * @note Steady speed is judged on velocity averaged over `kVelAvgWindow`, since
 *       raw velocity feedback tends to jitter above `kFrictionVelTolerance`
 *       (even when the sweep is steady).
 */
void Libra2Calibrator::AddFrictionSample(const PitchCycle& cycle,
                                         const bool& going_up,
                                         const double& vel) {
    FrictionRun& run = friction_run_;

    // Skip window edges, where ramps add inertia and CW may swing
    const double lo = run.angle_lo + run.margin;
    const double hi = run.angle_hi - run.margin;
    if (cycle.angle < lo || cycle.angle > hi) {
        return;  // near a window edge
    }
    ++run.range_samples;

    // Skip sudden jerks and jitters, which add error to calibration result
    const double mean_vel = run.vel_avg.Add(cycle.dt, cycle.vel);
    const double sweep_vel = going_up ? vel : -vel;
    if (std::abs(mean_vel - sweep_vel) > kFrictionVelTolerance * vel) {
        return;  // not at steady speed
    }
    ++run.steady_samples;

    // Bin by angle so up/down sweeps are compared at equal gravitational torque
    const std::size_t n_bins = run.up_sum.size();
    if (n_bins == 0) {
        return;
    }

    const auto bin = std::min(n_bins - 1,
                              static_cast<std::size_t>((cycle.angle - lo)
                                                       / kFrictionBinWidth));
    if (going_up) {
        run.up_sum[bin] += cycle.torque;
        ++run.up_count[bin];
    } else {
        run.down_sum[bin] += cycle.torque;
        ++run.down_count[bin];
    }
}

/**
 * @brief Turns one speed's binned sweeps into a friction point.
 *
 * @param vel Sweep speed, v (rad/s)
 *
 * @note Returns the reason for failure, if any.
 *
 * @note Code-specific equation based on Eqs. 18 and 19a at constant speed.
 *       Per angle bin, with actual actuator torque (so bin mean = -τ_ext,g):
 *         drag = (τ_up - τ_down) / 2
 *         τ_g(θ) = (τ_up + τ_down) / 2
 */
std::optional<std::string> Libra2Calibrator::FinishFrictionVel(
    const double& vel) {
    FrictionRun& run = friction_run_;

    const double lo = run.angle_lo + run.margin;

    Motion::FrictionPoint point;
    point.vel = vel;
    double drag_min = 0.0;
    double drag_max = 0.0;
    double grav_sum = 0.0;

    for (std::size_t i = 0; i < run.up_sum.size(); ++i) {
        if (run.up_count[i] < kFrictionMinBinSamples
            || run.down_count[i] < kFrictionMinBinSamples) {
            // Not enough samples to compare
            continue;
        }

        // Friction flips sign between sweeps and gravity doesn't, so half the
        // difference is friction and half the sum is holding torque
        const double up = run.up_sum[i] / run.up_count[i];
        const double down = run.down_sum[i] / run.down_count[i];
        const double drag = 0.5 * (up - down);
        const double holding = 0.5 * (up + down);

        // Track how drag varies across bins
        if (point.n_bins == 0) {
            drag_min = drag;
            drag_max = drag;
        }

        drag_min = std::min(drag_min, drag);
        drag_max = std::max(drag_max, drag);
        point.drag += drag;
        grav_sum += holding;

        // Save bin for calibration record
        Motion::FrictionBin bin;
        bin.angle = lo + ((static_cast<double>(i) + 0.5) * kFrictionBinWidth);
        bin.drag = drag;
        bin.holding = holding;
        bin.n_up = run.up_count[i];
        bin.n_down = run.down_count[i];
        point.bins.push_back(bin);

        ++point.n_bins;
    }

    if (point.n_bins < kFrictionMinBins) {
        return "only " + std::to_string(point.n_bins)
               + " angle bins had steady samples in both directions at "
               + Motion::FormatNum(vel) + " rad/s (need "
               + std::to_string(kFrictionMinBins) + "); use a wider window";
    }

    // Average drag over the bins and record this speed's friction point
    const auto n = static_cast<double>(point.n_bins);
    point.drag /= n;
    point.spread = drag_max - drag_min;

    // Low steady fraction = uneven sweep speed (stick-slip), so noisier drag
    point.steady_fraction = (run.range_samples > 0)
                                ? static_cast<double>(run.steady_samples)
                                      / run.range_samples
                                : 0.0;
    const auto steady_pct = static_cast<int>(
        std::lround(100.0 * point.steady_fraction));
    run.points.push_back(point);

    logger_->Debug("Friction speed " + std::to_string(run.vel_idx + 1) + "/"
                   + std::to_string(kFrictionVelFractions.size()) + " done ("
                   + Motion::FormatRadAsDeg(vel)
                   + " deg/s):"
                     "\n  drag:           "
                   + Motion::FormatNum(point.drag) + " Nm (spread "
                   + Motion::FormatNum(point.spread) + " Nm over "
                   + std::to_string(point.n_bins)
                   + " bins)"
                     "\n  steady samples: "
                   + std::to_string(steady_pct)
                   + "% of samples in the sampled range"
                     "\n  holding torque: "
                   + Motion::FormatNum(grav_sum / n) + " Nm (mean)");

    return std::nullopt;
}

/**
 * @brief Fits the friction line through all speeds and hands it over.
 *
 * @note Code-specific rule. A negative slope means friction is flat within
 *       noise, so b = 0 and τ_c is the mean drag. A negative τ_c within the
 *       largest bin spread means friction is too small to measure, so τ_c = 0
 *       and b is refit through the origin; otherwise, the calibration fails.
 */
void Libra2Calibrator::FinishFrictionCalibration() {
    const FrictionRun& run = friction_run_;

    std::vector<double> vels;
    std::vector<double> drags;
    for (const auto& p : run.points) {
        vels.push_back(p.vel);
        drags.push_back(p.drag);
    }

    auto fit = FitFrictionLine(vels, drags);
    if (!fit) {
        FinishCalibration(false, "friction line fit failed (need two or more "
                                 "speeds)");
        return;
    }

    Motion::FrictionModel& model = *fit;
    if (model.viscous < 0.0) {
        logger_->Warn("Fitted b is negative (" + Motion::FormatNum(model.viscous)
                      + " Nm*s/rad), so friction looks flat; using b = 0");

        model.viscous = 0.0;
        model.coulomb = 0.0;
        for (const double& d : drags) {
            model.coulomb += d;
        }
        model.coulomb /= static_cast<double>(drags.size());
    }

    if (model.coulomb < 0.0) {
        // Bin spread is the drag noise floor
        double noise = 0.0;
        for (const auto& p : run.points) {
            noise = std::max(noise, p.spread);
        }

        if (-model.coulomb > noise) {
            FinishCalibration(false, "fitted tau_c is negative ("
                                         + Motion::FormatNum(model.coulomb)
                                         + " Nm), past the bin spread ("
                                         + Motion::FormatNum(noise)
                                         + " Nm); check the run");
            return;  // nonsensical result
        }

        logger_->Info(
            "Fitted tau_c is negative (" + Motion::FormatNum(model.coulomb)
            + " Nm) but within bin spread (" + Motion::FormatNum(noise)
            + " Nm), so friction is negligible; using tau_c = 0");

        // Refit b through the origin
        double s_vv = 0.0;
        double s_vd = 0.0;
        for (std::size_t i = 0; i < vels.size(); ++i) {
            s_vv += vels[i] * vels[i];
            s_vd += vels[i] * drags[i];
        }

        model.coulomb = 0.0;
        model.viscous = std::max(0.0, s_vd / s_vv);
    }

    double sum_sq = 0.0;
    for (std::size_t i = 0; i < vels.size(); ++i) {
        const double r = drags[i] - (model.coulomb + (model.viscous * vels[i]));
        sum_sq += r * r;
    }

    model.residual_rms = std::sqrt(sum_sq / static_cast<double>(drags.size()));

    // Hand over model
    model.angle_lo = run.angle_lo;
    model.angle_hi = run.angle_hi;
    model.volume = run.volume;
    model.points = run.points;

    output_.friction = model;
}
