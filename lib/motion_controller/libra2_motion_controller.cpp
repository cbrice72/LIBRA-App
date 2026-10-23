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
#include <array>
#include <cmath>
#include <string>

// Project Headers
#include "joint_defs.h"

/* --- TABLE OF CONTENTS ---
 * !Local Helpers
 * !Counterweight Model
 * !Pure Equations
 * !Rest Sampler
 * !Class Management
 * !Settings & Arm Model
 * !Calibration Commands
 * !Calibration Record
 * !Control Cycle
 * !Hooks
 * !Model Calibration Helpers
 * !Friction Calibration Helpers
 */

// Physical Constants

constexpr double kWaterDensity = 1.0;  // ρ_water (kg/L) -- Eq. 28
constexpr double kGrav = 9.80665;      // g (m/s²)

// Mechanical Constants (heights above the arm/CW centerline)

constexpr double kPitchJointHeight = 0.1005;  // pitch joint O (m)
constexpr double kCwBracketHeight = 0.0427;   // CW hang points (m)

// Model Parameters

constexpr double kCwPivotDist = 0.7777;      // r_P at 0 liters, A (m)
constexpr double kCwPivotSlope = -0.001155;  // r_P change per liter, B (m/L)
constexpr double kCwHangOffset = kCwBracketHeight
                                 - kPitchJointHeight;  // q_P (m)
constexpr double kArmInertia =
    47.6;  // I_CAD, from Inertial Properties -> Global -> Izz (kg·m²)
// TODO: set from experimental data (or run the friction calibration)
constexpr double kDefaultFrictionCoulomb = 0.0;  // τ_c (Nm)
constexpr double kDefaultFrictionViscous = 0.0;  // b (Nm·s/rad)

// Predictive CW Command

// TODO: guess; prevents valve chatter
constexpr double kCwVolumeTolerance = 0.01;  // "target reached" band (L)

// Model Calibration

// TODO: all of these are guesses; tune via experiments
constexpr double kMaxLevelAngle = 0.0175;    // |θ| limit for level cal (rad)
constexpr double kReachTolerance = 0.01;     // "arrived at target" (rad)
constexpr double kRestVelTolerance = 0.005;  // "at rest" speed (rad/s)
constexpr double kRestPosTolerance = 0.005;  // allowed drift in window (rad)
constexpr double kRestVolTolerance = 0.01;   // allowed CW change in window (L)
constexpr double kRestWindowTime = 3.0;      // averaging window (s)
constexpr double kMoveTimeout = 180.0;       // per move stage (s)
constexpr double kSampleTimeout = 60.0;      // per sampling stage (s)
constexpr double kMinCosForBalance = 1e-3;   // |cosθ| floor, Eq. 34/38'

// Friction Calibration

// TODO: all of these are guesses; tune via experiments
constexpr double kFrictionRampDist = 0.0175;     // to reach speed, ~1 deg (rad)
constexpr double kFrictionSampleMargin = 0.035;  // unsampled ends, ~2 deg (rad)
constexpr double kFrictionBinWidth = 0.0087;     // angle bin, ~0.5 deg (rad)
constexpr double kFrictionSpeedTolerance = 0.1;  // allowed |θ̇ - v| / v
constexpr int kFrictionMinBinSamples = 3;        // per bin and direction
constexpr int kFrictionMinBins = 3;              // usable bins per speed
// Sweep speeds as fractions of the caller's max speed (~0.5, 1, 2, 3 deg/s)
constexpr std::array<double, 4> kFrictionSpeedFractions = {1.0 / 6.0, 1.0 / 3.0,
                                                           2.0 / 3.0, 1.0};

//------------------------------------------------------------------------------
// !Local Helpers
//------------------------------------------------------------------------------

namespace {

constexpr std::size_t kPitchIdx = Joint::Name::kPitch;

/**
 * @brief Gets the sign of a number (−1, 0, or 1).
 *
 * @param x Number to check
 */
double Sign(const double& x) {
    return (x > 0.0) - (x < 0.0);
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
 * @brief Checks whether two counterweight (CW) models are the same (within
 *        a tolerance).
 *
 * @param a First CW model
 * @param b Second CW model
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
// !Counterweight Model
//------------------------------------------------------------------------------

/**
 * @brief Builds and checks the counterweight (CW) model (pivot distance vs.
 *        volume).
 *
 * @param cw_pivot_dist CW pivot distance at 0 liters, A (m); sign is ignored
 *                      (0 = "not measured yet" and leaves model invalid)
 * @param cw_pivot_slope Change in CW pivot distance per liter, B (m/L); may be
 *                       negative
 * @param cw_hang_offset CW hang point offset (normal to arm) relative to pitch
 *                       joint O, q_P (m)
 * @param capacity Max. combined CW volume (L)
 *
 * @note This is a code-specific extension of Eq. 14b, where r_P varies linearly
 *       with volume:
 *         r_P(V) = A + B·V.
 */
CwModel::CwModel(const double& cw_pivot_dist, const double& cw_pivot_slope,
                 const double& cw_hang_offset, const double& capacity)
    : capacity_(capacity) {
    if (!std::isfinite(capacity) || capacity <= 0.0) {
        error_ = "CW capacity must be positive";
        return;
    }
    if (!std::isfinite(cw_pivot_dist) || !std::isfinite(cw_pivot_slope)
        || !std::isfinite(cw_hang_offset)) {
        error_ = "One or more of r_P components or q_P is not a number";
        return;
    }
    if (cw_pivot_dist == 0.0) {
        error_ = "r_P is not set (intercept = 0)";
        return;
    }

    r_P_0_ = std::abs(cw_pivot_dist);
    dr_P_dV_ = cw_pivot_slope;
    q_P_ = cw_hang_offset;

    if (!(r_P_0_ + 2.0 * dr_P_dV_ * capacity_ > 0.0)) {
        error_ = "r_P line is too steep: the CW moment must increase with "
                 "volume up to full (need A + 2*B*capacity > 0)";
    }
}

/**
 * @brief Gets the values that define the current model.
 */
Motion::CwModelParams CwModel::GetParams() const {
    Motion::CwModelParams params;
    params.pivot_dist = r_P_0_;
    params.pivot_slope = dr_P_dV_;
    params.hang_offset = q_P_;
    params.capacity = capacity_;
    return params;
}

/**
 * @brief Gets the expected mass of a full CW (kg).
 */
double CwModel::GetCapacityMass() const {
    return capacity_ * kWaterDensity;
}

/**
 * @brief Computes the distance (m) along the arm from pitch joint O to CW pivot
 *        P at a given CW volume.
 *
 * @param volume CW volume, V (L)
 *
 * @note Code-specific linear model:
 *         r_P(V) = A + B·V
 */
double CwModel::PivotDist(const double& volume) const {
    return r_P_0_ + dr_P_dV_ * volume;
}

/**
 * @brief Computes the CW's gravitational moment along the arm (kg·m).
 *
 * @param mass CW mass, m_cw (kg)
 *
 * @note Code-specific extension of Eqs. 12' and 14b, with Eq. 28 for volume:
 *         m_cw·r_P(V), where V = m_cw / ρ_water
 */
double CwModel::Moment(const double& mass) const {
    return mass * PivotDist(mass / kWaterDensity);
}

/**
 * @brief Computes the CW's gravitational moment normal to arm (kg·m).
 *
 * @param mass CW mass, m_cw (kg)
 *
 * @note Sine-term moment of Eq. 12':
 *         m_cw·q_P
 *
 * @see Libra2Model::CwGravTorque
 */
double CwModel::HangMoment(const double& mass) const {
    return mass * q_P_;
}

/**
 * @brief Computes the CW moment (kg·m) that the balance equation compares at
 *        an angle.
 *
 * @param mass CW mass, m_cw (kg)
 * @param theta Pitch angle, θ (rad)
 *
 * @note Code-specific extension of Eq. 38' for r_P that varies with volume;
 *       dividing CW torque by g·cosθ gives:
 *         m_cw·(r_P(V) + q_P·tanθ)
 */
double CwModel::EffectiveMoment(const double& mass, const double& theta) const {
    return Moment(mass) + HangMoment(mass) * std::tan(theta);
}

/**
 * @brief Finds the CW mass (kg) that gives a requested moment at an angle
 *        (inverse of `EffectiveMoment()`).
 *
 * @param moment Requested effective moment, M (kg·m)
 * @param theta Pitch angle, θ (rad)
 *
 * @note Result is bound to [0, capacity]. Returns none if the model is invalid
 *       or the effective lever is too short at the given angle (i.e., if moment
 *       no longer rises with mass, the answer is ambiguous).
 *
 * @note Code-specific extension: with A' = A + q_P·tanθ, solves
 *       m_cw·(A' + B·m_cw / ρ_water) = M in a form that stays accurate for
 *       small B:
 *         m_cw = 2·M / (A' + √(A'² + 4·B·M / ρ_water))
 *
 * @see CwModel::EffectiveMoment
 */
std::optional<double> CwModel::MassForMoment(const double& moment,
                                             const double& theta) const {
    if (!IsValid()) {
        return std::nullopt;
    }

    // Effective lever must keep the moment rising from empty to full
    const double a_eff = r_P_0_ + q_P_ * std::tan(theta);
    if (!(a_eff > 0.0) || !(a_eff + 2.0 * dr_P_dV_ * capacity_ > 0.0)) {
        return std::nullopt;
    }

    // Clamp to [0, capacity]
    if (!(moment > 0.0)) {
        return 0.0;
    }

    const double cap = GetCapacityMass();
    if (moment >= EffectiveMoment(cap, theta)) {
        return cap;
    }

    // Solve for m_cw
    const double c = dr_P_dV_ / kWaterDensity;
    const double disc = std::max(0.0, a_eff * a_eff + 4.0 * c * moment);
    return 2.0 * moment / (a_eff + std::sqrt(disc));
}

/**
 * @brief Computes the CW moment of inertia (kg·m²) about pitch joint O.
 *
 * @param mass CW mass, m_cw (kg)
 *
 * @note Code-specific CW term of the inertia in Eq. 15, as a point mass at CW
 *       pivot P:
 *         m_cw·(r_P(V)² + q_P²)
 */
double CwModel::Inertia(const double& mass) const {
    const double r_P = PivotDist(mass / kWaterDensity);
    return mass * (r_P * r_P + q_P_ * q_P_);
}

/**
 * @brief Computes the change in CW moment of inertia per kg of CW (m²).
 *
 * @param mass CW mass, m_cw (kg)
 *
 * @note Code-specific derivative of `Inertia()` for the inertia rate in Eq. 15:
 *         d(inertia)/dm_cw = r_P·(r_P + 2·B·V) + q_P²
 *
 * @see CwModel::Inertia
 */
double CwModel::InertiaSlope(const double& mass) const {
    const double volume = mass / kWaterDensity;
    const double r_P = PivotDist(volume);
    return r_P * (r_P + 2.0 * dr_P_dV_ * volume) + q_P_ * q_P_;
}

//------------------------------------------------------------------------------
// !Pure Equations
//------------------------------------------------------------------------------

/**
 * @brief Stateless equations.
 *
 * @see CwModel
 */
namespace Libra2Model {

/**
 * @brief Converts CW volume (L) to mass (kg).
 *
 * @param volume CW volume, V (L)
 *
 * @note From Eq. 28:
 *         m_cw = ρ_water·V
 */
double CwMass(const double& volume) {
    return kWaterDensity * volume;
}

/**
 * @brief Computes the CW's gravitational torque (Nm) about pitch joint O.
 *
 * @param cw_moment CW gravitational moment along arm, m_cw·r_P (kg·m)
 * @param cw_hang_moment CW gravitational moment normal to arm, m_cw·q_P (kg·m)
 * @param grav Gravitational acceleration, g (m/s²)
 * @param theta Pitch angle, θ (rad)
 *
 * @note From Eq. 29b':
 *         τ_g2(θ) = m_cw·g·(r_P·cosθ + q_P·sinθ)
 */
double CwGravTorque(const double& cw_moment, const double& cw_hang_moment,
                    const double& grav, const double& theta) {
    return grav
           * (cw_moment * std::cos(theta) + cw_hang_moment * std::sin(theta));
}

/**
 * @brief Computes the arm's gravitational torque (Nm), i.e., measured torque
 *        minus the CW's component.
 *
 * @param torque_meas Measured torque about pitch joint, τ_meas(θ) (Nm)
 * @param cw_grav_torque CW gravitational torque, τ_g2(θ) (Nm), from
 *                       `CwGravTorque()`
 *
 * @note From Eq. 29a:
 *         τ_g1(θ) = τ_meas(θ) - τ_g2(θ)
 */
double ArmGravTorque(const double& torque_meas, const double& cw_grav_torque) {
    return torque_meas - cw_grav_torque;
}

/**
 * @brief Computes the arm's gravitational torque (Nm) at level from one
 *        measurement at any angle.
 *
 * @param arm_torque Arm's gravitational torque at `theta`, τ_g1(θ) (Nm)
 * @param theta Pitch angle, θ (rad)
 * @param arm_hang_torque Arm hang torque, m_arm·h_1·g (Nm)
 *
 * @note Returns none if cosθ is too small.
 *
 * @note From Eq. 34 (Eq. 31 when θ = 0):
 *         τ_g1,0 = (τ_g1(θ) - g·m_arm·h_1·sinθ) / cosθ
 */
std::optional<double> ArmLevelTorque(const double& arm_torque,
                                     const double& theta,
                                     const double& arm_hang_torque) {
    const double c = std::cos(theta);
    if (std::abs(c) < kMinCosForBalance) {
        return std::nullopt;
    }
    return (arm_torque - arm_hang_torque * std::sin(theta)) / c;
}

/**
 * @brief Solves the arm's level and hang torques from a set of two-angle
 *        measurements.
 *
 * @param arm_torque_a Arm's gravitational torque at angle a, τ_g1(θ_a) (Nm)
 * @param theta_a Angle a, θ_a (rad)
 * @param arm_torque_b Arm's gravitational torque at angle b, τ_g1(θ_b) (Nm)
 * @param theta_b Angle b, θ_b (rad)
 *
 * @note Fails if the angles are too close together.
 *
 * @note From Eq. 35c:
 *         τ_g1,0 = (τ_g1(θ_a)·sinθ_b - τ_g1(θ_b)·sinθ_a) / sin(θ_b - θ_a)
 *         m_arm·h_1·g = (τ_g1(θ_b)·cosθ_a - τ_g1(θ_a)·cosθ_b)
 *                       / sin(θ_b - θ_a)
 */
std::optional<Motion::ArmModel> SolveTwoAngle(const double& arm_torque_a,
                                              const double& theta_a,
                                              const double& arm_torque_b,
                                              const double& theta_b) {
    const double s = std::sin(theta_b - theta_a);
    if (std::abs(s) < std::sin(0.5 * Motion::kMinAngleSeparation)) {
        return std::nullopt;
    }

    Motion::ArmModel model;
    model.level_torque = (arm_torque_a * std::sin(theta_b)
                          - arm_torque_b * std::sin(theta_a))
                         / s;
    model.hang_torque = (arm_torque_b * std::cos(theta_a)
                         - arm_torque_a * std::cos(theta_b))
                        / s;
    return model;
}

/**
 * @brief Computes the net gravitational torque (Nm) about pitch joint O.
 *
 * @param arm_level_torque Arm gravitational torque at level, τ_g1,0 (Nm)
 * @param arm_hang_torque Arm hang torque, m_arm·h_1·g (Nm)
 * @param cw_moment CW gravitational moment along arm, m_cw·r_P (kg·m)
 * @param cw_hang_moment CW gravitational moment normal to arm, m_cw·q_P (kg·m)
 * @param grav Gravitational acceleration, g (m/s²)
 * @param theta Pitch angle, θ (rad)
 *
 * @note From Eq. 36':
 *         τ_ext,g(θ) = (τ_g1,0 + m_cw·g·r_P)·cosθ
 *                      + (m_arm·h_1·g + m_cw·g·q_P)·sinθ
 */
double NetGravTorque(const double& arm_level_torque,
                     const double& arm_hang_torque, const double& cw_moment,
                     const double& cw_hang_moment, const double& grav,
                     const double& theta) {
    return (arm_level_torque + grav * cw_moment) * std::cos(theta)
           + (arm_hang_torque + grav * cw_hang_moment) * std::sin(theta);
}

/**
 * @brief Computes the total sine coefficient of the net gravitational torque
 *        (Nm); the arm is stable at equilibrium only while this is negative.
 *
 * @param arm_hang_torque Arm hang torque, m_arm·h_1·g (Nm)
 * @param cw_hang_moment CW gravitational moment normal to arm, m_cw·q_P (kg·m)
 * @param grav Gravitational acceleration, g (m/s²)
 *
 * @note Numerator g·S of Eq. 26c'; stability per Eq. 27' is S < 0:
 *         τ' = (m_arm·h_1·g + m_cw·g·q_P) / cosθ*
 */
double StabilityTorque(const double& arm_hang_torque,
                       const double& cw_hang_moment, const double& grav) {
    return arm_hang_torque + grav * cw_hang_moment;
}

/**
 * @brief Computes the actuator torque feedforward (Nm) that follows commanded
 *        motion while canceling gravity.
 *
 * @param inertia Moment of inertia (kg·m²)
 * @param inertia_rate Rate of change of inertia (kg·m²/s)
 * @param friction_viscous Viscous friction coefficient, b (Nm·s/rad)
 * @param friction_coulomb Coulomb friction torque, τ_c (Nm)
 * @param ref_acc Reference acceleration, θ̈_ref (rad/s²)
 * @param ref_vel Reference velocity, θ̇_ref (rad/s)
 * @param net_grav_torque Measured net gravitational torque, τ_ext,g(θ) (Nm)
 *
 * @note Result is not clamped.
 *
 * @note From Eq. 37:
 *         τ_actuator = I·θ̈_ref + (b + İ)·θ̇_ref + τ_c·sign(θ̇_ref)
 *                      - τ_ext,g(θ)
 */
double FeedforwardTorque(const double& inertia, const double& inertia_rate,
                         const double& friction_viscous,
                         const double& friction_coulomb, const double& ref_acc,
                         const double& ref_vel, const double& net_grav_torque) {
    return inertia * ref_acc + (friction_viscous + inertia_rate) * ref_vel
           + friction_coulomb * Sign(ref_vel) - net_grav_torque;
}

/**
 * @brief Computes the effective CW moment (kg·m) that balances the arm at a
 *        given angle.
 *
 * @param arm_level_torque Arm gravitational torque at level, τ_g1,0 (Nm)
 * @param arm_hang_torque Arm hang torque, m_arm·h_1·g (Nm)
 * @param grav Gravitational acceleration, g (m/s²)
 * @param theta Angle to balance at, θ (rad)
 *
 * @note Returns none if cosθ is too small.
 *
 * @note Code-specific extension of Eq. 38'. Since r_P varies with CW volume,
 *       `CwModel::MassForMoment()` inverts the effective moment instead of
 *       dividing by a fixed r_P:
 *         m_cw,bal·(r_P + q_P·tanθ) = -(τ_g1,0 + m_arm·h_1·g·tanθ) / g
 */
std::optional<double> BalanceCwMoment(const double& arm_level_torque,
                                      const double& arm_hang_torque,
                                      const double& grav, const double& theta) {
    if (std::abs(std::cos(theta)) < kMinCosForBalance) {
        return std::nullopt;
    }
    return -(arm_level_torque + arm_hang_torque * std::tan(theta)) / grav;
}

/**
 * @brief Predicts the net gravitational torque (Nm) at a target angle with the
 *        current CW state.
 *
 * @param grav Gravitational acceleration, g (m/s²)
 * @param theta_tgt Target angle, θ_tgt (rad)
 * @param cw_moment_eff Current effective CW moment at target (kg·m), from
 *                      `CwModel::EffectiveMoment()`
 * @param cw_moment_bal Effective CW moment balancing arm at target (kg·m),
 *                      from `BalanceCwMoment()`
 *
 * @note Code-specific extension of Eq. 39' for r_P that varies with volume,
 *       with the effective moments of `BalanceCwMoment()`:
 *         τ_ext,g(θ_tgt) = g·cosθ_tgt·(M_eff(m_cw) - M_eff(m_cw,bal))
 */
double TargetNetGravTorque(const double& grav, const double& theta_tgt,
                           const double& cw_moment_eff,
                           const double& cw_moment_bal) {
    return grav * std::cos(theta_tgt) * (cw_moment_eff - cw_moment_bal);
}

/**
 * @brief Fits a least-squares line through the friction sweep results (drag vs.
 *        speed).
 *
 * @param speeds Sweep speeds, v (rad/s)
 * @param drags Drag at each speed, (τ_up - τ_down) / 2 (Nm)
 *
 * @note Returns the fit (intercept τ_c, slope b, and RMS of residuals; other
 *       fields empty), or none with fewer than two distinct speeds.
 *
 * @note Isolates the friction component by averaging the up and down sweeps,
 *       which cancels the gravitational torque:
 *         (τ_up - τ_down) / 2 = b·v + τ_c
 */
std::optional<Motion::FrictionModel> FitFrictionLine(
    const std::vector<double>& speeds, const std::vector<double>& drags) {
    const std::size_t n = speeds.size();
    if (n < 2 || drags.size() != n) {
        return std::nullopt;
    }

    // Compute means
    double mean_v = 0.0;
    double mean_f = 0.0;
    for (std::size_t i = 0; i < n; ++i) {
        mean_v += speeds[i];
        mean_f += drags[i];
    }
    mean_v /= static_cast<double>(n);
    mean_f /= static_cast<double>(n);

    // Compute slope and intercept
    double s_vv = 0.0;
    double s_vf = 0.0;
    for (std::size_t i = 0; i < n; ++i) {
        s_vv += (speeds[i] - mean_v) * (speeds[i] - mean_v);
        s_vf += (speeds[i] - mean_v) * (drags[i] - mean_f);
    }
    if (!(s_vv > 0.0)) {
        return std::nullopt;
    }

    // Determine friction coefficients (+ residuals)
    Motion::FrictionModel model;
    model.viscous = s_vf / s_vv;
    model.coulomb = mean_f - model.viscous * mean_v;

    double sum_sq = 0.0;
    for (std::size_t i = 0; i < n; ++i) {
        const double r = drags[i] - (model.coulomb + model.viscous * speeds[i]);
        sum_sq += r * r;
    }
    model.residual_rms = std::sqrt(sum_sq / static_cast<double>(n));

    return model;
}

}  // namespace Libra2Model

//------------------------------------------------------------------------------
// !Rest Sampler
//------------------------------------------------------------------------------

/**
 * @brief Discards the active sampling window.
 */
void RestSampler::Reset() {
    count_ = 0;
    t_elapsed_s_ = 0.0;
    sum_theta_ = 0.0;
    sum_torque_ = 0.0;
    sum_volume_ = 0.0;
}

/**
 * @brief Adds one cycle of measurements to the active sampling window; returns
 *        averages once window is complete.
 *
 * @param dt Time since last cycle (s)
 * @param pos Actual angle, θ (rad)
 * @param vel Actual velocity, θ̇ (rad/s)
 * @param torque Actual actuator torque, τ_actuator (Nm)
 * @param trajectory_active Whether a trajectory is running
 * @param volume Latest CW volume (L), if known
 */
std::optional<Motion::RestSample> RestSampler::Add(
    const double& dt, const double& pos, const double& vel,
    const double& torque, const bool& trajectory_active,
    const std::optional<double>& volume) {
    const bool still = !trajectory_active && std::abs(vel) <= kRestVelTolerance
                       && volume.has_value();
    if (!still) {
        Reset();
        return std::nullopt;
    }

    // Restart window if pose or CW volume drifted from where it began
    if (count_ > 0
        && (std::abs(pos - anchor_theta_) > kRestPosTolerance
            || std::abs(*volume - anchor_volume_) > kRestVolTolerance)) {
        Reset();
    }

    if (count_ == 0) {
        anchor_theta_ = pos;
        anchor_volume_ = *volume;
    } else {
        t_elapsed_s_ += dt;
    }

    sum_theta_ += pos;
    sum_torque_ += torque;
    sum_volume_ += *volume;
    ++count_;

    if (t_elapsed_s_ >= kRestWindowTime && count_ >= 2) {
        Motion::RestSample sample;
        sample.theta = sum_theta_ / count_;
        sample.torque = sum_torque_ / count_;
        sample.volume = sum_volume_ / count_;
        Reset();
        return sample;
    }
    return std::nullopt;
}

//------------------------------------------------------------------------------
// !Class Management
//------------------------------------------------------------------------------

/**
 * @brief Delegating constructor. Uses this file's model constants/defaults.
 *
 * @param cw_capacity CW capacity (L)
 * @param torque_comp_lower_bound ATC "soft" limit (Nm)
 * @param torque_comp_upper_bound ATC "hard" limit (Nm)
 * @param torque_comp_slow_rate Trajectory playback rate between ATC limits
 * @param debug_mode Whether to output verbose debug text
 */
Libra2MotionController::Libra2MotionController(
    const double& cw_capacity, const double& torque_comp_lower_bound,
    const double& torque_comp_upper_bound, const double& torque_comp_slow_rate,
    const bool& debug_mode)
    : Libra2MotionController(kCwPivotDist, kCwPivotSlope, kCwHangOffset,
                             cw_capacity, kDefaultFrictionCoulomb,
                             kDefaultFrictionViscous, kArmInertia,
                             torque_comp_lower_bound, torque_comp_upper_bound,
                             torque_comp_slow_rate, debug_mode) {}

/**
 * @brief Standard constructor.
 *
 * @param cw_pivot_dist CW pivot distance at 0 liters, A (m); 0 = not measured
 * @param cw_pivot_slope Change in CW pivot distance per liter, B (m/L)
 * @param cw_hang_offset CW hang point offset (normal to arm) relative to pitch
 *                       joint O, q_P (m)
 * @param cw_capacity CW capacity (L)
 * @param friction_coulomb Coulomb friction torque, τ_c (Nm)
 * @param friction_viscous Viscous friction coefficient, b (Nm·s/rad)
 * @param arm_inertia Arm's moment of inertia about pitch joint O (kg·m²)
 * @param torque_comp_lower_bound ATC "soft" limit (Nm)
 * @param torque_comp_upper_bound ATC "hard" limit (Nm)
 * @param torque_comp_slow_rate Trajectory playback rate between ATC limits
 * @param debug_mode Whether to output verbose debug text
 *
 * @note The CW's share of the inertia comes from the CW model and flow rate.
 */
Libra2MotionController::Libra2MotionController(
    const double& cw_pivot_dist, const double& cw_pivot_slope,
    const double& cw_hang_offset, const double& cw_capacity,
    const double& friction_coulomb, const double& friction_viscous,
    const double& arm_inertia, const double& torque_comp_lower_bound,
    const double& torque_comp_upper_bound, const double& torque_comp_slow_rate,
    const bool& debug_mode)
    : AbstractMotionController({Joint::Name::kPitch}, debug_mode),
      cw_model_(cw_pivot_dist, cw_pivot_slope, cw_hang_offset, cw_capacity),
      friction_coulomb_(friction_coulomb),
      friction_viscous_(friction_viscous),
      arm_inertia_(arm_inertia),
      torque_comp_lower_bound_(torque_comp_lower_bound),
      torque_comp_upper_bound_(torque_comp_upper_bound),
      torque_comp_slow_rate_(torque_comp_slow_rate) {
    if (!cw_model_.IsValid()) {
        logger_->Warn("CW model invalid: " + cw_model_.GetError()
                      + " (calibration and feedforward are unavailable)");
    }
}

//------------------------------------------------------------------------------
// !Settings & Arm Model
//------------------------------------------------------------------------------

/**
 * @brief Sets the motion planning mode.
 *
 * @param mode Mode to use from next target onward
 *
 * @note A mode change may be refused (e.g., CW-primary needs valid/non-zero arm
 *       and CW models, and no move in progress). Switching to Tandem is always
 *       accepted.
 */
bool Libra2MotionController::SetMotionMode(const Motion::MotionMode& mode) {
    std::lock_guard<std::mutex> lock(mutex_);

    // Validate mode change
    if (mode == Motion::MotionMode::kCwPrimary
        && !(arm_calibrated_ && cw_model_.IsValid())) {
        logger_->Warn("CW-primary mode needs a calibrated arm model and a "
                      "valid CW model; staying in the current mode");
        return false;
    }
    if (mode == Motion::MotionMode::kCwPrimary && IsMovingLocked()) {
        logger_->Warn("Can't switch to CW-primary while the arm is moving; "
                      "staying in the current mode");
        return false;
    }

    // Ensure safe transition
    if (cw_phase_ == CwPhase::kCwMoving) {
        logger_->Warn("Left CW-primary while the CW was moving the arm; "
                      "handing off to the actuator");

        cw_phase_ = CwPhase::kIdle;
        cw_assist_done_ = true;
        if (target_pos_) {
            pending_move_ = (*target_pos_)[kPitchIdx];
        }

    } else if (!IsMovingLocked()) {
        // Keep active CW state until the next target
        cw_phase_ = CwPhase::kIdle;
        cw_assist_done_ = false;
    }

    mode_ = mode;
    logger_->Info(
        std::string("Motion mode: ")
        + (mode == Motion::MotionMode::kTandem ? "Tandem" : "CW-primary"));
    return true;
}

/**
 * @brief Gets the active motion mode.
 */
Motion::MotionMode Libra2MotionController::GetMotionMode() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return mode_;
}

/**
 * @brief Checks whether the arm is currently moving or about to move.
 *
 * @note Trajectory state is as of the last `Update()`.
 */
bool Libra2MotionController::IsMoving() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return IsMovingLocked();
}

/**
 * @brief Stores latest CW state (from WaterController).
 *
 * @param volume CW volume, V (L)
 * @param flow Net CW flow rate, V̇ (L/s); positive = filling
 */
void Libra2MotionController::SetCwState(const double& volume,
                                        const double& flow) {
    std::lock_guard<std::mutex> lock(mutex_);

    if (!std::isfinite(volume) || volume < 0.0 || !std::isfinite(flow)) {
        logger_->Warn("Ignoring invalid CW state: " + Format(volume) + " L, "
                      + Format(flow) + " L/s");
        return;
    }

    cw_vol_ = volume;
    cw_flow_ = flow;
}

/**
 * @brief Enables or disables automatic torque compensation (ATC).
 *
 * @param enabled Whether to enable ATC
 */
void Libra2MotionController::EnableTorqueComp(const bool& enabled) {
    std::lock_guard<std::mutex> lock(mutex_);

    atc_en_ = enabled;
    if (!atc_en_) {
        movement_en_ = true;  // "manual" mode: movement is always allowed
    }

    logger_->Info("Automatic torque compensation (ATC) "
                  + std::string(enabled ? "enabled" : "disabled"));
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
 * @brief Sets the ATC's soft and hard torque limits.
 *
 * @param lower_bound Soft limit (Nm); movement slows at or above it
 * @param upper_bound Hard limit (Nm); movement pauses above it
 */
bool Libra2MotionController::SetTorqueCompBounds(const double& lower_bound,
                                                 const double& upper_bound) {
    std::lock_guard<std::mutex> lock(mutex_);

    if (!(lower_bound >= 0.0) || !(lower_bound < upper_bound)) {
        logger_->Warn("Ignoring invalid torque compensation bounds: lower="
                      + Format(lower_bound)
                      + " Nm, upper=" + Format(upper_bound) + " Nm");
        return false;
    }

    torque_comp_lower_bound_ = lower_bound;
    torque_comp_upper_bound_ = upper_bound;
    logger_->Info("Torque compensation bounds updated: lower="
                  + Format(lower_bound) + " Nm, upper=" + Format(upper_bound)
                  + " Nm");

    return true;
}

/**
 * @brief Sets the CW-primary mode's hand-off band.
 *
 * @param band Range around target angle (rad); inside it, CW stops and
 *             actuator takes over
 */
bool Libra2MotionController::SetHandoffBand(const double& band) {
    std::lock_guard<std::mutex> lock(mutex_);

    if (!(band > 0.0) || !std::isfinite(band)) {
        logger_->Warn("Ignoring invalid hand-off band: " + Format(band)
                      + " rad");
        return false;
    }

    handoff_band_ = band;
    logger_->Info("Hand-off band updated: " + Format(band) + " rad");

    return true;
}

/**
 * @brief Directly sets the arm model parameters.
 *
 * @param arm_level_torque Arm gravitational torque at level, τ_g1,0 (Nm)
 * @param arm_hang_torque Arm hang torque, m_arm·h_1·g (Nm)
 *
 * @note No raw samples are kept, so a saved record of it can't be recomputed
 *       if the CW model changes. Use `LoadCalibrationRecord()` for saved
 *       calibrations.
 */
void Libra2MotionController::SetArmModel(const double& arm_level_torque,
                                         const double& arm_hang_torque) {
    std::lock_guard<std::mutex> lock(mutex_);

    arm_level_torque_ = arm_level_torque;
    arm_hang_torque_ = arm_hang_torque;
    arm_calibrated_ = true;
    arm_source_ = "set";
    arm_cw_params_ = cw_model_.GetParams();
    arm_samples_.clear();
    logger_->Info("Arm model set: arm_level_torque=" + Format(arm_level_torque)
                  + " Nm, arm_hang_torque=" + Format(arm_hang_torque) + " Nm");

    CheckStability(arm_hang_torque_);
}

/**
 * @brief Resets the arm model.
 */
void Libra2MotionController::ClearArmModel() {
    std::lock_guard<std::mutex> lock(mutex_);

    arm_level_torque_ = 0.0;
    arm_hang_torque_ = 0.0;
    arm_calibrated_ = false;
    arm_source_.clear();
    arm_samples_.clear();
    mode_ = Motion::MotionMode::kTandem;  // safe default
    cw_phase_ = CwPhase::kIdle;

    logger_->Info("Arm model cleared; feedforward and predictive CW disabled");
}

/**
 * @brief Checks whether an arm model is available (calibrated, loaded, or set
 *        via `SetArmModel()`).
 */
bool Libra2MotionController::IsArmCalibrated() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return arm_calibrated_;
}

/**
 * @brief Gets the arm's gravitational torque at level, τ_g1,0 (Nm).
 */
double Libra2MotionController::GetArmLevelTorque() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return arm_level_torque_;
}

/**
 * @brief Gets the arm's hang torque, m_arm·h_1·g (Nm).
 */
double Libra2MotionController::GetArmHangTorque() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return arm_hang_torque_;
}

/**
 * @brief Computes the net gravitational torque (Nm) about pitch joint O for a
 *        given angle and CW mass.
 *
 * @param theta Pitch angle, θ (rad)
 * @param cw_mass CW mass, m_cw (kg)
 *
 * @note Mutex lock not required since `Update()` calls it while holding the
 *       lock. External callers should only call it while the motion controller
 *       isn't running.
 *
 * @see Libra2Model::NetGravTorque
 */
double Libra2MotionController::ComputeNetGravTorque(
    const double& theta, const double& cw_mass) const {
    return Libra2Model::NetGravTorque(arm_level_torque_, arm_hang_torque_,
                                      cw_model_.Moment(cw_mass),
                                      cw_model_.HangMoment(cw_mass), kGrav,
                                      theta);
}

//------------------------------------------------------------------------------
// !Calibration Commands
//------------------------------------------------------------------------------

/**
 * @brief Starts the level calibration (arm must already be level and at rest).
 *
 * @note Uses Eq. 34 at the measured angle (near 0), which reduces to Eq. 31:
 *         τ_g1,0 = τ_meas(0) - m_cw·g·r_P
 */
bool Libra2MotionController::StartLevelCalibration() {
    std::lock_guard<std::mutex> lock(mutex_);

    if (Motion::IsCalibrationRunning(status_)) {
        logger_->Warn("Calibration already running; ignoring level request");
        return false;
    }
    if (!cw_model_.IsValid()) {
        logger_->Error("Can't calibrate: invalid CW model ("
                       + cw_model_.GetError() + ")");
        return false;
    }

    // Set up and start calibration
    two_angle_ = false;

    logger_->Info(
        "Starting level calibration (arm assumed to be level and at rest)");
    BeginCalibrationStage(Motion::CalibrationStatus::kArmSamplingLevel);

    return true;
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
 * @see Libra2Model::SolveTwoAngle
 */
bool Libra2MotionController::StartTwoAngleCalibration(const double& angle_a,
                                                      const double& angle_b) {
    std::lock_guard<std::mutex> lock(mutex_);

    if (Motion::IsCalibrationRunning(status_)) {
        logger_->Warn(
            "Calibration already running; ignoring two-angle request");
        return false;
    }
    if (!cw_model_.IsValid()) {
        logger_->Error("Can't calibrate: invalid CW model ("
                       + cw_model_.GetError() + ")");
        return false;
    }

    // Validate params
    if (!std::isfinite(angle_a) || !std::isfinite(angle_b)
        || angle_a == angle_b) {
        logger_->Warn("Two-angle calibration rejected: need two distinct "
                      "finite angles (got a="
                      + Format(angle_a) + ", b=" + Format(angle_b) + ")");
        return false;
    } else if (std::abs(angle_a) > Motion::kMaxCalibrationAngle
               || std::abs(angle_b) > Motion::kMaxCalibrationAngle) {
        logger_->Warn("Two-angle calibration rejected: need |angle| <= "
                      + Format(Motion::kMaxCalibrationAngle) + " rad (got a="
                      + Format(angle_a) + ", b=" + Format(angle_b) + ")");
        return false;
    } else if (std::abs(angle_b - angle_a) < Motion::kMinAngleSeparation) {
        logger_->Warn("Two-angle calibration rejected: need |b - a| >= "
                      + Format(Motion::kMinAngleSeparation) + " rad (got a="
                      + Format(angle_a) + ", b=" + Format(angle_b) + ")");
        return false;
    }

    // Set up and start calibration
    two_angle_ = true;
    angle_a_ = angle_a;
    angle_b_ = angle_b;

    logger_->Info("Starting two-angle calibration: a=" + Format(angle_a)
                  + " rad, b=" + Format(angle_b) + " rad");
    BeginCalibrationStage(Motion::CalibrationStatus::kArmMovingToA);

    return true;
}

/**
 * @brief Starts the friction calibration.
 *
 * @param angle_a One end of the window (rad)
 * @param angle_b Other end of the window (rad)
 * @param max_speed Max. actuator speed (rad/s); sweep speeds are fractions
 *                  of it
 *
 * @note Doesn't need an arm or CW model; gravity cancels between the up and
 *       down sweeps.
 *
 * @note Isolates the friction component by averaging the up and down sweeps,
 *       which cancels the gravitational torque:
 *         (τ_up - τ_down) / 2 = b·v + τ_c
 *
 * @see kFrictionSpeedFractions Libra2Model::FitFrictionLine
 */
bool Libra2MotionController::StartFrictionCalibration(const double& angle_a,
                                                      const double& angle_b,
                                                      const double& max_speed) {
    std::lock_guard<std::mutex> lock(mutex_);

    if (Motion::IsCalibrationRunning(status_)) {
        logger_->Warn("Calibration already running; ignoring friction request");
        return false;
    }

    // Validate params
    if (!cw_vol_) {
        logger_->Warn("Friction calibration rejected: CW volume unknown");
        return false;
    }
    if (!(max_speed > 0.0) || !std::isfinite(max_speed)) {
        logger_->Error("Friction calibration rejected: invalid max speed "
                       + Format(max_speed) + " rad/s");
        return false;
    }

    const double lo = std::min(angle_a, angle_b);
    const double hi = std::max(angle_a, angle_b);
    if (!std::isfinite(lo) || !std::isfinite(hi) || lo == hi) {
        logger_->Warn("Friction calibration rejected: need |angle| <=  "
                      "calibration rejected: need two distinct "
                      "finite angles (got a="
                      + Format(angle_a) + ", b=" + Format(angle_b) + ")");
        return false;
    } else if (std::abs(lo) > Motion::kMaxCalibrationAngle
               || std::abs(hi) > Motion::kMaxCalibrationAngle) {
        logger_->Warn("Friction calibration rejected: need |angle| <= "
                      + Format(Motion::kMaxCalibrationAngle) + " rad (got a="
                      + Format(angle_a) + ", b=" + Format(angle_b) + ")");
        return false;
    } else if (hi - lo < Motion::kFrictionMinWindow) {
        logger_->Warn("Friction calibration rejected: need |b - a| >= "
                      + Format(Motion::kFrictionMinWindow) + " rad (got a="
                      + Format(angle_a) + ", b=" + Format(angle_b) + ")");
        return false;
    }

    // Ensure slowest sweep fits in one move stage
    const double slowest = max_speed * kFrictionSpeedFractions.front();
    const double t_slowest_s = (hi - lo + 2.0 * kFrictionRampDist) / slowest;
    if (t_slowest_s > kMoveTimeout) {
        logger_->Warn("Friction calibration rejected: slowest sweep takes "
                      + Format(t_slowest_s) + " s (limit "
                      + Format(kMoveTimeout) + " s); use a smaller window");
        return false;
    }

    // Set up and start calibration
    friction_run_ = FrictionRun();
    friction_run_.angle_lo = lo;
    friction_run_.angle_hi = hi;
    friction_run_.max_speed = max_speed;
    friction_run_.volume = *cw_vol_;
    two_angle_ = false;

    logger_->Info("Starting friction calibration: window " + Format(lo) + " to "
                  + Format(hi) + " rad, "
                  + std::to_string(kFrictionSpeedFractions.size())
                  + " speeds up to " + Format(max_speed)
                  + " rad/s (CW stays idle; aborts above "
                  + Format(torque_comp_upper_bound_) + " Nm)");
    BeginCalibrationStage(Motion::CalibrationStatus::kFrictionMovingToStart);

    return true;
}

/**
 * @brief Cancels a running calibration.
 */
void Libra2MotionController::CancelCalibration() {
    std::lock_guard<std::mutex> lock(mutex_);
    CancelCalibrationLocked();
}

/**
 * @brief Gets calibration progress.
 */
Motion::CalibrationStatus Libra2MotionController::GetCalibrationStatus() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return status_;
}

/**
 * @brief Gets a description of the last calibration, if any.
 */
std::string Libra2MotionController::GetCalibrationMessage() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return calibration_msg_;
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

    if (arm_calibrated_) {
        Motion::ArmModel arm;
        arm.source = arm_source_;
        arm.level_torque = arm_level_torque_;
        arm.hang_torque = arm_hang_torque_;
        arm.cw_model = arm_cw_params_;
        arm.samples = arm_samples_;
        record.arm_model = arm;
    }
    record.friction = friction_model_;

    return record;
}

/**
 * @brief Applies the given arm and/or friction models.
 *
 * @param record A full or partial calibration
 *
 * @note If the CW model changed since the arm model was made, the arm model is
 *       recomputed from its raw samples with the current CW model.
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
        const Motion::ArmModel& saved = *record.arm_model;
        Motion::ArmModel arm = saved;

        if (!IsSameCwModel(saved.cw_model, cw_model_.GetParams())) {
            if (const auto recomputed = RecomputeArmModel(saved)) {
                arm = *recomputed;
                logger_->Info(
                    "CW model has changed since this calibration was saved; "
                    "arm model was recomputed from its raw samples (saved: "
                    "arm_level_torque="
                    + Format(saved.level_torque) + " Nm, arm_hang_torque="
                    + Format(saved.hang_torque) + " Nm)");
            } else {
                logger_->Warn(
                    "CW model has changed since this calibration was saved, "
                    "but its arm model can't be recomputed (no raw samples, or "
                    "current CW model invalid); using saved values as-is");
            }
        }

        logger_->Info("Arm model loaded: arm_level_torque "
                      + Format(arm_level_torque_) + " -> "
                      + Format(arm.level_torque) + " Nm, arm_hang_torque "
                      + Format(arm_hang_torque_) + " -> "
                      + Format(arm.hang_torque) + " Nm (" + arm.source + ")");

        // Apply and validate
        arm_level_torque_ = arm.level_torque;
        arm_hang_torque_ = arm.hang_torque;
        arm_calibrated_ = true;
        arm_source_ = arm.source;
        arm_cw_params_ = arm.cw_model;
        arm_samples_ = arm.samples;

        CheckStability(arm_hang_torque_);
    }

    // Apply the friction model
    if (record.friction) {
        logger_->Info("Friction loaded: tau_c " + Format(friction_coulomb_)
                      + " -> " + Format(record.friction->coulomb) + " Nm, b "
                      + Format(friction_viscous_) + " -> "
                      + Format(record.friction->viscous) + " Nm*s/rad");

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
 * @brief Applies three-step hysteresis to throttle the trajectory based on
 *        sensed torque (like LIBRA-I's motion control).
 *
 * @param torque_magnitude Absolute value of torque feedback (Nm)
 * @param direction CW adjustment direction, opposite to torque
 * @param braking Whether actuator torque opposes travel (CW leads)
 *
 * @note Slowing the trajectory only helps while the actuator lifts the arm.
 *       While it brakes, slowing would raise the torque, so the soft limit
 *       keeps full speed and only the CW reacts. The hard limit always stops.
 *
 * @see HebiThread::CheckTorqueControl
 */
Libra2MotionController::AtcResult Libra2MotionController::UpdateTorqueComp(
    const double& torque_magnitude, const Motion::CwCommand& direction,
    const bool& braking) {
    AtcResult result;

    if (!atc_en_) {
        movement_en_ = true;
        return result;  // default: full speed, no CW command
    }

    bool in_slow_band = false;

    if (torque_magnitude > torque_comp_upper_bound_) {
        // Hard limit: stop movement, then report direction (same as LIBRA-I)
        if (movement_en_) {
            movement_en_ = false;

            logger_->Info("Movement disabled due to high torque ("
                          + Format(torque_magnitude) + " Nm > "
                          + Format(torque_comp_upper_bound_) + " Nm)");
        } else {
            result.cw_command = direction;
        }

    } else if (torque_magnitude >= torque_comp_lower_bound_) {
        // Soft limit: keep reporting direction, slow trajectory (unless braking)
        result.cw_command = direction;
        in_slow_band = !braking;

    } else {
        // Acceptable torque: stop balancing and re-enable movement
        result.cw_command = Motion::CwCommand::kIdle;
        if (!movement_en_) {
            movement_en_ = true;

            logger_->Info("Movement re-enabled (" + Format(torque_magnitude)
                          + " Nm < " + Format(torque_comp_lower_bound_)
                          + " Nm)");
        }
    }

    if (!movement_en_) {
        result.playback_rate = 0.0;
    } else {
        result.playback_rate = in_slow_band ? torque_comp_slow_rate_ : 1.0;
    }

    return result;
}

/**
 * @brief Computes the CW mass (kg) that balances the arm at a target angle,
 *        m_cw,bal(θ_tgt), clamped to the CW's capacity.
 *
 * @param theta_tgt Target angle, θ_tgt (rad)
 *
 * @note Returns none if balance is undefined at this angle.
 *
 * @see Libra2Model::BalanceCwMoment CwModel::MassForMoment
 */
std::optional<double> Libra2MotionController::TargetCwMass(
    const double& theta_tgt) const {
    const auto moment_bal = Libra2Model::BalanceCwMoment(arm_level_torque_,
                                                         arm_hang_torque_,
                                                         kGrav, theta_tgt);
    if (!moment_bal) {
        return std::nullopt;
    }
    return cw_model_.MassForMoment(*moment_bal, theta_tgt);
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
    if (std::abs(diff) <= kCwVolumeTolerance * kWaterDensity) {
        return Motion::CwCommand::kIdle;
    }

    // TODO: if valve latency makes the CW overshoot, stop early by latency·V̇
    return (diff > 0.0) ? Motion::CwCommand::kFill : Motion::CwCommand::kDrain;
}

/**
 * @brief Runs one CW-primary cycle: pitch joint allowed to spin freely so the
 *        CW moves it.
 *
 * @param theta Actual angle, θ (rad)
 * @param theta_tgt Target angle, θ_tgt (rad)
 * @param[in,out] command Command with CW adjustment and free-spinning pitch
 *
 * @note Returns true while CW controls the free-spinning arm, and false at
 *       hand-off (where actuator takes over and a proper move is requested).
 */
bool Libra2MotionController::RunCwPrimary(const double& theta,
                                          const double& theta_tgt,
                                          Motion::Libra2Command& command) {
    const auto cw_mass_tgt = TargetCwMass(theta_tgt);
    if (!cw_mass_tgt) {
        logger_->Warn("CW-primary: target mass undefined at this angle; "
                      "handing off to actuator");

        cw_phase_ = CwPhase::kIdle;
        cw_assist_done_ = true;
        pending_move_ = theta_tgt;
        return false;
    }
    const double mass_tgt = *cw_mass_tgt;
    const Motion::CwCommand cw = PredictiveCwCommand(mass_tgt, *cw_mass_);

    const bool volume_reached = (cw == Motion::CwCommand::kIdle);
    const bool angle_reached = std::abs(theta - theta_tgt) <= handoff_band_;

    if (volume_reached || angle_reached) {
        // Log hand-off and set up for actuator takeover
        logger_->Info("CW-primary hand-off ("
                      + std::string(volume_reached ? "volume" : "")
                      + std::string(volume_reached && angle_reached ? "+" : "")
                      + std::string(angle_reached ? "angle" : "")
                      + " reached):\n  actual angle=" + Format(theta)
                      + " rad, target angle=" + Format(theta_tgt)
                      + " rad, angle error=" + Format(theta - theta_tgt)
                      + " rad\n  actual volume="
                      + Format(*cw_mass_ / kWaterDensity) + " L, target volume="
                      + Format(mass_tgt / kWaterDensity) + " L, volume error="
                      + Format((*cw_mass_ - mass_tgt) / kWaterDensity) + " L");

        cw_phase_ = CwPhase::kIdle;
        cw_assist_done_ = true;
        pending_move_ = theta_tgt;
        return false;
    }

    // Arm spins freely; CW does the moving
    command.mode[kPitchIdx] = Motion::CommandMode::kTorqueOnly;
    command.cw_command = cw;
    command.playback_rate = 1.0;
    movement_en_ = true;

    return true;
}

/**
 * @brief Runs one control cycle: decides what the actuator and CW do.
 *
 * @param dt Time since last cycle (s)
 * @param actual Actual joint values
 * @param commanded Commanded joint values from previous cycle
 * @param trajectory_active Whether a trajectory is running
 *
 * @note If anything CW-primary needs is lost while the arm is free (model, CW
 *       volume, target, or ATC), the actuator takes over immediately.
 */
Motion::Libra2Command Libra2MotionController::Update(
    const double& dt, const Motion::ActualState& actual,
    const Motion::CommandedState& commanded, const bool& trajectory_active) {
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
    const double theta = actual.pos[kPitchIdx];
    const double actual_vel = actual.vel[kPitchIdx];
    const double actual_torque = actual.torque[kPitchIdx];
    trajectory_active_ = trajectory_active;

    // Eq. 28: CW mass from latest volume
    cw_mass_ = cw_vol_ ? std::optional<double>(Libra2Model::CwMass(*cw_vol_))
                       : std::nullopt;

    // Eq. 15: inertia and its rate, with the CW's share (ṁ_cw = ρ_water·V̇)
    if (cw_mass_ && cw_model_.IsValid()) {
        inertia_ = arm_inertia_ + cw_model_.Inertia(*cw_mass_);
        inertia_rate_ = cw_model_.InertiaSlope(*cw_mass_) * kWaterDensity
                        * cw_flow_;
    } else {
        inertia_ = arm_inertia_;
        inertia_rate_ = 0.0;
    }

    const bool calibrating = Motion::IsCalibrationRunning(status_);
    const bool model_ready = arm_calibrated_ && cw_model_.IsValid()
                             && cw_mass_.has_value();

    std::optional<double> target;
    if (target_pos_) {
        target = (*target_pos_)[kPitchIdx];
    }

    // Determine movement and CW commands
    Motion::Libra2Command command(Motion::kNumJoints);
    bool arm_free = false;

    if (mode_ == Motion::MotionMode::kCwPrimary
        && cw_phase_ == CwPhase::kCwMoving) {
        // (A) CW-primary: arm spins freely while CW moves it
        if (!model_ready || !target || calibrating || !atc_en_) {
            logger_->Warn("CW-primary: model, CW volume, target, or ATC "
                          "unavailable; handing off to the actuator");

            cw_phase_ = CwPhase::kIdle;
            cw_assist_done_ = true;
            if (target) {
                pending_move_ = *target;
            }
        } else {
            arm_free = RunCwPrimary(theta, *target, command);
        }
    }

    if (!arm_free && IsFrictionStage(status_)) {
        // Friction sweep: no throttle or CW command necessary
        command.playback_rate = 1.0;
        command.cw_command = Motion::CwCommand::kIdle;
        RunFrictionCalibration(dt, trajectory_active, theta, actual_vel,
                               actual_torque);

    } else if (!arm_free) {
        // (B) Tandem/Holding: ATC throttle applies,
        const Motion::CwCommand direction = (actual_torque > 0.0)
                                                ? Motion::CwCommand::kFill
                                                : Motion::CwCommand::kDrain;
        const bool braking = target
                             && (actual_torque * (*target - theta) < 0.0);
        const AtcResult atc = UpdateTorqueComp(std::abs(actual_torque),
                                               direction, braking);

        command.playback_rate = atc.playback_rate;
        command.cw_command = atc.cw_command;  // reactive (LIBRA-I style)

        if (calibrating) {
            // Feedforward off and CW reactive while calibrating
            RunArmCalibration(dt, trajectory_active, theta, actual_vel,
                              actual_torque);
        } else {
            // Predictive CW replaces reactive (except after hand-off)
            if (model_ready && target && !cw_assist_done_) {
                const auto cw_mass_tgt = TargetCwMass(*target);
                if (cw_mass_tgt) {
                    command.cw_command = PredictiveCwCommand(*cw_mass_tgt,
                                                             *cw_mass_);
                }
            }

            if (model_ready) {
                const double torque_ext_g = ComputeNetGravTorque(theta,
                                                                 *cw_mass_);
                const double inertia = inertia_ff_en_ ? inertia_ : 0.0;
                const double inertia_rate = inertia_ff_en_ ? inertia_rate_
                                                           : 0.0;
                command.ff_torque[kPitchIdx] =
                    Libra2Model::FeedforwardTorque(inertia, inertia_rate,
                                                   friction_viscous_,
                                                   friction_coulomb_,
                                                   commanded.acc[kPitchIdx],
                                                   commanded.vel[kPitchIdx],
                                                   torque_ext_g);
            }
        }
    }

    // Deal with requests
    if (pending_stop_) {
        command.stop_trajectory = true;
        pending_stop_ = false;
    }
    if (pending_move_) {
        std::vector<double> move = actual.pos;
        move[kPitchIdx] = *pending_move_;
        command.move_to = move;
        pending_move_.reset();
    }
    if (pending_sweep_) {
        command.sweep = pending_sweep_;
        pending_sweep_.reset();
    }

    command.diagnostics = ComputeDiagnostics(theta, target);

    return command;
}

/**
 * @brief Computes diagnostic data about the active model.
 *
 * @param theta Actual angle, θ (rad)
 * @param theta_tgt Target angle, θ_tgt (rad), if any
 *
 * @note Entries that can't be computed (i.e., no model, CW volume, or target)
 *       are left empty.
 *
 * @see Libra2Model::CwMass Libra2Model::NetGravTorque
 *      Libra2Model::BalanceCwMoment Libra2Model::TargetNetGravTorque
 */
Motion::Libra2Diagnostics Libra2MotionController::ComputeDiagnostics(
    const double& theta, const std::optional<double>& theta_tgt) const {
    Motion::Libra2Diagnostics diagnostics;
    diagnostics.cw_mass = cw_mass_;
    if (!arm_calibrated_ || !cw_model_.IsValid()) {
        return diagnostics;
    }

    if (cw_mass_) {
        diagnostics.torque_ext_g = ComputeNetGravTorque(theta, *cw_mass_);
    }

    // Eqs. 38'-40
    if (theta_tgt) {
        const auto moment_bal = Libra2Model::BalanceCwMoment(arm_level_torque_,
                                                             arm_hang_torque_,
                                                             kGrav, *theta_tgt);
        const auto mass_tgt = moment_bal ? cw_model_.MassForMoment(*moment_bal,
                                                                   *theta_tgt)
                                         : std::nullopt;
        if (mass_tgt) {
            diagnostics.cw_mass_tgt = *mass_tgt;
            if (cw_mass_) {
                diagnostics.cw_mass_change = *mass_tgt - *cw_mass_;
                diagnostics.torque_ext_g_tgt = Libra2Model::TargetNetGravTorque(
                    kGrav, *theta_tgt,
                    cw_model_.EffectiveMoment(*cw_mass_, *theta_tgt),
                    *moment_bal);
            }
        }
    }

    return diagnostics;
}

//------------------------------------------------------------------------------
// !Hooks
//------------------------------------------------------------------------------

/**
 * @brief Handles a new target, cancelling any calibration before requesting the
 *        move.
 *
 * @note HebiThread starts every LIBRA-II trajectory from `move_to`, so the
 *       choice of how to reach a target is made here only.
 */
void Libra2MotionController::OnTargetChanged() {
    CancelCalibrationLocked();

    // Mode change applies from the next target
    cw_assist_done_ = false;
    if (mode_ == Motion::MotionMode::kCwPrimary) {
        cw_phase_ = CwPhase::kCwMoving;
        pending_move_.reset();  // eventual hand-off requests the move
    } else {
        cw_phase_ = CwPhase::kIdle;
        pending_move_ = (*target_pos_)[kPitchIdx];
    }
}

/**
 * @brief Handles a cleared target: cancels any calibration and CW assistance.
 */
void Libra2MotionController::OnStop() {
    CancelCalibrationLocked();

    cw_phase_ = CwPhase::kIdle;
    cw_assist_done_ = false;
    pending_move_.reset();
}

/**
 * @brief Handles a reset: clears runtime state. Keeps settings, arm model, and
 *        friction.
 */
void Libra2MotionController::OnReset() {
    CancelCalibrationLocked();

    cw_phase_ = CwPhase::kIdle;
    cw_assist_done_ = false;
    pending_move_.reset();
    pending_stop_ = false;
    trajectory_active_ = false;
    movement_en_ = true;
    state_warned_ = false;
}

//------------------------------------------------------------------------------
// !Model Calibration Helpers
//------------------------------------------------------------------------------

/**
 * @brief Cancels a running calibration (`mutex_` must already be held).
 */
void Libra2MotionController::CancelCalibrationLocked() {
    if (Motion::IsCalibrationRunning(status_)) {
        status_ = Motion::CalibrationStatus::kIdle;

        pending_move_.reset();
        pending_sweep_.reset();
        if (IsFrictionStage(status_)) {
            pending_stop_ = true;
        }

        sampler_.Reset();

        calibration_msg_ = "Calibration cancelled";
        logger_->Info(calibration_msg_);
    }
}

/**
 * @brief Checks whether the arm is moving, like `IsMoving()` (`mutex_` must
 *        already be held).
 */
bool Libra2MotionController::IsMovingLocked() const {
    return trajectory_active_ || (cw_phase_ == CwPhase::kCwMoving)
           || pending_move_.has_value() || pending_sweep_.has_value()
           || Motion::IsCalibrationRunning(status_);
}

/**
 * @brief Enters a calibration stage and, if applicable, requests a move.
 *
 * @param status Calibration stage to transition to
 */
void Libra2MotionController::BeginCalibrationStage(
    const Motion::CalibrationStatus& status) {
    status_ = status;
    t_stage_s_ = 0.0;
    sampler_.Reset();

    FrictionRun& fr = friction_run_;
    switch (status) {
        case Motion::CalibrationStatus::kArmMovingToA:
            pending_move_ = angle_a_;
            break;
        case Motion::CalibrationStatus::kArmMovingToB:
            pending_move_ = angle_b_;
            break;
        case Motion::CalibrationStatus::kFrictionMovingToStart:
            pending_move_ = fr.angle_lo;
            break;
        case Motion::CalibrationStatus::kFrictionSweepUp:
        case Motion::CalibrationStatus::kFrictionSweepDown:
            {
                const bool going_up =
                    (status == Motion::CalibrationStatus::kFrictionSweepUp);

                Motion::SweepRequest sweep;
                sweep.from = going_up ? fr.angle_lo : fr.angle_hi;
                sweep.to = going_up ? fr.angle_hi : fr.angle_lo;
                sweep.speed = fr.max_speed
                              * kFrictionSpeedFractions.at(fr.speed_idx);
                sweep.ramp_dist = kFrictionRampDist;

                pending_sweep_ = sweep;
                fr.trajectory_seen = false;

                if (going_up) {
                    // Each run starts with an upwards sweep and fresh history
                    const auto n_bins = static_cast<std::size_t>(std::ceil(
                        (fr.angle_hi - fr.angle_lo - 2.0 * kFrictionSampleMargin)
                        / kFrictionBinWidth));
                    fr.up_sum.assign(n_bins, 0.0);
                    fr.down_sum.assign(n_bins, 0.0);
                    fr.up_count.assign(n_bins, 0);
                    fr.down_count.assign(n_bins, 0);
                }
                break;
            }
        default:
            break;
    }
}

/**
 * @brief Ends a calibration and logs the outcome.
 *
 * @param success Whether the arm model or friction was updated
 * @param message Descriptive outcome (success: the result; failure: the reason)
 */
void Libra2MotionController::FinishCalibration(const bool& success,
                                               const std::string& message) {
    status_ = success ? Motion::CalibrationStatus::kSucceeded
                      : Motion::CalibrationStatus::kFailed;

    pending_move_.reset();
    pending_sweep_.reset();
    sampler_.Reset();

    if (success) {
        calibration_msg_ = message;
        logger_->Info(calibration_msg_);
    } else {
        calibration_msg_ = "Calibration failed: " + message;
        logger_->Error(calibration_msg_);
    }
}

/**
 * @brief Converts an averaged rest sample into the arm's own gravitational
 *        torque (Nm).
 *
 * @param sample Averaged measurements
 *
 * @note At rest, measured torque is the opposite of actual actuator torque, per
 *       Eq. 19b. Using Eq. 29a removes the CW torque, and with Eq. 28 for CW
 *       mass:
 *         τ_meas(θ) = -τ_actuator τ_g1(θ) = τ_meas(θ) - τ_g2(θ)
 */
double Libra2MotionController::SampleToArmTorque(
    const Motion::RestSample& sample) const {
    const double cw_mass = Libra2Model::CwMass(sample.volume);
    const double cw_torque = Libra2Model::CwGravTorque(cw_model_.Moment(cw_mass),
                                                       cw_model_.HangMoment(
                                                           cw_mass),
                                                       kGrav, sample.theta);

    return Libra2Model::ArmGravTorque(-sample.torque, cw_torque);
}

/**
 * @brief Recomputes a saved arm model from its raw samples using the current
 *        CW model.
 *
 * @param saved Arm model (one sample = level, two = two-angle)
 */
std::optional<Motion::ArmModel> Libra2MotionController::RecomputeArmModel(
    const Motion::ArmModel& saved) const {
    if (!cw_model_.IsValid()) {
        return std::nullopt;
    }

    Motion::ArmModel model = saved;
    model.cw_model = cw_model_.GetParams();

    // Level calibrated
    if (saved.samples.size() == 1) {
        // Eq. 34 with the saved arm_hang_torque
        const Motion::RestSample& s = saved.samples.front();
        const auto level = Libra2Model::ArmLevelTorque(SampleToArmTorque(s),
                                                       s.theta,
                                                       saved.hang_torque);
        if (!level) {
            return std::nullopt;
        }
        model.level_torque = *level;
        return model;
    }

    // Two-angle calibrated
    if (saved.samples.size() == 2) {
        // Eq. 35
        const Motion::RestSample& a = saved.samples[0];
        const Motion::RestSample& b = saved.samples[1];
        const auto solved = Libra2Model::SolveTwoAngle(SampleToArmTorque(a),
                                                       a.theta,
                                                       SampleToArmTorque(b),
                                                       b.theta);
        if (!solved) {
            return std::nullopt;
        }
        model.level_torque = solved->level_torque;
        model.hang_torque = solved->hang_torque;
        return model;
    }

    return std::nullopt;
}

/**
 * @brief Checks whether the arm is stable at every possible CW volume.
 *
 * @param arm_hang_torque Arm hang torque, m_arm·h_1·g (Nm)
 *
 * @note S in Eq. 27' is linear in m_cw, so checking only empty and full covers
 *       every volume.
 */
bool Libra2MotionController::CheckStability(const double& arm_hang_torque) {
    const double cap = cw_model_.GetCapacityMass();
    const double empty = Libra2Model::StabilityTorque(arm_hang_torque,
                                                      cw_model_.HangMoment(0.0),
                                                      kGrav);
    const double full = Libra2Model::StabilityTorque(arm_hang_torque,
                                                     cw_model_.HangMoment(cap),
                                                     kGrav);

    const std::string values = Format(empty) + " Nm empty, " + Format(full)
                               + " Nm full";
    if (std::max(empty, full) < 0.0) {
        logger_->Info("Arm model is stable at all CW volumes");
        logger_->Debug("Stability (g*S < 0): " + values);
        return true;
    }

    // Linear in m_cw, so a sign change between empty and full has one crossover
    std::string unstable_range_str = "at every CW volume";
    if ((empty < 0.0) != (full < 0.0)) {
        const double crossover = cap * empty / (empty - full) / kWaterDensity;
        unstable_range_str = std::string(empty < 0.0 ? "above " : "below ")
                             + Format(crossover) + " L";
    }
    logger_->Warn("Arm model is unstable " + unstable_range_str
                  + " (g*S >= 0): " + values + "; check the calibration");

    return false;
}

/**
 * @brief Advances the active arm calibration (level or two-angle) by one cycle.
 *
 * @param dt Time since last cycle (s)
 * @param trajectory_active Whether a trajectory is running
 * @param theta Actual angle, θ (rad)
 * @param actual_vel Actual velocity, θ̇ (rad/s)
 * @param actual_torque Actual actuator torque, τ_actuator (Nm)
 */
void Libra2MotionController::RunArmCalibration(const double& dt,
                                               const bool& trajectory_active,
                                               const double& theta,
                                               const double& actual_vel,
                                               const double& actual_torque) {
    t_stage_s_ += dt;

    const bool moving_stage =
        (status_ == Motion::CalibrationStatus::kArmMovingToA
         || status_ == Motion::CalibrationStatus::kArmMovingToB);
    const bool at_a = (status_ == Motion::CalibrationStatus::kArmMovingToA
                       || status_ == Motion::CalibrationStatus::kArmSamplingA);
    const double target = at_a ? angle_a_ : angle_b_;

    if (t_stage_s_ > (moving_stage ? kMoveTimeout : kSampleTimeout)) {
        FinishCalibration(false, moving_stage
                                     ? "timed out moving to target"
                                     : "timed out waiting for steady state");
        return;
    }

    // Sampling stages: average once steady
    std::optional<Motion::RestSample> sample;
    if (!moving_stage) {
        // Sampling is invalid if pose drifts (except for level calibration)
        if (status_ != Motion::CalibrationStatus::kArmSamplingLevel
            && std::abs(theta - target) > 2.0 * kReachTolerance) {
            sampler_.Reset();
            return;
        }

        sample = sampler_.Add(dt, theta, actual_vel, actual_torque,
                              trajectory_active, cw_vol_);
        if (!sample) {
            return;
        }
    }

    switch (status_) {
        case Motion::CalibrationStatus::kArmMovingToA:
        case Motion::CalibrationStatus::kArmMovingToB:
            // Start next step of an active calibration
            if (!trajectory_active
                && std::abs(theta - target) <= kReachTolerance
                && std::abs(actual_vel) <= kRestVelTolerance) {
                BeginCalibrationStage(
                    at_a ? Motion::CalibrationStatus::kArmSamplingA
                         : Motion::CalibrationStatus::kArmSamplingB);
            }
            break;

        case Motion::CalibrationStatus::kArmSamplingLevel:
            FinishLevelCalibration(*sample);
            break;

        case Motion::CalibrationStatus::kArmSamplingA:
            sample_a_ = *sample;
            BeginCalibrationStage(Motion::CalibrationStatus::kArmMovingToB);
            break;

        case Motion::CalibrationStatus::kArmSamplingB:
            FinishTwoAngleCalibration(*sample);
            break;

        default:
            break;
    }
}

/**
 * @brief Solves the arm model from a level sample and applies it.
 *
 * @param sample Averaged sample with the arm level
 *
 * @note Uses Eq. 34 with the current arm hang torque (0 if never calibrated),
 *       since one sample can't separate the two arm terms.
 */
void Libra2MotionController::FinishLevelCalibration(
    const Motion::RestSample& sample) {
    if (std::abs(sample.theta) > kMaxLevelAngle) {
        FinishCalibration(false, "arm is not level (theta="
                                     + Format(sample.theta) + " rad)");
        return;
    }

    // Eq. 34 with current arm_hang_torque_
    const auto level = Libra2Model::ArmLevelTorque(SampleToArmTorque(sample),
                                                   sample.theta,
                                                   arm_hang_torque_);
    if (!level) {
        FinishCalibration(false, "cos(theta) too small");
        return;
    }
    if (!arm_calibrated_) {
        logger_->Warn("arm_hang_torque not calibrated: using "
                      "arm_hang_torque = 0 (run two-angle calibration to "
                      "measure it)");
    }

    arm_level_torque_ = *level;
    arm_calibrated_ = true;
    arm_source_ = "level";
    arm_cw_params_ = cw_model_.GetParams();
    arm_samples_ = {sample};

    FinishCalibration(true, "Level calibration succeeded: arm_level_torque="
                                + Format(arm_level_torque_)
                                + " Nm, arm_hang_torque="
                                + Format(arm_hang_torque_) + " Nm");
}

/**
 * @brief Solves the arm model from the samples at both angles and applies it.
 *
 * @param sample_b Averaged sample at the second angle (`sample_a_` is already
 *                 stored internally)
 *
 * @see Libra2Model::SolveTwoAngle
 */
void Libra2MotionController::FinishTwoAngleCalibration(
    const Motion::RestSample& sample_b) {
    // Eq. 35 with measured angles
    const auto solved = Libra2Model::SolveTwoAngle(SampleToArmTorque(sample_a_),
                                                   sample_a_.theta,
                                                   SampleToArmTorque(sample_b),
                                                   sample_b.theta);
    if (!solved) {
        FinishCalibration(false, "measured angles too close together");
        return;
    }

    arm_level_torque_ = solved->level_torque;
    arm_hang_torque_ = solved->hang_torque;
    arm_calibrated_ = true;
    arm_source_ = "two-angle";
    arm_cw_params_ = cw_model_.GetParams();
    arm_samples_ = {sample_a_, sample_b};
    const bool stable = CheckStability(arm_hang_torque_);

    FinishCalibration(true,
                      "Two-angle calibration succeeded: arm_level_torque="
                          + Format(arm_level_torque_)
                          + " Nm, arm_hang_torque=" + Format(arm_hang_torque_)
                          + " Nm" + (stable ? "" : " (UNSTABLE; see log)"));
}

//------------------------------------------------------------------------------
// !Friction Calibration Helpers
//------------------------------------------------------------------------------

/**
 * @brief Advances the running friction calibration by one cycle.
 *
 * @param dt Time since last cycle (s)
 * @param trajectory_active Whether a trajectory is running
 * @param theta Actual angle, θ (rad)
 * @param actual_vel Actual velocity, θ̇ (rad/s)
 * @param actual_torque Actual actuator torque, τ_actuator (Nm)
 *
 * @note Aborts (and stops the sweep) if the torque passes the ATC hard limit
 *       or the CW volume changes.
 */
void Libra2MotionController::RunFrictionCalibration(
    const double& dt, const bool& trajectory_active, const double& theta,
    const double& actual_vel, const double& actual_torque) {
    FrictionRun& run = friction_run_;
    t_stage_s_ += dt;

    // Abort guards
    std::string abort_reason;
    if (t_stage_s_ > kMoveTimeout) {
        abort_reason = "timed out moving or sweeping";
    } else if (std::abs(actual_torque) > torque_comp_upper_bound_) {
        abort_reason = "torque " + Format(actual_torque)
                       + " Nm is past the hard limit ("
                       + Format(torque_comp_upper_bound_)
                       + " Nm); balance the CW near the window first";
    } else if (!cw_vol_ || std::abs(*cw_vol_ - run.volume) > kRestVolTolerance) {
        abort_reason = "CW volume changed during the sweeps";
    }

    if (!abort_reason.empty()) {
        pending_stop_ = true;
        FinishCalibration(false, abort_reason);
        return;
    }

    const bool going_up = (status_
                           == Motion::CalibrationStatus::kFrictionSweepUp);
    const double end =
        (status_ == Motion::CalibrationStatus::kFrictionSweepDown
         || status_ == Motion::CalibrationStatus::kFrictionMovingToStart)
            ? run.angle_lo
            : run.angle_hi;
    const double speed = run.max_speed
                         * kFrictionSpeedFractions.at(run.speed_idx);

    if (status_ != Motion::CalibrationStatus::kFrictionMovingToStart) {
        if (trajectory_active) {
            run.trajectory_seen = true;
            AddFrictionSample(theta, actual_vel, actual_torque, going_up, speed);
            return;
        }
        if (!run.trajectory_seen) {
            return;  // sweep requested but not started yet
        }
    }

    // Wait until the move/sweep has ended at rest at its end
    const bool arrived = !trajectory_active
                         && std::abs(theta - end) <= kReachTolerance
                         && std::abs(actual_vel) <= kRestVelTolerance;
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
            {
                std::string error;
                if (!FinishFrictionSpeed(speed, error)) {
                    FinishCalibration(false, error);
                    return;
                }
                ++run.speed_idx;
                if (run.speed_idx < kFrictionSpeedFractions.size()) {
                    BeginCalibrationStage(
                        Motion::CalibrationStatus::kFrictionSweepUp);
                } else {
                    FinishFrictionCalibration();
                }
                break;
            }

        default:
            break;
    }
}

/**
 * @brief Bins a friction sweep sample if it is inside the sampled span and at
 *        steady speed.
 *
 * @param theta Actual angle, θ (rad)
 * @param actual_vel Actual velocity, θ̇ (rad/s)
 * @param actual_torque Actual actuator torque, τ_actuator (Nm)
 * @param going_up Whether this is the up sweep
 * @param speed Sweep speed, v (rad/s)
 */
void Libra2MotionController::AddFrictionSample(const double& theta,
                                               const double& actual_vel,
                                               const double& actual_torque,
                                               const bool& going_up,
                                               const double& speed) {
    FrictionRun& run = friction_run_;

    // Skip window edges, where ramps add inertia and the CW may swing
    const double lo = run.angle_lo + kFrictionSampleMargin;
    const double hi = run.angle_hi - kFrictionSampleMargin;
    if (theta < lo || theta > hi) {
        return;  // near a window edge
    }

    const double sweep_vel = going_up ? speed : -speed;
    if (std::abs(actual_vel - sweep_vel) > kFrictionSpeedTolerance * speed) {
        return;  // not at steady speed
    }

    // Bin by angle so up/down sweeps are compared at equal gravitational torque
    const std::size_t n_bins = run.up_sum.size();
    if (n_bins == 0) {
        return;
    }

    const auto bin = std::min(n_bins - 1, static_cast<std::size_t>(
                                              (theta - lo) / kFrictionBinWidth));
    if (going_up) {
        run.up_sum[bin] += actual_torque;
        ++run.up_count[bin];
    } else {
        run.down_sum[bin] += actual_torque;
        ++run.down_count[bin];
    }
}

/**
 * @brief Turns one speed's binned sweeps into a friction point, and logs the
 *        gravity curve they give.
 *
 * @param speed Sweep speed, v (rad/s)
 * @param[out] error Descriptive reason for failure, if any
 *
 * @note Per angle bin, with actual actuator torque (so bin mean = holding
 *       torque, -τ_ext,g):
 *         drag = (τ_up - τ_down) / 2
 *         τ_g(θ) = (τ_up + τ_down) / 2
 */
bool Libra2MotionController::FinishFrictionSpeed(const double& speed,
                                                 std::string& error) {
    FrictionRun& run = friction_run_;

    const bool model_ready = arm_calibrated_ && cw_model_.IsValid();
    const double lo = run.angle_lo + kFrictionSampleMargin;
    const double cw_mass = Libra2Model::CwMass(run.volume);

    Motion::FrictionPoint point;
    point.speed = speed;
    double drag_min = 0.0;
    double drag_max = 0.0;
    double grav_sum = 0.0;
    double model_err_sum = 0.0;

    for (std::size_t i = 0; i < run.up_sum.size(); ++i) {
        if (run.up_count[i] < kFrictionMinBinSamples
            || run.down_count[i] < kFrictionMinBinSamples) {
            // Not enough samples to compare up/down at this angle
            continue;
        }

        // Friction flips sign between sweeps and gravity doesn't, so half the
        // difference is friction and half the sum is holding torque
        const double up = run.up_sum[i] / run.up_count[i];
        const double down = run.down_sum[i] / run.down_count[i];
        const double drag = 0.5 * (up - down);
        const double holding = 0.5 * (up + down);

        // Track how much drag varies across bins
        if (point.n_bins == 0) {
            drag_min = drag;
            drag_max = drag;
        }

        drag_min = std::min(drag_min, drag);
        drag_max = std::max(drag_max, drag);
        point.drag += drag;
        grav_sum += holding;

        // Holding torque should equal -τ_ext,g at bin center
        if (model_ready) {
            const double theta_bin = lo
                                     + (static_cast<double>(i) + 0.5)
                                           * kFrictionBinWidth;
            model_err_sum += holding + ComputeNetGravTorque(theta_bin, cw_mass);
        }

        ++point.n_bins;
    }

    if (point.n_bins < kFrictionMinBins) {
        error = "only " + std::to_string(point.n_bins)
                + " angle bins had steady samples in both directions at "
                + Format(speed) + " rad/s (need "
                + std::to_string(kFrictionMinBins) + "); use a wider window";
        return false;
    }

    // Average drag over the bins and record this speed's friction point
    const auto n = static_cast<double>(point.n_bins);
    point.drag /= n;
    point.spread = drag_max - drag_min;
    run.points.push_back(point);

    std::string msg = "Friction sweep at " + Format(speed)
                      + " rad/s: drag=" + Format(point.drag) + " Nm (spread "
                      + Format(point.spread) + " Nm over "
                      + std::to_string(point.n_bins)
                      + " bins), mean holding torque=" + Format(grav_sum / n)
                      + " Nm";
    if (model_ready) {
        msg += " (model error " + Format(model_err_sum / n) + " Nm)";
    }
    logger_->Info(msg);

    return true;
}

/**
 * @brief Fits the friction line through all speeds and applies it.
 *
 * @note A negative slope means friction is flat within noise, so b = 0 and τ_c
 *       is the mean drag. A negative τ_c fails the calibration.
 */
void Libra2MotionController::FinishFrictionCalibration() {
    const FrictionRun& run = friction_run_;

    std::vector<double> speeds;
    std::vector<double> drags;
    for (const auto& p : run.points) {
        speeds.push_back(p.speed);
        drags.push_back(p.drag);
    }

    auto fit = Libra2Model::FitFrictionLine(speeds, drags);
    if (!fit) {
        FinishCalibration(false, "friction line fit failed (need two or more "
                                 "speeds)");
        return;
    }

    Motion::FrictionModel& model = *fit;
    if (model.viscous < 0.0) {
        logger_->Warn("Fitted b is negative (" + Format(model.viscous)
                      + " Nm*s/rad), so friction looks flat; using b = 0");

        model.viscous = 0.0;
        model.coulomb = 0.0;
        for (const double& d : drags) {
            model.coulomb += d;
        }
        model.coulomb /= static_cast<double>(drags.size());

        double sum_sq = 0.0;
        for (const double& d : drags) {
            sum_sq += (d - model.coulomb) * (d - model.coulomb);
        }
        model.residual_rms = std::sqrt(sum_sq
                                       / static_cast<double>(drags.size()));
    }

    if (model.coulomb < 0.0) {
        FinishCalibration(false, "fitted tau_c is negative ("
                                     + Format(model.coulomb)
                                     + " Nm); check the run");
        return;
    }

    // Apply model
    model.angle_lo = run.angle_lo;
    model.angle_hi = run.angle_hi;
    model.volume = run.volume;
    model.points = run.points;

    friction_coulomb_ = model.coulomb;
    friction_viscous_ = model.viscous;
    friction_model_ = model;

    FinishCalibration(true, "Friction calibration succeeded: tau_c="
                                + Format(model.coulomb) + " Nm, b="
                                + Format(model.viscous) + " Nm*s/rad (fit RMS "
                                + Format(model.residual_rms) + " Nm)");
}
