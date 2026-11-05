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
#include <iomanip>
#include <sstream>
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

// ETA (Eq. 44; unverified)

// Tubes u_1 (smallest) to u_15
constexpr std::array<double, 15> kEtaTubeMass = {
    0.0386, 0.0450, 0.0592, 0.0674, 0.0755, 0.0847, 0.0932, 0.0995,
    0.1147, 0.1233, 0.1306, 0.1389, 0.1520, 0.1550, 0.1608,
};  // m_ui (kg)
constexpr std::array<double, 15> kEtaTubeLength = {
    0.600, 0.600, 0.640, 0.640, 0.640, 0.640, 0.640, 0.640,
    0.640, 0.640, 0.640, 0.640, 0.640, 0.640, 0.640,
};                                         // L_ui (m)
constexpr double kEtaTubeOverlap = 0.150;  // L_ov (m)

constexpr double kEtaConduitMassPerLength = 0.182;  // μ_conduit (kg/m)
// TODO: input after measuring
constexpr double kEtaConduitMinLength = 0.0;  // L_conduit,min (m)

constexpr double kSensorSuiteMass = 0.2917;  // D456 + SF45/B w/ mounts (kg)
constexpr double kEtaTipMountMass = 0.0215;  // (kg)
constexpr double kEtaTipMass = kSensorSuiteMass
                               + kEtaTipMountMass;  // m_tip (kg)

// Predictive CW Command

// TODO: arbitrary; tune via experiments
constexpr double kCwVolumeTolerance = 0.01;  // "target reached" band (L)

// Model Calibration

constexpr double kReachTolerance = 0.0175;  // "arrived at target", ~1 deg (rad)
constexpr double kRestPosTolerance = 0.005;  // allowed drift in window (rad)
constexpr double kRestVelTolerance = 0.005;  // allowed drift in window (rad/s)
constexpr double kRestVolumeTolerance = 0.01;  // allowed CW change in window (L)
constexpr double kRestWindowTime = 3.0;        // averaging window (s)

constexpr double kProgressLogPeriod = 15.0;  // heartbeat message (s)
constexpr double kMoveTimeout = 180.0;       // (s)
constexpr double kSamplingTimeout = 60.0;    // (s)

// Velocity averaging window for rest and steady speed checks, arbitrary (s);
// at rest, raw vel noise is ~0.6 deg/s but its 0.75 s mean stays < 0.2 deg/s
constexpr double kVelAvgWindow = 0.75;
constexpr double kMinCosForBalance = 1e-3;  // |cosθ| floor, Eq. 34/38'

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
constexpr double kFrictionVelTolerance = 0.1;  // allowed |θ̇ - v| / v (rad/s)

constexpr double kFrictionBinWidth = 0.0087;  // angle bin, ~0.5 deg (rad)
constexpr int kFrictionMinBinSamples = 3;     // per bin and direction
constexpr int kFrictionMinBins = 3;           // usable bins per speed

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
 * @brief Formats an angle (rad) or speed (rad/s) in degrees.
 *
 * @param rad Value to format
 */
std::string FormatDeg(const double& rad) {
    constexpr double kRadToDeg = 180.0 / M_PI;

    std::ostringstream oss;
    oss << std::fixed << std::setprecision(1) << rad * kRadToDeg;
    return oss.str();
}

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
// !Counterweight Model
//------------------------------------------------------------------------------

/**
 * @brief Builds and checks the counterweight (CW) model (pivot distance vs.
 *        volume).
 *
 * @param cw_pivot_dist CW pivot distance at 0 liters, A (m); sign is ignored
 *                      (0 = "not measured yet" and leaves model invalid)
 * @param cw_pivot_slope Change in CW pivot distance per liter, B (m/L)
 * @param cw_mean_sq_dist Bags' mean squared hang distance (along arm, from O)
 *                        at 0 liters, C (m²) (0 = "not measured yet" and leaves
 *                        model invalid)
 * @param cw_mean_sq_dist_slope Change in mean squared hang distance per liter,
 *                              D (m²/L)
 * @param cw_hang_offset CW hang point offset (normal to arm) relative to pitch
 *                       joint O, q_P (m)
 * @param capacity Max. combined CW volume (L)
 *
 * @note Extension of Eq. 14b, where r_P varies linearly with volume:
 *         r_P(V) = A + B·V.
 */
CwModel::CwModel(const double& cw_pivot_dist, const double& cw_pivot_slope,
                 const double& cw_mean_sq_dist,
                 const double& cw_mean_sq_dist_slope,
                 const double& cw_hang_offset, const double& capacity)
    : capacity_(capacity) {
    // Validate params
    if (!std::isfinite(capacity) || capacity <= 0.0) {
        error_ = "CW capacity must be positive";
        return;
    }
    if (!std::isfinite(cw_pivot_dist) || !std::isfinite(cw_pivot_slope)
        || !std::isfinite(cw_mean_sq_dist)
        || !std::isfinite(cw_mean_sq_dist_slope)
        || !std::isfinite(cw_hang_offset)) {
        error_ = "One or more of r_P components, mean r^2 components, or q_P "
                 "is not a number";
        return;
    }
    if (cw_pivot_dist == 0.0) {
        error_ = "r_P is not set (intercept = 0)";
        return;
    }
    if (cw_mean_sq_dist <= 0.0) {
        error_ = "mean r^2 is not set (intercept <= 0)";
        return;
    }

    // Set model properties
    r_P_0_ = std::abs(cw_pivot_dist);
    dr_P_dV_ = cw_pivot_slope;
    r2_0_ = cw_mean_sq_dist;
    dr2_dV_ = cw_mean_sq_dist_slope;
    q_P_ = cw_hang_offset;

    // Validate mechanics
    if (!(r_P_0_ + (2.0 * dr_P_dV_ * capacity_) > 0.0)) {
        error_ = "r_P line is too steep: the CW moment must increase with "
                 "volume up to full (need A + 2*B*capacity > 0)";
        return;
    }
    if (!(r2_0_ + (2.0 * dr2_dV_ * capacity_) > 0.0)) {
        error_ = "mean r^2 line is too steep: the CW inertia must increase "
                 "with volume up to full (need C + 2*D*capacity > 0)";
    }
}

/**
 * @brief Gets the values that define the current model.
 */
Motion::CwModelParams CwModel::GetParams() const {
    Motion::CwModelParams params;
    params.pivot_dist = r_P_0_;
    params.pivot_slope = dr_P_dV_;
    params.mean_sq_dist = r2_0_;
    params.mean_sq_dist_slope = dr2_dV_;
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
    return r_P_0_ + (dr_P_dV_ * volume);
}

/**
 * @brief Computes the bags' mean squared hang distance (m²) along the arm from
 *        pitch joint O at a given CW volume.
 *
 * @param volume CW volume, V (L)
 *
 * @note Code-specific linear model, fitted to per-bag weighings from Step 1 of
 *       the motion controller experiments (2026/10/22):
 *         ΣV_i·r_bag,i² / V = C + D·V
 */
double CwModel::MeanSqDist(const double& volume) const {
    return r2_0_ + (dr2_dV_ * volume);
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
 * @param angle Pitch angle, θ (rad)
 *
 * @note Code-specific extension of Eq. 38' for r_P that varies with volume;
 *       dividing CW torque by g·cosθ gives:
 *         m_cw·(r_P(V) + q_P·tanθ)
 */
double CwModel::EffectiveMoment(const double& mass, const double& angle) const {
    return Moment(mass) + (HangMoment(mass) * std::tan(angle));
}

/**
 * @brief Finds the CW mass (kg) that gives a requested moment at an angle
 *        (inverse of `EffectiveMoment()`).
 *
 * @param moment Requested effective moment, M (kg·m)
 * @param angle Pitch angle, θ (rad)
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
                                             const double& angle) const {
    if (!IsValid()) {
        return std::nullopt;
    }

    // Effective lever must keep the moment rising from empty to full
    const double a_eff = r_P_0_ + (q_P_ * std::tan(angle));
    if (!(a_eff > 0.0) || !(a_eff + (2.0 * dr_P_dV_ * capacity_) > 0.0)) {
        return std::nullopt;
    }

    // Clamp to [0, capacity]
    if (!(moment > 0.0)) {
        return 0.0;
    }

    const double cap = GetCapacityMass();
    if (moment >= EffectiveMoment(cap, angle)) {
        return cap;
    }

    // Solve for m_cw
    const double c = dr_P_dV_ / kWaterDensity;
    const double disc = std::max(0.0, (a_eff * a_eff) + (4.0 * c * moment));
    return 2.0 * moment / (a_eff + std::sqrt(disc));
}

/**
 * @brief Computes the CW moment of inertia (kg·m²) about pitch joint O.
 *
 * @param mass CW mass, m_cw (kg)
 *
 * @note Uses `MeanSqDist()` for bags' spread since only total volume is known:
 *         I_cw = ρ_water·ΣV_i·r_bag,i² + m_cw·q_P² = m_cw·(C + D·V + q_P²)
 */
double CwModel::Inertia(const double& mass) const {
    // TODO: use after completing Step 5 of the motion controller experiments
    /*
    const double r_P = PivotDist(mass / kWaterDensity);
    return mass * (r_P * r_P + q_P_ * q_P_);
    */
    return mass * (MeanSqDist(mass / kWaterDensity) + (q_P_ * q_P_));
}

/**
 * @brief Computes the change in CW moment of inertia per kg of CW (m²).
 *
 * @param mass CW mass, m_cw (kg)
 *
 * @note Code-specific derivative of `Inertia()` for the inertia rate in Eq. 15:
 *         d(inertia)/dm_cw = C + 2·D·V + q_P²
 *
 * @see CwModel::Inertia
 */
double CwModel::InertiaSlope(const double& mass) const {
    // TODO: use after completing Step 5 of the motion controller experiments
    /*
    const double volume = mass / kWaterDensity;
    const double r_P = PivotDist(volume);
    return r_P * (r_P + 2.0 * dr_P_dV_ * volume) + q_P_ * q_P_;
    */
    const double volume = mass / kWaterDensity;
    return r2_0_ + (2.0 * dr2_dV_ * volume) + (q_P_ * q_P_);
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
 * @brief Computes the contribution of ETA extension to the arm's moment,
 *        Δ(m_arm·r_1) (kg·m).
 *
 * @param extension Change in ETA extension (from min.), ΔL (m)
 *
 * @note From Eq. 44 (from extended equations in `ARM_EXTENSION_ESTIMATION.md`),
 *       where ΔL = ΔL_conduit since the conduit is assumed to be fed in from
 *       the pitch axis:
 *         Δ(m_arm·r_1) = (tubes u_1-15) + (conduit) + (tip cap, payload)
 *                      = Σ m_ui·max(0, ΔL - S_(i-1))
 *                        + μ_conduit·(L_conduit,min·ΔL + ΔL²/2)
 *                        + m_tip·ΔL
 *
 *       Tubes assumed to slide out smallest-first, so tube u_i only moves once
 *       the tubes inside it have used up their strokes:
 *         S_(i-1) = Σ_(j<i) (L_uj - L_ov)
 *
 * @todo Arm inertia also grows with extension, but isn't modeled yet.
 */
double EtaMomentChange(const double& extension) {
    // Tubes
    double moment = 0.0;
    double stroke_before = 0.0;  // S_(i-1)
    for (std::size_t i = 0; i < kEtaTubeMass.size(); ++i) {
        moment += kEtaTubeMass.at(i) * std::max(0.0, extension - stroke_before);
        stroke_before += kEtaTubeLength.at(i) - kEtaTubeOverlap;
    }

    // Conduit
    moment += kEtaConduitMassPerLength
              * ((kEtaConduitMinLength * extension)
                 + (0.5 * extension * extension));

    // Tip cap, payload
    moment += kEtaTipMass * extension;
    return moment;
}

/**
 * @brief Computes the CW's gravitational torque (Nm) about pitch joint O.
 *
 * @param cw_moment CW gravitational moment along arm, m_cw·r_P (kg·m)
 * @param cw_hang_moment CW gravitational moment normal to arm, m_cw·q_P (kg·m)
 * @param grav Gravitational acceleration, g (m/s²)
 * @param angle Pitch angle, θ (rad)
 *
 * @note From Eq. 29b':
 *         τ_g2(θ) = m_cw·g·(r_P·cosθ + q_P·sinθ)
 */
double CwGravTorque(const double& cw_moment, const double& cw_hang_moment,
                    const double& grav, const double& angle) {
    return grav
           * ((cw_moment * std::cos(angle))
              + (cw_hang_moment * std::sin(angle)));
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
 * @param arm_torque Arm's gravitational torque at `angle`, τ_g1(θ) (Nm)
 * @param angle Pitch angle, θ (rad)
 * @param arm_hang_torque Arm hang torque, m_arm·h_1·g (Nm)
 *
 * @note Returns none if cosθ is too small.
 *
 * @note From Eq. 34 (Eq. 31 when θ = 0):
 *         τ_g1,0 = (τ_g1(θ) - g·m_arm·h_1·sinθ) / cosθ
 */
std::optional<double> ArmLevelTorque(const double& arm_torque,
                                     const double& angle,
                                     const double& arm_hang_torque) {
    const double c = std::cos(angle);
    if (std::abs(c) < kMinCosForBalance) {
        return std::nullopt;
    }
    return (arm_torque - (arm_hang_torque * std::sin(angle))) / c;
}

/**
 * @brief Solves the arm's level and hang torques from a set of two-angle
 *        measurements.
 *
 * @param arm_torque_a Arm's gravitational torque at angle a, τ_g1(θ_a) (Nm)
 * @param angle_a Angle a, θ_a (rad)
 * @param arm_torque_b Arm's gravitational torque at angle b, τ_g1(θ_b) (Nm)
 * @param angle_b Angle b, θ_b (rad)
 *
 * @note Fails if the angles are too close together.
 *
 * @note From Eqs. 35c and 35d:
 *         τ_g1,0 = (τ_g1(θ_a)·sinθ_b - τ_g1(θ_b)·sinθ_a) / sin(θ_b - θ_a)
 *         m_arm·h_1·g = (τ_g1(θ_b)·cosθ_a - τ_g1(θ_a)·cosθ_b)
 *                       / sin(θ_b - θ_a)
 */
std::optional<Motion::ArmModel> SolveTwoAngle(const double& arm_torque_a,
                                              const double& angle_a,
                                              const double& arm_torque_b,
                                              const double& angle_b) {
    const double s = std::sin(angle_b - angle_a);
    if (std::abs(s) < std::sin(0.5 * Motion::kMinAngleSeparation)) {
        return std::nullopt;
    }

    Motion::ArmModel model;
    model.level_torque = ((arm_torque_a * std::sin(angle_b))
                          - (arm_torque_b * std::sin(angle_a)))
                         / s;
    model.hang_torque = ((arm_torque_b * std::cos(angle_a))
                         - (arm_torque_a * std::cos(angle_b)))
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
 * @param angle Pitch angle, θ (rad)
 *
 * @note From Eq. 36':
 *         τ_ext,g(θ) = (τ_g1,0 + m_cw·g·r_P)·cosθ
 *                      + (m_arm·h_1·g + m_cw·g·q_P)·sinθ
 */
double NetGravTorque(const double& arm_level_torque,
                     const double& arm_hang_torque, const double& cw_moment,
                     const double& cw_hang_moment, const double& grav,
                     const double& angle) {
    return ((arm_level_torque + (grav * cw_moment)) * std::cos(angle))
           + ((arm_hang_torque + (grav * cw_hang_moment)) * std::sin(angle));
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
    return arm_hang_torque + (grav * cw_hang_moment);
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
    return (inertia * ref_acc) + ((friction_viscous + inertia_rate) * ref_vel)
           + (friction_coulomb * Sign(ref_vel)) - net_grav_torque;
}

/**
 * @brief Computes the effective CW moment (kg·m) that balances the arm at a
 *        given angle.
 *
 * @param arm_level_torque Arm gravitational torque at level, τ_g1,0 (Nm)
 * @param arm_hang_torque Arm hang torque, m_arm·h_1·g (Nm)
 * @param grav Gravitational acceleration, g (m/s²)
 * @param angle Angle to balance at, θ (rad)
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
                                      const double& grav, const double& angle) {
    if (std::abs(std::cos(angle)) < kMinCosForBalance) {
        return std::nullopt;
    }
    return -(arm_level_torque + (arm_hang_torque * std::tan(angle))) / grav;
}

/**
 * @brief Predicts the net gravitational torque (Nm) at a target angle with the
 *        current CW state.
 *
 * @param grav Gravitational acceleration, g (m/s²)
 * @param angle_tgt Target angle, θ_tgt (rad)
 * @param cw_moment_eff Current effective CW moment at target (kg·m), from
 *                      `CwModel::EffectiveMoment()`
 * @param cw_moment_bal Effective CW moment balancing arm at target (kg·m),
 *                      from `BalanceCwMoment()`
 *
 * @note Code-specific extension of Eq. 39' for r_P that varies with volume,
 *       with the effective moments of `BalanceCwMoment()`:
 *         τ_ext,g(θ_tgt) = g·cosθ_tgt·(M_eff(m_cw) - M_eff(m_cw,bal))
 */
double TargetNetGravTorque(const double& grav, const double& angle_tgt,
                           const double& cw_moment_eff,
                           const double& cw_moment_bal) {
    return grav * std::cos(angle_tgt) * (cw_moment_eff - cw_moment_bal);
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
 * @note Code-specific equation, based on Eq. 18 at constant speed (θ̈ = 0,
 *       İ = 0). Isolates the friction component by averaging the up/down
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

}  // namespace Libra2Model

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
 * @param vel Actual velocity, θ̇ (rad/s)
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
 * @param vel Actual velocity, θ̇ (rad/s)
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
    : Libra2MotionController(kCwPivotDist, kCwPivotSlope, kCwMeanSqDist,
                             kCwMeanSqDistSlope, kCwHangOffset, cw_capacity,
                             kDefaultFrictionCoulomb, kDefaultFrictionViscous,
                             kArmInertia, torque_comp_lower_bound,
                             torque_comp_upper_bound, torque_comp_slow_rate,
                             debug_mode) {}

/**
 * @brief Standard constructor.
 *
 * @param cw_pivot_dist CW pivot distance at 0 liters, A (m); 0 = not measured
 * @param cw_pivot_slope Change in CW pivot distance per liter, B (m/L)
 * @param cw_mean_sq_dist Bags' mean squared hang distance at 0 liters, C (m²);
 *                        0 = not measured
 * @param cw_mean_sq_dist_slope Change in mean squared hang distance per liter,
 *                              D (m²/L)
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
    const double& cw_mean_sq_dist, const double& cw_mean_sq_dist_slope,
    const double& cw_hang_offset, const double& cw_capacity,
    const double& friction_coulomb, const double& friction_viscous,
    const double& arm_inertia, const double& torque_comp_lower_bound,
    const double& torque_comp_upper_bound, const double& torque_comp_slow_rate,
    const bool& debug_mode)
    : AbstractMotionController({Joint::Name::kPitch}, debug_mode),
      cw_model_(cw_pivot_dist, cw_pivot_slope, cw_mean_sq_dist,
                cw_mean_sq_dist_slope, cw_hang_offset, cw_capacity),
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
 * @note A mode change may be refused (e.g., CW Primary needs valid/non-zero arm
 *       and CW models, and no move in progress). Switching to Tandem is always
 *       accepted.
 */
bool Libra2MotionController::SetMotionMode(const Motion::MotionMode& mode) {
    std::lock_guard<std::mutex> lock(mutex_);

    // Validate mode change
    if (mode == Motion::MotionMode::kCwPrimary
        && !(arm_calibrated_ && cw_model_.IsValid())) {
        logger_->Warn("CW Primary mode needs a calibrated arm model and a "
                      "valid CW model; staying in the current mode");
        return false;
    }
    if (mode == Motion::MotionMode::kCwPrimary && IsMovingLocked()) {
        logger_->Warn("Can't switch to CW Primary while the arm is moving; "
                      "staying in the current mode");
        return false;
    }

    // Ensure safe transition
    if (cw_phase_ == CwPhase::kCwMoving) {
        logger_->Warn("Left CW Primary while the CW was moving the arm; "
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
        + (mode == Motion::MotionMode::kTandem ? "Tandem" : "CW Primary"));
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

    cw_volume_ = volume;
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
 * @brief Sets the CW Primary mode's hand-off band.
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
        logger_->Warn("Ignoring invalid ETA extension: " + Format(extension)
                      + " m");
        return;
    }

    eta_extension_ = extension;

    logger_->Info("ETA extension set to " + Format(extension * 1000.0) + " mm");
}

/**
 * @brief Gets the ETA's extension (m).
 */
double Libra2MotionController::GetEtaExtension() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return eta_extension_;
}

/**
 * @brief Directly sets the arm model parameters.
 *
 * @param arm_level_torque Arm gravitational torque at level and min. ETA
 *                         extension, τ_g1,0 (Nm)
 * @param arm_hang_torque Arm hang torque, m_arm·h_1·g (Nm)
 */
void Libra2MotionController::SetArmModel(const double& arm_level_torque,
                                         const double& arm_hang_torque) {
    std::lock_guard<std::mutex> lock(mutex_);

    arm_level_torque_ = arm_level_torque;
    arm_hang_torque_ = arm_hang_torque;
    arm_calibrated_ = true;
    arm_source_ = "set";
    arm_cw_params_ = cw_model_.GetParams();
    logger_->Info("Arm model set: τ_g1,0=" + Format(arm_level_torque)
                  + " Nm, m_arm*h_1*g=" + Format(arm_hang_torque) + " Nm");

    CheckStability(arm_hang_torque_);
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

    arm_level_torque_ = 0.0;
    arm_hang_torque_ = 0.0;
    arm_calibrated_ = false;
    arm_source_.clear();
    mode_ = Motion::MotionMode::kTandem;  // safe default
    cw_phase_ = CwPhase::kIdle;

    friction_coulomb_ = 0.0;
    friction_viscous_ = 0.0;
    friction_model_.reset();  // back to startup defaults

    logger_->Info("Arm and friction models cleared; feedforward and predictive "
                  "CW disabled");

    return true;
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
 * @brief Gets the arm's gravitational torque at level and min. ETA extension,
 *        τ_g1,0 (Nm).
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
 * @param angle Pitch angle, θ (rad)
 * @param cw_mass CW mass, m_cw (kg)
 *
 * @note Mutex lock not required since `Update()` calls it while holding the
 *       lock. External callers should only call it while the motion controller
 *       isn't running.
 *
 * @see Libra2Model::NetGravTorque
 *      Libra2MotionController::LevelTorqueAtExtension
 */
double Libra2MotionController::ComputeNetGravTorque(
    const double& angle, const double& cw_mass) const {
    return Libra2Model::NetGravTorque(LevelTorqueAtExtension(),
                                      arm_hang_torque_,
                                      cw_model_.Moment(cw_mass),
                                      cw_model_.HangMoment(cw_mass), kGrav,
                                      angle);
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
 * @note Mutex lock not required since callers hold it (see
 *       `ComputeNetGravTorque()`).
 *
 * @see Libra2Model::EtaMomentChange
 */
double Libra2MotionController::LevelTorqueAtExtension() const {
    return arm_level_torque_
           - (kGrav * Libra2Model::EtaMomentChange(eta_extension_));
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
    if (!arm_calibrated_) {
        logger_->Warn("Can't calibrate: missing required arm hang torque "
                      "(m_arm*h_1*g) from a two-angle (or loaded) calibration");
        return false;
    }

    // Set up and start calibration
    two_angle_ = false;

    logger_->Info("Starting level calibration");
    logger_->Debug(
        "Level calibration: torque will be averaged when system is at rest for "
        + Format(kRestWindowTime) + " s");
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

    logger_->Info("Starting two-angle calibration: a=" + FormatDeg(angle_a)
                  + " deg, b=" + FormatDeg(angle_b) + " deg");
    logger_->Debug("Two-angle calibration: (1) move to a, (2) average at "
                   "rest, (3) move to b, (4) average at rest");
    BeginCalibrationStage(Motion::CalibrationStatus::kArmMovingToA);

    return true;
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
 * @see kFrictionVelFractions Libra2Model::FitFrictionLine
 */
bool Libra2MotionController::StartFrictionCalibration(const double& angle_a,
                                                      const double& angle_b,
                                                      const double& max_vel) {
    std::lock_guard<std::mutex> lock(mutex_);

    if (Motion::IsCalibrationRunning(status_)) {
        logger_->Warn("Calibration already running; ignoring friction request");
        return false;
    }

    // Validate params
    if (!cw_volume_) {
        logger_->Warn("Friction calibration rejected: CW volume unknown");
        return false;
    }
    if (!(max_vel > 0.0) || !std::isfinite(max_vel)) {
        logger_->Error("Friction calibration rejected: invalid max speed "
                       + Format(max_vel) + " rad/s");
        return false;
    }

    const double lo = std::min(angle_a, angle_b);
    const double hi = std::max(angle_a, angle_b);
    if (!std::isfinite(lo) || !std::isfinite(hi) || lo == hi) {
        logger_->Warn("Friction calibration rejected: need two distinct "
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

    // Ramps widen with speed, so fastest sweep determines unsampled ends
    const double inertia = arm_inertia_
                           + (cw_model_.IsValid() ? cw_model_.Inertia(
                                  Libra2Model::CwMass(*cw_volume_))
                                                  : 0.0);
    const double longest_ramp = RampDist(max_vel, inertia);
    const double margin = longest_ramp + kFrictionSampleMargin;

    const double min_window = (2.0 * margin)
                              + (kFrictionMinBins * kFrictionBinWidth);
    if (hi - lo < min_window) {
        logger_->Warn("Friction calibration rejected: window is "
                      + FormatDeg(hi - lo) + " deg, need at least "
                      + FormatDeg(min_window) + " deg (the " + FormatDeg(max_vel)
                      + " deg/s sweeps take " + FormatDeg(longest_ramp)
                      + " deg at each end to speed up and slow down)");
        return false;
    }

    // Ensure slowest sweep fits in one move stage
    const double slowest = max_vel * kFrictionVelFractions.front();
    const double t_slowest_s = SweepTime(lo, hi, slowest,
                                         RampDist(slowest, inertia));
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
    friction_run_.max_vel = max_vel;
    friction_run_.volume = *cw_volume_;
    friction_run_.inertia = inertia;
    friction_run_.margin = margin;
    two_angle_ = false;

    // Sweep speeds, for the start log
    std::string vels_deg;
    for (const double& fraction : kFrictionVelFractions) {
        vels_deg += (vels_deg.empty() ? "" : ", ")
                    + FormatDeg(max_vel * fraction);
    }

    logger_->Info("Starting friction calibration: " + FormatDeg(lo) + " to "
                  + FormatDeg(hi) + " deg at " + vels_deg + " deg/s");
    logger_->Debug(
        "Friction calibration ("
        + std::to_string(1 + (2 * kFrictionVelFractions.size()))
        + " steps):"
          "\n  sampled:  "
        + FormatDeg(lo + margin) + " to " + FormatDeg(hi - margin)
        + " deg"
          "\n  steps:    move to start, followed by up/down sweep at each speed"
          "\n  CW:       stays idle at "
        + Format(*cw_volume_)
        + " L"
          "\n  aborts:   above "
        + Format(kMaxFrictionTorque) + " Nm");
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

/**
 * @brief Gets the running calibration's current step (e.g., "Friction
 *        calibration [3/9]: ..."); empty if none is running.
 */
std::string Libra2MotionController::GetCalibrationProgress() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return calibration_progress_;
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
        Libra2Model::BalanceCwMoment(LevelTorqueAtExtension(), arm_hang_torque_,
                                     kGrav, angle_tgt);
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
    if (std::abs(diff) <= kCwVolumeTolerance * kWaterDensity) {
        return Motion::CwCommand::kIdle;
    }

    // TODO: if valve latency causes CW overshoot, stop early by around
    //       latency·[V̇] ...
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

        cw_phase_ = CwPhase::kIdle;
        cw_assist_done_ = true;
        pending_move_ = angle_tgt;
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
        logger_->Info("CW handing control to actuator (" + reached
                      + " reached)");
        logger_->Debug(
            "CW Primary hand-off (" + reached + " reached):\n  actual angle="
            + Format(angle) + " rad, target angle=" + Format(angle_tgt)
            + " rad, angle error=" + Format(angle - angle_tgt)
            + " rad\n  actual volume=" + Format(*cw_mass_ / kWaterDensity)
            + " L, target volume=" + Format(mass_tgt / kWaterDensity)
            + " L, volume error="
            + Format((*cw_mass_ - mass_tgt) / kWaterDensity) + " L");

        cw_phase_ = CwPhase::kIdle;
        cw_assist_done_ = true;
        pending_move_ = angle_tgt;
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
 * @note If anything CW Primary needs is lost while the arm is free (model, CW
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

    // Eq. 15: inertia and its rate, with the CW's share (ṁ_cw = ρ_water·V̇);
    // CW terms are code-specific (see `CwModel::Inertia()`/`InertiaSlope()`)
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
        // (A) CW Primary: arm spins freely while CW moves it
        if (!model_ready || !target || calibrating || !atc_en_) {
            logger_->Warn("CW Primary: model, CW volume, target, or ATC "
                          "unavailable; handing control to the actuator");

            cw_phase_ = CwPhase::kIdle;
            cw_assist_done_ = true;
            if (target) {
                pending_move_ = *target;
            }
        } else {
            arm_free = RunCwPrimary(angle, *target, command);
        }
    }

    if (!arm_free && IsFrictionStage(status_)) {
        // Friction sweep: no throttle or CW command necessary
        command.playback_rate = 1.0;
        command.cw_command = Motion::CwCommand::kIdle;
        RunFrictionCalibration(dt, trajectory_active, angle, actual_vel,
                               actual_torque);

    } else if (!arm_free) {
        // (B) Tandem/Holding: ATC throttle applies,
        const Motion::CwCommand direction = (actual_torque > 0.0)
                                                ? Motion::CwCommand::kFill
                                                : Motion::CwCommand::kDrain;
        const bool braking = target
                             && (actual_torque * (*target - angle) < 0.0);
        const AtcResult atc = UpdateTorqueComp(std::abs(actual_torque),
                                               direction, braking);

        command.playback_rate = atc.playback_rate;
        command.cw_command = atc.cw_command;  // reactive (LIBRA-I style)

        if (calibrating) {
            // Feedforward off and CW reactive while calibrating
            RunArmCalibration(dt, trajectory_active, angle, actual_vel,
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

            command.ff_torque[kPitchIdx] = ModelFeedforward(angle, ref_vel,
                                                            ref_acc,
                                                            model_ready);
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

    command.diagnostics = ComputeDiagnostics(angle, target);

    return command;
}

/**
 * @brief Computes the actuator torque feedforward (Nm) for the commanded motion
 *        (Eq. 37).
 *
 * @param angle Actual angle, θ (rad)
 * @param ref_vel Reference velocity, θ̇_ref (rad/s)
 * @param ref_acc Reference acceleration, θ̈_ref (rad/s²)
 * @param model_ready Whether the arm model and CW mass are known; if not, the
 *                    gravity term is left out (PID loop and CW handle it)
 *
 * @note Friction and inertia terms always apply, so a move gets the effort to
 *       overcome friction even before the arm model is calibrated.
 *
 * @see Libra2Model::FeedforwardTorque
 */
double Libra2MotionController::ModelFeedforward(const double& angle,
                                                const double& ref_vel,
                                                const double& ref_acc,
                                                const bool& model_ready) const {
    const double torque_ext_g = model_ready
                                    ? ComputeNetGravTorque(angle, *cw_mass_)
                                    : 0.0;
    const double inertia = inertia_ff_en_ ? inertia_ : 0.0;
    const double inertia_rate = inertia_ff_en_ ? inertia_rate_ : 0.0;

    return Libra2Model::FeedforwardTorque(inertia, inertia_rate,
                                          friction_viscous_, friction_coulomb_,
                                          ref_acc, ref_vel, torque_ext_g);
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
 * @see Libra2Model::CwMass Libra2Model::NetGravTorque
 *      Libra2Model::BalanceCwMoment Libra2Model::TargetNetGravTorque
 *      Libra2MotionController::LevelTorqueAtExtension
 */
Motion::Libra2Diagnostics Libra2MotionController::ComputeDiagnostics(
    const double& angle, const std::optional<double>& angle_tgt) const {
    Motion::Libra2Diagnostics diagnostics;
    diagnostics.cw_mass = cw_mass_;
    if (!arm_calibrated_ || !cw_model_.IsValid()) {
        return diagnostics;
    }

    if (cw_mass_) {
        diagnostics.torque_ext_g = ComputeNetGravTorque(angle, *cw_mass_);
    }

    // Eqs. 38'-40
    if (angle_tgt) {
        const auto moment_bal =
            Libra2Model::BalanceCwMoment(LevelTorqueAtExtension(),
                                         arm_hang_torque_, kGrav, *angle_tgt);
        const auto mass_tgt = moment_bal ? cw_model_.MassForMoment(*moment_bal,
                                                                   *angle_tgt)
                                         : std::nullopt;
        if (mass_tgt) {
            diagnostics.cw_mass_tgt = *mass_tgt;
            if (cw_mass_) {
                diagnostics.cw_mass_change = *mass_tgt - *cw_mass_;
                diagnostics.torque_ext_g_tgt = Libra2Model::TargetNetGravTorque(
                    kGrav, *angle_tgt,
                    cw_model_.EffectiveMoment(*cw_mass_, *angle_tgt),
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
        // Stop in-progress moves (check BEFORE clearing status)
        if (IsFrictionStage(status_)) {
            pending_stop_ = true;
        }

        status_ = Motion::CalibrationStatus::kIdle;

        pending_move_.reset();
        pending_sweep_.reset();

        sampler_.Reset();
        calibration_progress_.clear();

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
                sweep.vel = fr.max_vel * kFrictionVelFractions.at(fr.vel_idx);
                sweep.ramp_dist = RampDist(sweep.vel, fr.inertia);

                pending_sweep_ = sweep;
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
std::string Libra2MotionController::DescribeCalibrationStage() const {
    const FrictionRun& fr = friction_run_;
    const auto n_vels = static_cast<int>(kFrictionVelFractions.size());
    const std::string at_rest = "averaging torque once system at rest for "
                                + Format(kRestWindowTime) + " s";

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
                                      + FormatDeg(at_a ? angle_a_ : angle_b_)
                                      + " deg"
                                : "at " + label + ", " + at_rest;
                break;
            }
        case Motion::CalibrationStatus::kFrictionMovingToStart:
            name = "Friction";
            step = 1;
            total = 1 + (2 * n_vels);
            action = "moving to window start " + FormatDeg(fr.angle_lo)
                     + " deg";
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
                         + std::to_string(n_vels) + " (" + FormatDeg(vel)
                         + " deg/s), sweeping " + (going_up ? "up" : "down")
                         + " to "
                         + FormatDeg(going_up ? fr.angle_hi : fr.angle_lo)
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
 * @param dt Time since last cycle (s)
 * @param timeout Time limit of active step (s)
 * @param angle Actual angle, θ (rad)
 * @param actual_vel Actual velocity, θ̇ (rad/s)
 * @param trajectory_active Whether a trajectory is running
 */
void Libra2MotionController::LogCalibrationProgress(
    const double& dt, const double& timeout, const double& angle,
    const double& actual_vel, const bool& trajectory_active) {
    t_progress_log_s_ += dt;
    if (t_progress_log_s_ < kProgressLogPeriod) {
        return;
    }
    t_progress_log_s_ = 0.0;

    logger_->Debug(calibration_progress_ + "\n  still running: "
                   + std::to_string(static_cast<int>(t_stage_s_)) + " of "
                   + std::to_string(static_cast<int>(timeout))
                   + " s before timeout"
                     "\n  angle:         "
                   + FormatDeg(angle)
                   + " deg"
                     "\n  speed:         "
                   + FormatDeg(actual_vel)
                   + " deg/s"
                     "\n  trajectory:    "
                   + (trajectory_active ? "running" : "finished"));
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
 * @brief Converts an averaged rest sample into the arm's own gravitational
 *        torque (Nm).
 *
 * @param sample Averaged measurements
 *
 * @note At rest, measured torque is the opposite of actual actuator torque, per
 *       Eq. 19b. Using Eq. 29a removes the CW torque, and with Eq. 28 for CW
 *       mass:
 *         τ_meas(θ) = -τ_actuator
 *         τ_g1(θ) = τ_meas(θ) - τ_g2(θ)
 */
double Libra2MotionController::SampleToArmTorque(
    const Motion::RestSample& sample) const {
    const double cw_mass = Libra2Model::CwMass(sample.volume);
    const double cw_torque = Libra2Model::CwGravTorque(cw_model_.Moment(cw_mass),
                                                       cw_model_.HangMoment(
                                                           cw_mass),
                                                       kGrav, sample.angle);

    return Libra2Model::ArmGravTorque(-sample.torque, cw_torque);
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

    // NOTE: Code-specific equation. S is linear in m_cw, so a sign change
    //       between empty and full has one crossover:
    //         V_cross = m_cap·g·S_empty / (g·S_empty - g·S_full) / ρ_water
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
 * @param angle Actual angle, θ (rad)
 * @param actual_vel Actual velocity, θ̇ (rad/s)
 * @param actual_torque Actual actuator torque, τ_actuator (Nm)
 */
void Libra2MotionController::RunArmCalibration(const double& dt,
                                               const bool& trajectory_active,
                                               const double& angle,
                                               const double& actual_vel,
                                               const double& actual_torque) {
    t_stage_s_ += dt;

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
    LogCalibrationProgress(dt, timeout, angle, actual_vel, trajectory_active);

    // Sampling stages: average once steady
    std::optional<Motion::RestSample> sample;
    if (!moving_stage) {
        // Sampling is invalid if pose drifts (except for level calibration)
        if (status_ != Motion::CalibrationStatus::kArmSamplingLevel
            && std::abs(angle - target) > 2.0 * kReachTolerance) {
            sampler_.Reset();
            return;
        }

        sample = sampler_.Add(dt, angle, actual_vel, actual_torque,
                              trajectory_active, cw_volume_);
        if (!sample) {
            return;
        }
    }

    switch (status_) {
        case Motion::CalibrationStatus::kArmMovingToA:
        case Motion::CalibrationStatus::kArmMovingToB:
            // Start next step of an active calibration
            if (!trajectory_active
                && std::abs(angle - target) <= kReachTolerance
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
        FinishCalibration(false, "arm is not level (theta="
                                     + Format(sample.angle) + " rad)");
        return;
    }

    // Eq. 34 with current arm_hang_torque_
    const auto level = Libra2Model::ArmLevelTorque(SampleToArmTorque(sample),
                                                   sample.angle,
                                                   arm_hang_torque_);
    if (!level) {
        FinishCalibration(false, "cos(theta) too small");
        return;
    }

    // Convert to min. ETA extension (Eq. 44)
    arm_level_torque_ = *level
                        + (kGrav * Libra2Model::EtaMomentChange(eta_extension_));
    arm_calibrated_ = true;
    arm_source_ = "level";
    arm_cw_params_ = cw_model_.GetParams();

    FinishCalibration(true,
                      "Level calibration succeeded: τ_g1,0="
                          + Format(arm_level_torque_) + " Nm"
                          + (eta_extension_ > 0.0 ? " (at min. extension)" : "")
                          + ", m_arm*h_1*g=" + Format(arm_hang_torque_) + " Nm");
}

/**
 * @brief Solves the arm model from the samples at both angles and applies it.
 *
 * @param sample_b Averaged sample at the second angle (`sample_a_` is already
 *                 stored internally)
 *
 * @note τ_g1,0 is converted to min. ETA extension (see
 *       `LevelTorqueAtExtension()`).
 *
 * @see Libra2Model::SolveTwoAngle Libra2Model::EtaMomentChange
 */
void Libra2MotionController::FinishTwoAngleCalibration(
    const Motion::RestSample& sample_b) {
    // Eq. 35 with measured angles
    const auto solved = Libra2Model::SolveTwoAngle(SampleToArmTorque(sample_a_),
                                                   sample_a_.angle,
                                                   SampleToArmTorque(sample_b),
                                                   sample_b.angle);
    if (!solved) {
        FinishCalibration(false, "measured angles too close together");
        return;
    }

    // Convert to min. ETA extension (Eq. 44)
    arm_level_torque_ = solved->level_torque
                        + (kGrav * Libra2Model::EtaMomentChange(eta_extension_));
    arm_hang_torque_ = solved->hang_torque;
    arm_calibrated_ = true;
    arm_source_ = "two-angle";
    arm_cw_params_ = cw_model_.GetParams();
    const bool stable = CheckStability(arm_hang_torque_);

    FinishCalibration(true,
                      "Two-angle calibration succeeded: τ_g1,0="
                          + Format(arm_level_torque_) + " Nm"
                          + (eta_extension_ > 0.0 ? " (at min. extension)" : "")
                          + ", m_arm*h_1*g=" + Format(arm_hang_torque_) + " Nm"
                          + (stable ? "" : " (UNSTABLE; see log)"));
}

//------------------------------------------------------------------------------
// !Friction Calibration Helpers
//------------------------------------------------------------------------------

/**
 * @brief Advances the running friction calibration by one cycle.
 *
 * @param dt Time since last cycle (s)
 * @param trajectory_active Whether a trajectory is running
 * @param angle Actual angle, θ (rad)
 * @param actual_vel Actual velocity, θ̇ (rad/s)
 * @param actual_torque Actual actuator torque, τ_actuator (Nm)
 *
 * @note Aborts (and stops the sweep) if the torque stays above
 *       `kMaxFrictionTorque` for `kMaxFrictionTorqueTime`, or if the CW volume
 *       changes.
 */
void Libra2MotionController::RunFrictionCalibration(
    const double& dt, const bool& trajectory_active, const double& angle,
    const double& actual_vel, const double& actual_torque) {
    FrictionRun& run = friction_run_;
    t_stage_s_ += dt;

    // Brief spikes shouldn't count against the limit
    if (std::abs(actual_torque) > kMaxFrictionTorque) {
        run.t_over_max_s += dt;
    } else {
        run.t_over_max_s = 0.0;
    }

    // Abort guards
    std::string abort_reason;
    if (t_stage_s_ > kMoveTimeout) {
        abort_reason = "timed out moving or sweeping";
    } else if (run.t_over_max_s >= kMaxFrictionTorqueTime) {
        abort_reason = "torque (" + Format(actual_torque)
                       + " Nm) stayed above the calibration's torque limit ("
                       + Format(kMaxFrictionTorque) + " Nm) for "
                       + Format(kMaxFrictionTorqueTime)
                       + " s; balance the CW or narrow/move the window";
    } else if (!cw_volume_
               || std::abs(*cw_volume_ - run.volume) > kRestVolumeTolerance) {
        abort_reason = "CW volume changed during the sweeps";
    }

    if (!abort_reason.empty()) {
        pending_stop_ = true;
        FinishCalibration(false, abort_reason);
        return;
    }
    LogCalibrationProgress(dt, kMoveTimeout, angle, actual_vel,
                           trajectory_active);

    const bool going_up = (status_
                           == Motion::CalibrationStatus::kFrictionSweepUp);
    const double end =
        (status_ == Motion::CalibrationStatus::kFrictionSweepDown
         || status_ == Motion::CalibrationStatus::kFrictionMovingToStart)
            ? run.angle_lo
            : run.angle_hi;
    const double vel = run.max_vel * kFrictionVelFractions.at(run.vel_idx);

    if (status_ != Motion::CalibrationStatus::kFrictionMovingToStart) {
        if (trajectory_active) {
            run.trajectory_seen = true;
            AddFrictionSample(dt, angle, actual_vel, actual_torque, going_up,
                              vel);
            return;
        }
        if (!run.trajectory_seen) {
            return;  // sweep requested but not started yet
        }
    }

    // Wait until the move/sweep has ended at rest at its end
    const bool arrived = !trajectory_active
                         && std::abs(angle - end) <= kReachTolerance
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
                if (!FinishFrictionVel(vel, error)) {
                    FinishCalibration(false, error);
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
            }

        default:
            break;
    }
}

/**
 * @brief Bins a friction sweep sample if it is inside the sampled range and at
 *        steady speed.
 *
 * @param dt Time since last cycle (s)
 * @param angle Actual angle, θ (rad)
 * @param actual_vel Actual velocity, θ̇ (rad/s)
 * @param actual_torque Actual actuator torque, τ_actuator (Nm)
 * @param going_up Whether this is the up sweep
 * @param vel Sweep speed, v (rad/s)
 *
 * @note Steady speed is judged on velocity averaged over `kVelAvgWindow`, since
 *       raw velocity feedback tends to jitter above `kFrictionVelTolerance`
 *       (even when the sweep is steady).
 */
void Libra2MotionController::AddFrictionSample(
    const double& dt, const double& angle, const double& actual_vel,
    const double& actual_torque, const bool& going_up, const double& vel) {
    FrictionRun& run = friction_run_;

    // Skip window edges, where ramps add inertia and CW may swing
    const double lo = run.angle_lo + run.margin;
    const double hi = run.angle_hi - run.margin;
    if (angle < lo || angle > hi) {
        return;  // near a window edge
    }
    ++run.range_samples;

    // Skip sudden jerks and jitters, which add error to calibration result
    const double mean_vel = run.vel_avg.Add(dt, actual_vel);
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

    const auto bin = std::min(n_bins - 1, static_cast<std::size_t>(
                                              (angle - lo) / kFrictionBinWidth));
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
 * @param vel Sweep speed, v (rad/s)
 * @param[out] error Descriptive reason for failure, if any
 *
 * @note Code-specific equation based on Eqs. 18 and 19a at constant speed.
 *       Per angle bin, with actual actuator torque (so bin mean = -τ_ext,g):
 *         drag = (τ_up - τ_down) / 2 τ_g(θ) = (τ_up + τ_down) / 2
 */
bool Libra2MotionController::FinishFrictionVel(const double& vel,
                                               std::string& error) {
    FrictionRun& run = friction_run_;

    const bool model_ready = arm_calibrated_ && cw_model_.IsValid();
    const double lo = run.angle_lo + run.margin;
    const double cw_mass = Libra2Model::CwMass(run.volume);

    Motion::FrictionPoint point;
    point.vel = vel;
    double drag_min = 0.0;
    double drag_max = 0.0;
    double grav_sum = 0.0;
    double model_err_sum = 0.0;

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
        const double angle_bin = lo
                                 + ((static_cast<double>(i) + 0.5)
                                    * kFrictionBinWidth);
        Motion::FrictionBin bin;
        bin.angle = angle_bin;
        bin.drag = drag;
        bin.holding = holding;
        bin.n_up = run.up_count[i];
        bin.n_down = run.down_count[i];
        point.bins.push_back(bin);

        // Holding torque should equal -τ_ext,g at bin center
        if (model_ready) {
            model_err_sum += holding + ComputeNetGravTorque(angle_bin, cw_mass);
        }

        ++point.n_bins;
    }

    if (point.n_bins < kFrictionMinBins) {
        error = "only " + std::to_string(point.n_bins)
                + " angle bins had steady samples in both directions at "
                + Format(vel) + " rad/s (need "
                + std::to_string(kFrictionMinBins) + "); use a wider window";
        return false;
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

    std::string msg = "Friction speed " + std::to_string(run.vel_idx + 1) + "/"
                      + std::to_string(kFrictionVelFractions.size()) + " done ("
                      + FormatDeg(vel)
                      + " deg/s):"
                        "\n  drag:           "
                      + Format(point.drag) + " Nm (spread "
                      + Format(point.spread) + " Nm over "
                      + std::to_string(point.n_bins)
                      + " bins)"
                        "\n  steady samples: "
                      + std::to_string(steady_pct)
                      + "% of samples in the sampled range"
                        "\n  holding torque: "
                      + Format(grav_sum / n) + " Nm (mean)";
    if (model_ready) {
        msg += "\n  model error:    " + Format(model_err_sum / n)
               + " Nm (mean)";
    }
    logger_->Debug(msg);

    return true;
}

/**
 * @brief Fits the friction line through all speeds and applies it.
 *
 * @note Code-specific rule. A negative slope means friction is flat within
 *       noise, so b = 0 and τ_c is the mean drag. A negative τ_c within the
 *       largest bin spread means friction is too small to measure, so τ_c = 0
 *       and b is refit through the origin; otherwise, the calibration fails.
 */
void Libra2MotionController::FinishFrictionCalibration() {
    const FrictionRun& run = friction_run_;

    std::vector<double> vels;
    std::vector<double> drags;
    for (const auto& p : run.points) {
        vels.push_back(p.vel);
        drags.push_back(p.drag);
    }

    auto fit = Libra2Model::FitFrictionLine(vels, drags);
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
    }

    if (model.coulomb < 0.0) {
        // Bin spread is the drag noise floor
        double noise = 0.0;
        for (const auto& p : run.points) {
            noise = std::max(noise, p.spread);
        }

        if (-model.coulomb > noise) {
            FinishCalibration(false, "fitted tau_c is negative ("
                                         + Format(model.coulomb)
                                         + " Nm), past the bin spread ("
                                         + Format(noise)
                                         + " Nm); check the run");
            return;  // nonsensical result
        }

        logger_->Info("Fitted tau_c is negative (" + Format(model.coulomb)
                      + " Nm) but within bin spread (" + Format(noise)
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
