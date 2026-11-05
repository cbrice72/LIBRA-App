/******************************************************************************
 * @file   libra2_model.cpp
 * @brief  Physical approximation of LIBRA-II; implementation file.
 *
 * @author Christian Brice
 ******************************************************************************/

// Related Header
#include "libra2_model.h"

// C++ Standard Library Headers
#include <algorithm>
#include <array>
#include <cmath>

/* --- TABLE OF CONTENTS ---
 * !Counterweight Model
 * !Equations
 */

using Libra2Model::kWaterDensity;

// ETA (Eq. 44; unverified)

// - Tubes u_1 (smallest) to u_15
constexpr std::array<double, 15> kEtaTubeMass = {
    0.0386, 0.0450, 0.0592, 0.0674, 0.0755, 0.0847, 0.0932, 0.0995,
    0.1147, 0.1233, 0.1306, 0.1389, 0.1520, 0.1550, 0.1608,
};  // m_ui (kg)
constexpr std::array<double, 15> kEtaTubeLength = {
    0.600, 0.600, 0.640, 0.640, 0.640, 0.640, 0.640, 0.640,
    0.640, 0.640, 0.640, 0.640, 0.640, 0.640, 0.640,
};                                         // L_ui (m)
constexpr double kEtaTubeOverlap = 0.150;  // L_ov (m)

// - Spiral conduit
constexpr double kEtaConduitMassPerLength = 0.182;  // μ_conduit (kg/m)
// TODO: input after measuring
constexpr double kEtaConduitMinLength = 0.0;  // L_conduit,min (m)

// - Cables fed out with the conduit
constexpr double kCameraCableMassPerLength = 0.0642;  // RealSense D456 (kg/m)
// TODO: input after measuring
constexpr double kCameraCableMinLength = 0.0;        // RealSense D456 (m)
constexpr double kLidarCableMassPerLength = 0.0292;  // SF45/B (kg/m)
// TODO: input after measuring
constexpr double kLidarCableMinLength = 0.0;  // SF45/B (m)

constexpr double kSensorSuiteMass = 0.2917;  // D456 + SF45/B w/ mounts (kg)
constexpr double kEtaTipMountMass = 0.0215;  // (kg)
constexpr double kEtaTipMass = kSensorSuiteMass
                               + kEtaTipMountMass;  // m_tip (kg)

constexpr double kMinCosForBalance = 1e-3;  // |cosθ| floor, Eq. 34/38'

//------------------------------------------------------------------------------
// !Counterweight Model
//------------------------------------------------------------------------------

/**
 * @brief Builds and checks the counterweight (CW) model (pivot distance vs.
 *        volume).
 *
 * @param params CW geometry; `pivot_dist` sign is ignored, and `pivot_dist` = 0
 *               or `mean_sq_dist` <= 0 means "not measured yet" (leaves model
 *               invalid)
 *
 * @note Extension of Eq. 14b, where r_P varies linearly with volume:
 *         r_P(V) = A + B·V.
 */
CwModel::CwModel(const Motion::CwModelParams& params)
    : capacity_(params.capacity) {
    // Validate params
    if (!std::isfinite(params.capacity) || params.capacity <= 0.0) {
        error_ = "CW capacity must be positive";
        return;
    }
    if (!std::isfinite(params.pivot_dist) || !std::isfinite(params.pivot_slope)
        || !std::isfinite(params.mean_sq_dist)
        || !std::isfinite(params.mean_sq_dist_slope)
        || !std::isfinite(params.hang_offset)) {
        error_ = "One or more of r_P components, mean r^2 components, or q_P "
                 "is not a number";
        return;
    }
    if (params.pivot_dist == 0.0) {
        error_ = "r_P is not set (intercept = 0)";
        return;
    }
    if (params.mean_sq_dist <= 0.0) {
        error_ = "mean r^2 is not set (intercept <= 0)";
        return;
    }

    // Set model properties
    r_P_0_ = std::abs(params.pivot_dist);
    dr_P_dV_ = params.pivot_slope;
    r2_0_ = params.mean_sq_dist;
    dr2_dV_ = params.mean_sq_dist_slope;
    q_P_ = params.hang_offset;

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
// !Equations
//------------------------------------------------------------------------------

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
 * @note From Eq. 44 (see extended equations in `ARM_EXTENSION_ESTIMATION.md`),
 *       where ΔL = ΔL_conduit since the conduit is assumed to be fed in from
 *       the pitch axis:
 *         Δ(m_arm·r_1) = (tubes u_1-15) + (conduit, cables) + (tip, payload)
 *                      = Σ m_ui·max(0, ΔL - S_(i-1))
 *                        + Σ μ_k·(L_k,min·ΔL + ΔL²/2)
 *                        + m_tip·ΔL
 *
 *       Tubes are assumed to slide out smallest-first, so tube u_i only moves once
 *       the tubes inside it have used up their strokes:
 *         S_(i-1) = Σ_(j<i) (L_uj - L_ov)
 *
 *       Camera and LiDAR cables (k = cam, lidar) are a code-specific addition,
 *       assumed to change at the same rate as the conduit (k = conduit).
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

    // Conduit, cables
    const auto fed_out_moment = [&](const double& mass_per_length,
                                    const double& min_length) {
        return mass_per_length
               * ((min_length * extension) + (0.5 * extension * extension));
    };
    moment += fed_out_moment(kEtaConduitMassPerLength, kEtaConduitMinLength);
    moment += fed_out_moment(kCameraCableMassPerLength, kCameraCableMinLength);
    moment += fed_out_moment(kLidarCableMassPerLength, kLidarCableMinLength);

    // Tip cap, payload
    moment += kEtaTipMass * extension;
    return moment;
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
 * @brief Computes the effective CW moment (kg·m) that balances the arm at a
 *        given angle.
 *
 * @param arm_level_torque Arm gravitational torque at level, τ_g1,0 (Nm)
 * @param arm_hang_torque Arm hang torque, m_arm·h_1·g (Nm)
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
                                      const double& angle) {
    if (std::abs(std::cos(angle)) < kMinCosForBalance) {
        return std::nullopt;
    }
    return -(arm_level_torque + (arm_hang_torque * std::tan(angle))) / kGrav;
}

}  // namespace Libra2Model
