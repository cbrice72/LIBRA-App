/******************************************************************************
 * @file   libra2_model.h
 * @brief  Physical approximation of LIBRA-II (CW and arm gravity); header file.
 *
 * @note   Equation numbers (Eq. ##) refer to the notebook scans in
 *         `LIBRA Archive/2026-10-02 LIBRA-II Controller Equations/`.
 *         All values are in joint space (positive pitch = arm up).
 *
 * @author Christian Brice
 ******************************************************************************/

#pragma once

// C++ Standard Library Headers
#include <optional>
#include <string>

// Other Library Headers
// (none)

// Project Headers
#include "motion_defs.h"

/**
 * @brief Hanging counterweight (CW) model. Its gravitational torque treats the
 *        water bags as one mass at pivot P, and its inertia uses the bags' mean
 *        squared hang distance; both distances shift with water volume.
 *
 * @note Defines the CW's share of the gravitational torque at the pitch joint
 *       (Eq. 12'), the arm's inertia (Eq. 15), and the CW mass that balances
 *       the arm at a given angle (Eq. 38').
 */
class CwModel {
  public:
    explicit CwModel(const Motion::CwModelParams& params);

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
                                         const double& angle) const;
    [[nodiscard]] std::optional<double> MassForMoment(const double& moment,
                                                      const double& angle) const;

    [[nodiscard]] double Inertia(const double& mass) const;
    [[nodiscard]] double InertiaSlope(const double& mass) const;

  private:
    [[nodiscard]] double PivotDist(const double& volume) const;
    [[nodiscard]] double MeanSqDist(const double& volume) const;

    double r_P_0_{0.0};    // r_P at 0 liters, A (m)
    double dr_P_dV_{0.0};  // r_P change per liter, B (m/L)
    double r2_0_{0.0};     // mean r_bag² at 0 liters, C (m²)
    double dr2_dV_{0.0};   // mean r_bag² change per liter, D (m²/L)
    double q_P_{0.0};      // hang-point offset normal to arm, from O (m)

    double capacity_;  // CW capacity (L)

    std::string error_;
};

/**
 * @brief Stateless equations.
 *
 * @see CwModel
 */
namespace Libra2Model {

inline constexpr double kWaterDensity = 1.0;  // ρ_water (kg/L) -- Eq. 28
inline constexpr double kGrav = 9.80665;      // g (m/s²)

[[nodiscard]] double CwMass(const double& volume);
[[nodiscard]] double EtaMomentChange(const double& extension);

[[nodiscard]] std::optional<double> ArmLevelTorque(
    const double& arm_torque, const double& angle,
    const double& arm_hang_torque);
[[nodiscard]] std::optional<Motion::ArmModel> SolveTwoAngle(
    const double& arm_torque_a, const double& angle_a,
    const double& arm_torque_b, const double& angle_b);

[[nodiscard]] std::optional<double> BalanceCwMoment(
    const double& arm_level_torque, const double& arm_hang_torque,
    const double& angle);

}  // namespace Libra2Model
