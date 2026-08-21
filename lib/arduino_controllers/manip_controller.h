/******************************************************************************
 * @file   manip_controller.h
 * @brief  Manip Arduino (tip servos) device management; header file.
 *
 * @author Christian Brice
 ******************************************************************************/

#pragma once

// C++ Standard Library Headers
#include <array>

// Other Library Headers
// (none)

// Project Headers
#include "generic_serial_device.h"

/**
 * @brief Controls LIBRA-I's 3-servo, 2-DoF manipulator via serial connection.
 *
 * @note This manipulator is no longer installed on LIBRA-I due to it being too
 *       weak to lift the sensor suite.
 */
class ManipController : public GenericSerialDevice {
    // NOLINTBEGIN: required by Qt
    Q_OBJECT
    // NOLINTEND

  public:
    explicit ManipController(QObject* parent, const bool& debug_mode);
    ~ManipController() override = default;

  public slots:
    // --- Arduino Commands ---

    void EnablePitchCorrection(const bool& enabled);
    void UpdatePitchFeedback(const double& pitch);

    void SetTarget(const double& pan, const double& tilt, const bool& move_slow);

  signals:
    // --- Arduino Updates ---

    void ReportPosition(const double& base, const double& pan,
                        const double& tilt);

  protected:
    void OnUpdate() override;

  private:
    // --- Helper Functions ---

    // --- Data Members ---

    std::array<double, 3> current_pos_{0};
    std::array<double, 3> target_pos_{0};

    std::array<int, 3> slow_direction_{0};

    bool manip_correction_enabled_{true};
};
