/******************************************************************************
 * @file   flow_controller.h
 * @brief  Flow Arduino (flow sensor system) device management; header file.
 *
 * @author Christian Brice
 ******************************************************************************/

#pragma once

// C++ Standard Library Headers
// (none)

// Other Library Headers
// (none)

// Project Headers
#include "abstract_serial_device.h"

/**
 * @brief Controls the flow sensor system via serial connection.
 */
class FlowController : public AbstractSerialDevice {
    // NOLINTBEGIN: required by Qt
    Q_OBJECT
    // NOLINTEND

  public:
    explicit FlowController(QObject* parent, const bool& debug_mode);
    ~FlowController() override = default;

  public slots:
    // --- Arduino Commands ---

    // TODO

  signals:
    // --- Arduino Updates ---

    // TODO

  protected:
    void OnUpdate() override;

  private:
    // --- Helper Functions ---

    // TODO

    // --- Data Members ---

    // TODO
};
