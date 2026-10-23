/******************************************************************************
 * @file   flow_controller.cpp
 * @brief  Flow Arduino (flow sensor system) device management;
 *         implementation file.
 *
 * @author Christian Brice
 ******************************************************************************/

// Related Header
#include "flow_controller.h"

// C++ Standard Library Headers
//   (none)

// Other Library Headers
#include <QByteArray>  // Qt::Core

// Project Headers
//   (none)

/* --- TABLE OF CONTENTS ---
 * !Class Management
 * !Class Helpers
 * !Arduino Commands (slots)
 */

//------------------------------------------------------------------------------
// !Class Management
//------------------------------------------------------------------------------

/**
 * @brief Standard constructor.
 *
 * @param parent Owning Qt widget
 * @param debug_mode Whether to output verbose debug text
 */
FlowController::FlowController(QObject* parent, const bool& debug_mode)
    : GenericSerialDevice(parent, debug_mode, "Flow") {}

//------------------------------------------------------------------------------
// !Class Helpers
//------------------------------------------------------------------------------

/**
 * @brief Reads the current flow values (in L/s) from the FlowArduino.
 */
void FlowController::OnUpdate() {
    // Read all lines (in case loops misalign); keep only most recent
    QString str_data;
    while (serial_port_->canReadLine()) {
        const QString line = QString::fromUtf8(serial_port_->readLine())
                                 .trimmed();
        if (line.isEmpty() || line.startsWith('?')) {
            continue;  // skip blanks and device-ID query replies
        }
        str_data = line;
    }
    if (str_data.isEmpty()) {
        return;
    }

    // Prepare data for parsing
    const QStringList str_values = str_data.split(',');
    if (str_values.size() != 2) {
        logger_->Warn("Malformed flow data payload (\"" + str_data.toStdString()
                      + "\"); ignoring");
        return;
    }

    // Parse and validate data
    auto parse_flow_value = [this, &str_data](const QString& str,
                                              const char* field_name) -> double {
        bool ok = false;
        const double val = str.toDouble(&ok);
        if (!ok) {
            logger_->Warn(std::string("Invalid ") + field_name + " field (\""
                          + str.toStdString()
                          + "\") in payload: " + str_data.toStdString());
            return -1.0;  // indicate fault
        }
        return val;
    };

    const double inflow = parse_flow_value(str_values[0], "inflow");
    const double outflow = parse_flow_value(str_values[1], "outflow");

    // Resolve into a single signed reading
    double flow;
    if (inflow == -1.0 || outflow == -1.0) {
        flow = -1.0;  // sensor fault, malformed data
    } else if (outflow > 0.0) {
        // Outflow sensor is guaranteed to not have flow disturbances,
        // so it is safe to prioritize it over the inflow sensor
        flow = -outflow;
    } else {
        flow = inflow;
    }

    emit ReportStatus(flow);
}

//------------------------------------------------------------------------------
// !Arduino Commands (slots)
//------------------------------------------------------------------------------

// NOTE: FlowArduino is a read-only device.
