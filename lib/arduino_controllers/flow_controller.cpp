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
// (none)

// Other Library Headers
// (none)

// Project Headers
// (none)

/* --- TABLE OF CONTENTS ---
 * !Local Helpers
 * !Class Management
 * !Class Helpers
 * !Arduino Commands (slots)
 */

//------------------------------------------------------------------------------
// !Local Helpers
//------------------------------------------------------------------------------

namespace {}  // namespace

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
    : AbstractSerialDevice(parent, debug_mode, "Flow") {
    // TODO
}

//------------------------------------------------------------------------------
// !Class Helpers
//------------------------------------------------------------------------------

/**
 * @brief TODO: documentation.
 */
void FlowController::OnUpdate() {
    // TODO
}

//------------------------------------------------------------------------------
// !Arduino Commands (slots)
//------------------------------------------------------------------------------
