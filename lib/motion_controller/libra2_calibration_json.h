/******************************************************************************
 * @file   libra2_calibration_json.h
 * @brief  Saves/loads LIBRA-II calibration records as JSON; header file.
 *
 * @author Christian Brice
 ******************************************************************************/

#pragma once

// C++ Standard Library Headers
#include <string>

// Project Headers
#include "motion_defs.h"

namespace Motion {

// Default folder for calibration files (relative to the run directory)
constexpr const char* kCalibrationDir = "./calibration";

void SaveCalibrationJson(const CalibrationRecord& record,
                         const std::string& file_path);
[[nodiscard]] CalibrationRecord LoadCalibrationJson(const std::string& file_path);

}  // namespace Motion
