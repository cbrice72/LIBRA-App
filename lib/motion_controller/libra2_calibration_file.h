/******************************************************************************
 * @file   libra2_calibration_file.h
 * @brief  Saves/loads LIBRA-II calibration records as JSON; header file.
 *
 * @author Christian Brice
 ******************************************************************************/

#pragma once

// C++ Standard Library Headers
#include <optional>
#include <string>

// Project Headers
#include "motion_defs.h"

namespace Motion {

// Default folder for calibration files (relative to the run directory)
constexpr const char* kCalibrationDir = "./calibration";

bool SaveCalibrationFile(const CalibrationRecord& record,
                         const std::string& file_path, std::string& error);
std::optional<CalibrationRecord> LoadCalibrationFile(
    const std::string& file_path, std::string& error);

}  // namespace Motion
