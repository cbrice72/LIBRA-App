/******************************************************************************
 * @file   colors.h
 * @brief  Colors for programmatic use in the LIBRA App; header-only.
 *
 * @author Christian Brice
 ******************************************************************************/

#pragma once

// C++ Standard Library Headers
#include <cstdio>

// Other Library Headers
#include <QColor>   // Qt::Gui
#include <QString>  // Qt::Core

/* --- TABLE OF CONTENTS ---
 * !Constants
 * !Helper Functions
 */

/**
 * @brief Colorblind-friendly palette for text used in the LIBRA app. Based on
 *        Bang Wong's proposal
 *
 * @see https://doi.org/10.1038/nmeth.1618
 */
namespace Color {

//------------------------------------------------------------------------------
// !Constants
//------------------------------------------------------------------------------

inline constexpr const char* kBlack = "rgb(0, 0, 0)";
inline constexpr const char* kOrange = "rgb(230, 159, 0)";
inline constexpr const char* kLightBlue = "rgb(86, 180, 233)";
inline constexpr const char* kGreen = "rgb(0, 158, 115)";
inline constexpr const char* kYellow = "rgb(240, 228, 66)";
inline constexpr const char* kBlue = "rgb(0, 114, 178)";
inline constexpr const char* kRed = "rgb(213, 94, 0)";
inline constexpr const char* kMagenta = "rgb(204, 121, 167)";

//------------------------------------------------------------------------------
// !Helper Functions
//------------------------------------------------------------------------------

/**
 * @brief Converts a palette color to a QColor, for APIs that don't take
 *        style sheets (e.g., item colors).
 *
 * @param rgb Palette color, as "rgb(r, g, b)"
 */
inline QColor ToQColor(const char* rgb) {
    int r = 0;
    int g = 0;
    int b = 0;
    std::sscanf(rgb, "rgb(%d, %d, %d)", &r, &g, &b);
    return {r, g, b};
}

}  // namespace Color
