/******************************************************************************
 * @file   utility.h
 * @brief  Namespace for convenient, general-use functions; header file.
 *
 * @author brice.c.aa
 ******************************************************************************/

#pragma once

// C++ Standard Library Headers
#include <string>
// Other Libraries' Headers
//   (none)
// Project Headers
//   (none)

namespace util {

void PrintEPOSErr(std::string func_name, uint err, int node_id = 0);

}  // namespace util
