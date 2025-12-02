################################################################################
# @file   EposSetup.cmake
# @brief  CMake module for loading EPOS libraries packaged with this project.
#
# @author brice.c.aa
################################################################################

# ========================= CMAKE SETUP =========================

# NOTE: There is no `add_subdirectory()` call for EPOS since it doesn't provide
#       a CMakeLists.txt, only a binary to link against
set(EPOS_INCLUDE_DIRS
    "${EPOS_DIR}/include"
    CACHE PATH "Path to EPOS SDK include directories" FORCE
)
