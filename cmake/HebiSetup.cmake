################################################################################
# @file   HebiSetup.cmake
# @brief  CMake module for loading HEBI libraries packaged with this project.
#
# @author brice.c.aa
################################################################################

# ========================= CMAKE SETUP =========================

add_subdirectory(${HEBI_DIR})
set(HEBI_INCLUDE_DIRS
    "${HEBI_DIR}/src"
    "${HEBI_DIR}/hebi/include"
    "${HEBI_DIR}/Eigen"
    PARENT_SCOPE
)
