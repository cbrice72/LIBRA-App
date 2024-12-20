################################################################################
# @file   CMakeLists.txt
# @brief  Project threads CMakeLists file.
#
# @author brice.c.aa
################################################################################

# Specify subdirectory files for use by parent CMakeLists.txt
set(THREAD_SOURCE
    # Actuator Definitions
    ${CMAKE_CURRENT_SOURCE_DIR}/actuator_defs.h
    ${CMAKE_CURRENT_SOURCE_DIR}/abstract_actuator_thread.h
    # EPOS (Maxon)
    ${CMAKE_CURRENT_SOURCE_DIR}/epos_thread.h
    ${CMAKE_CURRENT_SOURCE_DIR}/epos_thread.cpp
    # HEBI
    ${CMAKE_CURRENT_SOURCE_DIR}/hebi_thread.h
    ${CMAKE_CURRENT_SOURCE_DIR}/hebi_thread.cpp
    # Logging
    ${CMAKE_CURRENT_SOURCE_DIR}/log_thread.h
    ${CMAKE_CURRENT_SOURCE_DIR}/log_thread.cpp
    PARENT_SCOPE
)