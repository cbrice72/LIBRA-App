#!/usr/bin/env bash

set -e  # exit on any error

# ==================== LOCAL VARIABLES AND FUNCTIONS ====================

PROJECT_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
LIBRA_ROS2_TOOLS_DIR="$PROJECT_ROOT/LIBRA-ROS2-Tools"
ROS2_WS_DIR="$LIBRA_ROS2_TOOLS_DIR/ros2_ws"
INSTALL_DIR="$ROS2_WS_DIR/install"

LIBRA_VERSION="${LIBRA_VERSION:-1}"
BUILD_WITH_ROS2="ON"

CLEAN_BUILD=false

# Helper function for colorized output
color_echo() {
  local level=$1
  shift
  local msg="$*"

  case "$level" in
    INFO)
      # bold-cyan label + cyan message
      echo -e "\033[1;36m[INFO]\033[0;36m $msg\033[0m"
      ;;
    WARN)
      # bold-yellow label + yellow message
      echo -e "\033[1;33m[WARN]\033[0;33m $msg\033[0m"
      ;;
    ERROR)
      # bold-red label + red message
      echo -e "\033[1;31m[ERROR]\033[0;31m $msg\033[0m"
      ;;
    *)
      echo "[UNKNOWN] $msg"
      ;;
  esac
}

# Helper function to display usage information
usage() {
  color_echo INFO "Usage: $0 [-v|--version 1|2] [--disable-ros2] [-c|--clean]"
}

# =================== PRE-PROCESSING ====================

START_TIME=$(date +%s.%N)

# =================== (1) PARSE COMMAND-LINE ARGS ====================

while [[ $# -gt 0 ]]; do
  case "$1" in
    -v|--version)
      if [[ $# -lt 2 ]]; then
        color_echo ERROR "Missing value for $1."
        usage
        exit 1
      fi
      LIBRA_VERSION="$2"
      shift 2
      ;;
    --disable-ros2)
      BUILD_WITH_ROS2="OFF"
      shift
      ;;
    -c|--clean)
      CLEAN_BUILD=true
      shift
      ;;
    -h|--help)
      usage
      exit 0
      ;;
    *)
      color_echo ERROR "Unknown option: $1"
      usage
      exit 1
      ;;
  esac
done

if [[ "$BUILD_WITH_ROS2" == "ON" ]]; then
  ROS_STATUS="with"
else
  ROS_STATUS="without"
fi

color_echo INFO "The LIBRA App will be built for LIBRA-$LIBRA_VERSION $ROS_STATUS ROS2 support."
sleep 2;  # let user see this message

# ==================== (2) CLEAN BUILD ARTIFACTS ====================

if [[ "$CLEAN_BUILD" == true ]]; then
#-------------------------
  color_echo INFO "Cleaning build artifacts..."

  rm -rf "$PROJECT_ROOT/build"
  if [[ -d "$ROS2_WS_DIR" ]]; then
    rm -rf "$ROS2_WS_DIR/build" "$ROS2_WS_DIR/install" "$ROS2_WS_DIR/log"
  fi

  color_echo INFO "Cleaning build artifacts... SUCCESS"
#-------------------------
fi

# ==================== (3) BUILD LIBRA-ROS2-TOOLS (COLCON) ====================

if [[ "$BUILD_WITH_ROS2" == "ON" ]]; then
#-------------------------
  if [ ! -d "$LIBRA_ROS2_TOOLS_DIR" ] || [ ! "$(ls -A "$LIBRA_ROS2_TOOLS_DIR" 2>/dev/null)" ]; then
    color_echo INFO "Initializing LIBRA-ROS2-Tools submodule..."

    git submodule update --init --recursive

    color_echo INFO "Initializing LIBRA-ROS2-Tools submodule... SUCCESS"
  fi
#-------------------------
  color_echo INFO "Sourcing ROS2 Humble environment..."

  source /opt/ros/humble/setup.bash 2>/dev/null || {
    color_echo ERROR "> Unable to source a ROS2 Humble environment!\n          Please ensure it is installed at \"/opt/ros/humble\"."
    exit 1
  }

  color_echo INFO "Sourcing ROS2 Humble environment... SUCCESS"
#-------------------------
  color_echo INFO "Building ROS2 workspace..."

  (cd "$ROS2_WS_DIR" && colcon build --packages-up-to libra)

  color_echo INFO "Building ROS2 workspace... SUCCESS"
#-------------------------
  color_echo INFO "Sourcing LIBRA ROS Tools environment..."

  source "$INSTALL_DIR/setup.bash" 2>/dev/null || {
    color_echo ERROR "> Unable to source the LIBRA ROS Tools environment!\n          See colcon output for errors."
    exit 1
  }

  color_echo INFO "Sourcing LIBRA ROS Tools environment... SUCCESS"
#-------------------------
else
  color_echo INFO "ROS2 integration disabled; skipping LIBRA-ROS2-Tools build"
fi

# ==================== (4) BUILD LIBRA-APP (CMAKE) ====================

#-------------------------
color_echo INFO "Configuring LIBRA App..."

if [[ "$BUILD_WITH_ROS2" == "ON" ]]; then
  BUILD_TYPE="ros"
else
  BUILD_TYPE="standalone"
fi
BUILD_DIR="$PROJECT_ROOT/build"
cmake -S "$PROJECT_ROOT" -B "$BUILD_DIR" -DLIBRA_VERSION="$LIBRA_VERSION" -DBUILD_WITH_ROS2="$BUILD_WITH_ROS2"

color_echo INFO "Configuring LIBRA App... SUCCESS"
#-------------------------
color_echo INFO "Building LIBRA App..."

cmake --build "$BUILD_DIR" --target all
EXE_PATH="$BUILD_DIR/libra-${LIBRA_VERSION}-${BUILD_TYPE}/libra_app"
if [ ! -f "$EXE_PATH" ]; then
  color_echo ERROR "> \"libra_app\" was not installed to $(dirname "$EXE_PATH")!\n          Check CMake build output for errors."
  exit 1
elif [ ! -x "$EXE_PATH" ]; then
  color_echo ERROR "> \"libra_app\" exists at $EXE_PATH but is not executable!\n          Check file permissions."
  exit 1
fi

color_echo INFO "Building LIBRA App... SUCCESS"
#-------------------------

# ==================== POST-PROCESSING ====================

END_TIME=$(date +%s.%N)
ELAPSED_TIME=$(echo "$END_TIME - $START_TIME" | bc -l)
F_ELAPSED_TIME=$(printf "Script finished in %.2f s" "$ELAPSED_TIME")
color_echo INFO "$F_ELAPSED_TIME"
echo ""
color_echo INFO "Run the app with the following command:"
if [[ "$BUILD_WITH_ROS2" == "ON" ]]; then
  echo "source LIBRA-ROS2-Tools/ros2_ws/install/setup.bash && $EXE_PATH"
else
  echo "$EXE_PATH"
fi
