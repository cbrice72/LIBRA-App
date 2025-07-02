#!/usr/bin/env bash

set -e  # exit on any error

# ==================== LOCAL VARIABLES AND FUNCTIONS ====================

PROJECT_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
LIBRA_ROS2_TOOLS_DIR="$PROJECT_ROOT/LIBRA-ROS2-Tools"
ROS2_WS_DIR="$LIBRA_ROS2_TOOLS_DIR/ros2_ws"
INSTALL_DIR="$ROS2_WS_DIR/install"

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

# ==================== SCRIPT START ====================


echo "Building LIBRA App with ROS2 integration..."
sleep 1;  # let user see this message

# ==================== (1) BUILD LIBRA-ROS2-TOOLS (COLCON) ====================

#-------------------------
START_TIME=$(date +%s.%N)
SLEEP_TIME=0

if [ ! -d "$LIBRA_ROS2_TOOLS_DIR" ] || [ ! "$(ls -A "$LIBRA_ROS2_TOOLS_DIR" 2>/dev/null)" ]; then
    color_echo INFO "Initializing LIBRA-ROS2-Tools submodule..."
    git submodule update --init --recursive
fi
#-------------------------
color_echo INFO "Sourcing ROS2 Humble environment..."
sleep 2
SLEEP_TIME=$(echo "$SLEEP_TIME + 2" | bc)

source /opt/ros/humble/setup.bash 2>/dev/null || {
    color_echo ERROR "> Unable to source a ROS2 Humble environment!\n          Please ensure it is installed at \"/opt/ros/humble\"."
    exit 1
}

color_echo INFO "Sourcing ROS2 Humble environment... SUCCESS"
#-------------------------
color_echo INFO "Building ROS2 workspace..."
sleep 2
SLEEP_TIME=$(echo "$SLEEP_TIME + 2" | bc)

cd "$ROS2_WS_DIR"
colcon build --packages-up-to libra

color_echo INFO "Building ROS2 workspace... SUCCESS"
#-------------------------
color_echo INFO "Sourcing LIBRA ROS Tools environment..."
sleep 2
SLEEP_TIME=$(echo "$SLEEP_TIME + 2" | bc)

source $INSTALL_DIR/setup.bash 2>/dev/null || {
    color_echo ERROR "> Unable to source the LIBRA ROS Tools environment!\n          See colcon output for errors."
    exit 1
}

color_echo INFO "Sourcing LIBRA ROS Tools environment... SUCCESS"
#-------------------------

# ==================== (2) BUILD LIBRA-APP (CMAKE) ====================

#-------------------------
color_echo INFO "Building LIBRA App..."
sleep 2
SLEEP_TIME=$(echo "$SLEEP_TIME + 2" | bc)

cd "$PROJECT_ROOT"
mkdir -p build2 && cd build2
cmake ..
cmake --build . --target all

if [ ! -f "libra_app_gui" ]; then
    color_echo ERROR "> \"libra_app_gui\" was not installed to $PROJECT_ROOT/build!\n          Check CMake build output for errors."
    exit 1
elif [ ! -x "libra_app_gui" ]; then
    color_echo ERROR "> \"libra_app_gui\" exists at $PROJECT_ROOT/build but is not executable!\n          Check file permissions."
    exit 1
fi

color_echo INFO "Building LIBRA App... SUCCESS"
#-------------------------
END_TIME=$(date +%s.%N)
ELAPSED_TIME=$(echo "$END_TIME - $START_TIME - $SLEEP_TIME" | bc -l)
F_ELAPSED_TIME=$(printf "Total build time: %.2f s" "$ELAPSED_TIME")
color_echo INFO "$F_ELAPSED_TIME"
#-------------------------

# ==================== SCRIPT END ====================

echo -e "\nYou may now run the app with the following command:"
echo "  source LIBRA-ROS2-Tools/ros2_ws/install/setup.bash && ./build/libra_app_gui"
