#!/usr/bin/env bash

set -e  # exit on any error

PROJECT_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
LIBRA_ROS2_TOOLS_DIR="$PROJECT_ROOT/LIBRA-ROS2-Tools"
ROS2_WS_DIR="$LIBRA_ROS2_TOOLS_DIR/ros2_ws"
INSTALL_DIR="$ROS2_WS_DIR/install"

echo "Building LIBRA App with ROS2 integration..."

# Initialize and build the LIBRA-ROS2-Tools submodule via colcon
if [ ! -d "$LIBRA_ROS2_TOOLS_DIR" ] || [ ! "$(ls -A "$LIBRA_ROS2_TOOLS_DIR" 2>/dev/null)" ]; then
    echo "[INFO] Initializing LIBRA-ROS2-Tools submodule..."
    git submodule update --init --recursive
fi

echo "[INFO] Building ROS2 workspace..."

source /opt/ros/humble/setup.bash 2>/dev/null || {
    echo "[ERROR] > Unable to source a ROS2 Humble environment!"
    echo "          Please ensure it is installed at \"/opt/ros/humble\"."
    exit 1
}

echo "[INFO] > Sourced ROS2 Humble environment"

cd "$ROS2_WS_DIR"
colcon build --packages-up-to libra

source $INSTALL_DIR/setup.bash 2>/dev/null || {
    echo "[ERROR] > Unable to source the LIBRA ROS Tools environment!"
    echo "          See colcon output for errors."
    exit 1
}

echo "[INFO] > Sourced LIBRA ROS Tools environment"

echo "[INFO] ROS2 workspace built successfully!"

# Build the LIBRA-App via CMake
echo "[INFO] Building LIBRA App..."

cd "$PROJECT_ROOT"
mkdir -p build2 && cd build2
cmake ..
cmake --build . --target all

if [ ! -f "libra_app_gui" ]; then
    echo "[ERROR] > \"libra_app_gui\" was not installed to $PROJECT_ROOT/build!"
    echo "          Check CMake build output for errors."
    exit 1
elif [ ! -x "libra_app_gui" ]; then
    echo "[ERROR] > \"libra_app_gui\" exists at $PROJECT_ROOT/build but is not executable!"
    echo "          Check file permissions."
    exit 1
fi

echo "[INFO] LIBRA App built successfully!"

echo "You may now run the app with the following command:"
echo "  source LIBRA-ROS2-Tools/ros2_ws/install/setup.bash && ./build/libra_app_gui"
