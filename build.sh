#!/bin/bash
set -e

# 1. Source ROS 2 Jazzy Base
source /opt/ros/jazzy/setup.bash

WORKSPACE_ROOT="/ros2_ws"
PROJECT_ROOT="$WORKSPACE_ROOT/src/Mobile-Robot-Collision"
MUJOCO_SRC_DIR="$PROJECT_ROOT/src/mujoco_ros2_control"

cd $WORKSPACE_ROOT

# --- AUTOMATIC SETUP (Only runs if folders are missing) ---
if [ ! -d "$MUJOCO_SRC_DIR" ] || [ ! -d "lodepng_manual" ]; then
    echo ">>> Initial setup: Downloading MuJoCo bridge and fixing dependencies..."
    mkdir -p "$PROJECT_ROOT/src"
    wget -qO mujoco_bridge.zip https://github.com/ros-controls/mujoco_ros2_control/archive/refs/heads/main.zip
    unzip -q mujoco_bridge.zip && mv mujoco_ros2_control-main "$MUJOCO_SRC_DIR" && rm mujoco_bridge.zip
    
    mkdir -p lodepng_manual
    curl -sL https://raw.githubusercontent.com/lvandeve/lodepng/master/lodepng.cpp -o lodepng_manual/lodepng.cpp
    curl -sL https://raw.githubusercontent.com/lvandeve/lodepng/master/lodepng.h -o lodepng_manual/lodepng.h
    echo 'add_library(lodepng STATIC lodepng.cpp lodepng.h)' > lodepng_manual/CMakeLists.txt
fi

# --- SMART BUILDING ---
# If you provide a package name (e.g. ./build.sh emergency_mppi), it builds only that.
# If you run it without arguments, it builds your whole project.
PKGS_TO_BUILD=${1:-"collision_angle_mitigation emergency_mppi mujoco_ros2_control"}

echo ">>> Building: $PKGS_TO_BUILD (Workers: 2)"

colcon build --symlink-install \
    --parallel-workers 2 \
    --packages-up-to $PKGS_TO_BUILD \
    --base-paths "$PROJECT_ROOT" \
    --cmake-args \
    -DFETCHCONTENT_SOURCE_DIR_LODEPNG=$WORKSPACE_ROOT/lodepng_manual \
    -DCMAKE_BUILD_TYPE=Release

source install/setup.bash
echo ">>> Done."