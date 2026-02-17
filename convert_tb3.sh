#!/bin/bash
set -e  # Exit immediately if a command fails

# 1. Define Paths
INPUT_DIR="/ros2_ws/src/Mobile-Robot-Collision/src/Collision_Angle_Mitigation/models/URDFs"
OUTPUT_DIR="/ros2_ws/src/Mobile-Robot-Collision/src/Collision_Angle_Mitigation/models"
SYSTEM_MESH_PATH="/opt/ros/jazzy/share/turtlebot3_description"

mkdir -p "$OUTPUT_DIR"

# 2. Setup a temporary environment
echo "Setting up temporary MuJoCo environment..."
python3 -m venv /tmp/mujoco_converter
source /tmp/mujoco_converter/bin/activate

# 3. Install MuJoCo safely inside the venv
python3 -m pip install --upgrade pip
python3 -m pip install mujoco

# 4. Convert and Patch
echo "Converting TurtleBot3 Waffle..."
python3 -c "import mujoco; model = mujoco.MjModel.from_xml_path('$INPUT_DIR/turtlebot3_waffle.urdf'); mujoco.mj_saveLastXML('$OUTPUT_DIR/turtlebot3_waffle.xml', model)"

echo "Patching mesh paths..."
sed -i "s|package://turtlebot3_description|$SYSTEM_MESH_PATH|g" "$OUTPUT_DIR/turtlebot3_waffle.xml"

# 5. Done
deactivate
rm -rf /tmp/mujoco_converter
echo "SUCCESS: Model created at $OUTPUT_DIR/turtlebot3_waffle.xml"