#!/bin/bash
# Complete clean rebuild for Elderly Bot
# This ensures all Python changes are properly installed

set -e

echo "============================================"
echo "  Elderly Bot - Clean Rebuild Script"
echo "============================================"
echo ""

# Navigate to workspace
cd ~/catkin_ws

echo "[1/5] Cleaning build artifacts..."
rm -rf build/ devel/ install/ .catkin_tools/
echo "✓ Clean complete"
echo ""

echo "[2/5] Re-sourcing ROS environment..."
source /opt/ros/melodic/setup.bash
echo "✓ Environment ready"
echo ""

echo "[3/5] Building workspace..."
catkin_make
echo "✓ Build complete"
echo ""

echo "[4/5] Sourcing new workspace..."
source devel/setup.bash
echo "✓ Workspace sourced"
echo ""

echo "[5/5] Verifying installed script..."
if [ -f "install/lib/elderly_bot/sensors_actuators_node.py" ]; then
    echo "✓ Script installed successfully"
    echo ""
    echo "Installation path:"
    echo "  $(readlink -f install/lib/elderly_bot/sensors_actuators_node.py)"
else
    echo "⚠ Warning: Script not found in install directory"
fi
echo ""

echo "============================================"
echo "  Rebuild Complete!"
echo "============================================"
echo ""
echo "To run the robot:"
echo "  roslaunch elderly_bot bringup.launch"
echo ""
echo "To test sensors:"
echo "  rostopic echo /gas_detected"
echo "  rostopic echo /jetson_temperature"
echo "  rostopic pub /buzzer_command std_msgs/Bool \"data: true\""
echo ""
