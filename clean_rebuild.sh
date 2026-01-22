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

echo "[2/5] Making scripts executable..."
chmod +x src/elderly_bot/scripts/*.py
echo "✓ Scripts are executable"
echo ""

echo "[3/5] Re-sourcing ROS environment..."
source /opt/ros/melodic/setup.bash
echo "✓ Environment ready"
echo ""

echo "[4/6] Building workspace..."
catkin_make
echo "✓ Build complete (devel space)"
echo ""

echo "[5/6] Sourcing new workspace..."
source devel/setup.bash
echo "✓ Workspace sourced"
echo ""

echo "[6/6] Verifying installed script..."
DEVEL_SCRIPT="devel/lib/elderly_bot/sensors_actuators_node.py"
INSTALL_SCRIPT="install/lib/elderly_bot/sensors_actuators_node.py"

if [ -f "$DEVEL_SCRIPT" ]; then
    echo "✓ Script ready in devel space (default for roslaunch)"
    echo "  Location: $(readlink -f $DEVEL_SCRIPT)"
    
    # Check if it's executable
    if [ -x "$DEVEL_SCRIPT" ]; then
        echo "✓ Script is executable"
    else
        echo "⚠ Making script executable..."
        chmod +x "$DEVEL_SCRIPT"
    fi
elif [ -f "$INSTALL_SCRIPT" ]; then
    echo "✓ Script ready in install space"
    echo "  Location: $(readlink -f $INSTALL_SCRIPT)"
else
    echo "⚠ Script not found in expected locations"
    echo "  Checked: $DEVEL_SCRIPT"
    echo "  Checked: $INSTALL_SCRIPT"
    echo ""
    echo "  The script should still work from source:"
    echo "  $(readlink -f src/elderly_bot/scripts/sensors_actuators_node.py)"
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
