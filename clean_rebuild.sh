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

echo "[6/6] Verifying and fixing script deployment..."
SOURCE_SCRIPT="src/elderly_bot/scripts/sensors_actuators_node.py"
DEVEL_DIR="devel/lib/elderly_bot"
DEVEL_SCRIPT="$DEVEL_DIR/sensors_actuators_node.py"

# Check source script first
if [ ! -f "$SOURCE_SCRIPT" ]; then
    echo "✗ ERROR: Source script not found: $SOURCE_SCRIPT"
    exit 1
fi

# Make source executable
chmod +x "$SOURCE_SCRIPT"
echo "✓ Source script is executable"

# Create devel directory if needed
if [ ! -d "$DEVEL_DIR" ]; then
    mkdir -p "$DEVEL_DIR"
    echo "✓ Created devel directory: $DEVEL_DIR"
fi

# Copy script to devel space (fallback if catkin_install_python doesn't work)
cp "$SOURCE_SCRIPT" "$DEVEL_SCRIPT"
chmod +x "$DEVEL_SCRIPT"
echo "✓ Script deployed to devel space"
echo "  Location: $(readlink -f $DEVEL_SCRIPT)"
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
