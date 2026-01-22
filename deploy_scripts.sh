#!/bin/bash
# Direct script deployment (bypass catkin install issues)
# Use this if clean_rebuild.sh doesn't work

set -e

echo "============================================"
echo "  Direct Script Deployment"
echo "============================================"
echo ""

cd ~/catkin_ws

echo "[1/3] Making script executable..."
chmod +x src/elderly_bot/scripts/sensors_actuators_node.py
echo "✓ Script is executable"
echo ""

echo "[2/3] Building workspace (devel space)..."
source /opt/ros/melodic/setup.bash
catkin_make
echo "✓ Build complete"
echo ""

echo "[3/3] Verifying script location..."
SCRIPT_PATH="$(pwd)/devel/lib/elderly_bot/sensors_actuators_node.py"
if [ -f "$SCRIPT_PATH" ]; then
    echo "✓ Script found: $SCRIPT_PATH"
else
    echo "⚠ Script not in devel/lib, checking source..."
    SCRIPT_PATH="$(pwd)/src/elderly_bot/scripts/sensors_actuators_node.py"
    if [ -f "$SCRIPT_PATH" ]; then
        echo "✓ Script found in source: $SCRIPT_PATH"
    else
        echo "✗ ERROR: Script not found!"
        exit 1
    fi
fi
echo ""

echo "============================================"
echo "  Deployment Complete!"
echo "============================================"
echo ""
echo "The script will run from: devel/lib/elderly_bot/"
echo ""
echo "To run the robot:"
echo "  source ~/catkin_ws/devel/setup.bash"
echo "  roslaunch elderly_bot bringup.launch"
echo ""
