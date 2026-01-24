#!/bin/bash
# Fix ROS Melodic Python Environment Issues
# Run this script on Jetson Nano to resolve rospkg and Python version conflicts

set -e

echo "================================================"
echo "ROS Melodic Python Environment Fix"
echo "================================================"
echo ""

# Detect ROS distro
if [ -f /opt/ros/melodic/setup.bash ]; then
    ROS_DISTRO="melodic"
elif [ -f /opt/ros/noetic/setup.bash ]; then
    ROS_DISTRO="noetic"
else
    echo "❌ No ROS installation found!"
    exit 1
fi

echo "Detected ROS: $ROS_DISTRO"
echo ""

# Source ROS environment
source /opt/ros/$ROS_DISTRO/setup.bash

# Check Python version used by ROS
echo "[1/5] Checking Python configuration..."
echo "ROS Python path: $PYTHONPATH"
echo ""

if [ "$ROS_DISTRO" = "melodic" ]; then
    echo "ROS Melodic requires Python 2.7"
    PYTHON_CMD="python2"
    PIP_CMD="pip2"
    
    # Verify python2 is available
    if ! command -v $PYTHON_CMD &> /dev/null; then
        echo "❌ Python 2.7 not found! Installing..."
        sudo apt-get update
        sudo apt-get install -y python python-pip
    fi
else
    echo "ROS Noetic uses Python 3"
    PYTHON_CMD="python3"
    PIP_CMD="pip3"
fi

echo "✅ Using: $PYTHON_CMD"
$PYTHON_CMD --version
echo ""

# Install/reinstall ROS Python packages
echo "[2/5] Installing ROS Python dependencies..."
sudo apt-get update
sudo apt-get install -y \
    ros-$ROS_DISTRO-cv-bridge \
    ros-$ROS_DISTRO-image-transport \
    ros-$ROS_DISTRO-compressed-image-transport \
    python-rosdep \
    python-rosinstall \
    python-rosinstall-generator \
    python-wstool

if [ "$ROS_DISTRO" = "melodic" ]; then
    # Python 2 specific packages
    sudo apt-get install -y \
        python-rospkg \
        python-catkin-pkg \
        python-opencv \
        python-numpy
else
    # Python 3 specific packages
    sudo apt-get install -y \
        python3-rospkg \
        python3-catkin-pkg \
        python3-opencv \
        python3-numpy
fi

echo "✅ ROS dependencies installed"
echo ""

# Verify rospkg installation
echo "[3/5] Verifying rospkg installation..."
if $PYTHON_CMD -c "import rospkg; print('rospkg version: ' + rospkg.__version__)" 2>/dev/null; then
    echo "✅ rospkg is working"
else
    echo "⚠️  rospkg import failed, attempting manual install..."
    sudo $PIP_CMD install --upgrade rospkg catkin-pkg
fi
echo ""

# Check cv_bridge
echo "[4/5] Verifying cv_bridge..."
if $PYTHON_CMD -c "from cv_bridge import CvBridge; print('cv_bridge: OK')" 2>/dev/null; then
    echo "✅ cv_bridge is working"
else
    echo "❌ cv_bridge failed - may need to rebuild workspace"
fi
echo ""

# Fix script shebangs in elderly_bot package
echo "[5/5] Checking script shebangs..."
if [ -d ~/catkin_ws/src/elderly_bot/scripts ]; then
    cd ~/catkin_ws/src/elderly_bot/scripts
    
    echo "Verifying Python scripts use correct interpreter..."
    for script in *.py; do
        if [ -f "$script" ]; then
            SHEBANG=$(head -n1 "$script")
            if [[ "$SHEBANG" == *"python3"* ]] && [ "$ROS_DISTRO" = "melodic" ]; then
                echo "⚠️  $script uses python3 but ROS Melodic needs python2!"
                echo "   Please update shebang to: #!/usr/bin/env python"
            elif [[ "$SHEBANG" == *"python"* ]]; then
                echo "✅ $script: $SHEBANG"
            fi
        fi
    done
fi
echo ""

# Environment check
echo "================================================"
echo "Environment Summary"
echo "================================================"
echo "ROS Distribution: $ROS_DISTRO"
echo "Python Command: $PYTHON_CMD"
echo "Python Version: $($PYTHON_CMD --version 2>&1)"
echo "PYTHONPATH: $PYTHONPATH"
echo ""

# Test imports
echo "Testing critical imports..."
$PYTHON_CMD << 'PYEOF'
import sys
print("Python executable: " + sys.executable)

try:
    import rospy
    print("✅ rospy")
except ImportError as e:
    print("❌ rospy: " + str(e))

try:
    import rospkg
    print("✅ rospkg")
except ImportError as e:
    print("❌ rospkg: " + str(e))

try:
    from cv_bridge import CvBridge
    print("✅ cv_bridge")
except ImportError as e:
    print("❌ cv_bridge: " + str(e))

try:
    import cv2
    print("✅ opencv (cv2)")
except ImportError as e:
    print("❌ opencv: " + str(e))
PYEOF

echo ""
echo "================================================"
echo "Next Steps"
echo "================================================"
echo ""
echo "1. If any imports failed, rebuild your workspace:"
echo "   cd ~/catkin_ws"
echo "   catkin_make clean"
echo "   catkin_make"
echo "   source devel/setup.bash"
echo ""
echo "2. Ensure your .bashrc sources the correct ROS setup:"
echo "   echo 'source /opt/ros/$ROS_DISTRO/setup.bash' >> ~/.bashrc"
echo "   echo 'source ~/catkin_ws/devel/setup.bash' >> ~/.bashrc"
echo ""
echo "3. Test the camera node:"
echo "   roslaunch elderly_bot camera_streaming.launch enable_kvs:=false"
echo ""
