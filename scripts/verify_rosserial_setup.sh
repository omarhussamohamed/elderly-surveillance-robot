#!/bin/bash
# Verify rosserial setup and dependencies
# Run this on Jetson to check all versions and compatibility

echo "=========================================="
echo "rosserial Setup Verification"
echo "=========================================="
echo ""

# Check ROS version
echo "1. ROS Version:"
rosversion -d
echo ""

# Check rosserial_python version
echo "2. rosserial_python version:"
dpkg -l | grep rosserial-python || apt-cache show ros-melodic-rosserial-python | grep Version
echo ""

# Check rosserial_arduino version
echo "3. rosserial_arduino version:"
dpkg -l | grep rosserial-arduino || apt-cache show ros-melodic-rosserial-arduino | grep Version
echo ""

# Check if ros_lib exists
echo "4. ros_lib location:"
if [ -d ~/Arduino/libraries/ros_lib ]; then
    echo "   Found: ~/Arduino/libraries/ros_lib"
    echo "   Last modified: $(stat -c %y ~/Arduino/libraries/ros_lib 2>/dev/null | cut -d' ' -f1)"
    echo "   Checking key files..."
    if [ -f ~/Arduino/libraries/ros_lib/time.h ]; then
        echo "   ✓ time.h exists"
        # Check if it has the right content
        if grep -q "ros::Time" ~/Arduino/libraries/ros_lib/time.h 2>/dev/null; then
            echo "   ✓ time.h appears valid"
        else
            echo "   ✗ time.h appears corrupted or wrong version"
        fi
    else
        echo "   ✗ time.h MISSING - ros_lib is incomplete!"
    fi
    
    if [ -f ~/Arduino/libraries/ros_lib/nav_msgs/Odometry.h ]; then
        echo "   ✓ nav_msgs/Odometry.h exists"
    else
        echo "   ✗ nav_msgs/Odometry.h MISSING"
    fi
    
    if [ -f ~/Arduino/libraries/ros_lib/geometry_msgs/Twist.h ]; then
        echo "   ✓ geometry_msgs/Twist.h exists"
    else
        echo "   ✗ geometry_msgs/Twist.h MISSING"
    fi
else
    echo "   ✗ ros_lib NOT FOUND at ~/Arduino/libraries/ros_lib"
    echo "   Run: rosrun rosserial_arduino make_libraries.py ~/Arduino/libraries"
fi
echo ""

# Check Python serial library
echo "5. Python pyserial version:"
python -c "import serial; print('   pyserial version:', serial.VERSION)" 2>/dev/null || \
python3 -c "import serial; print('   pyserial version:', serial.VERSION)" 2>/dev/null || \
echo "   ✗ pyserial not installed - install with: sudo apt-get install python-serial"
echo ""

# Check USB port
echo "6. USB Serial Ports:"
if [ -e /dev/ttyUSB0 ]; then
    echo "   /dev/ttyUSB0 exists"
    echo "   Permissions: $(ls -l /dev/ttyUSB0 | awk '{print $1, $3, $4}')"
    if [ -r /dev/ttyUSB0 ] && [ -w /dev/ttyUSB0 ]; then
        echo "   ✓ Readable and writable"
    else
        echo "   ✗ Permission denied - run: sudo chmod 666 /dev/ttyUSB0"
    fi
else
    echo "   ✗ /dev/ttyUSB0 NOT FOUND"
fi
echo ""

# Check rosserial_arduino library on system
echo "7. rosserial_arduino ROS package files:"
if rospack find rosserial_arduino > /dev/null 2>&1; then
    ROS_LIB_GEN=$(rospack find rosserial_arduino)/src/rosserial_arduino/make_libraries.py
    if [ -f "$ROS_LIB_GEN" ]; then
        echo "   ✓ Found: $ROS_LIB_GEN"
        echo "   Last modified: $(stat -c %y "$ROS_LIB_GEN" 2>/dev/null | cut -d' ' -f1)"
    else
        echo "   ✗ make_libraries.py not found"
    fi
else
    echo "   ✗ rosserial_arduino ROS package not found"
fi
echo ""

# Check message definitions match
echo "8. ROS Message Definitions:"
if rospack find geometry_msgs > /dev/null 2>&1 && rospack find nav_msgs > /dev/null 2>&1; then
    echo "   ✓ geometry_msgs package found"
    echo "   ✓ nav_msgs package found"
    
    # Check Twist message
    if [ -f "$(rospack find geometry_msgs)/msg/Twist.msg" ]; then
        echo "   ✓ Twist.msg exists"
    fi
    
    # Check Odometry message
    if [ -f "$(rospack find nav_msgs)/msg/Odometry.msg" ]; then
        echo "   ✓ Odometry.msg exists"
    fi
else
    echo "   ✗ Message packages not found"
fi
echo ""

echo "=========================================="
echo "Diagnostic Summary"
echo "=========================================="
echo ""
echo "Next steps if issues found:"
echo "1. Regenerate ros_lib:"
echo "   cd ~/Arduino/libraries && rm -rf ros_lib"
echo "   rosrun rosserial_arduino make_libraries.py ."
echo ""
echo "2. Fix USB permissions:"
echo "   sudo chmod 666 /dev/ttyUSB0"
echo "   sudo usermod -a -G dialout $USER"
echo ""
echo "3. Reinstall rosserial (if needed):"
echo "   sudo apt-get update"
echo "   sudo apt-get install --reinstall ros-melodic-rosserial-python ros-melodic-rosserial-arduino"
echo ""

