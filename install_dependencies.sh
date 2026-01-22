#!/bin/bash

###############################################################################
# Elderly Bot Dependency Installation Script
# 
# This script installs all required ROS packages and dependencies for the
# Elderly Bot autonomous monitoring robot.
#
# Target Platform: Ubuntu 18.04 with ROS Melodic
# Target Hardware: Jetson Nano
#
# Usage:
#   chmod +x install_dependencies.sh
#   ./install_dependencies.sh
###############################################################################

set -e  # Exit on error

echo "=========================================="
echo "Elderly Bot Dependency Installation"
echo "=========================================="
echo ""

# Check if running on Ubuntu 18.04
if [ -f /etc/os-release ]; then
    . /etc/os-release
    if [ "$VERSION_ID" != "18.04" ]; then
        echo "WARNING: This script is designed for Ubuntu 18.04"
        echo "Current version: $VERSION_ID"
        read -p "Continue anyway? (y/n) " -n 1 -r
        echo
        if [[ ! $REPLY =~ ^[Yy]$ ]]; then
            exit 1
        fi
    fi
fi

# Check if ROS Melodic is installed
if [ ! -f /opt/ros/melodic/setup.bash ]; then
    echo "ERROR: ROS Melodic not found!"
    echo "Please install ROS Melodic first:"
    echo "  http://wiki.ros.org/melodic/Installation/Ubuntu"
    exit 1
fi

echo "ROS Melodic detected"
source /opt/ros/melodic/setup.bash

# Update package list
echo ""
echo "Updating package list..."
sudo apt-get update

# Install core ROS packages
echo ""
echo "Installing core ROS packages..."
sudo apt-get install -y \
    ros-melodic-desktop-full \
    ros-melodic-tf \
    ros-melodic-tf2-ros

# Install navigation stack
echo ""
echo "Installing navigation stack..."
sudo apt-get install -y \
    ros-melodic-navigation \
    ros-melodic-move-base \
    ros-melodic-move-base-msgs \
    ros-melodic-amcl \
    ros-melodic-map-server \
    ros-melodic-navfn \
    ros-melodic-dwa-local-planner \
    ros-melodic-costmap-2d

# Install robot_localization
echo ""
echo "Installing robot_localization..."
sudo apt-get install -y \
    ros-melodic-robot-localization

# Install gmapping
echo ""
echo "Installing gmapping..."
sudo apt-get install -y \
    ros-melodic-gmapping

# Install explore_lite
echo ""
echo "Installing explore_lite..."
sudo apt-get install -y \
    ros-melodic-explore-lite

# Install RPLidar ROS package
echo ""
echo "Installing RPLidar ROS package..."
sudo apt-get install -y \
    ros-melodic-rplidar-ros

# Install rosserial for ESP32 communication
echo ""
echo "Installing rosserial..."
sudo apt-get install -y \
    ros-melodic-rosserial \
    ros-melodic-rosserial-python \
    ros-melodic-rosserial-arduino

# Install additional tools
echo ""
echo "Installing additional tools..."
sudo apt-get install -y \
    python-pip \
    python-yaml \
    python-rospkg \
    python-catkin-tools

# Install Python dependencies
echo ""
echo "Installing Python dependencies..."
pip install --user \
    pyserial \
    pyyaml

# Setup udev rules for serial devices
echo ""
echo "Setting up udev rules for serial devices..."

# RPLidar udev rule
sudo bash -c 'cat > /etc/udev/rules.d/99-rplidar.rules << EOF
# RPLidar A1
KERNEL=="ttyUSB*", ATTRS{idVendor}=="10c4", ATTRS{idProduct}=="ea60", MODE:="0666", GROUP:="dialout", SYMLINK+="rplidar"
EOF'

# ESP32 udev rule
sudo bash -c 'cat > /etc/udev/rules.d/99-esp32.rules << EOF
# ESP32 Development Board
KERNEL=="ttyUSB*", ATTRS{idVendor}=="1a86", ATTRS{idProduct}=="7523", MODE:="0666", GROUP:="dialout", SYMLINK+="esp32"
KERNEL=="ttyUSB*", ATTRS{idVendor}=="10c4", ATTRS{idProduct}=="ea60", MODE:="0666", GROUP:="dialout"
EOF'

# Reload udev rules
sudo udevadm control --reload-rules
sudo udevadm trigger

# Add user to dialout group (required for serial access)
echo ""
echo "Adding user to dialout group..."
sudo usermod -a -G dialout $USER

# Create workspace directories if they don't exist
echo ""
echo "Setting up workspace directories..."
mkdir -p ~/catkin_ws/src

# Check if elderly_bot package exists
if [ ! -d ~/catkin_ws/src/elderly_bot ]; then
    echo "WARNING: elderly_bot package not found in ~/catkin_ws/src/"
    echo "Please ensure the package is in the correct location"
fi

# Create maps directory
mkdir -p ~/catkin_ws/src/elderly_bot/maps

# Create rviz directory
mkdir -p ~/catkin_ws/src/elderly_bot/rviz

# Build workspace
echo ""
echo "Building catkin workspace..."
cd ~/catkin_ws
catkin_make

# Source workspace in bashrc if not already present
if ! grep -q "source ~/catkin_ws/devel/setup.bash" ~/.bashrc; then
    echo ""
    echo "Adding workspace to ~/.bashrc..."
    echo "" >> ~/.bashrc
    echo "# ROS Melodic and Elderly Bot workspace" >> ~/.bashrc
    echo "source /opt/ros/melodic/setup.bash" >> ~/.bashrc
    echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc
fi

# Print Arduino IDE instructions
echo ""
echo "=========================================="
echo "Arduino IDE Setup for ESP32"
echo "=========================================="
echo ""
echo "To program the ESP32, you need to:"
echo ""
echo "1. Install Arduino IDE (if not already installed):"
echo "   Download from: https://www.arduino.cc/en/software"
echo ""
echo "2. Add ESP32 board support:"
echo "   - Open Arduino IDE"
echo "   - Go to File > Preferences"
echo "   - Add to 'Additional Board Manager URLs':"
echo "     https://raw.githubusercontent.com/espressif/arduino-esp32/gh-pages/package_esp32_index.json"
echo "   - Go to Tools > Board > Board Manager"
echo "   - Search for 'esp32' and install"
echo ""
echo "3. Install required Arduino libraries:"
echo "   - Go to Sketch > Include Library > Manage Libraries"
echo "   - Install: 'Rosserial Arduino Library'"
echo "   - Install: 'MPU9250' by hideakitai"
echo ""
echo "4. Configure rosserial for Arduino (CRITICAL - Must be done on Jetson!):"
echo "   cd ~/Arduino/libraries"
echo "   rm -rf ros_lib"
echo "   rosrun rosserial_arduino make_libraries.py ."
echo "   IMPORTANT: ros_lib MUST be generated on Jetson (where rosserial_python runs)"
echo "   If you generated it on Windows, it will cause 'Unable to sync' errors!"
echo "   After generating, copy the ros_lib folder to your Windows Arduino IDE libraries folder"
echo ""
echo "5. Open firmware file:"
echo "   ~/catkin_ws/src/elderly_bot/firmware/elderly_bot_esp32_wifi.ino"
echo ""
echo "6. Select board: ESP32 Dev Module"
echo "7. Upload to ESP32"
echo ""

# Print optional packages info
echo ""
echo "=========================================="
echo "Optional Python Packages"
echo "=========================================="
echo ""
echo "For additional features (gas sensor, buzzer, cloud bridge), install:"
echo ""
echo "# Gas sensor + buzzer + Jetson monitoring:"
echo "sudo pip3 install jetson-stats Jetson.GPIO smbus2"
echo ""
echo "# Cloud bridge (AWS IoT Core):"
echo "sudo pip3 install AWSIoTPythonSDK"
echo ""
echo "IMPORTANT: Reboot after installing jetson-stats!"
echo "sudo reboot"
echo ""
echo "These packages enable:"
echo "  - scripts/sensors_actuators_node.py (gas sensor, buzzer, Jetson stats)"
echo "  - scripts/cloud_bridge_node.py (AWS IoT Core integration)"
echo ""
echo "To enable these nodes, see:"
echo "  - launch/SENSORS_ACTUATORS_LAUNCH_SNIPPET.xml"
echo "  - launch/CLOUD_BRIDGE_LAUNCH_SNIPPET.xml"
echo ""

# Print completion message
echo ""
echo "=========================================="
echo "Installation Complete!"
echo "=========================================="
echo ""
echo "Next steps:"
echo ""
echo "1. Log out and log back in (for dialout group to take effect)"
echo "   OR run: newgrp dialout"
echo ""
echo "2. Program the ESP32 with the firmware (see instructions above)"
echo ""
echo "3. Connect hardware:"
echo "   - RPLidar to /dev/ttyUSB1"
echo "   - ESP32 via WiFi to Jetson (192.168.1.16:11411)"
echo "   - MPU-9250 IMU to I2C Bus 1 (magnetometer DISABLED)"
echo "   - (Optional) MQ-6 gas sensor via ADS1115 ADC"
echo "   - (Optional) Active buzzer to GPIO pin"
echo ""
echo "4. Test hardware bringup:"
echo "   roslaunch elderly_bot bringup.launch"
echo ""
echo "5. Create a map (MODE 1):"
echo "   roslaunch elderly_bot mapping.launch"
echo "   (When complete, save map with: rosrun map_server map_saver -f ~/catkin_ws/src/elderly_bot/maps/house_map)"
echo ""
echo "6. Navigate and patrol (MODE 2):"
echo "   roslaunch elderly_bot navigation.launch"
echo "   rosrun elderly_bot patrol_client.py"
echo ""
echo "For more information, see the README.md file"
echo ""
echo "=========================================="


