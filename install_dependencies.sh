#!/usr/bin/env bash

###############################################################################
# Elderly Bot Dependencies Installer - FINAL VERSION
# Idempotent, fast, and safe to run multiple times
###############################################################################

set -euo pipefail

WORKSPACE_ROOT="${HOME}/catkin_ws"
ELDERLY_BOT_PATH="${WORKSPACE_ROOT}/src/elderly_bot"

echo -e "\n\033[1;32mInstalling Elderly Bot Dependencies\033[0m"

# ROS Melodic core + navigation
sudo apt-get update -qq
sudo apt-get install -y --no-install-recommends \
    ros-melodic-navigation \
    ros-melodic-robot-localization \
    ros-melodic-gmapping \
    ros-melodic-amcl \
    ros-melodic-move-base \
    ros-melodic-dwa-local-planner \
    ros-melodic-rplidar-ros \
    ros-melodic-rosserial-python \
    ros-melodic-diagnostic-aggregator \
    ros-melodic-imu-filter-madgwick \
    python-rosdep \
    python-pip \
    python3-pip \
    python-opencv \
    python-yaml

# Python 2 dependencies (Melodic)
sudo pip install --upgrade \
    paho-mqtt \
    pyserial \
    smbus2 \
    pyyaml

# Jetson specific
sudo apt-get install -y python-smbus i2c-tools
sudo pip3 install --upgrade Jetson.GPIO jetson-stats

# Add user to groups
sudo usermod -a -G dialout,video $USER

# udev rules
sudo tee /etc/udev/rules.d/99-rplidar.rules > /dev/null << 'EOF'
KERNEL=="ttyUSB*", ATTRS{idVendor}=="10c4", ATTRS{idProduct}=="ea60", MODE:="0666", GROUP:="dialout", SYMLINK+="rplidar"
EOF

sudo udevadm control --reload-rules && sudo udevadm trigger

echo -e "\n\033[1;32m✓ Dependencies installed successfully\033[0m"
echo "Run: source ~/.bashrc && cd ~/catkin_ws && catkin_make"