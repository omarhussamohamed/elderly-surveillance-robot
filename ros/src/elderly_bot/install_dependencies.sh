#!/usr/bin/env bash

###############################################################################
# Elderly Bot Dependencies Installer
# ROS Melodic (Ubuntu 18.04) — Python 2 based
# Safe to run multiple times (idempotent)
###############################################################################

set -euo pipefail

echo -e "\n\033[1;32mInstalling Elderly Bot Dependencies\033[0m"

sudo apt-get update -qq

# ── Core ROS packages ─────────────────────────────────────────
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
    python-opencv \
    python-yaml \
    i2c-tools \
    python-smbus

# ── Python 2 packages (ROS Melodic) ───────────────────────────
sudo pip install --upgrade \
    paho-mqtt \
    smbus2 \
    pyyaml \
    pyserial

# ── Jetson-specific utilities (safe even if not on Jetson) ───
sudo pip install --upgrade jetson-stats || true
sudo pip install --upgrade Jetson.GPIO || true

# ── User permissions ─────────────────────────────────────────
sudo usermod -a -G dialout,video $USER

# ── RPLidar udev rule ────────────────────────────────────────
sudo tee /etc/udev/rules.d/99-rplidar.rules > /dev/null << 'EOF'
KERNEL=="ttyUSB*", ATTRS{idVendor}=="10c4", ATTRS{idProduct}=="ea60", MODE:="0666", GROUP:="dialout", SYMLINK+="rplidar"
EOF

sudo udevadm control --reload-rules
sudo udevadm trigger

echo -e "\n\033[1;32m✓ Dependencies installed successfully\033[0m"
echo "Reboot recommended. Then run:"
echo "  cd ~/catkin_ws && catkin_make"
