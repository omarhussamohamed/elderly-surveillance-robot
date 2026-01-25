#!/bin/bash

# Ensure script is run as root
if [ "$EUID" -ne 0 ]; then
  echo "Please run as root"
  exit
fi

# Update and install dependencies
apt-get update
apt-get install -y \
    python3.8 \
    python3.8-venv \
    python3.8-dev \
    ros-melodic-rosbridge-server \
    gstreamer1.0-plugins-bad \
    gstreamer1.0-plugins-good \
    gstreamer1.0-plugins-ugly \
    gstreamer1.0-tools \
    gstreamer1.0-libav

# Create Python 3.8 virtual environment
VENV_DIR="/opt/livekit_venv"
python3.8 -m venv $VENV_DIR
source $VENV_DIR/bin/activate

# Install Python packages
pip install --upgrade pip
pip install \
    opencv-python==4.2.0.34 \
    numpy==1.19.5 \
    roslibpy==1.2.0 \
    livekit==1.0.0

deactivate

# Install LiveKit CLI
curl -s https://raw.githubusercontent.com/livekit/livekit-cli/main/install.sh | bash

# Print completion message
echo "Installation complete. To activate the virtual environment, run:"
echo "source $VENV_DIR/bin/activate"