#!/bin/bash
# Setup script for Camera + AWS KVS on Jetson Nano
# Run this once to install all dependencies

set -e

echo "=========================================="
echo "ElderlyBot Camera & KVS Setup"
echo "=========================================="

# Update system
echo "[1/6] Updating system packages..."
sudo apt-get update

# Install camera and CV dependencies
echo "[2/6] Installing camera tools and OpenCV..."
sudo apt-get install -y \
  v4l-utils \
  python3-opencv \
  ros-noetic-cv-bridge \
  ros-noetic-image-transport \
  ros-noetic-compressed-image-transport

# Install GStreamer
echo "[3/6] Installing GStreamer..."
sudo apt-get install -y \
  gstreamer1.0-tools \
  gstreamer1.0-plugins-base \
  gstreamer1.0-plugins-good \
  gstreamer1.0-plugins-bad \
  gstreamer1.0-plugins-ugly \
  libgstreamer1.0-dev \
  libgstreamer-plugins-base1.0-dev

# Install AWS CLI
echo "[4/6] Installing AWS CLI..."
if ! command -v aws &> /dev/null; then
    curl "https://awscli.amazonaws.com/awscli-exe-linux-aarch64.zip" -o "awscliv2.zip"
    unzip -q awscliv2.zip
    sudo ./aws/install
    rm -rf aws awscliv2.zip
else
    echo "AWS CLI already installed"
fi

# Install AWS KVS Producer SDK
echo "[5/6] Installing AWS Kinesis Video Streams Producer SDK..."

# Install prerequisites
sudo apt-get install -y \
  build-essential \
  cmake \
  git \
  libssl-dev \
  libcurl4-openssl-dev \
  liblog4cplus-dev \
  libgtest-dev \
  libx264-dev

# Clone and build KVS Producer SDK
KVS_DIR="$HOME/amazon-kinesis-video-streams-producer-sdk-cpp"

if [ ! -d "$KVS_DIR" ]; then
    cd ~
    git clone --recursive https://github.com/awslabs/amazon-kinesis-video-streams-producer-sdk-cpp.git
    cd amazon-kinesis-video-streams-producer-sdk-cpp
    mkdir -p build
    cd build
    cmake .. -DBUILD_GSTREAMER_PLUGIN=ON -DBUILD_DEPENDENCIES=OFF
    make -j$(nproc)
    sudo make install
    sudo ldconfig
else
    echo "KVS SDK already exists at $KVS_DIR"
fi

# Make scripts executable
echo "[6/6] Setting up ROS scripts..."
SCRIPTS_DIR="$(rospack find elderly_bot)/scripts"
chmod +x "$SCRIPTS_DIR/camera_node.py"
chmod +x "$SCRIPTS_DIR/kvs_streamer_node.py"

echo ""
echo "=========================================="
echo "✅ Setup Complete!"
echo "=========================================="
echo ""
echo "Next steps:"
echo "1. Configure AWS credentials:"
echo "   aws configure"
echo ""
echo "2. Create KVS stream:"
echo "   aws kinesisvideo create-stream --stream-name elderly-bot-stream --data-retention-in-hours 1 --region eu-north-1"
echo ""
echo "3. Test camera:"
echo "   roslaunch elderly_bot camera_streaming.launch enable_kvs:=false"
echo ""
echo "4. Start full streaming:"
echo "   roslaunch elderly_bot camera_streaming.launch"
echo ""
