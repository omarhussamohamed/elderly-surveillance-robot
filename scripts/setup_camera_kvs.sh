#!/bin/bash
# Setup script for Camera + AWS KVS on Jetson Nano
# Run this once to install all dependencies

set -e

echo "=========================================="
echo "ElderlyBot Camera & KVS Setup"
echo "=========================================="

# Detect ROS version
if [ -z "$ROS_DISTRO" ]; then
    if [ -f "/opt/ros/melodic/setup.bash" ]; then
        export ROS_DISTRO="melodic"
        source /opt/ros/melodic/setup.bash
    elif [ -f "/opt/ros/noetic/setup.bash" ]; then
        export ROS_DISTRO="noetic"
        source /opt/ros/noetic/setup.bash
    else
        echo "ERROR: No ROS installation found!"
        exit 1
    fi
fi

echo "Detected ROS version: $ROS_DISTRO"

# Detect JetPack version
if [ -f "/etc/nv_tegra_release" ]; then
    echo "JetPack version:"
    cat /etc/nv_tegra_release
fi

# Update system
echo "[1/6] Updating system packages..."
sudo apt-get update

# Install camera and CV dependencies
echo "[2/6] Installing camera tools and OpenCV..."
sudo apt-get install -y \
  v4l-utils \
  python3-opencv \
  python3-pip \
  ros-${ROS_DISTRO}-cv-bridge \
  ros-${ROS_DISTRO}-image-transport \
  ros-${ROS_DISTRO}-compressed-image-transport

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
    echo "Downloading AWS CLI for ARM64..."
    cd /tmp
    curl -o "awscliv2.zip" "https://awscli.amazonaws.com/awscli-exe-linux-aarch64.zip"
    unzip -q awscliv2.zip
    sudo ./aws/install
    rm -rf aws awscliv2.zip
    cd -
    echo "AWS CLI installed successfully"
else
    echo "AWS CLI already installed: $(aws --version)"
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
  libx264-dev \
  pkg-config \
  m4 \
  automake \
  libtool

# Clone and build KVS Producer SDK
KVS_DIR="$HOME/amazon-kinesis-video-streams-producer-sdk-cpp"

if [ ! -d "$KVS_DIR" ]; then
    echo "Cloning KVS Producer SDK..."
    cd ~
    git clone --recursive https://github.com/awslabs/amazon-kinesis-video-streams-producer-sdk-cpp.git
    cd amazon-kinesis-video-streams-producer-sdk-cpp
    
    echo "Building KVS SDK (this takes ~20-30 minutes on Jetson Nano)..."
    mkdir -p build
    cd build
    
    # Build with GStreamer plugin for Jetson
    cmake .. \

# Source workspace and find package
if [ -f "$HOME/catkin_ws/devel/setup.bash" ]; then
    source $HOME/catkin_ws/devel/setup.bash
    SCRIPTS_DIR="$(rospack find elderly_bot 2>/dev/null)/scripts"
    
    if [ -d "$SCRIPTS_DIR" ]; then
        chmod +x "$SCRIPTS_DIR/camera_node.py" 2>/dev/null || true
        chmod +x "$SCRIPTS_DIR/kvs_streamer_node.py" 2>/dev/null || true
        echo "Scripts made executable"
    else
        echo "Warning: Could not find elderly_bot package. Make sure to run catkin_make first."
    fi
else
    echo "Warning: catkin workspace not built yet. Run 'catkin_make' first."
fi

echo ""
echo "=========================================="
echo "✅ Setup Complete!"
echo "=========================================="
echo ""
echo "System Configuration:"
echo "  ROS Version: $ROS_DISTRO"
echo "  Camera: /dev/video0 (1280x720 @ 30fps MJPEG)"
echo "  AWS Region: eu-north-1"
echo ""
echo "Next steps:"
echo ""
echo "1. Build ROS workspace:"
echo "   cd ~/catkin_ws"
echo "   catkin_make"
echo "   source devel/setup.bash"
echo ""
echo "2. Configure AWS credentials (AFTER rotating exposed keys!):"
echo "   aws configure"
echo "   # Enter your NEW Access Key and Secret Key"
echo "   # Region: eu-north-1"
echo ""
echo "3. Create KVS stream:"
echo "   aws kinesisvideo create-stream \\"
echo "     --stream-name elderly-bot-stream \\"
echo "     --data-retention-in-hours 1 \\"
echo "     --region eu-north-1"
echo ""
echo "4. Test camera (without AWS):"
echo "   roslaunch elderly_bot camera_streaming.launch enable_kvs:=false"
echo ""
echo "5. Verify camera feed:"
echo "   rostopic hz /camera/image_raw"
echo "   # Should show ~30 Hz"
echo ""
echo "6. Start full AWS streaming:"
echo "   roslaunch elderly_bot camera_streaming.launch"
echo ""
echo "7. Check stream status:"
echo "   rostopic echo /kvs/streaming"
echo ""
echo "Troubleshooting:"
echo "  - Camera permissions: sudo usermod -a -G video $USER (then logout/login)"
echo "  - Check camera: v4l2-ctl --device=/dev/video0 --all"
echo "  - View feed locally: rqt_image_view
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
