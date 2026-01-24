#!/bin/bash
###############################################################################
# AWS KVS FINAL INTEGRATION - EXECUTE ON JETSON NANO
# Run this script with: bash JETSON_EXECUTE_NOW.sh
###############################################################################

set -e  # Exit on error

echo "=========================================="
echo "STEP 1: ADD USER TO VIDEO GROUP"
echo "=========================================="
sudo usermod -a -G video omar
echo "✓ User 'omar' added to video group"
echo "  NOTE: You must LOG OUT and LOG BACK IN for this to take effect"

echo ""
echo "=========================================="
echo "STEP 2: CONFIGURE ENVIRONMENT (~/.bashrc)"
echo "=========================================="

# Backup existing bashrc
cp ~/.bashrc ~/.bashrc.backup.$(date +%Y%m%d_%H%M%S)
echo "✓ Backed up ~/.bashrc"

# Add KVS environment variables
cat << 'EOF' >> ~/.bashrc

# ============================================================================
# AWS KVS Environment Configuration (added $(date))
# ============================================================================
export GST_PLUGIN_PATH=$GST_PLUGIN_PATH:/home/omar/amazon-kinesis-video-streams-producer-sdk-cpp/build
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/home/omar/amazon-kinesis-video-streams-producer-sdk-cpp/build

# AWS Credentials for KVS
export AWS_ACCESS_KEY_ID="AKIAQFLXNXMMILSXZBPW"
export AWS_SECRET_ACCESS_KEY="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"
export AWS_DEFAULT_REGION="eu-west-1"
EOF

echo "✓ Added KVS environment variables to ~/.bashrc"

# Source bashrc for current session
source ~/.bashrc
echo "✓ Sourced ~/.bashrc"

echo ""
echo "=========================================="
echo "STEP 3: VERIFY GSTREAMER PLUGIN"
echo "=========================================="
if gst-inspect-1.0 kvssink &>/dev/null; then
    echo "✓ kvssink plugin found"
    gst-inspect-1.0 kvssink | head -n 10
else
    echo "✗ ERROR: kvssink plugin NOT found!"
    echo "  Check: $GST_PLUGIN_PATH"
    exit 1
fi

echo ""
echo "=========================================="
echo "STEP 4: BUILD ROS WORKSPACE"
echo "=========================================="
cd ~/catkin_ws
source /opt/ros/melodic/setup.bash
catkin_make
echo "✓ catkin_make completed"
source devel/setup.bash
echo "✓ Workspace sourced"

echo ""
echo "=========================================="
echo "STEP 5: INSTALL SYSTEMD SERVICE"
echo "=========================================="
sudo cp ~/catkin_ws/src/elderly_bot/config/elderly-bot-kvs.service /etc/systemd/system/
echo "✓ Service file copied to /etc/systemd/system/"

sudo systemctl daemon-reload
echo "✓ systemd daemon reloaded"

sudo systemctl enable elderly-bot-kvs.service
echo "✓ Service enabled (will start on boot)"

echo ""
echo "=========================================="
echo "INSTALLATION COMPLETE!"
echo "=========================================="
echo ""
echo "IMPORTANT: LOG OUT AND LOG BACK IN to apply video group membership"
echo ""
echo "After re-login, test with:"
echo "  roslaunch elderly_bot kvs_stream.launch"
echo ""
echo "Or start the systemd service:"
echo "  sudo systemctl start elderly-bot-kvs.service"
echo "  sudo systemctl status elderly-bot-kvs.service"
echo ""
echo "View stream at:"
echo "  https://eu-west-1.console.aws.amazon.com/kinesisvideo/home?region=eu-west-1#/streams/RobotStream"
echo ""
