#!/bin/bash
# Setup and verify Cloud Bridge functionality

echo "=== Cloud Bridge Setup Verification ==="
echo ""

# Check if running as omar
if [ "$USER" != "omar" ]; then
    echo "ERROR: Must run as user 'omar'"
    exit 1
fi

# 1. Check if AWS IoT Python SDK is installed
echo "1. Checking AWS IoT Python SDK..."
if python -c "import AWSIoTPythonSDK" 2>/dev/null; then
    echo "   ✓ AWSIoTPythonSDK installed"
else
    echo "   ✗ AWSIoTPythonSDK not installed"
    echo "   Installing with: pip install AWSIoTPythonSDK"
    pip install AWSIoTPythonSDK
fi

# 2. Check certificate files exist
echo ""
echo "2. Checking certificate files..."
CERTS_DIR="/home/omar/catkin_ws/src/elderly_bot/aws_certs"
REQUIRED_FILES=("AmazonRootCA1.pem" "certificate.pem.crt" "private.pem.key")

for file in "${REQUIRED_FILES[@]}"; do
    if [ -f "$CERTS_DIR/$file" ]; then
        echo "   ✓ $file exists"
    else
        echo "   ✗ $file NOT FOUND in $CERTS_DIR/"
        echo "   Download certificates from AWS IoT Console:"
        echo "   1. Go to AWS IoT Core → Manage → Things"
        echo "   2. Select 'robot_nano' thing"
        echo "   3. Click 'Security' → Download certificates"
    fi
done

# 3. Check configuration file
echo ""
echo "3. Checking configuration..."
if [ -f "/home/omar/catkin_ws/src/elderly_bot/config/cloud_config.yaml" ]; then
    echo "   ✓ cloud_config.yaml exists"
    # Check for critical parameters
    if grep -q "a1k8itxfx77i0w-ats.iot.us-east-1.amazonaws.com" /home/omar/catkin_ws/src/elderly_bot/config/cloud_config.yaml; then
        echo "   ✓ Endpoint configured"
    else
        echo "   ✗ Endpoint missing or incorrect"
    fi
else
    echo "   ✗ cloud_config.yaml not found"
fi

# 4. Verify node is executable
echo ""
echo "4. Checking Python node..."
if [ -f "/home/omar/catkin_ws/src/elderly_bot/scripts/cloud_bridge_node.py" ]; then
    echo "   ✓ cloud_bridge_node.py exists"
    chmod +x /home/omar/catkin_ws/src/elderly_bot/scripts/cloud_bridge_node.py
else
    echo "   ✗ cloud_bridge_node.py not found in scripts/"
fi

# 5. Check ROS environment
echo ""
echo "5. Checking ROS environment..."
if type roslaunch >/dev/null 2>&1; then
    echo "   ✓ ROS installed"
else
    echo "   ✗ ROS not found in PATH"
    echo "   Source with: source /opt/ros/melodic/setup.bash"
fi

# 6. Create a test launch file
echo ""
echo "6. Creating test launch file..."
cat > /tmp/test_cloud.launch << 'EOF'
<?xml version="1.0"?>
<launch>
  <node name="cloud_bridge_test" 
        pkg="elderly_bot" 
        type="cloud_bridge_node.py"
        output="screen">
    <rosparam>
      enable_cloud: true
      aws_endpoint: "a1k8itxfx77i0w-ats.iot.us-east-1.amazonaws.com"
      client_id: "robot_nano"
      port: 8883
      root_ca_path: "/home/omar/catkin_ws/src/elderly_bot/aws_certs/AmazonRootCA1.pem"
      cert_path: "/home/omar/catkin_ws/src/elderly_bot/aws_certs/certificate.pem.crt"
      key_path: "/home/omar/catkin_ws/src/elderly_bot/aws_certs/private.pem.key"
      mqtt_topic_telemetry: "elderly_bot/telemetry"
      mqtt_topic_alerts: "elderly_bot/alerts"
      mqtt_topic_commands: "elderly_bot/commands"
      publish_rate: 1.0
      keepalive_interval: 30
    </rosparam>
  </node>
</launch>
EOF
echo "   ✓ Test launch file created: /tmp/test_cloud.launch"

echo ""
echo "=== SETUP COMPLETE ==="
echo ""
echo "To run the Cloud Bridge:"
echo "Method 1: Using bringup.launch"
echo "  roslaunch elderly_bot bringup.launch enable_cloud:=true"
echo ""
echo "Method 2: Using dedicated launch file"
echo "  roslaunch elderly_bot cloud_bridge.launch"
echo ""
echo "Method 3: Direct node execution (for testing)"
echo "  rosrun elderly_bot cloud_bridge_node.py _enable_cloud:=true"
echo ""
echo "To test AWS IoT connection:"
echo "1. Go to AWS IoT Console → Test → MQTT test client"
echo "2. Subscribe to topic: elderly_bot/telemetry"
echo "3. Subscribe to topic: elderly_bot/alerts"
echo "4. Publish to topic: elderly_bot/commands"
echo "   {\"command\": \"buzzer\", \"value\": 1000}"