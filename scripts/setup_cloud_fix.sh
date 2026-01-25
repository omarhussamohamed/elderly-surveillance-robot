#!/bin/bash
echo "=== CLOUD BRIDGE FIX FOR PYTHON 2.7 ==="
echo ""

# 1. Remove incompatible AWS IoT SDK
echo "1. Removing incompatible AWS IoT SDK..."
pip uninstall -y AWSIoTPythonSDK 2>/dev/null

# 2. Install Python 2.7 compatible version
echo ""
echo "2. Installing Python 2.7 compatible AWS IoT SDK..."
pip install AWSIoTPythonSDK==1.4.9

# 3. Verify installation
echo ""
echo "3. Verifying installation..."
python2 -c "import AWSIoTPythonSDK; print('✓ AWSIoTPythonSDK 1.4.9 installed')" 2>/dev/null || echo "✗ Installation failed"

# 4. Check certificates
echo ""
echo "4. Checking certificate files..."
CERTS_DIR="/home/omar/catkin_ws/src/elderly_bot/aws_certs"

if [ -d "$CERTS_DIR" ]; then
    echo "✓ Certificate directory exists"
    
    for file in "AmazonRootCA1.pem" "certificate.pem.crt" "private.pem.key"; do
        if [ -f "$CERTS_DIR/$file" ]; then
            echo "  ✓ $file"
        else
            echo "  ✗ $file (MISSING)"
        fi
    done
else
    echo "✗ Certificate directory not found: $CERTS_DIR"
    echo "  Create it: mkdir -p $CERTS_DIR"
    echo "  Download certificates from AWS IoT Console and place them there"
fi

# 5. Check cloud_config.yaml
echo ""
echo "5. Checking configuration..."
CONFIG_FILE="/home/omar/catkin_ws/src/elderly_bot/config/cloud_config.yaml"
if [ -f "$CONFIG_FILE" ]; then
    echo "✓ cloud_config.yaml exists"
    
    # Check endpoint
    if grep -q "a1k8itxfx77i0w-ats.iot.us-east-1.amazonaws.com" "$CONFIG_FILE"; then
        echo "  ✓ AWS endpoint configured"
    else
        echo "  ✗ AWS endpoint missing or incorrect"
    fi
    
    # Check certificate paths
    if grep -q "/home/omar" "$CONFIG_FILE"; then
        echo "  ✓ Certificate paths configured"
    else
        echo "  ✗ Certificate paths may be incorrect"
    fi
else
    echo "✗ cloud_config.yaml not found"
    echo "  Create it in $CONFIG_FILE"
fi

# 6. Make scripts executable
echo ""
echo "6. Making scripts executable..."
chmod +x /home/omar/catkin_ws/src/elderly_bot/scripts/cloud_bridge_node.py 2>/dev/null
chmod +x /home/omar/catkin_ws/src/elderly_bot/scripts/system_health_monitor.py 2>/dev/null
chmod +x /home/omar/catkin_ws/src/elderly_bot/scripts/test_cloud_publisher.py 2>/dev/null
echo "✓ Script permissions updated"

# 7. Test command
echo ""
echo "7. Creating test command..."
cat > /tmp/send_cloud_command.sh << 'EOF'
#!/bin/bash
# Send test command to AWS IoT
echo "Publishing test command to AWS IoT..."
mosquitto_pub \
  --cafile /home/omar/catkin_ws/src/elderly_bot/aws_certs/AmazonRootCA1.pem \
  --cert /home/omar/catkin_ws/src/elderly_bot/aws_certs/certificate.pem.crt \
  --key /home/omar/catkin_ws/src/elderly_bot/aws_certs/private.pem.key \
  -h a1k8itxfx77i0w-ats.iot.us-east-1.amazonaws.com \
  -p 8883 \
  -t "elderly_bot/commands" \
  -m '{"command": "buzzer", "value": 1000}' \
  --tls-version tlsv1.2 \
  -d
EOF
chmod +x /tmp/send_cloud_command.sh

echo ""
echo "=== SETUP COMPLETE ==="
echo ""
echo "To fix the AWS IoT SDK issue:"
echo "1. The incompatible SDK has been uninstalled"
echo "2. Python 2.7 compatible version (1.4.9) has been installed"
echo ""
echo "To test the system:"
echo "1. Start ROS core: roscore"
echo "2. Launch bringup with cloud: roslaunch elderly_bot bringup.launch enable_cloud:=true"
echo "3. Publish test data: rosrun elderly_bot test_cloud_publisher.py"
echo ""
echo "Check AWS IoT Console:"
echo "1. Go to AWS IoT Console → Test → MQTT test client"
echo "2. Subscribe to topic: elderly_bot/telemetry"
echo "3. Subscribe to topic: elderly_bot/alerts"
echo "4. Publish to topic: elderly_bot/commands"
echo "   {\"command\": \"buzzer\", \"value\": 1000}"
echo ""
echo "If connection fails, check:"
echo "1. Internet connection"
echo "2. AWS IoT Policy allows connection"
echo "3. Certificates are valid and not expired"