# AWS IoT Core Integration - Deployment Guide

## 🎯 Overview

This guide covers AWS IoT Core integration for the Elderly Bot, enabling cloud monitoring and remote control.

---

## 📋 Prerequisites

1. **Hardware**: Jetson Nano with internet connectivity
2. **Software**: ROS Melodic, Python 3.x
3. **AWS Account**: Active AWS account with IoT Core access
4. **Certificates**: Downloaded from AWS IoT Core console

---

## 🚀 Quick Start (5 Steps)

### **Step 1: Install AWS IoT SDK**

```bash
cd ~/catkin_ws/src/elderly_bot
pip3 install AWSIoTPythonSDK pyyaml
```

### **Step 2: Validate Certificates**

```bash
cd ~/catkin_ws/src/elderly_bot/scripts
chmod +x validate_aws_certs.sh
./validate_aws_certs.sh
```

**Expected output:**
```
✅ ALL VALIDATIONS PASSED
✓ Root CA certificate is valid
✓ Device certificate is valid
✓ Private key and certificate MATCH
```

### **Step 3: Configure AWS Endpoint**

Edit `config/cloud_config.yaml`:

```bash
nano ~/catkin_ws/src/elderly_bot/config/cloud_config.yaml
```

**Update these fields:**
```yaml
aws_endpoint: "YOUR_ENDPOINT-ats.iot.us-east-1.amazonaws.com"  # From AWS console
client_id: "elderly_bot_01"  # Must match AWS Thing name
enable_cloud: false  # Keep false until Step 4 passes
```

**Get your endpoint:**
1. Go to [AWS IoT Core Console](https://console.aws.amazon.com/iot/)
2. Click **Settings** (left menu)
3. Copy **Device data endpoint**

### **Step 4: Test Connection (Without ROS)**

```bash
cd ~/catkin_ws/src/elderly_bot/scripts
python3 aws_connection_test.py
```

**Expected output:**
```
✅ CONNECTION SUCCESSFUL!
✅ AWS IoT CORE CONNECTION TEST PASSED
```

**If connection fails**, check:
- Internet connectivity: `ping 8.8.8.8`
- Endpoint URL (no typos)
- Certificate paths in `cloud_config.yaml`
- AWS IoT policy attached and activated

### **Step 5: Enable Cloud Bridge**

If Step 4 passed, update `config/cloud_config.yaml`:

```yaml
enable_cloud: true  # Now safe to enable
```

Launch with cloud bridge:

```bash
roslaunch elderly_bot bringup.launch enable_cloud:=true
```

**Monitor in terminal:**
```bash
rostopic echo /gas_detected
# Should see data flowing to AWS
```

**Monitor in AWS Console:**
1. Go to AWS IoT Core → **Test** → **MQTT test client**
2. Subscribe to: `robot/sensors/data`
3. You should see JSON messages every second

---

## 📂 File Structure

```
elderly_bot/
├── aws_certs/
│   ├── AmazonRootCA1.pem          # Root CA (from Amazon)
│   ├── certificate.pem.crt        # Device certificate (from AWS IoT)
│   ├── private.pem.key            # Private key (KEEP SECURE!)
│   └── .gitignore                 # Prevents git commits
├── config/
│   └── cloud_config.yaml          # AWS connection config
├── scripts/
│   ├── cloud_bridge_node.py       # Main ROS bridge node
│   ├── aws_connection_test.py     # Standalone connection test
│   ├── validate_aws_certs.sh      # Certificate validation
│   └── setup_aws_iot.sh           # Automated setup script
└── launch/
    └── bringup.launch             # Main launch file (enable_cloud arg)
```

---

## 🔒 Security Best Practices

### Certificate Protection

1. **Never commit certificates to git**
   - Already configured in `.gitignore`
   - Verify: `git status` should NOT show `.pem` files

2. **Set correct permissions**
   ```bash
   chmod 600 ~/catkin_ws/src/elderly_bot/aws_certs/private.pem.key
   chmod 644 ~/catkin_ws/src/elderly_bot/aws_certs/*.pem.crt
   ```

3. **Backup certificates securely**
   ```bash
   # Create encrypted backup
   tar -czf aws_certs_backup.tar.gz aws_certs/
   gpg -c aws_certs_backup.tar.gz
   # Delete unencrypted backup
   rm aws_certs_backup.tar.gz
   ```

### AWS IoT Policy

Minimum required policy for robot:

```json
{
  "Version": "2012-10-17",
  "Statement": [
    {
      "Effect": "Allow",
      "Action": "iot:Connect",
      "Resource": "arn:aws:iot:REGION:ACCOUNT:client/elderly_bot_01"
    },
    {
      "Effect": "Allow",
      "Action": ["iot:Publish", "iot:Receive"],
      "Resource": "arn:aws:iot:REGION:ACCOUNT:topic/robot/*"
    },
    {
      "Effect": "Allow",
      "Action": "iot:Subscribe",
      "Resource": "arn:aws:iot:REGION:ACCOUNT:topicfilter/robot/*"
    }
  ]
}
```

Replace `REGION` and `ACCOUNT` with your AWS values.

---

## 📡 Message Formats

### Robot → Cloud (Published)

**Topic:** `robot/sensors/data`

```json
{
  "timestamp": 1234567890.123,
  "robot_id": "elderly_bot_01",
  "sensors": {
    "gas_detected": false,
    "temperature": 45.2,
    "power": 12.5
  }
}
```

### Cloud → Robot (Subscribed)

**Topic:** `robot/commands`

**Buzzer control:**
```json
{"command": "buzzer", "value": true}
```

**Emergency stop:**
```json
{"command": "stop"}
```

**Movement (use with caution!):**
```json
{
  "command": "move",
  "value": {
    "linear": 0.1,
    "angular": 0.0
  }
}
```

---

## 🔧 Troubleshooting

### Connection Timeout

**Symptom:** `Connection timeout` or `Failed to connect`

**Solutions:**
1. Check internet: `ping google.com`
2. Verify endpoint URL (no typos)
3. Check firewall allows port 8883
4. Test with `aws_connection_test.py`

### Certificate Verification Failed

**Symptom:** `Certificate verification failed` or `SSL handshake failed`

**Solutions:**
1. Run `validate_aws_certs.sh` to check integrity
2. Ensure certificate is **activated** in AWS IoT Console
3. Check policy is attached to certificate
4. Verify file permissions (600 for private key)

### Permission Denied

**Symptom:** `Not authorized` or `Connection refused (5)`

**Solutions:**
1. Verify IoT policy attached to certificate
2. Check Thing name matches `client_id` in config
3. Ensure policy allows `iot:Connect`, `iot:Publish`, `iot:Subscribe`

### SDK Not Installed

**Symptom:** `AWSIoTPythonSDK not found`

**Solution:**
```bash
pip3 install AWSIoTPythonSDK
```

### Wrong Python Version

**Symptom:** `cloud_bridge_node.py` fails to start

**Solution:**
- Node uses Python 3 (shebang: `#!/usr/bin/env python3`)
- Check: `python3 --version` (should be 3.6+)

---

## 🧪 Testing Checklist

- [ ] AWS SDK installed: `python3 -c "import AWSIoTPythonSDK"`
- [ ] Certificates validated: `./validate_aws_certs.sh`
- [ ] Config updated: `cloud_config.yaml` has correct endpoint
- [ ] Standalone test passed: `python3 aws_connection_test.py`
- [ ] ROS integration works: `roslaunch elderly_bot bringup.launch enable_cloud:=true`
- [ ] Messages visible in AWS console: MQTT test client subscribed to `robot/sensors/data`
- [ ] Commands work: Publish `{"command": "buzzer", "value": true}` to `robot/commands`

---

## 🚀 Production Deployment

### 1. Enable Cloud by Default

Edit `config/cloud_config.yaml`:
```yaml
enable_cloud: true
```

### 2. Auto-Start with ROS

Edit `launch/bringup.launch`:
```xml
<arg name="enable_cloud" default="true" />
```

### 3. Systemd Service (Optional)

Create `/etc/systemd/system/elderly_bot.service`:

```ini
[Unit]
Description=Elderly Bot ROS System
After=network.target

[Service]
Type=simple
User=jetson
WorkingDirectory=/home/jetson/catkin_ws
ExecStart=/bin/bash -c "source /opt/ros/melodic/setup.bash && source /home/jetson/catkin_ws/devel/setup.bash && roslaunch elderly_bot bringup.launch enable_cloud:=true"
Restart=on-failure
RestartSec=10

[Install]
WantedBy=multi-user.target
```

Enable:
```bash
sudo systemctl enable elderly_bot
sudo systemctl start elderly_bot
```

---

## 📊 Monitoring

### ROS Topics

```bash
# Monitor gas sensor
rostopic echo /gas_detected

# Monitor temperature
rostopic echo /jetson_temperature

# Monitor cloud connection status
rostopic echo /cloud_status  # (if implemented)
```

### AWS Console

1. **MQTT Test Client**
   - Subscribe to: `robot/sensors/data`
   - Publish to: `robot/commands`

2. **CloudWatch Logs** (if configured)
   - View connection events
   - Monitor message throughput

3. **IoT Analytics** (if configured)
   - Historical sensor data
   - Anomaly detection

---

## 📚 Additional Resources

- [AWS IoT Core Documentation](https://docs.aws.amazon.com/iot/)
- [AWS IoT SDK for Python](https://github.com/aws/aws-iot-device-sdk-python)
- [ROS roslaunch Documentation](http://wiki.ros.org/roslaunch)

---

## 🆘 Support

If you encounter issues:

1. Check logs: `rosnode list` and `rosnode info cloud_bridge_node`
2. Run validation: `./validate_aws_certs.sh`
3. Test standalone: `python3 aws_connection_test.py`
4. Verify AWS policy and certificate activation

---

**Status:** ✅ Ready for Deployment

**Last Updated:** January 23, 2026
