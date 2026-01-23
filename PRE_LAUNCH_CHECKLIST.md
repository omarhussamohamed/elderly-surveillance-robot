# AWS IoT Integration - Pre-Launch Verification Guide
# ====================================================

## System Configuration

**Thing Name:** robot_nano
**AWS Endpoint:** a1k8itxfx77i0w-ats.iot.eu-north-1.amazonaws.com
**Port:** 8883 (Standard MQTT over TLS)
**Region:** eu-north-1
**Account:** 011524750104

---

## Pre-Launch Checklist

### 1. Run Permissions Check

```bash
cd ~/catkin_ws/src/elderly_bot
chmod +x scripts/check_permissions.sh
./scripts/check_permissions.sh
```

**Expected Output:** `✅ ALL CHECKS PASSED`

### 2. Validate Configuration

```bash
# Check YAML syntax
python -c "import yaml; print(yaml.safe_load(open('config/cloud_config.yaml'))['aws_iot_config'])"
```

**Expected Output:**
```python
{
  'client_id': 'robot_nano',
  'endpoint': 'a1k8itxfx77i0w-ats.iot.eu-north-1.amazonaws.com',
  'port': 8883,
  'root_ca': '/home/omar/catkin_ws/src/elderly_bot/aws_certs/AmazonRootCA1.pem',
  'device_cert': '/home/omar/catkin_ws/src/elderly_bot/aws_certs/certificate.pem.crt',
  'private_key': '/home/omar/catkin_ws/src/elderly_bot/aws_certs/private.pem.key'
}
```

### 3. Test AWS Connection

```bash
cd ~/catkin_ws/src/elderly_bot/scripts
python final_handshake.py
```

**Expected Output:**
```
✅ FINAL HANDSHAKE SUCCESSFUL!
  ✓ Thing Name: robot_nano
  ✓ Client ID: robot_nano connected successfully
  ✓ Endpoint: a1k8itxfx77i0w-ats.iot.eu-north-1.amazonaws.com:8883
  ✓ Bidirectional communication verified
```

---

## Launch Commands

### Launch with Cloud Enabled

```bash
cd ~/catkin_ws
source devel/setup.bash
roslaunch elderly_bot bringup.launch enable_cloud:=true
```

### Monitor Cloud Bridge Node

```bash
# In separate terminal
rostopic echo /rosout | grep cloud_bridge
```

**Expected Messages:**
```
[ INFO] Cloud Bridge Node Starting
[ INFO] Configuration validated
[ INFO] Connecting to AWS IoT Core...
[ INFO]   Endpoint: a1k8itxfx77i0w-ats.iot.eu-north-1.amazonaws.com:8883
[ INFO]   Client ID: robot_nano
[ INFO] Connected to AWS IoT Core
[ INFO] Subscribed to MQTT topic: elderly_bot/commands
```

### View Published Telemetry

Open AWS Console → IoT Core → MQTT Test Client:
```
https://eu-north-1.console.aws.amazon.com/iot/home?region=eu-north-1#/test
```

Subscribe to: `elderly_bot/#`

**Expected Messages:**
- `elderly_bot/telemetry` (every 1 second)
- `elderly_bot/alerts` (when gas detected)

---

## Troubleshooting

### Issue: "Not authorized to connect"

**Cause:** AWS IoT Policy doesn't allow client_id `robot_nano`

**Fix:** Update policy Resource ARN:
```json
"Resource": "arn:aws:iot:eu-north-1:011524750104:client/robot_nano"
```

### Issue: "Certificate verify failed"

**Cause:** Certificate not ACTIVE or wrong path

**Fix:**
1. Verify certificate is ACTIVE in AWS Console
2. Check certificate paths: `ls -l ~/catkin_ws/src/elderly_bot/aws_certs/`
3. Verify private key permissions: `600`

### Issue: "Connection timeout"

**Cause:** Network connectivity or firewall

**Fix:**
```bash
# Test connectivity
ping -c 3 a1k8itxfx77i0w-ats.iot.eu-north-1.amazonaws.com

# Test port 8883
nc -zv a1k8itxfx77i0w-ats.iot.eu-north-1.amazonaws.com 8883
```

### Issue: "ImportError: No module named AWSIoTPythonSDK"

**Cause:** Python package not installed

**Fix:**
```bash
pip install AWSIoTPythonSDK pyyaml
```

---

## AWS Console Verification

### Check Thing Configuration

1. Go to: IoT Core → Manage → Things
2. Search for: `robot_nano`
3. Verify: Thing exists

### Check Certificate Status

1. Go to: IoT Core → Security → Certificates
2. Find your certificate
3. Verify: Status = "Active" (green)
4. Go to "Policies" tab
5. Verify: Policy is attached

### Check Policy Permissions

1. Go to: IoT Core → Security → Policies
2. Click: RobotFullAccess (or your policy name)
3. Verify JSON contains:

```json
{
  "Version": "2012-10-17",
  "Statement": [
    {
      "Effect": "Allow",
      "Action": "iot:Connect",
      "Resource": "arn:aws:iot:eu-north-1:011524750104:client/robot_nano"
    },
    {
      "Effect": "Allow",
      "Action": ["iot:Publish", "iot:Receive"],
      "Resource": "arn:aws:iot:eu-north-1:011524750104:topic/elderly_bot/*"
    },
    {
      "Effect": "Allow",
      "Action": "iot:Subscribe",
      "Resource": "arn:aws:iot:eu-north-1:011524750104:topicfilter/elderly_bot/*"
    }
  ]
}
```

---

## Files Modified in This Audit

1. **config/cloud_config.yaml** - Restructured with aws_iot_config section, port 8883
2. **scripts/cloud_bridge_node.py** - Updated default port, enhanced error handling
3. **scripts/aws_connection_test.py** - Changed to port 8883 default
4. **scripts/final_handshake.py** - Complete rewrite with dual config support
5. **scripts/check_permissions.sh** - New diagnostic script

---

## Success Criteria

- [x] All Python scripts executable (`chmod +x`)
- [x] Private key permissions = 600
- [x] Client ID = robot_nano everywhere
- [x] Port = 8883 (standard MQTT/TLS)
- [x] Endpoint matches AWS Console
- [x] Certificate paths absolute and correct
- [x] final_handshake.py passes
- [x] No identity mismatches in logs

---

## Quick Commands Reference

```bash
# Check permissions
./scripts/check_permissions.sh

# Test connection
python scripts/final_handshake.py

# Launch ROS with cloud
roslaunch elderly_bot bringup.launch enable_cloud:=true

# Monitor logs
tail -f ~/.ros/log/latest/cloud_bridge_node*.log

# Check ROS topics
rostopic list | grep -E "jetson|gas|buzzer"

# Test publish to AWS
rostopic pub /gas_detected std_msgs/Bool "data: true" --once
```

---

**Status:** READY FOR DEPLOYMENT
**Last Updated:** January 23, 2026
**Standardized Identity:** robot_nano
