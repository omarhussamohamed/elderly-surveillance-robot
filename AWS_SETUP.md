# AWS IoT Core Configuration Summary
# ====================================

## Your AWS Account Details

```
Account ID: 011524750104
Region:     eu-north-1
Endpoint:   a1k8itxfx77i0w-ats.iot.eu-north-1.amazonaws.com
Policy:     RobotFullAccess (arn:aws:iot:eu-north-1:011524750104:policy/RobotFullAccess)
Client ID:  robot_nano
Port:       443 (ALPN enabled)
```

## Required AWS IoT Policy

**Policy Name:** RobotFullAccess

**For Testing (Recommended - Permissive):**
```json
{
  "Version": "2012-10-17",
  "Statement": [
    {
      "Effect": "Allow",
      "Action": "iot:*",
      "Resource": "*"
    }
  ]
}
```

**For Production (Restrictive):**
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

## How to Update Policy

1. **Go to AWS Console:**
   ```
   https://eu-north-1.console.aws.amazon.com/iot/home?region=eu-north-1#/policyhub/RobotFullAccess
   ```

2. **Edit Policy:**
   - Click "Edit active version"
   - Replace entire JSON with permissive policy above
   - Click "Save as new version"

3. **Verify Attachment:**
   - Go to: Security → Certificates → [Your Certificate]
   - Go to "Policies" tab
   - Verify "RobotFullAccess" is listed
   - If not: Click "Attach policy" → Select "RobotFullAccess"

4. **Activate Certificate:**
   - Certificate status must be "Active" (green)
   - If not: Click certificate → "Activate" button

## Test Connection

```bash
cd ~/catkin_ws/src/elderly_bot/scripts
python aws_connection_test.py
```

**Expected Output:**
```
✅ CONNECTION SUCCESSFUL!
```

## Launch with Cloud Enabled

```bash
cd ~/catkin_ws
source devel/setup.bash
roslaunch elderly_bot bringup.launch enable_cloud:=true
```

## Monitor in AWS Console

**MQTT Test Client:**
```
https://eu-north-1.console.aws.amazon.com/iot/home?region=eu-north-1#/test
```

**Subscribe to topics:**
- `elderly_bot/telemetry` (every 1 second)
- `elderly_bot/alerts` (when gas detected)

**Publish command:**
Topic: `elderly_bot/commands`
Message:
```json
{
  "command": "buzzer",
  "value": 2000
}
```

## Troubleshooting

| Issue | Fix |
|-------|-----|
| "Not authorized to connect" | Update policy to `iot:*` on `*` |
| "Certificate verify failed" | Check certificate is Active in AWS Console |
| Immediate disconnect | Verify policy attached to certificate |
| Connection timeout | Use port 443 (already configured) |

## Certificate Locations

```
Root CA:    /home/omar/catkin_ws/src/elderly_bot/aws_certs/AmazonRootCA1.pem
Device Cert: /home/omar/catkin_ws/src/elderly_bot/aws_certs/certificate.pem.crt
Private Key: /home/omar/catkin_ws/src/elderly_bot/aws_certs/private.pem.key
```

## Configuration Files

- `config/cloud_config.yaml` - Main configuration (client_id, endpoint, port)
- `scripts/cloud_bridge_node.py` - ROS-to-AWS bridge (Python 2.7)
- `scripts/aws_connection_test.py` - Standalone test (no ROS required)

## Quick Commands

```bash
# Test AWS connection
python scripts/aws_connection_test.py

# Check certificates
ls -lh aws_certs/*.pem*

# Validate YAML
python -c "import yaml; yaml.safe_load(open('config/cloud_config.yaml'))"

# Launch with cloud
roslaunch elderly_bot bringup.launch enable_cloud:=true

# Monitor logs
tail -f ~/.ros/log/latest/cloud_bridge_node*.log
```
