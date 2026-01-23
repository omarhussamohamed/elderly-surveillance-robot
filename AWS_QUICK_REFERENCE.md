# AWS IoT Core - Quick Reference

## 🚀 5-Minute Setup

```bash
# 1. Install SDK
pip3 install AWSIoTPythonSDK pyyaml

# 2. Validate certificates
cd ~/catkin_ws/src/elderly_bot/scripts
chmod +x *.sh
./validate_aws_certs.sh

# 3. Edit config (add your AWS endpoint)
nano ~/catkin_ws/src/elderly_bot/config/cloud_config.yaml

# 4. Test connection (without ROS)
python3 aws_connection_test.py

# 5. Enable and launch
# Edit cloud_config.yaml: set enable_cloud: true
roslaunch elderly_bot bringup.launch enable_cloud:=true
```

---

## 📁 Certificate Files

| File | Location | Permissions | Source |
|------|----------|-------------|--------|
| `AmazonRootCA1.pem` | `aws_certs/` | 644 | [Amazon Trust](https://www.amazontrust.com/repository/AmazonRootCA1.pem) |
| `certificate.pem.crt` | `aws_certs/` | 644 | AWS IoT Console |
| `private.pem.key` | `aws_certs/` | **600** | AWS IoT Console (SECURE!) |

**Set permissions:**
```bash
chmod 600 ~/catkin_ws/src/elderly_bot/aws_certs/private.pem.key
chmod 644 ~/catkin_ws/src/elderly_bot/aws_certs/*.pem
chmod 644 ~/catkin_ws/src/elderly_bot/aws_certs/*.crt
```

---

## 🔧 Configuration Template

**File:** `config/cloud_config.yaml`

```yaml
cloud_bridge_node:
  enable_cloud: true  # false until tested
  aws_endpoint: "YOUR_ENDPOINT-ats.iot.us-east-1.amazonaws.com"
  client_id: "elderly_bot_01"  # Must match AWS Thing name
  port: 8883
  
  # Certificate paths (relative to home ~)
  root_ca_path: "~/catkin_ws/src/elderly_bot/aws_certs/AmazonRootCA1.pem"
  cert_path: "~/catkin_ws/src/elderly_bot/aws_certs/certificate.pem.crt"
  key_path: "~/catkin_ws/src/elderly_bot/aws_certs/private.pem.key"
  
  # MQTT topics
  mqtt_topic_sensors: "robot/sensors/data"
  mqtt_topic_commands: "robot/commands"
```

---

## 🧪 Validation Scripts

| Script | Purpose | Usage |
|--------|---------|-------|
| `validate_aws_certs.sh` | Check certificate integrity | `./validate_aws_certs.sh` |
| `aws_connection_test.py` | Test AWS connection | `python3 aws_connection_test.py` |
| `setup_aws_iot.sh` | Automated setup | `./setup_aws_iot.sh` |

---

## 📡 MQTT Topics

### Robot → Cloud (Publish)

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

### Cloud → Robot (Subscribe)

**Topic:** `robot/commands`

```json
{"command": "buzzer", "value": true}
{"command": "stop"}
```

---

## 🔍 Testing Commands

```bash
# Check SDK installed
python3 -c "import AWSIoTPythonSDK; print('OK')"

# Validate certificates
./validate_aws_certs.sh

# Test AWS connection (no ROS)
python3 aws_connection_test.py

# Launch with cloud
roslaunch elderly_bot bringup.launch enable_cloud:=true

# Monitor sensor data
rostopic echo /gas_detected

# Check cloud node status
rosnode info cloud_bridge_node
```

---

## ⚠️ Common Issues

| Error | Solution |
|-------|----------|
| `AWSIoTPythonSDK not found` | `pip3 install AWSIoTPythonSDK` |
| `Connection timeout` | Check internet, verify endpoint URL |
| `Certificate verification failed` | Run `validate_aws_certs.sh`, check AWS console |
| `Permission denied` | Check AWS IoT policy attached and activated |
| `SSL handshake failed` | Check certificate expiration, verify paths |

---

## 🔒 Security Checklist

- [x] Certificates in `.gitignore` (never commit!)
- [x] Private key permissions: `chmod 600`
- [x] AWS IoT policy attached and activated
- [x] Thing name matches `client_id`
- [x] Policy allows `iot:Connect`, `iot:Publish`, `iot:Subscribe`

---

## 📊 AWS Console Quick Links

1. **Get Endpoint:**
   - AWS IoT Core → Settings → Device data endpoint

2. **Create Thing:**
   - AWS IoT Core → Manage → Things → Create

3. **Download Certificates:**
   - During Thing creation → Auto-generate certificate

4. **Test MQTT:**
   - AWS IoT Core → Test → MQTT test client
   - Subscribe to: `robot/sensors/data`

5. **Create Policy:**
   - AWS IoT Core → Security → Policies → Create

---

## 🚀 Launch Commands

```bash
# Test mode (cloud disabled)
roslaunch elderly_bot bringup.launch

# Production mode (cloud enabled)
roslaunch elderly_bot bringup.launch enable_cloud:=true

# Test AWS only (no robot)
python3 ~/catkin_ws/src/elderly_bot/scripts/aws_connection_test.py
```

---

## 📞 Get AWS Endpoint

```bash
# From AWS CLI (if configured)
aws iot describe-endpoint --endpoint-type iot:Data-ATS

# Or from console
# AWS IoT Core → Settings → Copy "Device data endpoint"
```

---

**Last Updated:** January 23, 2026
