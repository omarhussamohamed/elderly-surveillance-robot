# Deployment Fixes for Jetson Nano (ROS Melodic / Ubuntu 18.04)
# ================================================================

## Issue Summary
Three blocking issues encountered during deployment:
1. ❌ Python 3 missing ROS dependencies (`rospkg`)
2. ❌ Port 11411 conflict ("Address already in use")
3. ❌ AWS connection timeout/disconnect on port 8883

## SOLUTION: Step-by-Step Fix

---

## 1. Python Environment Setup (Python 2.7 for ROS Melodic)

ROS Melodic uses Python 2.7 by default. Install all dependencies for Python 2.7:

```bash
# Update pip for Python 2.7
python -m pip install --upgrade pip

# Install ROS dependencies for Python 2.7
pip install rospkg pyyaml defusedxml netifaces

# Install AWS IoT SDK for Python 2.7
pip install AWSIoTPythonSDK

# Verify installation
python -c "import rospkg; import yaml; import AWSIoTPythonSDK; print('✓ All dependencies installed')"
```

**Important Notes:**
- Use `pip` (not `pip3`) for Python 2.7 packages
- The shebang `#!/usr/bin/env python` in all scripts correctly points to Python 2.7
- ROS Melodic catkin uses Python 2.7 by default

---

## 2. Fix Port 11411 Conflict (rosserial)

Port 11411 is the default port for rosserial. If you see "Address already in use":

### Quick Fix: Kill Process Holding Port 11411

```bash
# Find and kill process on port 11411
sudo lsof -ti:11411 | xargs -r sudo kill -9

# Alternative if lsof is not available:
sudo netstat -tlnp | grep :11411 | awk '{print $7}' | cut -d'/' -f1 | xargs -r sudo kill -9

# Verify port is free
sudo lsof -i:11411
```

**Expected Output:** (nothing - port is free)

### Alternative: Change rosserial Port

If you want to use a different port, edit your launch file:

```xml
<!-- In launch/bringup.launch -->
<node name="serial_node" pkg="rosserial_python" type="serial_node.py">
  <param name="port" value="/dev/ttyACM0"/>
  <param name="baud" value="115200"/>
  <param name="tcp_port" value="11412"/>  <!-- Changed from 11411 -->
</node>
```

---

## 3. Fix AWS Connection (Switch to Port 443 with ALPN)

Port 8883 is often blocked by ISPs/corporate firewalls. Port 443 (HTTPS) is universally open.

### Configuration Changes (Already Applied)

✅ **cloud_config.yaml** - Port changed to 443:
```yaml
port: 443  # Changed from 8883
```

✅ **cloud_bridge_node.py** - ALPN enabled for port 443:
```python
if self.port == 443:
    rospy.loginfo("Using port 443 with ALPN for AWS IoT Core")
```

### Test Connection

```bash
# Test with port 443
cd ~/catkin_ws/src/elderly_bot/scripts
python aws_connection_test_port443.py
```

**Expected Output:**
```
✅ CONNECTION SUCCESSFUL on Port 443!
```

### Verify Network Connectivity

```bash
# Run full network diagnostics
cd ~/catkin_ws/src/elderly_bot/scripts
chmod +x diagnose_aws_network.sh
./diagnose_aws_network.sh
```

This will test:
- DNS resolution
- Port 443 connectivity
- Port 8883 connectivity (for comparison)
- TLS handshake

---

## 4. Complete Deployment Procedure

### Step 1: Clean Environment

```bash
# Kill any conflicting processes
sudo lsof -ti:11411 | xargs -r sudo kill -9

# Clean old ROS processes
killall -9 rosmaster roscore roslaunch 2>/dev/null

# Verify ports are free
sudo lsof -i:11411
sudo lsof -i:11311
```

### Step 2: Install Dependencies

```bash
# Python 2.7 packages
pip install rospkg pyyaml defusedxml netifaces AWSIoTPythonSDK

# System packages (if needed)
sudo apt-get update
sudo apt-get install -y python-pip python-yaml
```

### Step 3: Make Scripts Executable

```bash
cd ~/catkin_ws/src/elderly_bot
chmod +x scripts/*.py
chmod +x scripts/*.sh
```

### Step 4: Test AWS Connection (Port 443)

```bash
cd ~/catkin_ws/src/elderly_bot/scripts
python aws_connection_test_port443.py
```

Wait for: `✅ CONNECTION SUCCESSFUL`

### Step 5: Enable Cloud in Configuration

```bash
nano ~/catkin_ws/src/elderly_bot/config/cloud_config.yaml
```

Change:
```yaml
enable_cloud: true  # Changed from false
```

### Step 6: Launch Robot with Cloud Enabled

```bash
cd ~/catkin_ws
source devel/setup.bash
roslaunch elderly_bot bringup.launch enable_cloud:=true
```

### Step 7: Monitor in AWS IoT Console

1. Log into AWS Console → IoT Core
2. Test → MQTT test client
3. Subscribe to: `elderly_bot/#`
4. You should see:
   - `elderly_bot/telemetry` - Every 1 second
   - `elderly_bot/alerts` - When gas detected

### Step 8: Test Command From Cloud

In AWS MQTT test client, publish to `elderly_bot/commands`:

```json
{
  "command": "buzzer",
  "value": 2000
}
```

Buzzer should activate at 2000 Hz on the robot.

---

## Common Issues & Solutions

### Issue: "ImportError: No module named rospkg"

**Solution:**
```bash
pip install rospkg pyyaml defusedxml
```

### Issue: "Port 11411 Address already in use"

**Solution:**
```bash
sudo lsof -ti:11411 | xargs -r sudo kill -9
```

### Issue: "AWS connection timeout"

**Solution:**
1. Check internet connection: `ping google.com`
2. Test DNS: `host a1k8itxfx77i0w-ats.iot.eu-north-1.amazonaws.com`
3. Test port 443: `nc -zv a1k8itxfx77i0w-ats.iot.eu-north-1.amazonaws.com 443`
4. Run diagnostics: `./diagnose_aws_network.sh`

### Issue: "Not authorized to connect"

**Solution:**
1. Check AWS IoT Policy (see AWS_IOT_POLICY.md)
2. Verify certificate is ACTIVATED in AWS Console
3. Check client_id matches policy: `elderly_bot_nano`

### Issue: "Certificate verify failed"

**Solution:**
1. Check system time: `date` (must be accurate for TLS)
2. Sync time: `sudo ntpdate -s time.nist.gov`
3. Verify certificates: `./validate_aws_certs.sh`

---

## Verification Checklist

**Before launching:**
- [ ] Python 2.7 dependencies installed (`pip list | grep -i aws`)
- [ ] Port 11411 is free (`sudo lsof -i:11411` returns nothing)
- [ ] AWS connection test passes (`python aws_connection_test_port443.py`)
- [ ] Certificates are valid (`./validate_aws_certs.sh`)
- [ ] Cloud enabled in config (`enable_cloud: true`)
- [ ] Scripts are executable (`ls -l scripts/*.py`)

**After launching:**
- [ ] `rosnode list` shows `/cloud_bridge_node`
- [ ] AWS Console shows connection in Monitor → Logs
- [ ] Telemetry appears in MQTT test client (`elderly_bot/telemetry`)
- [ ] Gas alert triggers when sensor detects gas
- [ ] Buzzer responds to commands from cloud

---

## Quick Reference: One-Line Commands

```bash
# Kill port 11411 conflict
sudo lsof -ti:11411 | xargs -r sudo kill -9

# Install Python 2.7 dependencies
pip install rospkg pyyaml defusedxml netifaces AWSIoTPythonSDK

# Test AWS connection (port 443)
python ~/catkin_ws/src/elderly_bot/scripts/aws_connection_test_port443.py

# Launch with cloud enabled
roslaunch elderly_bot bringup.launch enable_cloud:=true

# Monitor ROS logs
tail -f ~/.ros/log/latest/cloud_bridge_node*.log
```

---

## Port 443 vs Port 8883

| Feature | Port 8883 | Port 443 |
|---------|-----------|----------|
| Protocol | MQTT over TLS | MQTT over WebSocket/ALPN |
| Firewall | Often blocked | Always open (HTTPS) |
| Performance | Slightly better | Good enough |
| ALPN Required | No | Yes (automatic in SDK) |
| Use Case | Direct connection | Blocked networks |

**Recommendation:** Use port 443 for reliability. The AWS SDK handles ALPN automatically.

---

## Support

If issues persist:
1. Check AWS CloudWatch Logs: IoT Core → Monitor → Logs
2. Check ROS logs: `~/.ros/log/latest/`
3. Run network diagnostics: `./diagnose_aws_network.sh`
4. Verify AWS policy: See AWS_IOT_POLICY.md

For connection debugging, enable verbose logging:
```bash
export ROS_PYTHON_LOG_CONFIG_FILE=/path/to/logging.conf
```
