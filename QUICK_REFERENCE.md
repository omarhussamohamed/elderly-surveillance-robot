# Elderly Bot - Quick Reference Guide

## 🚀 One-Command Setup & Launch

```bash
# Run with bash (no chmod needed)
bash ~/catkin_ws/src/elderly_bot/update_robot.sh
```

**This single command does everything:**
- ✅ Cleans redundant files automatically
- ✅ Configures environment (~/.bashrc)
- ✅ Upgrades jetson-stats with pip3 (fixes authentication & dictionary parsing)
- ✅ Adds user to jetson_stats group
- ✅ Fixes GPIO permissions permanently
- ✅ Updates code from Git
- ✅ Builds workspace
- ✅ Deploys scripts
- ✅ Launches the robot

**No chmod required** - just run with `bash`

---

## 🔊 Buzzer Control (Manual Only - Latching)

**IMPORTANT:** Buzzer is completely decoupled from gas sensor.
- No automatic gas alarms
- Manual control only via /buzzer_command

### Start Continuous Beeping (Indefinite)
```bash
rostopic pub /buzzer_command std_msgs/Bool "data: true"
```
**Buzzer will beep continuously (0.1s ON/OFF) forever until you send false.**

### Stop Beeping
```bash
rostopic pub /buzzer_command std_msgs/Bool "data: false"
```

### Monitor Buzzer Status
```bash
# Watch the logs to see buzzer activity
rostopic echo /rosout | grep BUZZER
```

**Implementation Details:**
- ✅ **Manual control only** - NO automatic gas alarm
- ✅ **Latching behavior** - stays ON indefinitely until stopped
- ✅ Thread-based continuous beeping (not momentary)
- ✅ State-locked to prevent race conditions
- ✅ Logs every 10 seconds to confirm it's still running
- ✅ Clean shutdown on node exit

**Gas Sensor:**
- Gas detection still published to /gas_detected topic
- Gas level still published to /gas_level topic  
- **No connection to buzzer** - you must manually activate buzzer if needed

---

## 🔧 Jetson Stats Fix

The update_robot.sh automatically:
1. ✅ Upgrades jetson-stats: `sudo pip3 install -U jetson-stats`
2. ✅ Adds user to jetson_stats group: `sudo usermod -aG jetson_stats $USER`
3. ✅ Restarts jtop service: `sudo systemctl restart jtop.service`
4. ✅ Handles nested dictionary values: `data['temp']` instead of `float(data)`

**Manual fix if needed:**
```bash
# Upgrade jetson-stats (use pip3, not pip)
sudo pip3 install -U jetson-stats

# Add user to group (try both group names)
sudo usermod -aG jetson_stats $USER

# Logout and login for group membership to take effect
logout
```

**Dictionary Parsing Fix:**
The code now handles nested dictionaries like:
```python
{'CPU': {'temp': 33.5, 'online': True}}
# Extracts: temp = 33.5
```

---

## 📊 Monitoring Topics

```bash
# Gas sensor
rostopic echo /gas_detected
rostopic echo /gas_level

# Jetson health
rostopic echo /jetson_temperature
rostopic echo /jetson_power

# Buzzer status (check logs)
rostopic echo /rosout | grep -i buzzer
```

---

## 🐛 Troubleshooting

### Buzzer stops after 0.1 seconds

**This should not happen anymore** - buzzer runs indefinitely.

**Check:**
```bash
# Look for continuous beeping logs
rostopic echo /rosout | grep "Still beeping"
# Should see: "[BUZZER] Still beeping (count: X)"

# Check if buzzer thread is running
rostopic echo /rosout | grep "BUZZER START"
```

**If it stops:**
1. Check if a false command was sent
2. Check for node crashes: `rosnode list | grep sensors_actuators`
3. Restart node: `rosnode kill /sensors_actuators_node`

**Check 2: Is the node running?**
```bash
rosnode list | grep sensors_actuators
rosnode info /sensors_actuators_node
```

**Check 3: Restart the node**
```bash
rosnode kill /sensors_actuators_node
# It will auto-restart via launch file
```

**Check 4: Hardware test**
```bash
# Test GPIO directly (buzzer on Pin 16)
echo 23 | sudo tee /sys/class/gpio/export
echo out | sudo tee /sys/class/gpio/gpio23/direction
echo 1 | sudo tee /sys/class/gpio/gpio23/value  # ON
sleep 2
echo 0 | sudo tee /sys/class/gpio/gpio23/value  # OFF
```

### Jetson stats authentication error

**Solution (automatic in update_robot.sh):**
```bash
sudo pip3 install -U jetson-stats
sudo usermod -aG jetson_stats $USER
sudo systemctl restart jtop.service
# Then logout and login
```

### Dictionary parsing error (Invalid CPU temp)

**Problem:** `Invalid CPU temp: {'temp': 33.5, 'online': True}`

**Solution:** Code now handles nested dictionaries automatically.
- Extracts `data['temp']` from dict values
- Falls back to direct float conversion for simple values
- Uses `.get()` to avoid KeyError crashes

---

## 📁 Clean Package Structure

```
elderly_bot/
├── update_robot.sh              ⭐ ONLY SCRIPT YOU NEED
├── install_dependencies.sh      (first-time setup)
├── scripts/
│   ├── sensors_actuators_node.py  (gas/buzzer/stats)
│   ├── mpu9250_node.py
│   ├── patrol_client.py
│   └── cloud_bridge_node.py
├── launch/
├── config/
└── [documentation]
```

**All redundant scripts removed:**
- ❌ clean_rebuild.sh
- ❌ deploy_scripts.sh  
- ❌ fix_gpio_permissions.sh
- ❌ diagnose_tf_slam.sh
- ❌ validate_stationary_pose.sh

---

## ✅ Verification Checklist

After running update_robot.sh:

```bash
# 1. Check buzzer works continuously
rostopic pub /buzzer_command std_msgs/Bool "data: true"
# Listen for continuous beeping...
rostopic pub /buzzer_command std_msgs/Bool "data: false"

# 2. Check Jetson stats
rostopic echo /jetson_temperature
# Should see temperature values

# 3. Check gas sensor
rostopic echo /gas_detected
# Should see true/false

# 4. Check for errors
rostopic echo /rosout | grep ERROR
# Should be minimal/none
```

---

## 🎯 Summary

**Before:**
- Multiple scripts for different tasks
- Manual permission fixes
- Buzzer only beeps momentarily
- Jetson stats authentication errors
- Manual sourcing required

**After:**
- ONE script: `update_robot.sh`
- Automatic permission fixes
- Buzzer beeps continuously until stopped
- Jetson stats with robust error handling
- Automatic environment setup

**Result:** Production-ready, zero-maintenance system. 🎉
