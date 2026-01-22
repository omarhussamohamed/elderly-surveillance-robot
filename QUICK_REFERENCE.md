# Elderly Bot - Quick Reference Guide

## 🚀 One-Command Setup & Launch

```bash
~/catkin_ws/src/elderly_bot/update_robot.sh
```

**This single command does everything:**
- ✅ Cleans redundant files automatically
- ✅ Configures environment (~/.bashrc)
- ✅ Upgrades jetson-stats (fixes authentication & float parsing)
- ✅ Fixes GPIO permissions permanently
- ✅ Updates code from Git
- ✅ Builds workspace
- ✅ Deploys scripts
- ✅ Launches the robot

---

## 🔊 Buzzer Control (Continuous Beeping)

### Start Continuous Beeping
```bash
rostopic pub /buzzer_command std_msgs/Bool "data: true"
```
**Buzzer will beep continuously (0.1s ON/OFF) until you send false.**

### Stop Beeping
```bash
rostopic pub /buzzer_command std_msgs/Bool "data: false"
```

### Monitor Buzzer Thread
```bash
# Watch the logs to see buzzer thread activity
rostopic echo /rosout | grep BUZZER
```

**Implementation Details:**
- Thread-based continuous beeping (not momentary)
- State-locked to prevent race conditions
- Logs every 10 seconds to confirm it's still running
- Automatically stops on gas sensor clear
- Clean shutdown on node exit

---

## 🔧 Jetson Stats Fix

The update_robot.sh automatically:
1. ✅ Upgrades jetson-stats: `sudo -H pip install -U jetson-stats`
2. ✅ Adds user to jtop group: `sudo usermod -aG jtop $USER`
3. ✅ Restarts jtop service: `sudo systemctl restart jtop.service`

**Manual fix if needed:**
```bash
# Upgrade jetson-stats
sudo -H pip install -U jetson-stats

# Add user to group
sudo usermod -aG jtop $USER

# Logout and login for group membership to take effect
logout
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

### Buzzer only beeps once and stops

**Check 1: Is the thread still running?**
```bash
# Look for "[BUZZER THREAD] Still beeping..." messages
rostopic echo /rosout | grep "BUZZER THREAD"
```

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
sudo usermod -aG jtop $USER
sudo systemctl restart jtop.service
# Then logout and login
```

### Float parsing error

**Solution:** Update_robot.sh upgrades jetson-stats automatically.
The code now has try-except blocks for all float conversions.

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
