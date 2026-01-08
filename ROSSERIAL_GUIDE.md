# rosserial Setup and Troubleshooting Guide

## Overview

This guide consolidates all rosserial-related troubleshooting information, fixes, and working configurations for the Elderly Robot ESP32 firmware.

## Root Causes of Sync Failures

### 1. ESP32 Arduino Core Version Mismatch (MOST COMMON)

**Problem**: Using ESP32 Core 3.x or version other than 2.0.17 causes compatibility issues with rosserial.

**Solution**:
- In Arduino IDE: Tools → Board → Boards Manager
- Search "esp32" and install version **2.0.17** exactly
- Verify in Tools menu: "ESP32 Arduino Core 2.0.17"

### 2. ros_lib Version/Platform Mismatch

**Problem**: `ros_lib` generated on different system (Windows vs Linux) or different ROS version.

**Fix**:
```bash
# On Jetson Nano (where rosserial_python runs)
cd ~/Arduino/libraries
rm -rf ros_lib
rosrun rosserial_arduino make_libraries.py .

# Copy to Arduino IDE (Windows/Mac)
# Use SCP, USB drive, or network share to copy ros_lib to:
# Windows: C:\Users\YOUR_USERNAME\Documents\Arduino\libraries\
# Mac: ~/Documents/Arduino/libraries/
```

### 3. Serial Output During Sync Handshake

**Problem**: `Serial.println()` calls during ROS initialization corrupt sync packets.

**Fix**: ✅ **Already implemented in current firmware**
- No Serial output before `nh.connected() == true`
- All Serial prints happen after successful sync

### 4. Timing Issues

**Problem**: ESP32 not ready when rosserial tries to connect.

**Fix**: ✅ **Already implemented**
- 500ms delay after `Serial.begin(115200)`
- 200ms delay after `nh.getHardware()->setBaud(115200)`
- 10-second sync waiting loop with `nh.spinOnce()`

### 5. Baud Rate Mismatch

**Problem**: Firmware and rosserial using different baud rates.

**Verification**:
- Firmware: `nh.getHardware()->setBaud(115200);`
- Command: `rosrun rosserial_python serial_node.py _port:=/dev/ttyUSB0 _baud:=115200`
- Must match exactly: **115200**

### 6. USB Port/Permissions Issues

**Checks**:
```bash
# Verify device exists
ls -l /dev/ttyUSB*

# Check permissions
sudo chmod 666 /dev/ttyUSB0

# Add user to dialout group (permanent fix)
sudo usermod -a -G dialout $USER
# Logout and login for group change to take effect

# Check for conflicting processes
lsof /dev/ttyUSB0
```

## Working Sync Sequence (CRITICAL)

The current firmware implements this exact sequence:

```cpp
void setup() {
  // 1. Initialize Serial and wait
  Serial.begin(115200);
  delay(500);  // Wait for Serial to stabilize

  // 2. Initialize hardware first (NO Serial output)

  // 3. Configure ROS baud rate
  nh.getHardware()->setBaud(115200);
  delay(200);  // Additional delay before ROS init

  // 4. Initialize ROS node (starts sync process)
  nh.initNode();

  // 5. Wait for sync to complete
  unsigned long timeout = millis() + 10000;  // 10 second timeout
  while(!nh.connected() && millis() < timeout) {
    nh.spinOnce();
    delay(10);
  }

  // 6. Configure publishers/subscribers AFTER sync
  nh.subscribe(cmd_vel_sub);
  nh.advertise(odom_pub);

  // 7. Only NOW safe to use Serial.println()
  if(nh.connected()) {
    Serial.println("ROS Connected Successfully!");
  }
}
```

## Fixes Attempted and Results

### ✅ Successful Fixes (Implemented)

1. **ESP32 Core Version**: Downgraded to 2.0.17 - RESOLVED sync issues
2. **ros_lib Regeneration**: Generate on Jetson, copy to Arduino IDE - RESOLVED version mismatch
3. **Serial Output Removal**: No Serial.println() during sync - RESOLVED packet corruption
4. **Proper Delays**: Added 500ms + 200ms delays - RESOLVED timing issues
5. **Sync Waiting Loop**: 10-second wait with nh.spinOnce() - RESOLVED handshake completion
6. **Buffer Size**: Increased to 1024 bytes - RESOLVED message size issues

### ❌ Unsuccessful Approaches (Tried but didn't work)

1. **ESP32 Core 3.x**: Latest versions incompatible with rosserial
2. **WiFi rosserial**: TCP transport less reliable than USB serial
3. **Custom rosserial versions**: Modified versions caused more issues
4. **Different baud rates**: Anything other than 115200 unreliable

## Current Working Configuration

### Hardware
- **ESP32 Board**: ESP32 Dev Module
- **Arduino Core**: 2.0.17 (exactly)
- **Baud Rate**: 115200
- **USB Port**: `/dev/ttyUSB0` (Jetson)

### Software Versions
- **ROS**: Melodic (Ubuntu 18.04)
- **rosserial_python**: 0.8.0
- **rosserial_arduino**: 0.8.0
- **Arduino IDE**: Any recent version

### Firmware Settings
- **ROS_SERIAL_BUFFER_SIZE**: 1024
- **Control Loop**: 50Hz (20ms intervals)
- **Odometry Publish**: 10Hz (100ms intervals)
- **Safety Timeout**: 800ms

## Testing Procedure

### Step 1: Verify Setup
```bash
# On Jetson - check rosserial versions
dpkg -l | grep rosserial

# Check USB device
ls -l /dev/ttyUSB0
sudo chmod 666 /dev/ttyUSB0

# Start roscore in separate terminal
roscore
```

### Step 2: Upload Firmware
1. Open Arduino IDE
2. Select ESP32 Dev Module board
3. Verify ESP32 Arduino Core 2.0.17
4. Open `elderly_bot_esp32_wifi.ino`
5. Upload to ESP32
6. **Wait 5+ seconds** for ESP32 to boot

### Step 3: Test Connection
```bash
# In new terminal (after 5+ second wait)
rosrun rosserial_python serial_node.py _port:=/dev/ttyUSB0 _baud:=115200
```

**Expected Success Output:**
```
[INFO] Connecting to /dev/ttyUSB0 at 115200 baud
[INFO] Requesting topics...
[INFO] Note: publish buffer size is 512 bytes
[INFO] Setup publisher on wheel_odom [nav_msgs/Odometry]
[INFO] Note: subscribe buffer size is 512 bytes
[INFO] Setup subscriber on cmd_vel [geometry_msgs/Twist]
```

### Step 4: Verify Topics
```bash
rostopic list
# Should show:
/cmd_vel
/wheel_odom
/rosout
/rosout_agg

# Test odometry publishing
rostopic hz /wheel_odom
# Should show ~10 Hz

# Test motor control
rostopic pub /cmd_vel geometry_msgs/Twist "linear: {x: 0.1}" -r 10
```

## Common Mistakes to Avoid

1. ❌ **Opening Arduino Serial Monitor during sync** - Interrupts ESP32
2. ❌ **Wrong ESP32 Core version** - Must be exactly 2.0.17
3. ❌ **Old ros_lib** - Must regenerate after any ROS updates
4. ❌ **Wrong USB port** - ESP32 might be on `/dev/ttyUSB1`
5. ❌ **Not waiting after ESP32 boot** - Need 5+ seconds
6. ❌ **Multiple rosserial instances** - Only one per port
7. ❌ **Baud rate mismatch** - Must be exactly 115200

## Advanced Debugging

### Enable Debug Output
```bash
rosrun rosserial_python serial_node.py _port:=/dev/ttyUSB0 _baud:=115200 _log_level:=DEBUG
```

### Monitor ESP32 Serial Output
```bash
screen /dev/ttyUSB0 115200
# Look for "ROS Connected Successfully!"
# Ctrl+A, K to exit
```

### Check System Resources
```bash
# Monitor CPU usage
top

# Check USB errors
dmesg | grep ttyUSB

# Verify no process conflicts
lsof /dev/ttyUSB0
```

## Version Compatibility Matrix

| Component | Version | Status | Notes |
|-----------|---------|--------|-------|
| ROS Distribution | Melodic | ✅ Required | Ubuntu 18.04 |
| rosserial_python | 0.8.0 | ✅ Compatible | Must match |
| rosserial_arduino | 0.8.0 | ✅ Compatible | Must match |
| ESP32 Arduino Core | 2.0.17 | ✅ Required | Exactly this version |
| ESP32 Board | ESP32 Dev Module | ✅ Compatible | Tested |
| Arduino IDE | 1.8.x+ | ✅ Compatible | Any recent |

## Emergency Recovery

If nothing works:

1. **Minimal Test Firmware**: Upload `test_sync_minimal.ino` to verify basic rosserial works
2. **Fresh ros_lib**: Delete and regenerate completely
3. **ESP32 Reset**: Manually press reset button after upload
4. **Clean Arduino IDE**: Restart Arduino IDE, clear cache
5. **USB Cable**: Try different cable/port

## Final Working State

The current `elderly_bot_esp32_wifi.ino` firmware incorporates all successful fixes and should connect reliably to rosserial_python. The dual-core architecture ensures motor control remains responsive while ROS communication operates independently.

If sync still fails after following this guide, the issue is likely environmental (USB hardware, power supply, or system-specific) rather than software-related.
