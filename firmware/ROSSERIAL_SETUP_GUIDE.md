# rosserial Setup and Troubleshooting Guide

## Critical Issue: ros_lib Path Mismatch

**PROBLEM**: Your `ros_lib` is located at a Windows path (`C:\Users\omarh\Documents\Arduino\libraries\ros_lib`), but you need it generated on the **Jetson** (Linux) where rosserial_python runs.

**This is the #1 cause of "Unable to sync with device" errors!**

## Solution: Regenerate ros_lib on Jetson

### Step 1: On Jetson, Generate ros_lib

```bash
# Navigate to your Arduino libraries directory (or create one)
cd ~/Arduino/libraries

# Remove old ros_lib if it exists (IMPORTANT!)
rm -rf ros_lib

# Generate fresh ros_lib from your Jetson's ROS Melodic installation
rosrun rosserial_arduino make_libraries.py .

# Verify it was created
ls -la ros_lib/
```

### Step 2: Copy ros_lib to Your Windows Arduino IDE

**Option A: Using SCP (if you have SSH access)**
```bash
# On Jetson
cd ~/Arduino/libraries
scp -r ros_lib username@your-windows-ip:/c/Users/omarh/Documents/Arduino/libraries/
```

**Option B: Using USB/Network Share**
1. Copy `~/Arduino/libraries/ros_lib` from Jetson to a USB drive
2. Copy it to `C:\Users\omarh\Documents\Arduino\libraries\` on Windows
3. Make sure the entire `ros_lib` folder is replaced (delete old one first!)

### Step 3: Verify Version Match

Check that both systems have compatible versions:

**On Jetson:**
```bash
dpkg -l | grep rosserial
# Should show: ros-melodic-rosserial-arduino and ros-melodic-rosserial-python
```

**In Arduino IDE:**
- Check that `ros_lib` version matches ROS Melodic message definitions
- The generated files should have timestamps from when you ran `make_libraries.py` on Jetson

## Common rosserial Sync Errors

### Error: "Unable to sync with device; possible link problem or link software version mismatch"

**Root Causes:**
1. ❌ **ros_lib generated on different system/ROS version** (MOST COMMON)
2. ❌ Baud rate mismatch
3. ❌ USB port permissions
4. ❌ Buffer size mismatch
5. ❌ Firmware crashes during initialization

**Fix Checklist:**

1. ✅ **Regenerate ros_lib on Jetson** (see above)
2. ✅ **Verify baud rates match:**
   - Firmware: `nh.getHardware()->setBaud(115200);`
   - Launch file: `<param name="baud" value="115200" />`
3. ✅ **Check USB permissions:**
   ```bash
   ls -l /dev/ttyUSB0
   sudo chmod 666 /dev/ttyUSB0
   sudo usermod -a -G dialout $USER
   # Log out and back in for group change to take effect
   ```
4. ✅ **Verify port:**
   ```bash
   # Check what's connected
   ls -l /dev/ttyUSB*
   # ESP32 should be on /dev/ttyUSB0
   # RPLidar should be on /dev/ttyUSB1
   ```
5. ✅ **Check Serial Monitor on ESP32:**
   - Should see "Elderly Bot ESP32 Starting..."
   - Should see "Waiting for ROS connection..."
   - If it crashes/resets, check for buffer overflow or memory issues

### Error: "Checksum does not match"

**Cause**: Serial communication corruption or buffer overflow

**Fix:**
- Increase buffer size (already set to 1024 in firmware)
- Check USB cable quality
- Reduce publish rates if messages are too large
- Add delays in `setup()` before `nh.initNode()`

### Error: Connection drops after a few seconds

**Cause**: Buffer overflow or watchdog reset

**Fix:**
- New firmware uses FreeRTOS tasks and proper rate limiting
- Monitor ESP32 serial output for crash messages
- Check free heap: `Serial.print("Free heap: "); Serial.println(ESP.getFreeHeap());`

## Testing rosserial Connection

### Step 1: Verify ESP32 Serial Output

Connect to ESP32 via Serial Monitor (115200 baud):
```
Elderly Bot ESP32 Starting...
Firmware: Redesigned with FreeRTOS
Initializing MPU9250... OK - IMU calibrated!
Initializing ROS... OK
Creating FreeRTOS tasks...
Tasks created!
Elderly Bot Ready!
Waiting for ROS connection...
```

### Step 2: Start roscore

```bash
roscore
```

### Step 3: Start rosserial (in separate terminal)

```bash
rosrun rosserial_python serial_node.py _port:=/dev/ttyUSB0 _baud:=115200
```

**Expected output:**
```
[INFO] [timestamp]: ROS Serial Python Node
[INFO] [timestamp]: Connecting to /dev/ttyUSB0 at 115200 baud
[INFO] [timestamp]: Note: publish buffer size is 512 bytes
[INFO] [timestamp]: Setup publisher on wheel_odom [nav_msgs/Odometry]
[INFO] [timestamp]: Setup publisher on imu/data [sensor_msgs/Imu]
[INFO] [timestamp]: Setup subscriber on cmd_vel [geometry_msgs/Twist]
```

### Step 4: Verify Topics

```bash
rostopic list
# Should see:
# /cmd_vel
# /imu/data
# /wheel_odom
# /rosout
# /rosout_agg

rostopic hz /imu/data
# Should show ~50 Hz

rostopic hz /wheel_odom
# Should show ~20 Hz
```

## Version Compatibility Matrix

| Component | Version | Status |
|-----------|---------|--------|
| ROS Distribution | Melodic | ✅ Required |
| rosserial_python | 0.8.0 | ✅ Compatible |
| rosserial_arduino | 0.8.0 | ✅ Compatible |
| ESP32 Arduino Core | 1.0.6 | ✅ Compatible |
| Arduino IDE | 2.3.7 | ✅ Compatible |

## Advanced Debugging

### Enable Verbose rosserial Output

```bash
rosrun rosserial_python serial_node.py _port:=/dev/ttyUSB0 _baud:=115200 _log_level:=DEBUG
```

### Monitor Serial Traffic

```bash
# Install screen or minicom
sudo apt-get install screen

# Connect to ESP32 serial
screen /dev/ttyUSB0 115200

# To exit: Ctrl+A then K then Y
```

### Check ROS Node Status

```bash
rosnode list
# Should see: /esp32_serial_node

rosnode info /esp32_serial_node
# Shows published/subscribed topics
```

## Firmware Buffer Configuration

The redesigned firmware uses:
- `ROS_SERIAL_BUFFER_SIZE 1024` - Buffer for ROS messages
- FreeRTOS tasks for non-blocking operation
- Rate limiting: IMU 50Hz, Odometry 20Hz, Control 50Hz

If you still see buffer issues, you can increase (at cost of RAM):
```cpp
#define ROS_SERIAL_BUFFER_SIZE 2048  // Use only if needed
```

## Next Steps After Fixing Sync

1. ✅ Verify all topics are publishing
2. ✅ Test `/cmd_vel` subscriber: `rostopic pub /cmd_vel geometry_msgs/Twist ...`
3. ✅ Check IMU data quality: `rostopic echo /imu/data`
4. ✅ Verify odometry: `rostopic echo /wheel_odom`

