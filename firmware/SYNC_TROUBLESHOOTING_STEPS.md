# ESP32 Rosserial Sync Troubleshooting Steps

## Critical Steps (MUST DO IN ORDER)

### Step 1: Verify ESP32 Core Version
**This is the #1 cause of sync failures!**

```bash
# On your development machine (Windows)
# In Arduino IDE: Tools → Board → Boards Manager
# Search for "esp32" and ensure version 2.0.17 is installed
```

**CRITICAL**: Must be Core 2.0.17, NOT 3.x

### Step 2: Regenerate ros_lib on Jetson
```bash
# On Jetson Nano
cd ~/Arduino/libraries
rm -rf ros_lib
rosrun rosserial_arduino make_libraries.py .
```

**CRITICAL**: Must regenerate ros_lib after any ROS/rosserial updates

### Step 3: Upload New Firmware to ESP32
**You MUST upload the updated firmware!**

1. Open Arduino IDE on Windows
2. Open `catkin_ws/src/elderly_bot/firmware/elderly_bot_esp32_no_imu.ino`
3. Select Board: **ESP32 Dev Module**
4. Select Port: Your ESP32 port (check Device Manager)
5. Verify ESP32 Core: **2.0.17** (in Tools menu)
6. Click **Upload**
7. Wait for upload to complete

### Step 4: Reset ESP32
After upload, **manually press the RESET button** on the ESP32 board.

### Step 5: Wait 5+ Seconds
**CRITICAL**: ESP32 needs time to boot and initialize Serial.

Wait at least 5 seconds after seeing the upload complete message.

### Step 6: Connect with Rosserial
```bash
# On Jetson Nano (in a NEW terminal after waiting 5+ seconds)
rosrun rosserial_python serial_node.py _port:=/dev/ttyUSB0 _baud:=115200
```

## What to Check If Still Failing

### Check 1: Serial Port Correct?
```bash
# On Jetson
ls -l /dev/ttyUSB*
# Should see /dev/ttyUSB0 (or similar)
# Note which one appears/disappears when you plug/unplug ESP32
```

### Check 2: Permissions Correct?
```bash
# On Jetson
sudo chmod 666 /dev/ttyUSB0
# Or add user to dialout group (more permanent)
sudo usermod -a -G dialout $USER
# Then logout and login again
```

### Check 3: Roscore Running?
```bash
# On Jetson (in separate terminal)
roscore
# Should see "started core service"
```

### Check 4: Firmware Actually Uploaded?
Connect to ESP32 Serial Monitor (115200 baud) and verify you see:
```
Elderly Bot ESP32 Ready!
ROS Connected Successfully!
```

If you don't see this, the new firmware wasn't uploaded or there's a compilation error.

### Check 5: USB Cable/Port
- Try different USB cable
- Try different USB port
- USB 2.0 ports are more reliable than USB 3.0 for serial

## Quick Test: Upload Minimal Test Firmware

If main firmware still fails, upload `test_sync_minimal.ino` to verify:
- ESP32 Core version is correct
- ros_lib is correct
- USB connection works
- Rosserial version matches

If minimal test works but main firmware doesn't, issue is in main firmware code.

## Expected Behavior When Working

When rosserial connects successfully, you should see:
```
[INFO] [timestamp]: ROS Serial Python Node
[INFO] [timestamp]: Connecting to /dev/ttyUSB0 at 115200 baud
[INFO] [timestamp]: Requesting topics...
[INFO] [timestamp]: Note: publish buffer size is 512 bytes
[INFO] [timestamp]: Setup publisher on wheel_odom [nav_msgs/Odometry]
[INFO] [timestamp]: Note: subscribe buffer size is 512 bytes
[INFO] [timestamp]: Setup subscriber on cmd_vel [geometry_msgs/Twist]
```

## Still Not Working?

1. **Verify ESP32 Core 2.0.17** - This is the most common issue
2. **Regenerate ros_lib** - Must match ROS version
3. **Upload firmware again** - Old firmware might still be running
4. **Wait longer** - ESP32 needs time after boot
5. **Check Serial Monitor** - See what ESP32 is actually doing

