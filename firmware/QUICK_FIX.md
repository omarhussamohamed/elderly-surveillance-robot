# Quick Fix: "Unable to sync with device" Error

## The Problem
```
[ERROR] Unable to sync with device; possible link problem or link software version mismatch
```

## The Solution (3 Steps)

### Step 1: Regenerate ros_lib on Jetson Nano
```bash
cd ~/Arduino/libraries
rm -rf ros_lib
rosrun rosserial_arduino make_libraries.py .
```

### Step 2: Re-upload Firmware to ESP32
1. Open Arduino IDE
2. Open `elderly_bot_esp32.ino`
3. Verify Board: "ESP32 Dev Module"
4. Verify Port: `/dev/ttyUSB0` (or your ESP32 port)
5. Click Upload

### Step 3: Test Connection
```bash
# Terminal 1
roscore

# Terminal 2
rosrun rosserial_python serial_node.py _port:=/dev/ttyUSB0 _baud:=115200
```

## Expected Result
```
[INFO] ROS Serial Python Node
[INFO] Connecting to /dev/ttyUSB0 at 115200 baud
[INFO] Requesting topics...
[INFO] Setup subscriber on cmd_vel [geometry_msgs/Twist]
[INFO] Setup publisher on wheel_odom [nav_msgs/Odometry]
[INFO] Setup publisher on imu/data [sensor_msgs/Imu]
```

## Verify It Works
```bash
# Check topics exist
rostopic list

# Should show:
# /cmd_vel
# /wheel_odom
# /imu/data

# Check publishing rates
rostopic hz /wheel_odom  # Should show ~100 Hz
rostopic hz /imu/data    # Should show ~100 Hz
```

## Why This Happens

The `ros_lib` library on the ESP32 must be generated on the **same system** running rosserial_python (your Jetson Nano). 

If you generated ros_lib on a different computer or with a different ROS version, it won't match and you'll get the sync error.

## Still Not Working?

See `ROSSERIAL_TROUBLESHOOTING.md` for detailed debugging steps.

## Important Notes

- ✅ Always regenerate ros_lib on the Jetson (not your development PC)
- ✅ Always re-upload firmware after regenerating ros_lib
- ✅ Use ESP32 Arduino Core 2.0.17 (not 3.x)
- ✅ Baud rate must be 115200

