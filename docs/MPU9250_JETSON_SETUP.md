# MPU9250 Direct Connection to Jetson Guide

## Overview

Instead of reading MPU9250 data through the ESP32, connect it directly to the Jetson I2C bus. This approach offers:

✅ **Reliability**: Direct I2C connection, no serial communication dependency  
✅ **Bandwidth**: Removes IMU data from ESP32 communication (reduces USB/UART load)  
✅ **Processing**: Jetson has better resources for IMU fusion and calibration  
✅ **Separation**: Motor control (ESP32) and sensing (Jetson) are independent

### Sensor Features

The MPU9250 node now publishes:
- **IMU data** (`sensor_msgs/Imu`): Accelerometer (m/s²) + Gyroscope (rad/s) at `/imu/data`
- **Magnetometer data** (`sensor_msgs/MagneticField`): 3-axis compass from internal AK8963 at `/imu/mag`
- **Temperature** (`sensor_msgs/Temperature`): Internal temperature sensor (°C) at `/imu/temperature`

The magnetometer improves yaw estimation and orientation accuracy for localization, while temperature data is useful for sensor monitoring and calibration.  

---

## Hardware Setup

### Jetson Nano J21 Header Pinout

```
Pin 1:  3.3V        ← MPU9250 VCC
Pin 3:  I2C2_SDA    ← MPU9250 SDA
Pin 5:  I2C2_SCL    ← MPU9250 SCL
Pin 6:  GND         ← MPU9250 GND
```

### Wiring Diagram

```
MPU9250          Jetson Nano J21
--------         --------------
VCC       →      Pin 1  (3.3V)
GND       →      Pin 6  (GND)
SDA       →      Pin 3  (I2C2_SDA)
SCL       →      Pin 5  (I2C2_SCL)
```

**Note**: If using a different Jetson model, check your specific I2C pinout.

---

## Software Setup

### Step 1: Install Required Packages

```bash
# Install I2C tools and Python libraries
sudo apt-get update
sudo apt-get install -y i2c-tools python3-smbus

# Verify I2C is enabled
sudo i2cdetect -l
# Should show: i2c-1, i2c-2, etc.
```

### Step 2: Enable I2C (if not already enabled)

```bash
# Check if I2C modules are loaded
lsmod | grep i2c

# Enable I2C (usually enabled by default on Jetson)
# If needed, add to /etc/modules:
# i2c-dev
# i2c-i801
```

### Step 3: Test I2C Connection

```bash
# Scan I2C bus 1 for devices
sudo i2cdetect -y 1

# Or scan I2C bus 2
sudo i2cdetect -y 2

# MPU9250 should appear at address 0x68 (or 0x69 if AD0 is HIGH)
# Example output:
#      0  1  2  3  4  5  6  7  8  9  a  b  c  d  e  f
# 00:          -- -- -- -- -- -- -- -- -- -- -- -- -- -- 
# 10: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- 
# 20: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- 
# 30: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- 
# 40: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- 
# 50: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- 
# 60: -- -- -- -- -- -- -- -- -- 68 -- -- -- -- -- -- 
# 70: -- -- -- -- -- -- -- --
#                                  ^^ MPU9250 found!
```

### Step 4: Make ROS Node Executable

```bash
cd ~/catkin_ws/src/elderly_bot/scripts
chmod +x mpu9250_node.py
```

### Step 5: Build ROS Package

```bash
cd ~/catkin_ws
catkin_make
source devel/setup.bash
```

---

## Usage

### Option 1: Standalone IMU Node

```bash
# Terminal 1: Start roscore
roscore

# Terminal 2: Start MPU9250 node
rosrun elderly_bot mpu9250_node.py

# Terminal 3: Check topics
rostopic list
# Should see: 
#   /imu/data
#   /imu/mag
#   /imu/temperature

rostopic echo /imu/data
# Should show IMU data (accel + gyro) publishing at 50 Hz

rostopic echo /imu/mag
# Should show magnetometer data in Tesla

rostopic echo /imu/temperature
# Should show temperature in Celsius
```

### Option 2: Using Launch File

```bash
# Start with launch file (includes TF transform)
roslaunch elderly_bot mpu9250_jetson.launch
```

### Option 3: Integrated with Bringup

Modify `bringup.launch` to include MPU9250 node:

```xml
<!-- In bringup.launch, add: -->
<include file="$(find elderly_bot)/launch/mpu9250_jetson.launch" />

<!-- Comment out or remove ESP32 IMU publishing -->
```

---

## Configuration

### Adjust I2C Bus Number

If MPU9250 is on a different I2C bus:

```bash
# Check available buses
sudo i2cdetect -l

# Update launch file parameter
<arg name="i2c_bus" default="2" />  <!-- Change to your bus number -->
```

### Adjust I2C Address

If MPU9250 AD0 pin is HIGH, address is 0x69:

```bash
# Update launch file parameter
<arg name="i2c_address" default="0x69" />
```

### Adjust Publish Rate

```bash
# In launch file
<arg name="publish_rate" default="100" />  <!-- Change to desired Hz -->
```

---

## ESP32 Firmware Changes

Since IMU is now on Jetson, you can remove IMU code from ESP32:

### Firmware Configuration:

1. **Remove IMU includes:**
```cpp
// DELETE these lines:
#include <Wire.h>
#include <MPU9250.h>
```

2. **Remove IMU globals:**
```cpp
// DELETE:
MPU9250 mpu;
bool imu_ready = false;
```

3. **Remove IMU initialization in setup():**
```cpp
// DELETE the entire IMU initialization block:
// Wire.begin(...);
// mpu.calibrateAccelGyro();
```

4. **Remove IMU task:**
```cpp
// DELETE:
xTaskCreatePinnedToCore(imuTask, ...);
```

5. **Remove IMU publisher:**
```cpp
// DELETE:
ros::Publisher imu_pub("imu/data", &imu_msg);
nh.advertise(imu_pub);
```

6. **Remove IMU task function:**
```cpp
// DELETE entire imuTask() function
```

**Result**: ESP32 firmware becomes simpler and lighter, focusing only on motor control and odometry.

---

## Benefits of This Approach

### 1. **Reduced ESP32 Communication Load**

**Before (ESP32):**
- Motor control: ~20 Hz
- Odometry: ~20 Hz  
- IMU: ~50 Hz
- **Total**: ~90 Hz worth of messages

**After (ESP32 + Jetson):**
- ESP32: Motor control + Odometry = ~40 Hz
- Jetson: IMU = ~50 Hz (separate channel)
- **Result**: ~44% reduction in ESP32 serial traffic

### 2. **Improved Reliability**

- IMU data no longer depends on ESP32 serial connection
- If ESP32 communication fails, IMU still works
- Direct I2C is more reliable than USB serial

### 3. **Better Performance**

- Jetson can do advanced IMU fusion (Kalman filter, complementary filter)
- Lower latency (no serial transmission delay)
- More accurate timestamps

### 4. **Easier Debugging**

- Can test IMU independently
- No need to flash ESP32 firmware to change IMU settings
- Direct access to raw sensor data

---

## Troubleshooting

### MPU9250 Not Detected on I2C

```bash
# Check I2C bus
sudo i2cdetect -y 1

# If nothing appears:
1. Check wiring (SDA/SCL swapped?)
2. Check power (3.3V connected?)
3. Check GND connection
4. Try different I2C bus: i2cdetect -y 2
5. Check if MPU9250 AD0 pin is HIGH (address 0x69) or LOW (0x68)
```

### Magnetometer Not Working

```bash
# Check logs for magnetometer initialization
rosrun elderly_bot mpu9250_node.py

# Look for:
# - "AK8963 magnetometer initialized successfully" (good)
# - "AK8963 magnetometer initialization failed" (problem)

# If failed:
1. Magnetometer might be unresponsive - try power cycling
2. Check if AK8963 appears at address 0x0C: sudo i2cdetect -y 1
3. Node will continue without mag data if initialization fails
```

### Temperature Readings Seem Wrong

```bash
# MPU9250 temperature sensor has ±3°C accuracy
# Reading should be close to ambient temperature
# If way off:
1. Check if sensor is under mechanical stress
2. Verify calibration (temp = (raw/333.87) + 21.0)
3. Temperature can be affected by self-heating after prolonged operation
```

### Permission Denied

```bash
# Add user to i2c group
sudo usermod -a -G i2c $USER
# Log out and back in, or:
newgrp i2c
```

### Wrong Data / Garbled Output

```bash
# Check I2C address is correct
sudo i2cdetect -y 1

# Verify bus speed (MPU9250 supports up to 400kHz)
# Default is usually 100kHz, which is fine
```

### Topic Not Publishing

```bash
# Check node is running
rosnode list | grep mpu9250

# Check for errors
rosnode info /mpu9250_node

# Check topic exists
rostopic list | grep imu
```

---

## Comparison: ESP32 vs Jetson IMU

| Aspect | ESP32 (Current) | Jetson (Direct) |
|--------|-----------------|-----------------|
| **Connection** | I2C → ESP32 → USB → Jetson | I2C → Jetson |
| **Latency** | ~10-20ms | <1ms |
| **Reliability** | Depends on USB | Direct connection |
| **Bandwidth** | Shares with odometry | Independent |
| **Processing** | Limited on ESP32 | Full Jetson power |
| **Complexity** | Firmware code needed | Simple Python node |
| **Debugging** | Flash firmware to change | Edit Python file |

---

## Recommendation

**Use Jetson Direct Connection if:**
- ✅ USB serial is unreliable
- ✅ You want to reduce ESP32 communication load
- ✅ You need advanced IMU processing (sensor fusion)
- ✅ You want independent IMU operation

**Keep ESP32 IMU if:**
- ✅ USB serial is working fine
- ✅ You prefer single communication channel
- ✅ ESP32 processing is sufficient
- ✅ Wiring constraints favor ESP32

---

## Next Steps

1. **Wire MPU9250 to Jetson** (follow hardware setup above)
2. **Test I2C connection** (`i2cdetect`)
3. **Run ROS node** (`roslaunch elderly_bot mpu9250_jetson.launch`)
4. **Update bringup.launch** to include MPU9250 node
5. **Remove IMU code from ESP32 firmware** (optional, for simplification)

---

## Files Added

- `src/elderly_bot/scripts/mpu9250_node.py` - ROS node for MPU9250
- `src/elderly_bot/launch/mpu9250_jetson.launch` - Launch file
- `src/elderly_bot/docs/MPU9250_JETSON_SETUP.md` - This guide

