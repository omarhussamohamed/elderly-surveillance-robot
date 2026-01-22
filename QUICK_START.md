# Elderly Bot - Quick Start Guide

## Prerequisites Checklist

Before starting, ensure you have:
- ✅ Jetson Nano with Ubuntu 18.04 + ROS Melodic
- ✅ ESP32 with firmware uploaded (TICKS_PER_REV=3960)
- ✅ RPLidar A1 connected to /dev/ttyUSB0
- ✅ MPU9250 IMU on Jetson I2C bus
- ✅ 4WD robot assembled with JGB37-520 motors
- ✅ WiFi network configured (ESP32 and Jetson on same network)

## 1. Installation (One-Time Setup)

```bash
cd ~/catkin_ws/src/elderly_bot
chmod +x install_dependencies.sh
./install_dependencies.sh
```

**Required packages installed:**
- ros-melodic-robot-state-publisher
- ros-melodic-robot-localization
- ros-melodic-imu-filter-madgwick
- ros-melodic-gmapping
- ros-melodic-amcl
- ros-melodic-move-base
- And more... (see install_dependencies.sh)

**Optional Python packages (for sensors/actuators/cloud):**
```bash
# Gas sensor + buzzer + Jetson monitoring
sudo pip3 install jetson-stats Jetson.GPIO smbus2

# Cloud bridge (AWS IoT Core)
sudo pip3 install AWSIoTPythonSDK

# Reboot after jetson-stats installation
sudo reboot
```

Log out and log back in (for serial port permissions).

## 2. Program ESP32 (One-Time Setup)

⚠️ **CRITICAL**: Firmware must have TICKS_PER_REV=3960 for correct kinematic scaling

### Upload Latest Firmware:

1. Open Arduino IDE on Windows
2. Open `firmware/elderly_bot_esp32_wifi.ino`
3. **VERIFY** line 81: `const int TICKS_PER_REV = 3960;`
4. **VERIFY** line 78: `const float WHEEL_RADIUS = 0.0325;`
5. Select Board: "ESP32 Dev Module"
6. Select Port
7. Upload

**See [ESP32_FIRMWARE_UPLOAD.txt](ESP32_FIRMWARE_UPLOAD.txt) for detailed instructions**

## 3. Hardware Connections

- **RPLidar** → `/dev/ttyUSB0` (USB connection)
- **ESP32** → WiFi connection to 192.168.1.29:11411 (see HARDWARE_MAP.md)
- **MPU-9250 IMU** → Jetson I2C Bus 1:
  - SDA → Pin 3 (I2C2_SDA)
  - SCL → Pin 5 (I2C2_SCL)
  - VCC → 3.3V, GND → GND
  - **Note**: Magnetometer DISABLED (indoor EMI), only gyro+accel used
- **MQ-6 Gas Sensor (Optional)** → ADS1115 ADC → I2C Bus 1 (address 0x48)
- **Active Buzzer (Optional)** → Jetson GPIO pin (configure in config/sensors_actuators.yaml)
- **Power**: 12V to motors, ESP32 powered separately or via Jetson 5V

## 4. Verify System (MANDATORY AFTER ANY CHANGES)

### Step 4.1: Start System

```bash
# Kill any existing ROS processes
rosnode kill -a
killall -9 rosmaster roscore
sleep 3

# Start fresh
roscore &
sleep 3
roslaunch elderly_bot bringup.launch
```

**Expected console output:**
- `[INFO] Starting robot_state_publisher` ← Confirms TF fix applied
- `[INFO] rosserial: Connected to ESP32`
- `[INFO] Starting RPLidar node`
- `[INFO] Starting IMU pipeline`

### Step 4.2: Run TF Verification

**In new terminal:**
```bash
cd ~/catkin_ws/src/elderly_bot
chmod +x scripts/tf_verification_complete.sh
bash scripts/tf_verification_complete.sh
```

**Expected results:**
- ✅ Laser: 180° yaw rotation (NOT roll)
- ✅ IMU: Near-zero rotation (or known mounting offset)
- ✅ Gravity on Z-axis = +9.8 m/s²

**If any test fails:** See [COMPLETE_TF_DEPLOYMENT.md](COMPLETE_TF_DEPLOYMENT.md)

### Step 4.3: Run Master Validation Suite

```bash
chmod +x scripts/master_validator.sh
bash scripts/master_validator.sh
```

**All 4 stages must PASS:**
1. Communication: >8Hz wheel, >40Hz filtered odometry
2. Drift: <0.1° over 3 minutes stationary
3. Linear: 1.0m commanded = 1.0m traveled ±2cm
4. Angular: 360° rotation returns to start ±5°

**If any stage fails:** See troubleshooting in script output

### Step 4.4: Visual Verification in RViz

```bash
rviz
```

**Setup TF display:**
1. Add → TF
2. Check "Show Axes"
3. Marker Scale = 0.3
4. Fixed Frame = "base_footprint"

**Check axes alignment:**
- ✅ base_link RED (X) → forward
- ✅ laser RED → backward (180° opposite) ← Expected
- ✅ imu_link RED → forward (parallel to base_link)
- ✅ All GREEN (Y) arrows parallel → left
- ✅ All BLUE (Z) arrows parallel → up

## 5. Create Map (MODE 1)

```bash
# Start autonomous mapping
roslaunch elderly_bot mapping.launch

# Watch in RViz as robot explores
# When done, save map:
rosrun map_server map_saver -f ~/catkin_ws/src/elderly_bot/maps/house_map
```

## 6. Configure Patrol Waypoints

1. Launch navigation:
   ```bash
   roslaunch elderly_bot navigation.launch
   ```

2. In RViz, use "2D Pose Estimate" to set initial position

3. Drive robot around (or use "2D Nav Goal") to find waypoint coordinates

4. Edit `config/patrol_goals.yaml` with your coordinates:
   ```yaml
   patrol_goals:
     - name: "Living Room"
       x: 2.0
       y: 1.5
       yaw: 0.0
   ```

## 7. Run Patrol (MODE 2)

```bash
# Terminal 1: Navigation
roslaunch elderly_bot navigation.launch

# Terminal 2: Patrol
rosrun elderly_bot patrol_client.py
```

Robot will now patrol indefinitely!

## Common Commands

### Save a map
```bash
rosrun map_server map_saver -f ~/catkin_ws/src/elderly_bot/maps/my_map
```

### Set initial pose (if AMCL is lost)
```bash
rostopic pub /initialpose geometry_msgs/PoseWithCovarianceStamped '{header: {frame_id: "map"}, pose: {pose: {position: {x: 0, y: 0, z: 0}, orientation: {w: 1}}}}'
```

### Send single navigation goal
```bash
rostopic pub /move_base_simple/goal geometry_msgs/PoseStamped '{header: {frame_id: "map"}, pose: {position: {x: 2.0, y: 1.0, z: 0}, orientation: {w: 1}}}'
```

### Check TF tree
```bash
rosrun tf view_frames
evince frames.pdf
```

### Monitor navigation status
```bash
rostopic echo /move_base/status
rostopic echo /move_base/result
```

## Troubleshooting

### Robot doesn't move
- Check ESP32 connection: `rostopic list | grep wheel_odom`
- Verify motors are powered
- Check motor direction flags in firmware

### Poor localization
- Use "2D Pose Estimate" in RViz to set initial pose
- Ensure map has sufficient features
- Check IMU is publishing: `rostopic hz /imu/data`

### Navigation fails
- Check costmaps in RViz
- Verify footprint matches robot size
- Tune DWA planner parameters

### Lidar not working
- Check connection: `ls -l /dev/ttyUSB*`
- Test directly: `roslaunch rplidar_ros view_rplidar.launch`

## Key Documentation Files

- **[HARDWARE_MAP.md](HARDWARE_MAP.md)**: Complete hardware configuration reference
- **[ROSSERIAL_GUIDE.md](ROSSERIAL_GUIDE.md)**: rosserial setup and troubleshooting
- **[SYSTEM_OVERVIEW.md](SYSTEM_OVERVIEW.md)**: System architecture and operation modes

## File Locations

- **Firmware**: `~/catkin_ws/src/elderly_bot/firmware/`
- **Config**: `~/catkin_ws/src/elderly_bot/config/`
- **Maps**: `~/catkin_ws/src/elderly_bot/maps/`
- **Launch**: `~/catkin_ws/src/elderly_bot/launch/`

## Next Steps

- Test motor responsiveness with different cmd_vel commands
- Adjust navigation parameters in config files
- Add more patrol waypoints
- Verify IMU data quality

For detailed information, see SYSTEM_OVERVIEW.md


