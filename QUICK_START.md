# Elderly Bot - Quick Start Guide

## 1. Installation (One-Time Setup)

```bash
cd ~/catkin_ws/src/elderly_bot
chmod +x install_dependencies.sh
./install_dependencies.sh
```

Log out and log back in (for serial port permissions).

## 2. Program ESP32 (One-Time Setup)

⚠️ **IMPORTANT**: Use ESP32 Arduino Core 2.0.17 ONLY (not 3.x)

1. Open Arduino IDE
2. Install ESP32 board support:
   - Tools → Board → Board Manager
   - Search "esp32"
   - **Select version 2.0.17**
   - Install
3. Install libraries: Rosserial Arduino Library
4. Generate ROS library on Jetson:
   ```bash
   # On Jetson Nano
   cd ~/Arduino/libraries
   rm -rf ros_lib
   rosrun rosserial_arduino make_libraries.py .
   ```
5. Copy ros_lib to Arduino IDE (see ROSSERIAL_GUIDE.md for details)
6. Open `firmware/elderly_bot_esp32_wifi.ino`
7. Select Board: "ESP32 Dev Module"
8. **Verify**: Tools → Board → Board Manager shows esp32 2.0.17
9. Upload to ESP32

**If you get errors**: See `ROSSERIAL_GUIDE.md` for comprehensive troubleshooting

## 3. Hardware Connections

- **RPLidar** → `/dev/ttyUSB0`
- **ESP32** → WiFi connection (see HARDWARE_MAP.md for SSID/password)
- **MPU-9250 IMU** → Jetson I2C (direct connection)
- **Power**: 12V to motors, ESP32 powered via Jetson's 5V rail

## 4. Test Hardware

```bash
roslaunch elderly_bot bringup.launch

# In another terminal, test:
rostopic echo /scan
rostopic echo /wheel_odom
rostopic echo /imu/data

# Test motors:
rostopic pub /cmd_vel geometry_msgs/Twist "linear: {x: 0.1}" -r 10
```

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


