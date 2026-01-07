# Elderly Bot - Deployment Checklist

Use this checklist to deploy the Elderly Bot system on your Jetson Nano.

## Pre-Deployment

### Software Requirements
- [ ] Ubuntu 18.04 installed on Jetson Nano
- [ ] ROS Melodic installed
- [ ] Arduino IDE installed (for ESP32 programming)
- [ ] Git installed (optional, for version control)

### Hardware Requirements
- [ ] Jetson Nano with power supply
- [ ] ESP32 38-pin development board
- [ ] RPLidar A1 with USB adapter
- [ ] MPU-9250 IMU module
- [ ] 4× JGB37-520 motors with encoders
- [ ] 2× L298N motor drivers
- [ ] 12V battery for motors
- [ ] 5V power for ESP32
- [ ] USB cables (2×)
- [ ] Jumper wires for connections

## Step 1: Transfer Package to Jetson Nano

### Option A: USB Drive
```bash
# On development machine
cd ~/catkin_ws/src
zip -r elderly_bot.zip elderly_bot/

# Copy to USB, then on Jetson:
cd ~/catkin_ws/src
unzip elderly_bot.zip
```

### Option B: Git (if using version control)
```bash
cd ~/catkin_ws/src
git clone <your-repo-url> elderly_bot
```

### Option C: Direct Copy (if on same network)
```bash
# On development machine
scp -r elderly_bot/ jetson@<jetson-ip>:~/catkin_ws/src/
```

## Step 2: Install Dependencies

```bash
cd ~/catkin_ws/src/elderly_bot
chmod +x install_dependencies.sh
./install_dependencies.sh
```

**Important**: Log out and log back in after installation (for dialout group).

## Step 3: Build Package

```bash
cd ~/catkin_ws
catkin_make
source devel/setup.bash

# Add to .bashrc for automatic sourcing
echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc
```

## Step 4: Hardware Connections

### ESP32 Wiring

#### Motor Driver #1 (L298N)
| ESP32 Pin | Function | L298N Pin |
|-----------|----------|-----------|
| 32 | FL PWM | ENA |
| 14 | FL IN1 | IN1 |
| 33 | FL IN2 | IN2 |
| 25 | FR PWM | ENB |
| 26 | FR IN1 | IN3 |
| 27 | FR IN2 | IN4 |

#### Motor Driver #2 (L298N)
| ESP32 Pin | Function | L298N Pin |
|-----------|----------|-----------|
| 4 | RL PWM | ENA |
| 5 | RL IN1 | IN1 |
| 16 | RL IN2 | IN2 |
| 17 | RR PWM | ENB |
| 18 | RR IN1 | IN3 |
| 19 | RR IN2 | IN4 |

#### Encoders
| ESP32 Pin | Function | Notes |
|-----------|----------|-------|
| 34 | FL Encoder A | Input only |
| 35 | FL Encoder B | Input only |
| 36 | FR Encoder A | Input only |
| 39 | FR Encoder B | Input only |
| 13 | RL Encoder A | With pullup |
| 12 | RL Encoder B | With pullup |
| 15 | RR Encoder A | With pullup |
| 2 | RR Encoder B | With pullup |

#### IMU (MPU-9250)
| ESP32 Pin | Function |
|-----------|----------|
| 21 | SDA |
| 22 | SCL |
| 3.3V | VCC |
| GND | GND |

### Power Connections
- [ ] 12V battery → Motor drivers VCC
- [ ] Motor drivers GND → Common ground
- [ ] 5V regulator → ESP32 VIN
- [ ] ESP32 GND → Common ground
- [ ] Jetson Nano powered separately

### USB Connections
- [ ] RPLidar → Jetson USB port (will be /dev/ttyUSB0)
- [ ] ESP32 → Jetson USB port (will be /dev/ttyUSB1)

## Step 5: Program ESP32

### Setup Arduino IDE
```bash
# Generate ROS library for Arduino
cd ~/Arduino/libraries
rm -rf ros_lib
rosrun rosserial_arduino make_libraries.py .
```

### Upload Firmware
1. Open Arduino IDE
2. File → Open → `~/catkin_ws/src/elderly_bot/firmware/elderly_bot_esp32.ino`
3. Tools → Board → "ESP32 Dev Module"
4. Tools → Port → Select ESP32 port
5. Upload

### Verify Upload
- Check serial monitor (115200 baud)
- Should see "Elderly Bot ESP32 Firmware Starting..."

## Step 6: Test Hardware

### Test 1: Lidar
```bash
# Terminal 1
roscore

# Terminal 2
roslaunch rplidar_ros view_rplidar.launch

# Should see scan data in RViz
```

### Test 2: ESP32 Communication
```bash
# Terminal 1
roscore

# Terminal 2
rosrun rosserial_python serial_node.py /dev/ttyUSB1

# Terminal 3
rostopic list
# Should see /wheel_odom and /imu/data

rostopic echo /imu/data
# Should see IMU data
```

### Test 3: Motors
```bash
# Terminal 1
roslaunch elderly_bot bringup.launch

# Terminal 2
rostopic pub /cmd_vel geometry_msgs/Twist "linear: {x: 0.1}" -r 10

# Motors should spin slowly forward
# Press Ctrl+C to stop
```

### Test 4: Encoders
```bash
# With motors running
rostopic echo /wheel_odom

# Should see odometry updating as wheels turn
```

### Test 5: Complete Bringup
```bash
roslaunch elderly_bot bringup.launch

# Check all topics are publishing:
rostopic hz /scan
rostopic hz /wheel_odom
rostopic hz /imu/data

# All should show reasonable rates
```

## Step 7: Create Map

### Start Mapping
```bash
roslaunch elderly_bot mapping.launch
```

### Monitor Progress
- Watch RViz visualization
- Robot should start exploring autonomously
- Map builds in real-time

### Save Map
When exploration is complete:
```bash
rosrun map_server map_saver -f ~/catkin_ws/src/elderly_bot/maps/house_map
```

Verify files created:
- `house_map.yaml`
- `house_map.pgm`

## Step 8: Configure Patrol Waypoints

### Method 1: Using RViz
```bash
# Start navigation
roslaunch elderly_bot navigation.launch

# In RViz:
# 1. Set initial pose with "2D Pose Estimate"
# 2. Send robot to desired locations with "2D Nav Goal"
# 3. Note coordinates from /amcl_pose topic
```

### Method 2: Drive and Record
```bash
# Terminal 1
roslaunch elderly_bot navigation.launch

# Terminal 2
rostopic echo /amcl_pose

# Drive robot to each patrol point and record coordinates
```

### Edit Patrol Goals
```bash
nano ~/catkin_ws/src/elderly_bot/config/patrol_goals.yaml
```

Update with your coordinates:
```yaml
patrol_goals:
  - name: "Location 1"
    x: 2.0
    y: 1.5
    yaw: 0.0
  # Add more waypoints...
```

## Step 9: Test Navigation

### Single Goal Test
```bash
# Terminal 1
roslaunch elderly_bot navigation.launch

# Terminal 2
rostopic pub /move_base_simple/goal geometry_msgs/PoseStamped '{header: {frame_id: "map"}, pose: {position: {x: 2.0, y: 1.0, z: 0}, orientation: {w: 1}}}'
```

Robot should navigate to goal.

### Patrol Test
```bash
# Terminal 1
roslaunch elderly_bot navigation.launch

# Terminal 2
rosrun elderly_bot patrol_client.py
```

Robot should cycle through all patrol waypoints.

## Step 10: Final Verification

### System Health Checks
- [ ] All topics publishing at expected rates
- [ ] TF tree complete (no warnings)
- [ ] Robot responds to navigation goals
- [ ] Obstacle avoidance working
- [ ] Motors stop on timeout
- [ ] Localization stable

### Performance Checks
- [ ] Mapping creates clean map
- [ ] Navigation reaches goals reliably
- [ ] Patrol cycles continuously
- [ ] No excessive drift in odometry
- [ ] IMU data reasonable

### Safety Checks
- [ ] E-stop accessible
- [ ] Motors stop when cmd_vel stops
- [ ] Robot avoids obstacles
- [ ] No unexpected movements
- [ ] Battery monitoring (if implemented)

## Troubleshooting

### ESP32 Not Connecting
```bash
# Check USB connection
ls -l /dev/ttyUSB*

# Check permissions
groups
# Should include 'dialout'

# If not, add user and reboot
sudo usermod -a -G dialout $USER
sudo reboot
```

### Lidar Not Working
```bash
# Check device
ls -l /dev/ttyUSB*

# Test directly
roslaunch rplidar_ros view_rplidar.launch serial_port:=/dev/ttyUSB0
```

### Motors Not Responding
1. Check power connections
2. Verify motor driver LEDs
3. Test with Arduino serial monitor
4. Check motor direction flags in firmware

### Poor Localization
1. Set initial pose in RViz
2. Ensure map has features
3. Check IMU calibration
4. Verify TF tree timing

### Navigation Failures
1. Check costmap visualization
2. Verify footprint size
3. Reduce max velocities
4. Tune planner parameters

## Maintenance

### Regular Checks
- [ ] Clean lidar lens
- [ ] Check encoder connections
- [ ] Verify motor mounting
- [ ] Test battery voltage
- [ ] Update map if environment changes

### Log Monitoring
```bash
# View recent logs
roscd elderly_bot
cat ~/.ros/log/latest/*.log | grep ERROR
cat ~/.ros/log/latest/*.log | grep WARN
```

### Performance Tuning
- Adjust PID gains if needed
- Tune navigation parameters
- Update patrol waypoints
- Optimize map resolution

## Production Deployment

### Auto-Start on Boot (Optional)
Create systemd service:
```bash
sudo nano /etc/systemd/system/elderly_bot.service
```

Add:
```ini
[Unit]
Description=Elderly Bot Navigation
After=network.target

[Service]
Type=simple
User=jetson
Environment="ROS_HOME=/home/jetson/.ros"
ExecStart=/bin/bash -c "source /opt/ros/melodic/setup.bash && source /home/jetson/catkin_ws/devel/setup.bash && roslaunch elderly_bot navigation.launch"
Restart=on-failure

[Install]
WantedBy=multi-user.target
```

Enable:
```bash
sudo systemctl enable elderly_bot.service
sudo systemctl start elderly_bot.service
```

## Support Contacts

- ROS Wiki: http://wiki.ros.org
- Package Documentation: See README.md
- Quick Start: See QUICK_START.md
- System Overview: See SYSTEM_OVERVIEW.md

## Deployment Complete!

Your Elderly Bot is now ready for autonomous operation.

**Final Steps**:
1. Run a full patrol cycle to verify
2. Monitor for 30 minutes to ensure stability
3. Document any environment-specific tuning
4. Create backup of working configuration

**Congratulations!** 🎉


