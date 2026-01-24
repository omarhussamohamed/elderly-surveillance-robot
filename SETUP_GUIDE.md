# ElderlyBot Package - Setup and Testing Guide

This guide covers the setup, dependencies, and testing procedures for the elderly_bot ROS package.

## Package Overview

Autonomous indoor 4WD monitoring robot with:
- Mapping and navigation (gmapping, move_base)
- AWS IoT Core integration
- AWS Kinesis Video Streams (KVS)
- ESP32 motor controller via rosserial
- RPLidar 2D LIDAR
- MPU9250 IMU sensor
- Gas detection and buzzer alerts

## System Requirements

- **Platform**: Jetson Nano
- **OS**: Ubuntu 18.04 (JetPack 4.x)
- **ROS**: Melodic
- **Python**: 2.7 (ROS Melodic requirement)

## Dependencies

### ROS Packages (apt)
```bash
sudo apt-get install -y \
  ros-melodic-cv-bridge \
  ros-melodic-image-transport \
  ros-melodic-compressed-image-transport \
  ros-melodic-move-base \
  ros-melodic-amcl \
  ros-melodic-gmapping \
  ros-melodic-robot-localization \
  ros-melodic-rplidar-ros \
  ros-melodic-rosserial-python \
  ros-melodic-explore-lite \
  python-rospkg \
  python-catkin-pkg \
  python-opencv \
  python-yaml
```

### Python Packages (pip)
```bash
pip2 install --user \
  AWSIoTPythonSDK \
  jetson-stats \
  smbus2 \
  pyyaml
```

### Camera and AWS KVS Dependencies
```bash
cd ~/catkin_ws/src/elderly_bot/scripts
chmod +x setup_camera_kvs.sh
./setup_camera_kvs.sh  # Installs GStreamer, AWS CLI, KVS SDK (~30 min)
```

## AWS Configuration

### Hardcoded Credentials
The system uses hardcoded AWS credentials for college project deployment:
- **Access Key**: `AKIAQFLXNXMMILSXZBPW`
- **Region**: `eu-west-1`
- **KVS Stream**: `RobotStream`

### Configure AWS
```bash
cd ~/catkin_ws/src/elderly_bot/scripts
chmod +x configure_aws.sh
./configure_aws.sh  # Sets up credentials and creates KVS stream
```

### AWS IoT Core Setup
1. Place certificates in `~/catkin_ws/src/elderly_bot/aws_certs/`:
   - `root-ca.pem`
   - `robot-nano-certificate.pem.crt`
   - `robot-nano-private.pem.key`

2. Update `config/cloud_config.yaml` with your IoT endpoint

3. Test connection:
   ```bash
   python2 scripts/final_handshake.py
   ```

## Build and Run

### Build Workspace
```bash
cd ~/catkin_ws
catkin_make
source devel/setup.bash
```

### Verify Python Scripts Compile
```bash
cd ~/catkin_ws/src/elderly_bot/scripts
for script in *.py; do
    python2 -m py_compile "$script" && echo "[OK] $script" || echo "[ERROR] $script"
done
```

### Launch Files

**Hardware Bringup** (ESP32, RPLidar, IMU, EKF):
```bash
roslaunch elderly_bot bringup.launch esp32_ip:=192.168.1.100
```

**Camera Only**:
```bash
roslaunch elderly_bot camera_streaming.launch enable_kvs:=false
```

**Camera + AWS KVS Streaming**:
```bash
roslaunch elderly_bot camera_streaming.launch
```

**Mapping** (requires bringup):
```bash
roslaunch elderly_bot mapping.launch
```

**Navigation** (requires map):
```bash
roslaunch elderly_bot navigation.launch map_file:=/path/to/map.yaml
```

## Troubleshooting

### ModuleNotFoundError: No module named 'rospkg'
**Cause**: Python version mismatch (python3 vs python2)
**Fix**:
```bash
# Verify all scripts use #!/usr/bin/env python2
head -n1 scripts/*.py

# Install rospkg for Python 2
sudo apt-get install python-rospkg

# Rebuild workspace
cd ~/catkin_ws && catkin_make clean && catkin_make
```

### Camera Node Fails to Start
**Check camera device**:
```bash
ls -l /dev/video*
v4l2-ctl --list-devices
```

**Check permissions**:
```bash
sudo usermod -a -G video $USER
# Logout and login again
```

### AWS KVS Stream Connection Fails
**Verify credentials**:
```bash
aws sts get-caller-identity
aws kinesisvideo describe-stream --stream-name RobotStream --region eu-west-1
```

**Check GStreamer kvssink plugin**:
```bash
gst-inspect-1.0 kvssink
```

### Encoding Errors in Scripts
**Clean compiled Python files**:
```bash
find ~/catkin_ws/src/elderly_bot -name "*.pyc" -delete
```

**Verify no non-ASCII characters**:
```bash
file scripts/*.py  # Should show "ASCII text" or "UTF-8 text"
```

## File Structure

```
elderly_bot/
├── CMakeLists.txt
├── package.xml
├── config/                    # Configuration files
│   ├── cloud_config.yaml      # AWS IoT/KVS settings
│   ├── sensors_actuators.yaml # Hardware I/O config
│   ├── amcl.yaml              # Localization params
│   ├── gmapping.yaml          # SLAM params
│   └── *_costmap.yaml         # Navigation params
├── launch/                    # ROS launch files
│   ├── bringup.launch         # Core hardware
│   ├── camera_streaming.launch # Camera + KVS
│   ├── mapping.launch         # SLAM
│   └── navigation.launch      # Autonomous navigation
├── scripts/                   # Python nodes
│   ├── camera_node.py         # Camera publisher
│   ├── kvs_streamer_node.py   # AWS KVS streaming
│   ├── cloud_bridge_node.py   # AWS IoT Core bridge
│   ├── sensors_actuators_node.py # Gas/buzzer control
│   ├── mpu9250_node.py        # IMU driver
│   ├── configure_aws.sh       # AWS setup script
│   └── setup_camera_kvs.sh    # KVS dependencies installer
├── urdf/
│   └── elderly_bot.urdf       # Robot model (with camera_link)
├── firmware/
│   └── elderly_bot_esp32_wifi.ino # ESP32 motor controller
└── aws_certs/                 # IoT Core certificates (gitignored)
```

## Testing Checklist

- [ ] `python2 -m py_compile scripts/*.py` - All scripts compile
- [ ] `roscore` + `roslaunch elderly_bot bringup.launch` - Hardware starts
- [ ] `rostopic hz /scan` - LiDAR publishes ~5 Hz
- [ ] `rostopic hz /odom` - Odometry publishes ~20 Hz
- [ ] `rostopic hz /imu/data` - IMU publishes ~30 Hz
- [ ] `roslaunch elderly_bot camera_streaming.launch enable_kvs:=false` - Camera publishes
- [ ] `rostopic hz /camera/image_raw` - Camera ~30 Hz
- [ ] `./scripts/configure_aws.sh` - AWS credentials configured
- [ ] `roslaunch elderly_bot camera_streaming.launch` - KVS streaming works
- [ ] View stream in AWS Console at RobotStream
- [ ] `python2 scripts/final_handshake.py` - IoT Core bidirectional test passes

## Additional Resources

- **Hardware Docs**: `HARDWARE.md`
- **System Architecture**: `SYSTEM_OVERVIEW.md`
- **ESP32 Firmware**: `firmware/elderly_bot_esp32_wifi.ino`
- **AWS Console**: https://eu-west-1.console.aws.amazon.com/

## Support

For issues, check:
1. `dmesg | tail` - Hardware connection errors
2. `rosnode list` - Active ROS nodes
3. `rostopic list` - Available topics
4. `rosnode info <node_name>` - Node details
5. `/var/log/ros/` - ROS log files
