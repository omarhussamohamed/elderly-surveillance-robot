# Recent Changes Summary

This document summarizes all recent changes made to the elderly_bot project. All documentation, code, and configuration files have been updated to reflect these changes.

## Date: January 22, 2026

---

## 1. New Features Added

### 1.1 Sensors & Actuators Node
**File**: `scripts/sensors_actuators_node.py`

Added comprehensive sensor and actuator support with graceful degradation:

- **MQ-6 Gas Sensor**: LPG/natural gas detection via ADS1115 16-bit I2C ADC
  - I2C address: 0x48
  - Threshold-based detection with voltage reporting
  - Publishes to `/gas_level` (sensor_msgs/Range) and `/gas_detected` (std_msgs/Bool)
  
- **Active Buzzer**: Alert/alarm notifications via Jetson GPIO
  - Configurable GPIO pin (BOARD numbering)
  - Auto-shutoff after 5 seconds for safety
  - Listens to `/buzzer_command` (std_msgs/Bool)
  
- **Jetson Monitoring**: System health monitoring using jetson-stats
  - Publishes to `/jetson_temperature` (sensor_msgs/Temperature)
  - Publishes to `/jetson_power` (std_msgs/Float32)

**Configuration**: `config/sensors_actuators.yaml`
- All features disabled by default (safe without hardware)
- Safe threshold values and pin configurations
- Full documentation included

**Launch Integration**: `launch/bringup.launch`
- Optional launch with `enable_sensors:=true`
- See `launch/SENSORS_ACTUATORS_LAUNCH_SNIPPET.xml` for details

### 1.2 Cloud Bridge Node
**File**: `scripts/cloud_bridge_node.py`

Added AWS IoT Core MQTT bridge for remote monitoring and control:

- **Bidirectional Communication**:
  - Publishes sensor data (gas, temperature, power) to AWS MQTT topic
  - Receives commands from AWS (buzzer, stop, move) and publishes to ROS
  
- **Features**:
  - Certificate-based authentication
  - JSON message format for cloud team compatibility
  - Graceful failure if AWS unavailable
  - Thread-safe with connection management
  
- **Message Format**:
  ```json
  {
    "timestamp": "2026-01-22T10:30:45.123456",
    "robot_id": "robot_01",
    "gas_level": 0.8,
    "gas_detected": false,
    "temperature": 45.2,
    "power": 12.5
  }
  ```

**Configuration**: `config/cloud_config.yaml`
- AWS endpoint, client_id, certificate paths
- MQTT topic configuration
- Complete AWS IoT Core setup guide (Thing creation, certificates, policy)

**Launch Integration**: `launch/bringup.launch`
- Optional launch with `enable_cloud:=true`
- See `launch/CLOUD_BRIDGE_LAUNCH_SNIPPET.xml` for details

---

## 2. Major Code Changes

### 2.1 IMU Magnetometer Removal
**File**: `scripts/mpu9250_node.py`

**Removed (~150 lines)**:
- `from sensor_msgs.msg import MagneticField` import
- All AK8963 magnetometer constants (registers, addresses)
- `_init_ak8963()` method implementation (~60 lines)
- `_read_magnetometer()` implementation (now returns zeros with comment)
- `mag_pub` publisher and all magnetometer message publishing

**Reason**: Indoor EMI from motors/PSU/battery corrupts magnetometer readings. Only gyro+accel used for orientation fusion via Madgwick filter.

### 2.2 IMU Temperature Topic Removal
**File**: `scripts/mpu9250_node.py`

**Removed**:
- All IMU die temperature reading and publishing
- `/imu/temperature` topic references

**Reason**: Jetson temperature (via jetson-stats) is more actionable for system monitoring. IMU die temperature not useful for robot operations.

### 2.3 EKF Configuration Fix
**File**: `config/ekf.yaml`

**Changed**:
- `imu0_config`: orientation yaw=true (was false), angular velocity vyaw=false (was true)
- `imu0_differential`: false (was true)
- Comments updated to reflect orientation fusion instead of angular velocity

**Reason**: EKF must fuse IMU orientation output from Madgwick filter, not raw angular velocity. Previous config was incorrect.

---

## 3. Documentation Updates

### 3.1 Files Updated

All documentation files have been updated to reflect the changes:

| File | Changes |
|------|---------|
| **README.md** | - Updated IMU description (magnetometer DISABLED, gyro+accel only)<br>- Added gas sensor, buzzer, Jetson monitoring<br>- Added hardware connections for new sensors<br>- Removed magnetometer references |
| **QUICK_START.md** | - Added optional Python packages installation<br>- Updated hardware connections (removed magnetometer note)<br>- Added note about IMU magnetometer disabled |
| **SYSTEM_OVERVIEW.md** | - Updated sensor table with new sensors<br>- Changed IMU from "9-axis" to "6-axis (gyro+accel)"<br>- Added new ROS topics for gas, buzzer, Jetson stats<br>- Removed /imu/mag and /imu/temperature topics |
| **HARDWARE_MAP.md** | - Removed /imu/temperature topic<br>- Removed /imu/mag topic<br>- Added new sensor/actuator topics |
| **CHANGELOG.md** | - Updated sensor fusion pipeline description<br>- Removed magnetometer references<br>- Added new features section |
| **package.xml** | - Added comment about optional Python dependencies |
| **install_dependencies.sh** | - Added optional packages section<br>- Installation instructions for new features<br>- Updated hardware connection list |

### 3.2 Launch Files Updated

| File | Changes |
|------|---------|
| **launch/bringup.launch** | - Updated IMU pipeline comment (magnetometer DISABLED)<br>- Added optional sensors_actuators_node launch<br>- Added optional cloud_bridge_node launch<br>- Both disabled by default |
| **launch/imu_nav.launch** | - Removed /imu/mag remap<br>- Removed magnetometer calibration instructions<br>- Updated verification commands |

### 3.3 New Documentation Files

| File | Purpose |
|------|---------|
| **launch/SENSORS_ACTUATORS_LAUNCH_SNIPPET.xml** | Integration guide for sensors/actuators node |
| **launch/CLOUD_BRIDGE_LAUNCH_SNIPPET.xml** | Integration guide for cloud bridge with AWS setup |
| **RECENT_CHANGES_SUMMARY.md** | This file - complete change summary |

---

## 4. Configuration Files

### 4.1 New Configuration Files

| File | Purpose |
|------|---------|
| **config/sensors_actuators.yaml** | Gas sensor, buzzer, Jetson monitoring configuration |
| **config/cloud_config.yaml** | AWS IoT Core connection configuration |

### 4.2 Modified Configuration Files

| File | Changes |
|------|---------|
| **config/ekf.yaml** | Fixed orientation fusion (yaw=true, vyaw=false, differential=false) |

---

## 5. Dependencies

### 5.1 New Python Dependencies (Optional)

Install with pip3:
```bash
# Gas sensor + buzzer + Jetson monitoring
sudo pip3 install jetson-stats Jetson.GPIO smbus2

# Cloud bridge (AWS IoT Core)
sudo pip3 install AWSIoTPythonSDK

# IMPORTANT: Reboot after jetson-stats installation
sudo reboot
```

### 5.2 Existing Dependencies (Unchanged)

All existing ROS packages remain the same (installed via `install_dependencies.sh`):
- ros-melodic-robot-localization
- ros-melodic-imu-filter-madgwick
- ros-melodic-gmapping
- ros-melodic-amcl
- ros-melodic-move-base
- And more...

---

## 6. Testing & Deployment

### 6.1 Completed Testing

- ✅ All Python syntax validated (no errors)
- ✅ All /imu/mag references removed (grep search confirmed)
- ✅ All /imu/temperature references removed
- ✅ EKF configuration matches Madgwick output
- ✅ Documentation consistency verified
- ✅ Launch file integration tested (syntax)

### 6.2 Pending Testing (On Jetson Hardware)

- ⏳ Gas sensor hardware connection and reading
- ⏳ Buzzer GPIO control
- ⏳ Jetson monitoring with jetson-stats
- ⏳ AWS IoT Core connection and communication
- ⏳ Full system integration test

### 6.3 Deployment Checklist

On Jetson Nano:

1. **Transfer Files**:
   ```bash
   scp -r config/ launch/ scripts/ omar@192.168.1.29:~/catkin_ws/src/elderly_bot/
   ```

2. **Set Permissions**:
   ```bash
   chmod +x ~/catkin_ws/src/elderly_bot/scripts/*.py
   ```

3. **Install Optional Dependencies** (if using new features):
   ```bash
   sudo pip3 install jetson-stats Jetson.GPIO smbus2 AWSIoTPythonSDK
   sudo reboot
   ```

4. **Connect Hardware** (optional):
   - MQ-6 sensor → ADS1115 channel A0 → I2C bus 1 (0x48)
   - Active buzzer → Jetson GPIO pin (configure in yaml)
   - Verify I2C: `sudo i2cdetect -y -r 1` (should show 0x48)

5. **Configure AWS** (if using cloud bridge):
   - Create Thing in AWS IoT Console
   - Download certificates (Root CA, device cert, private key)
   - Copy to Jetson: `~/aws_certs/`
   - Update `config/cloud_config.yaml`: aws_endpoint, certificate paths
   - Set `enable_cloud: true`

6. **Rebuild Workspace**:
   ```bash
   cd ~/catkin_ws
   catkin_make
   source devel/setup.bash
   ```

7. **Test Basic System**:
   ```bash
   roslaunch elderly_bot bringup.launch
   ```

8. **Test New Features** (optional):
   ```bash
   # With sensors/actuators
   roslaunch elderly_bot bringup.launch enable_sensors:=true
   
   # With cloud bridge
   roslaunch elderly_bot bringup.launch enable_cloud:=true
   
   # With both
   roslaunch elderly_bot bringup.launch enable_sensors:=true enable_cloud:=true
   ```

---

## 7. System Architecture Summary

### 7.1 Core Hardware (Unchanged)
- Jetson Nano (Ubuntu 18.04 + ROS Melodic)
- ESP32 motor controller (WiFi rosserial)
- 4WD differential drive (JGB37-520 motors)
- RPLidar A1 (USB)
- MPU-9250 IMU (I2C bus 1, 0x68) - **gyro+accel only**

### 7.2 New Hardware (Optional)
- MQ-6 gas sensor via ADS1115 ADC (I2C bus 1, 0x48)
- Active buzzer (Jetson GPIO)

### 7.3 Software Stack
- **Localization**: robot_localization EKF fuses /wheel_odom + /imu/data
- **IMU Fusion**: imu_filter_madgwick (gyro+accel, **magnetometer DISABLED**)
- **Mapping**: gmapping SLAM
- **Navigation**: AMCL + move_base + DWA local planner
- **New**: sensors_actuators_node (gas/buzzer/stats)
- **New**: cloud_bridge_node (AWS IoT Core)

### 7.4 Key ROS Topics

**Core Topics** (existing):
- `/wheel_odom` - Wheel encoder odometry from ESP32
- `/imu/data_raw` - Raw gyro+accel from MPU-9250
- `/imu/data` - Fused orientation from Madgwick filter
- `/odometry/filtered` - EKF output (odom → base_footprint TF)
- `/scan` - Laser scan from RPLidar
- `/cmd_vel` - Velocity commands to motors

**New Topics** (optional):
- `/gas_level` - Gas sensor voltage (sensor_msgs/Range)
- `/gas_detected` - Gas detection boolean (std_msgs/Bool)
- `/buzzer_command` - Buzzer control (std_msgs/Bool)
- `/jetson_temperature` - Jetson die temperature (sensor_msgs/Temperature)
- `/jetson_power` - Jetson power consumption (std_msgs/Float32)

**Removed Topics**:
- ~~`/imu/mag`~~ - Magnetometer data (REMOVED)
- ~~`/imu/temperature`~~ - IMU die temperature (REMOVED)

---

## 8. Important Notes

### 8.1 Magnetometer Policy

**STRICT POLICY**: Magnetometer is **BANNED** from localization stack.

**Reason**: Indoor EMI from motors, PSU, and battery corrupts magnetometer readings, causing:
- Random yaw jumps in /imu/data
- Map rotation drift in gmapping
- Poor localization in AMCL

**Solution**: Use only gyro+accel for orientation fusion via Madgwick filter (gain=0.9, gyro-dominant).

### 8.2 Graceful Degradation

All new nodes follow graceful degradation pattern:
- Features individually enable/disable via parameters
- Safe defaults when hardware unavailable
- Extensive error handling with try-except blocks
- Clear error messages in logs

### 8.3 Optional Features

New features are **disabled by default**:
- No hardware required for basic operation
- Enable individually as needed
- No breaking changes to existing functionality

---

## 9. Contact & Support

For questions about these changes:
- Review this document
- Check individual file headers for detailed documentation
- See launch snippet files for integration examples
- Refer to config/*.yaml files for parameter documentation

---

**End of Document**
