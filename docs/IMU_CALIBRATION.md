# IMU Dynamic Calibration & Sensor Fusion Pipeline

## Overview

This document describes the MPU9250 IMU sensor configuration, including dynamic gyroscope calibration, magnetometer integration, and the complete sensor fusion pipeline.

## System Architecture

### Data Flow Pipeline

```
┌─────────────────────┐
│   MPU9250 Sensor    │
│  (I2C on Jetson)    │
└──────────┬──────────┘
           │
           v
┌─────────────────────┐      Topics Published:
│  mpu9250_node.py    │ ───► /imu/data_raw (accel + gyro)
│  (Python ROS Node)  │ ───► /imu/mag (magnetometer)
└──────────┬──────────┘ ───► /imu/temperature (internal sensor)
           │
           │ Raw IMU + Mag
           v
┌─────────────────────┐
│ imu_filter_madgwick │
│ (Sensor Fusion)     │
└──────────┬──────────┘
           │
           v
      /imu/data (fused orientation)
```

## MPU9250 Node Features

**Location**: `scripts/mpu9250_node.py`

### 1. Dynamic Gyroscope Calibration

#### Problem
Static gyroscope bias drifts with temperature changes, causing accumulated orientation error over time.

#### Solution: Startup Calibration

**On every node launch**:
1. Robot MUST be stationary
2. Collects 100 gyroscope samples (~1 second)
3. Averages samples to determine current static drift
4. Applies offset for entire session

**Implementation**:
```python
def _calibrate_gyro(self):
    num_samples = 100
    gyro_sum = [0.0, 0.0, 0.0]
    
    for i in range(num_samples):
        gyro_x, gyro_y, gyro_z = self._read_gyro_raw()
        gyro_sum[0] += gyro_x
        gyro_sum[1] += gyro_y
        gyro_sum[2] += gyro_z
        rospy.sleep(0.01)  # 100Hz sampling
    
    # Calculate average static drift
    self.gyro_offset[0] = gyro_sum[0] / num_samples
    self.gyro_offset[1] = gyro_sum[1] / num_samples
    self.gyro_offset[2] = gyro_sum[2] / num_samples
```

**Startup Sequence**:
```
[ INFO] Starting dynamic gyroscope calibration (100 samples)...
[ INFO] Keep robot STATIONARY for gyroscope calibration...
[ INFO] Calibration successful with 100 samples
[ INFO] Gyro calibration complete: X=0.1523, Y=0.1441, Z=0.0179 rad/s
```

**Benefits**:
- Temperature-adaptive (recalibrates each launch)
- No manual calibration required
- Eliminates orientation drift when stationary

### 2. Magnetometer (AK8963) Integration

#### Features

**3-axis compass** for absolute heading reference:
- Prevents yaw drift during long operation
- Provides North reference for localization
- Hard-iron bias correction applied

**Initialization Sequence**:
1. Enable I2C bypass mode to access AK8963
2. Verify WHO_AM_I register (0x48)
3. Read sensitivity adjustment values from fuse ROM
4. Configure 16-bit continuous mode at 100Hz
5. Apply factory calibration factors

**Hard-Iron Bias Calibration**:
```python
# Pre-calibrated values (environment-specific)
self.mag_offset = [-0.00004252, 0.00004791, 0.00002824]  # Tesla

# Applied after raw reading
mag_x -= self.mag_offset[0]
mag_y -= self.mag_offset[1]
mag_z -= self.mag_offset[2]
```

**Output**: `sensor_msgs/MagneticField`
- Frame: `imu_link`
- Units: Tesla (T)
- Typical Earth field: 25-65 µT (varies by location)

### 3. Temperature Monitoring

**Internal sensor** for thermal drift awareness:
- Range: -40°C to +85°C
- Accuracy: ±3°C
- Formula: `(raw_value / 333.87) + 21.0`

**Use cases**:
- Sensor health monitoring
- Thermal drift compensation
- Environmental logging

### 4. Published Topics

| Topic | Message Type | Rate | Content |
|-------|--------------|------|---------|
| `/imu/data_raw` | `sensor_msgs/Imu` | 100Hz | Accel + Gyro (no orientation) |
| `/imu/mag` | `sensor_msgs/MagneticField` | 100Hz | 3-axis magnetometer |
| `/imu/temperature` | `sensor_msgs/Temperature` | 100Hz | Internal temp sensor |

**Note**: Orientation quaternion in `/imu/data_raw` is zero - computed by Madgwick filter.

## Sensor Fusion Pipeline

### imu_filter_madgwick Configuration

**Location**: `launch/imu_nav.launch`

**Input Topics**:
- `/imu/data_raw` - Raw accelerometer + gyroscope
- `/imu/mag` - Magnetometer data

**Output Topic**:
- `/imu/data` - Fused IMU with computed orientation quaternion

**Key Parameters**:

```xml
<!-- Sensor Configuration -->
<param name="use_mag" value="true" />                    <!-- Enable magnetometer fusion -->
<param name="use_magnetic_field_msg" value="true" />     <!-- Use sensor_msgs/MagneticField -->
<param name="publish_tf" value="true" />                 <!-- Publish TF transform -->

<!-- Filter Tuning -->
<param name="gain" value="0.1" />                        <!-- Fusion gain (0.0-1.0) -->
<param name="zeta" value="0.0" />                        <!-- Gyro drift compensation -->

<!-- Magnetometer Calibration -->
<param name="mag_bias/x" value="-0.00004252" />
<param name="mag_bias/y" value="0.00004791" />
<param name="mag_bias/z" value="0.00002824" />

<!-- Gravity Handling -->
<param name="remove_gravity_vector" value="true" />      <!-- Clean linear acceleration -->
<param name="stateless" value="false" />                 <!-- Maintain state for smoothness -->

<!-- Frame Configuration -->
<param name="world_frame" value="enu" />                 <!-- East-North-Up convention -->
<param name="fixed_frame" value="odom" />
```

### Filter Gain Tuning

**Gain Parameter** (`0.0` to `1.0`):
- **Higher (0.2-0.5)**: Trust gyro more → Faster response, more noise
- **Lower (0.05-0.1)**: Trust accel/mag more → Smoother, slower response
- **Default (0.1)**: Balanced for indoor robots

**When to adjust**:
- Jittery orientation → Decrease gain
- Lagging orientation → Increase gain
- Drift despite mag → Check mag calibration first

## Magnetometer Calibration Procedure

### When to Calibrate

- First-time setup
- After moving to new location/building
- Near large metal structures
- Persistent yaw drift during rotation tests

### Hard-Iron Calibration Steps

**1. Collect Raw Data**:
```bash
roslaunch elderly_bot imu_nav.launch
rostopic echo /imu/mag > mag_calibration.txt

# Move robot in slow figure-8 pattern for 60 seconds
# Cover all orientations (pitch, roll, yaw)
```

**2. Analyze Data**:
```python
# Find min/max for each axis from mag_calibration.txt
x_min, x_max = ...
y_min, y_max = ...
z_min, z_max = ...

# Calculate bias (center of sphere)
bias_x = (x_min + x_max) / 2.0
bias_y = (y_min + y_max) / 2.0
bias_z = (z_min + z_max) / 2.0
```

**3. Update Launch File**:
```xml
<arg name="madgwick_mag_bias_x" default="<calculated_bias_x>" />
<arg name="madgwick_mag_bias_y" default="<calculated_bias_y>" />
<arg name="madgwick_mag_bias_z" default="<calculated_bias_z>" />
```

**4. Verify**:
```bash
rostopic echo /imu/data | grep orientation -A 4
# Rotate robot 360°, check if yaw returns close to initial value
```

## Hardware Setup

### I2C Wiring (Jetson Nano)

```
MPU9250          Jetson Nano J21 Header
-------          ---------------------
VCC       →      Pin 1  (3.3V)
GND       →      Pin 6  (GND)
SDA       →      Pin 3  (I2C2_SDA)
SCL       →      Pin 5  (I2C2_SCL)
```

### I2C Configuration

**Bus**: I2C-1 (`/dev/i2c-1`)
**Address**: `0x68` (AD0 pin LOW)
**Speed**: 100 kHz (default, sufficient)

**Verification**:
```bash
sudo i2cdetect -y 1
# Should show device at 0x68
# AK8963 magnetometer appears at 0x0C
```

## Usage

### Launch Complete Pipeline

```bash
roslaunch elderly_bot imu_nav.launch
```

**Expected output**:
```
[ INFO] Initialized I2C bus 1
[ INFO] MPU9250 detected (WHO_AM_I = 0x71)
[ INFO] AK8963 sensitivity adjustments: X=1.168, Y=1.164, Z=1.151
[ INFO] AK8963 magnetometer initialized successfully
[ INFO] Starting dynamic gyroscope calibration (100 samples)...
[ INFO] Keep robot STATIONARY for gyroscope calibration...
[ INFO] Calibration successful with 100 samples
[ INFO] Gyro calibration complete: X=0.1523, Y=0.1441, Z=0.0179 rad/s
[ INFO] MPU9250 node started. Publishing to /imu/data_raw at 100 Hz
```

### Verification Commands

**Check all topics active**:
```bash
rostopic list | grep imu
# /imu/data
# /imu/data_raw
# /imu/mag
# /imu/temperature
```

**Check publish rates**:
```bash
rostopic hz /imu/data_raw /imu/data /imu/mag
# All should be ~100 Hz
```

**Verify orientation**:
```bash
rostopic echo /imu/data | grep orientation -A 4
# Should show valid quaternion (w ≈ 1.0 when upright)
# NOT all zeros
```

## Troubleshooting

### Issue: Orientation all zeros

**Causes**:
- `imu_filter_madgwick` not running
- Topic remapping incorrect
- Orientation covariance set to -1.0

**Solutions**:
1. Check Madgwick node is running: `rosnode list | grep imu_filter`
2. Verify covariance in raw data: `rostopic echo /imu/data_raw | grep covariance`
3. Restart launch file

### Issue: Orientation drifts during rotation

**Causes**:
- Magnetometer not calibrated
- Magnetic interference (motors, metal)
- Gain too high

**Solutions**:
1. Perform magnetometer calibration (figure-8 pattern)
2. Move IMU away from motors/metal
3. Reduce Madgwick gain to 0.05
4. Verify mag data is stable: `rostopic echo /imu/mag`

### Issue: Gyro creep when stationary

**Causes**:
- Robot moved during startup calibration
- Temperature change after calibration
- Sensor hardware failure

**Solutions**:
1. Restart node with robot COMPLETELY stationary
2. Allow sensor to warm up 30 seconds before calibration
3. Increase calibration samples to 200
4. Check raw gyro readings: `rostopic echo /imu/data_raw | grep angular_velocity`

### Issue: Magnetometer initialization fails

**Causes**:
- I2C bus contention
- Wrong I2C address
- Hardware connection issue

**Solutions**:
1. Check I2C devices: `sudo i2cdetect -y 1` (should see 0x0C)
2. Verify bypass mode is enabled in MPU9250
3. Check wiring (SDA/SCL not swapped)
4. Node continues without mag if init fails

## Performance Benchmarks

### Accuracy

| Metric | Value |
|--------|-------|
| Gyro static noise | ±0.001 rad/s |
| Accel static noise | ±0.02 m/s² |
| Mag field strength | 25-65 µT (location dependent) |
| Orientation convergence | <2 seconds |
| Yaw drift (with mag) | <1°/minute |

### Timing

| Operation | Duration |
|-----------|----------|
| Gyro calibration | 1.0 second (100 samples) |
| Sensor read cycle | 10 ms (100 Hz) |
| Filter update | <1 ms |
| Total latency | <15 ms (sensor to /imu/data) |

## Integration with Robot Localization

The fused `/imu/data` topic is consumed by `robot_localization` EKF:

```yaml
# In ekf.yaml
imu0: /imu/data
imu0_config: [false, false, false,  # position (not used)
              false, false, true,   # orientation (yaw only)
              false, false, false,  # velocity (not used)
              false, false, true,   # angular velocity (yaw rate)
              true, true, true]     # acceleration (all axes)
```

**Why yaw only**: Pitch/roll from accelerometer can be noisy during motion, wheel odometry is more reliable.

## See Also

- [MPU9250_JETSON_SETUP.md](MPU9250_JETSON_SETUP.md) - Hardware setup guide
- [imu_nav.launch](../launch/imu_nav.launch) - Launch file configuration
- [ekf.yaml](../config/ekf.yaml) - EKF fusion parameters
- [HARDWARE_MAP.md](../HARDWARE_MAP.md) - I2C pin assignments
