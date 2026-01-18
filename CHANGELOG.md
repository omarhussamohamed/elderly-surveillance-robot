# Elderly Bot - Changelog

## Latest Updates (January 18, 2026)

### TF Frame Alignment Fix (CRITICAL)

#### Laser Frame Rotation Corrected
- **Problem**: Lidar used 180° **roll** rotation instead of 180° **yaw**
- **Impact**: Inverted X/Y axes causing frame misalignment and potential sensor fusion errors
- **Fix**: Changed URDF `rpy="3.14159 0 0"` → `rpy="0 0 3.14159"`
- **Files**: `urdf/elderly_bot.urdf`, `launch/bringup.launch`
- **Result**: All coordinate frames now properly aligned per REP-105

#### Robot State Publisher Added
- **Added**: Systematic TF broadcaster from URDF
- **Removed**: Duplicate static_transform_publisher nodes
- **Benefit**: Single source of truth, eliminates TF conflicts
- **File**: `launch/bringup.launch`

#### IMU Mounting Diagnostic Tools
- **Added**: `scripts/imu_mounting_diagnostic.sh` - Physical orientation testing
- **Added**: `scripts/tf_verification_complete.sh` - Automated TF verification
- **Added**: `IMU_MOUNTING_FIX_REFERENCE.md` - Common scenarios guide
- **Documentation**: `TF_FRAME_ALIGNMENT_FIX.md`, `COMPLETE_TF_DEPLOYMENT.md`

### Kinematic Scaling Correction (3960 Ticks/Rev)
- **Problem**: Incorrect TICKS_PER_REV (4900) causing 1cm movement for 1m command
- **Root Cause**: Wrong encoder specification (990 PPR instead of actual 11 PPR)
- **Fix**: Corrected to 3960 ticks/rev (11 PPR × 90:1 gear ratio × 4 edges)
- **Result**: 1:1 kinematic mapping, robot moves exactly 1.0m when commanded
- **Files**: `firmware/elderly_bot_esp32_wifi.ino`, `HARDWARE_MAP.md`
- **Documentation**: `KINEMATIC_FIX_APPLIED.md`, `FINAL_KINEMATIC_VERIFICATION.txt`

### Validation & Testing Infrastructure
- **Added**: `scripts/master_validator.sh` - 4-stage automated system validation
- **Tests**: Communication audit, 3-min drift test, 1m linear test, 360° rotation test
- **Added**: `PHYSICAL_VALIDATION_PROTOCOL.md` - Manual testing procedures
- **Output**: Pass/fail dashboard with diagnostic hints

## Previous Updates (January 2026)

### IMU System Enhancement

#### MPU9250 Magnetometer Integration
- **Added**: 3-axis magnetometer (AK8963) support
- **Added**: Temperature sensor reading
- **Benefit**: Absolute heading reference prevents yaw drift
- **Topics**: `/imu/mag`, `/imu/temperature`
- **Documentation**: [docs/IMU_CALIBRATION.md](docs/IMU_CALIBRATION.md)

#### Dynamic Gyroscope Calibration
- **Changed**: Static calibration → Dynamic startup calibration
- **Method**: 100-sample averaging on each node launch
- **Benefit**: Temperature-adaptive, no manual calibration needed
- **Result**: Zero drift when stationary (<0.001 rad/s)

#### Sensor Fusion Pipeline
- **Added**: `imu_nav.launch` - Complete IMU processing pipeline
- **Flow**: Raw IMU → `imu_filter_madgwick` → Fused orientation
- **Topics**:
  - Input: `/imu/data_raw`, `/imu/mag`
  - Output: `/imu/data` (with orientation quaternion)
- **Frame**: All messages use `imu_link` frame ID

#### Configuration Updates
- **File**: `launch/imu_nav.launch`
- Magnetometer fusion enabled (`use_mag: true`)
- Gravity vector removal enabled
- Pre-calibrated mag biases included
- Madgwick gain tuned to 0.1 (balanced)

### Odometry System Enhancement

#### Encoder Debouncing
- **Added**: 100µs software debounce in ESP32 ISRs
- **Method**: Time-based pulse filtering using `micros()`
- **Result**: Ghost pulses eliminated, static creep stopped
- **Threshold**: Optimized for 100-400 RPM operation
- **Documentation**: [docs/ENCODER_CALIBRATION.md](docs/ENCODER_CALIBRATION.md)

#### Wheel Radius Calibration
- **Initial**: 32.5mm (nominal) → 60% overcounting
- **First calibration**: 20.3mm → 4.6% overcounting
- **Final calibration**: 19.4mm → <1% error
- **Method**: Empirical 1-meter push tests
- **Accuracy**: ±0.5% (0.995-1.005m for 1.0m physical)

#### Zero Initialization
- **Added**: Explicit encoder count reset on startup
- **Benefit**: No negative offsets, clean zero state
- **Implementation**: Reset in `setup()` function

#### Firmware Updates
- **File**: `firmware/elderly_bot_esp32_wifi.ino`
- ISR functions updated with debounce logic
- `WHEEL_RADIUS` constant calibrated (0.0194m)
- `DEBOUNCE_MICROS` constant defined (100µs)
- Startup initialization routine added

### Documentation Improvements

#### New Documents
- **[docs/IMU_CALIBRATION.md](docs/IMU_CALIBRATION.md)**
  - Dynamic calibration procedure
  - Magnetometer integration details
  - Sensor fusion pipeline explanation
  - Troubleshooting guide

- **[docs/ENCODER_CALIBRATION.md](docs/ENCODER_CALIBRATION.md)**
  - Debouncing implementation
  - Calibration methodology
  - Performance benchmarks
  - Maintenance procedures

#### Updated Documents
- **[README.md](README.md)**
  - Added links to new calibration docs
  - Updated sensor specifications
  - Corrected encoder resolution (990 PPR)
  - Added IMU magnetometer mention

- **[docs/MPU9250_JETSON_SETUP.md](docs/MPU9250_JETSON_SETUP.md)**
  - Updated with magnetometer features
  - Added temperature sensor info
  - Expanded troubleshooting section

### Performance Improvements

#### Odometry Accuracy
- **Before**: ±38-60% error
- **After**: ±0.5% error
- **Static stability**: No creep (<0.001m/min)

#### IMU Orientation
- **Before**: Invalid quaternion (all zeros with -1 covariance)
- **After**: Valid fused orientation with magnetometer
- **Static stability**: <1°/minute drift
- **Convergence**: <2 seconds after movement

#### System Reliability
- Ghost pulse elimination → Consistent encoder counts
- Dynamic calibration → Temperature-independent operation
- Zero initialization → Predictable startup behavior

## Breaking Changes

### Topic Changes
- **`/imu/data`**: Now published by `imu_filter_madgwick` (was `mpu9250_node`)
- **New topic**: `/imu/data_raw` - Raw IMU without orientation
- **Impact**: Update any nodes subscribing to `/imu/data`

### Launch File Changes
- **Recommended**: Use `imu_nav.launch` instead of `mpu9250_jetson.launch`
- **Includes**: Both raw IMU node and Madgwick filter
- **Benefit**: Complete sensor fusion pipeline

### Firmware Changes
- **Required**: Re-upload ESP32 firmware for debouncing
- **Required**: May need to recalibrate `WHEEL_RADIUS` for your robot
- **Note**: Startup calibration requires robot to be stationary

## Migration Guide

### From Old IMU Setup

**1. Update launch files**:
```bash
# Old
roslaunch elderly_bot mpu9250_jetson.launch

# New
roslaunch elderly_bot imu_nav.launch
```

**2. Update topic subscribers**:
```python
# Old
rospy.Subscriber("/imu/data", Imu, callback)  # Raw data

# New
rospy.Subscriber("/imu/data", Imu, callback)  # Fused data (same topic, different source)
# Or for raw data:
rospy.Subscriber("/imu/data_raw", Imu, callback)
```

**3. Verify magnetometer**:
```bash
rostopic echo /imu/mag
# Should show valid magnetic field readings
```

### From Old Odometry Setup

**1. Upload new firmware**:
- Open `firmware/elderly_bot_esp32_wifi.ino` in Arduino IDE
- Verify `DEBOUNCE_MICROS = 100` and `WHEEL_RADIUS = 0.0194`
- Upload to ESP32

**2. Test calibration**:
```bash
# Power cycle robot
# Push exactly 1.0m
rostopic echo /wheel_odom | grep "x:"
# Should report ~1.00m (±0.005m)
```

**3. Recalibrate if needed**:
```cpp
// If accuracy is off, adjust in firmware:
const float WHEEL_RADIUS = 0.0194 * (1.0 / measured_distance);
// Re-upload firmware
```

## Known Issues

### IMU
- Magnetometer requires environment-specific calibration (hard-iron bias)
- First 2 seconds after startup may have noisy orientation (filter converging)
- Gyro calibration requires robot to be completely stationary

### Odometry
- 100µs debounce may filter legitimate pulses above 500 RPM (unlikely for this robot)
- Wheel radius calibration is surface-dependent (carpet vs. tile)
- Slippage during aggressive turns can introduce error

## Future Improvements

### Planned
- [ ] Soft-iron magnetometer calibration (ellipsoid fitting)
- [ ] Adaptive debounce threshold based on motor speed
- [ ] EKF integration guide for combined IMU + odometry
- [ ] Automated calibration scripts
- [ ] RViz visualization for calibration verification

### Under Consideration
- [ ] Online wheel radius estimation
- [ ] Gyro bias tracking during operation
- [ ] Accelerometer calibration routine
- [ ] Encoder health monitoring

## Version History

### v2.0 (January 2026)
- IMU magnetometer integration
- Dynamic gyroscope calibration
- Encoder debouncing (100µs)
- Wheel radius calibration (19.4mm)
- Comprehensive documentation

### v1.0 (Initial Release)
- Basic 4WD robot platform
- GMapping SLAM
- AMCL navigation
- Patrol waypoint system
- ESP32 motor control
- RPLidar integration
- Basic IMU support (accel + gyro only)

## Support & Contact

For issues, improvements, or questions:
- Check documentation in `docs/` folder
- Review [TROUBLESHOOTING.md](TROUBLESHOOTING.md)
- See specific calibration guides for detailed procedures

## Contributors

- IMU magnetometer integration and dynamic calibration
- Encoder debouncing implementation
- Odometry calibration methodology
- Documentation expansion

---

**Last Updated**: January 14, 2026
