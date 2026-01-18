# Elderly Bot - Documentation Index

**Last Updated:** January 18, 2026  
**System Status:** Production Ready - All Critical Fixes Applied

---

## 🚀 Quick Navigation

### For New Users - Start Here
1. [README.md](README.md) - Project overview and quick links
2. [QUICK_START.md](QUICK_START.md) - Step-by-step setup guide
3. [HARDWARE_MAP.md](HARDWARE_MAP.md) - Hardware specifications and connections

### For Deployment - Essential Reading
1. [COMPLETE_TF_DEPLOYMENT.md](COMPLETE_TF_DEPLOYMENT.md) - **Complete deployment procedure**
2. Firmware Upload: See firmware/elderly_bot_esp32_wifi.ino comments (TICKS_PER_REV=3960)
3. [PHYSICAL_VALIDATION_PROTOCOL.md](PHYSICAL_VALIDATION_PROTOCOL.md) - Testing checklist

---

## 📚 Core Documentation

### System Architecture
- **[SYSTEM_OVERVIEW.md](SYSTEM_OVERVIEW.md)** - Complete system architecture, operational modes, TF tree
- **[HARDWARE_MAP.md](HARDWARE_MAP.md)** - Hardware specs, pinouts, calibration values
- **[CHANGELOG.md](CHANGELOG.md)** - Complete change history

### Setup & Configuration
- **[QUICK_START.md](QUICK_START.md)** - Installation and first-time setup
- **[ROSSERIAL_GUIDE.md](ROSSERIAL_GUIDE.md)** - WiFi rosserial setup and troubleshooting
- **[docs/MPU9250_JETSON_SETUP.md](docs/MPU9250_JETSON_SETUP.md)** - IMU hardware I2C setup
- **[docs/IMU_CALIBRATION.md](docs/IMU_CALIBRATION.md)** - IMU sensor fusion configuration

---

## 🔧 Critical Fixes & Calibration

### TF Frame Alignment (Latest Fix - Jan 18, 2026)
- **[TF_FRAME_ALIGNMENT_FIX.md](TF_FRAME_ALIGNMENT_FIX.md)** - ⭐ **Technical details of TF coordinate frame fix**
- **[COMPLETE_TF_DEPLOYMENT.md](COMPLETE_TF_DEPLOYMENT.md)** - 🚀 **Step-by-step deployment procedure**
- **[IMU_MOUNTING_FIX_REFERENCE.md](IMU_MOUNTING_FIX_REFERENCE.md)** - IMU rotation scenarios and URDF fixes

**What was fixed:**
- Laser frame: 180° roll → 180° yaw (correct backward-facing lidar)
- Added robot_state_publisher for systematic TF broadcasting
- Removed duplicate static transform publishers
- Created diagnostic tools for IMU orientation verification

### Drift Fix (Gyro Drift / Map Ghosting)
- **[DRIFT_FIX_APPLIED.md](DRIFT_FIX_APPLIED.md)** - ✅ **Solution for stationary rotation drift**

**What was fixed:**
- EKF now fuses IMU absolute yaw orientation (not angular velocity)
- Madgwick filter bias correction enabled (zeta=0.01)
- TCP nodelay optimization for reduced latency
- Result: <0.05° drift over 3 minutes

### Kinematic Scaling Fix (1cm→1m Bug)
- **[KINEMATIC_FIX_APPLIED.md](KINEMATIC_FIX_APPLIED.md)** - 🎯 **Solution for odometry scaling error**

**What was fixed:**
- TICKS_PER_REV corrected from 4900 to 3960 (11 PPR × 90 × 4)
- WHEEL_RADIUS verified at 0.0325m
- Float math casting fixed in ticks_to_m calculation
- Result: 1:1 kinematic mapping, robot moves exactly 1.0m when commanded

---

## 🧪 Testing & Validation

### Automated Testing Scripts
Located in [scripts/](scripts/)

**Comprehensive Validation:**
- **master_validator.sh** - Complete 4-stage system validation
  - Stage 1: Communication audit (topic rates)
  - Stage 2: 3-minute drift stress test
  - Stage 3: 1-meter linear scaling test
  - Stage 4: 360° rotation test

**TF & Frame Diagnostics:**
- **tf_verification_complete.sh** - Automated TF frame verification
- **imu_mounting_diagnostic.sh** - Physical IMU orientation testing

**Individual Tests:**
- **final_drift_validation.sh** - Drift and rate monitoring
- **autonomous_square_test.sh** - 1m×1m autonomous mapping test
- **boss_level_test.sh** - Combined validation wrapper

### Testing Documentation
- **[PHYSICAL_VALIDATION_PROTOCOL.md](PHYSICAL_VALIDATION_PROTOCOL.md)** - Manual testing procedures with pass/fail checklist
- **[IMU_MOUNTING_FIX_REFERENCE.md](IMU_MOUNTING_FIX_REFERENCE.md)** - IMU diagnostic scenarios

---

## 📖 Reference Documentation

### Configuration Files
Located in [config/](config/)
- `ekf.yaml` - Robot localization sensor fusion (50Hz)
- `gmapping.yaml` - SLAM parameters (minimumScore=50)
- `amcl.yaml` - Localization parameters
- `dwa_local_planner.yaml` - Local path planner
- `costmap_common_params.yaml` - Obstacle detection
- `global_costmap.yaml`, `local_costmap.yaml` - Costmap configs
- `patrol_goals.yaml` - Autonomous patrol waypoints

### Launch Files
Located in [launch/](launch/)
- `bringup.launch` - Core hardware interfaces (RPLidar, ESP32, IMU, robot_state_publisher)
- `imu_nav.launch` - IMU processing pipeline (Madgwick filter)
- `mapping.launch` - Autonomous mapping mode (gmapping + explore_lite)
- `navigation.launch` - Patrol mode (AMCL + move_base)
- `mpu9250_jetson.launch` - Standalone IMU test

### Robot Description
Located in [urdf/](urdf/)
- `elderly_bot.urdf` - Complete robot URDF with corrected TF frames
  - base_footprint → base_link (0.07m height offset)
  - base_link → laser (0.23m height, 180° yaw for backward mounting)
  - base_link → imu_link (centered, aligned with robot frame)

### Firmware
Located in [firmware/](firmware/)
- `elderly_bot_esp32_wifi.ino` - Production ESP32 firmware
  - TICKS_PER_REV = 3960
  - WHEEL_RADIUS = 0.0325m
  - WiFi rosserial TCP connection
  - Dual-core architecture (Core 0: WiFi, Core 1: motors)
- `motor_and_encoder_HW_test.ino` - Hardware diagnostic
- `motor_and_encoder_SW_test.ino` - Software diagnostic

---

## 🗺️ Maps
Located in [maps/](maps/)
- Store generated maps here
- Use: `rosrun map_server map_saver -f ~/catkin_ws/src/elderly_bot/maps/map_name`

---

## 📊 System Status Summary

### ✅ Verified Working (Jan 18, 2026)
- [x] TF coordinate frames properly aligned (laser 180° yaw)
- [x] Robot state publisher broadcasting URDF transforms
- [x] Kinematic scaling 1:1 (TICKS_PER_REV=3960)
- [x] Stationary drift <0.1° over 3 minutes
- [x] EKF absolute yaw fusion from IMU
- [x] TCP nodelay optimization for low latency
- [x] Gmapping minimumScore=50 for stable maps
- [x] Master validation suite (all 4 stages passing)

### 🔄 Requires Per-Robot Verification
- [ ] IMU physical mounting orientation (use imu_mounting_diagnostic.sh)
- [ ] WiFi network credentials (ESP32 and Jetson)
- [ ] Serial port permissions (add user to dialout group)
- [ ] Map generation for specific environment

### 📝 Operational Checklist
Before each mission:
1. Run tf_verification_complete.sh - verify TF frames aligned
2. Run master_validator.sh - verify all 4 stages pass
3. Visual check in RViz - TF axes parallel
4. Test 1m forward movement - verify ±2cm accuracy

---

## 🛠️ Troubleshooting

### Common Issues & Solutions

**"Laser frame still has 180° roll"**
→ URDF not reloaded, restart ROS completely (kill roscore)
→ See: [COMPLETE_TF_DEPLOYMENT.md](COMPLETE_TF_DEPLOYMENT.md) Phase 2

**"Robot moves 6cm instead of 1m"**
→ Firmware not uploaded with TICKS_PER_REV=3960
→ See: [ESP32_FIRMWARE_UPLOAD.txt](ESP32_FIRMWARE_UPLOAD.txt)

**"Map hKINEMATIC_FIX_APPLIED.md](KINEMATIC_FIX_APPLIED.md) for firmware upload
→ Check drift with master_validator.sh Stage 2
→ See: [DRIFT_FIX_APPLIED.md](DRIFT_FIX_APPLIED.md)

**"IMU axes misaligned in RViz"**
→ Run imu_mounting_diagnostic.sh to identify rotation
→ See: [IMU_MOUNTING_FIX_REFERENCE.md](IMU_MOUNTING_FIX_REFERENCE.md)

**"rosserial not connecting"**
→ Check WiFi, firewall port 11411
→ See: [ROSSERIAL_GUIDE.md](ROSSERIAL_GUIDE.md)

---

## 📞 Support Resources

### Internal Documentation (This Package)
- All MD files in root directory
- [docs/](docs/) - Detailed setup guides
- [scripts/](scripts/) - Validation and diagnostic tools

### External References
- [ROS Robot Localization](http://docs.ros.org/en/melodic/api/robot_localization/html/index.html)
- [REP-105: Coordinate Frames](https://www.ros.org/reps/rep-0105.html)
- [Gmapping Documentation](http://wiki.ros.org/gmapping)
- [ESP32 Arduino Core](https://github.com/espressif/arduino-esp32)

---

## 🎯 Next Steps After Setup

1. **Verify System**: Run complete validation suite
2. **Create First Map**: Use mapping.launch to explore environment
3. **Test Navigation**: Load map and test waypoint navigation
4. **Define Patrol Route**: Edit config/patrol_goals.yaml
5. **Production Deployment**: Run full autonomous patrol mission

---

**Package Maintainer:** elderly_bot development team  
**ROS Version:** Melodic (Ubuntu 18.04)  
**Last System Audit:** January 18, 2026  
**Status:** Production Ready ✅
