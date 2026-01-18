# Map Ghosting & Stationary Drift Fix - Applied Changes

**Date:** January 18, 2026  
**Issue:** Robot showing -13° rotation drift while stationary, causing map duplication/ghosting in Foxglove/RViz

## Problem Analysis

The -13.196° stationary drift in `odom → base_footprint` transform was caused by:
1. EKF integrating IMU angular velocity without absolute orientation reference
2. Gyroscope drift accumulating over time
3. Madgwick filter not applying bias correction
4. Insufficient rejection thresholds in EKF

## Applied Fixes

### 1. EKF Configuration Changes (`config/ekf.yaml`)

**Changed IMU fusion strategy:**
- **Before:** Fused only angular velocity (vyaw) from IMU, letting EKF estimate orientation
- **After:** Fuse absolute yaw orientation from Madgwick filter
- **Reason:** Prevents drift accumulation by using sensor-fused absolute orientation

```yaml
# OLD (caused drift)
imu0_config: [false, false, false,  
              false, false, false,  # no orientation
              false, false, false,  
              false, false, true,   # only vyaw (accumulates drift)
              true,  true,  false]

# NEW (fixes drift)
imu0_config: [false, false, false,  
              false, false, true,   # fuse yaw orientation
              false, false, false,  
              false, false, false,  # no vyaw when using orientation
              true,  true,  false]
```

**Increased rejection thresholds:**
- `imu0_pose_rejection_threshold`: 0.8 → **5.0**
- `imu0_twist_rejection_threshold`: 0.8 → **2.0**
- `imu0_linear_acceleration_rejection_threshold`: 0.8 → **2.0**

**Reduced yaw process noise:**
- Yaw process noise: 0.06 → **0.01**
- Trusts sensor fusion more, reduces random walk

### 2. Madgwick Filter Changes (`launch/imu_nav.launch`)

**Enabled gyroscope bias correction:**
- `madgwick_zeta`: 0.0 → **0.01**
- Allows filter to compensate for gyro drift in real-time
- Automatically corrects for temperature-dependent bias

### 3. Rosserial Optimization (`launch/bringup.launch`)

**Added TCP parameters for ESP32:**
```xml
<param name="tcp_nodelay" value="1" />
<param name="baud" value="115200" />
```
- Reduces latency in wheel odometry messages
- Improves time synchronization

## Verification Steps

After restarting the system, verify the fix:

### 1. Check Stationary Drift
```bash
# Robot must be completely still
rosrun tf tf_echo odom base_footprint
```
**Expected result:** Rotation should stay at 0.000 (±0.01°) when stationary

### 2. Monitor EKF Diagnostics
```bash
rostopic echo /diagnostics | grep ekf
```
**Expected:** No constant rejection messages

### 3. Verify Map Quality
- Launch mapping: `roslaunch elderly_bot mapping.launch`
- Keep robot stationary for 30 seconds
- **Expected:** Map should not rotate or drift, walls should be sharp single lines

### 4. Test Autonomous Movement
```bash
# Move robot slowly forward
rostopic pub /cmd_vel geometry_msgs/Twist "linear: {x: 0.1, y: 0.0, z: 0.0} angular: {x: 0.0, y: 0.0, z: 0.0}"
```
**Expected:** Map should build consistently without wall duplication

## Technical Explanation

### Why This Fixes The Problem

**Before:**
1. IMU gyroscope has small bias (e.g., +0.002 rad/s drift)
2. EKF integrates this continuously: θ = θ + ω×Δt
3. Over 30 seconds: drift = 0.002 × 30 = 0.06 rad = 3.4°
4. No absolute reference to correct this
5. Result: Continuous rotation even when stationary

**After:**
1. Madgwick filter fuses gyro + accel + mag → absolute orientation
2. `madgwick_zeta=0.01` estimates and removes gyro bias
3. EKF fuses this absolute yaw orientation (not velocity)
4. Any drift is immediately corrected by absolute reference
5. Result: Stable orientation when stationary

### REP-105 Compliance

All transforms follow REP-105 standard:
- `map` → `odom`: Published by gmapping (SLAM correction)
- `odom` → `base_footprint`: Published by EKF (fused odometry)
- `base_footprint` → `base_link`: Static (robot geometry)
- `base_link` → `laser`: Static, 180° rotation (motor backward)
- `base_link` → `imu_link`: Static (sensor mounting)

### TF Rates (All Synchronized)
- Static transforms: 100 Hz (via static_transform_publisher)
- EKF output: 50 Hz
- Lidar scans: ~6.5 Hz
- IMU: 100 Hz → Madgwick → 100 Hz
- Wheel odometry: 10 Hz

## Rollback Instructions

If issues arise, revert changes:

```bash
cd ~/catkin_ws/src/elderly_bot
git diff config/ekf.yaml
git diff launch/bringup.launch
git diff launch/imu_nav.launch

# Revert if needed
git checkout config/ekf.yaml launch/bringup.launch launch/imu_nav.launch
```

## Additional Recommendations

### If Drift Persists:

1. **Recalibrate IMU gyroscope:**
   - Keep robot perfectly still during startup
   - `mpu9250_node.py` will auto-calibrate gyro offsets
   - Check startup logs for calibration values

2. **Verify magnetometer calibration:**
   - Hard-iron bias already configured in `imu_nav.launch`
   - If building has strong magnetic disturbances, disable mag:
     ```bash
     # In imu_nav.launch, set use_mag="false"
     ```

3. **Check wheel encoder accuracy:**
   - Verify `HARDWARE_MAP.md` wheel diameter (38.8mm) is accurate
   - Measure actual distance traveled vs odometry
   - Adjust if needed

4. **Increase Madgwick filter gain (if turns are sluggish):**
   - `madgwick_gain`: 0.1 → 0.2 (trust gyro more during motion)
   - Keep `madgwick_zeta=0.01` for drift correction

## Expected Improvements

✅ Zero rotation drift when stationary  
✅ Sharp, single-line walls in map  
✅ No map duplication/ghosting  
✅ Consistent odometry during slow movements  
✅ Reliable autonomous navigation  
✅ Stable 100% autonomous mapping capability

## Status
- [x] Fixes applied
- [ ] System restarted
- [ ] Stationary drift verified
- [ ] Mapping tested
- [ ] Autonomous movement validated
