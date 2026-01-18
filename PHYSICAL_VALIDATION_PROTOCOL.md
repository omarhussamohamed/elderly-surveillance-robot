# PHYSICAL VALIDATION PROTOCOL
## Comprehensive System Validation After 3960 Ticks/Rev & Drift Fixes

**Date:** January 18, 2026  
**Objective:** Verify kinematic scaling, drift elimination, and SLAM integrity in real-world conditions  
**Prerequisites:** ESP32 firmware uploaded with TICKS_PER_REV=3960, system running `roslaunch elderly_bot bringup.launch`

---

## TEST 1: "GOLDEN METER" - Linear Scaling Validation

### Purpose
Confirm that 1.0m commanded = 1.0m traveled (±2cm)

### Setup
1. Place robot on flat, open floor
2. Mark starting position with tape at robot center
3. Measure and mark 100cm forward with tape
4. Ensure robot has clear path (no obstacles for 1.5m)

### Command
```bash
# SSH to Jetson (192.168.1.29)
timeout 10 rostopic pub -r 10 /cmd_vel geometry_msgs/Twist "linear: {x: 0.1, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}"
```

### Expected Result
- **Duration:** 10 seconds
- **Distance:** 100cm ± 2cm from start tape to robot center
- **Velocity:** Smooth acceleration, constant 0.1 m/s, smooth deceleration

### Pass/Fail Criteria
- ✅ **PASS:** Robot stops between 98-102cm from start
- ❌ **FAIL:** Robot stops at ~6-10cm → Firmware not uploaded correctly
- ❌ **FAIL:** Robot stops at 80-90cm → TICKS_PER_REV too high (wheel slippage or wrong value)
- ❌ **FAIL:** Robot stops at 110-120cm → TICKS_PER_REV too low

### Measured Result
```
Actual Distance: __________ cm
Status: [ ] PASS  [ ] FAIL
Notes: _________________________________________________________________
```

---

## TEST 2: "TRUE NORTH" - Stationary Drift Validation

### Purpose
Verify absolute yaw fusion eliminates drift (max 0.05° over 3 minutes)

### Setup
1. Place robot on flat floor
2. Ensure robot is completely stationary (no vibrations, no touching)
3. Mark robot orientation with tape line

### Command
```bash
# Terminal 1: Start recording
rostopic echo /odometry/filtered | grep -A 3 "orientation:" > drift_test.log &
DRIFT_PID=$!

# Wait 180 seconds (DO NOT TOUCH ROBOT)
sleep 180

# Stop recording
kill $DRIFT_PID

# Analyze yaw drift
python3 << 'EOF'
import math
import re

def quaternion_to_yaw(x, y, z, w):
    return math.atan2(2.0*(w*z + x*y), 1.0 - 2.0*(y*y + z*z))

with open('drift_test.log', 'r') as f:
    data = f.read()

quats = re.findall(r'x: ([-\d.]+)\s+y: ([-\d.]+)\s+z: ([-\d.]+)\s+w: ([-\d.]+)', data)

if len(quats) >= 2:
    yaw_start = math.degrees(quaternion_to_yaw(float(quats[0][0]), float(quats[0][1]), float(quats[0][2]), float(quats[0][3])))
    yaw_end = math.degrees(quaternion_to_yaw(float(quats[-1][0]), float(quats[-1][1]), float(quats[-1][2]), float(quats[-1][3])))
    drift = yaw_end - yaw_start
    print(f"Start Yaw: {yaw_start:.4f}°")
    print(f"End Yaw: {yaw_end:.4f}°")
    print(f"Total Drift: {drift:.4f}°")
    print(f"Status: {'PASS ✅' if abs(drift) <= 0.05 else 'FAIL ❌'}")
else:
    print("ERROR: Insufficient data collected")
EOF
```

### Alternative Quick Check
```bash
# Watch live for 3 minutes
rostopic echo /odometry/filtered/pose/pose/orientation
# Manually record z and w values at start and end
```

### Expected Result
- **Duration:** 180 seconds
- **Yaw Change:** ≤0.05° (essentially zero drift)
- **Visual Check:** Tape line still aligned with robot front

### Pass/Fail Criteria
- ✅ **PASS:** |Drift| ≤ 0.05° (0.00° is ideal)
- ⚠️ **MARGINAL:** 0.05° < |Drift| ≤ 0.20° (acceptable but investigate IMU mounting)
- ❌ **FAIL:** |Drift| > 0.20° → EKF config not loaded or IMU not calibrated

### Measured Result
```
Start Yaw: __________ °
End Yaw: __________ °
Total Drift: __________ °
Status: [ ] PASS  [ ] MARGINAL  [ ] FAIL
Notes: _________________________________________________________________
```

---

## TEST 3: "FULL CIRCLE" - Angular Scaling Validation

### Purpose
Verify 360° commanded = 360° actual rotation (±5°)

### Setup
1. Place robot on flat floor with 1m clearance
2. Mark robot front orientation with tape arrow on floor
3. Stand back to observe full rotation

### Command
```bash
# 360° rotation at 0.25 rad/s
# Duration = 2π / 0.25 = 25.13 seconds
timeout 26 rostopic pub -r 10 /cmd_vel geometry_msgs/Twist "linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.25}"
```

### Expected Result
- **Duration:** ~25 seconds
- **Rotation:** Robot completes full 360° and faces original direction (tape arrow aligned)
- **Path:** Robot stays centered (minimal translation)

### Pass/Fail Criteria
- ✅ **PASS:** Robot faces original direction ±5°
- ⚠️ **MARGINAL:** 5° < Error ≤ 15° → Wheel slippage or track width needs calibration
- ❌ **FAIL:** Error > 15° → WHEEL_TRACK incorrect (measure physical track width)

### Measured Result
```
Final Orientation Error: __________ ° (visual estimate)
Path Drift During Rotation: __________ cm
Status: [ ] PASS  [ ] MARGINAL  [ ] FAIL
Notes: _________________________________________________________________
```

---

## TEST 4: "WALL SNAP" - SLAM Integrity Validation

### Purpose
Verify Gmapping minimumScore=50 prevents map ghosting and ensures scan matching locks to existing features

### Setup
1. Launch mapping: `roslaunch elderly_bot mapping.launch`
2. Open Rviz: `rviz -d ~/catkin_ws/src/elderly_bot/rviz/mapping.rviz`
3. Choose test environment with clear corners (hallway, room with furniture)

### Procedure
```bash
# Drive manually for 2 minutes
# 1. Drive forward 2m
# 2. Turn 90° to face a wall/corner
# 3. Drive forward 1m
# 4. Turn 180° and return along same path
# 5. Observe corner in Rviz - should have single sharp line, not double/blurred
```

### Manual Drive Commands (use these or teleop)
```bash
# Forward 2m
timeout 20 rostopic pub -r 10 /cmd_vel geometry_msgs/Twist "linear: {x: 0.1}"

# Rotate 90° left (π/2 rad at 0.25 rad/s = 6.3s)
timeout 7 rostopic pub -r 10 /cmd_vel geometry_msgs/Twist "angular: {z: 0.25}"

# Forward 1m
timeout 10 rostopic pub -r 10 /cmd_vel geometry_msgs/Twist "linear: {x: 0.1}"

# Rotate 180° (π rad at 0.25 rad/s = 12.6s)
timeout 13 rostopic pub -r 10 /cmd_vel geometry_msgs/Twist "angular: {z: 0.25}"

# Return
timeout 10 rostopic pub -r 10 /cmd_vel geometry_msgs/Twist "linear: {x: 0.1}"
```

### Expected Result
- **Wall Lines:** Single pixel width, sharp, no ghosting
- **Corners:** Sharp 90° angles, no doubled geometry
- **Loop Closure:** When returning to start, scans "snap" to existing map
- **No Jumping:** Robot pose in Rviz moves smoothly, no teleportation

### Pass/Fail Criteria
- ✅ **PASS:** Walls are sharp single lines, corners precise, no ghosting
- ⚠️ **MARGINAL:** Minor blur (<2cm) in high-speed turns only
- ❌ **FAIL:** Significant ghosting, doubled walls → Drift still present or minimumScore not applied

### Observed Result
```
Wall Sharpness: [ ] Sharp  [ ] Slightly Blurred  [ ] Heavily Ghosted
Corner Quality: [ ] 90° Crisp  [ ] Rounded  [ ] Doubled
Loop Closure: [ ] Snap to Existing  [ ] Small Offset (<5cm)  [ ] Large Offset (>10cm)
Status: [ ] PASS  [ ] MARGINAL  [ ] FAIL
Notes: _________________________________________________________________
```

---

## TEST 5: "NETWORK PULSE" - Communication Latency Validation

### Purpose
Verify tcp_nodelay=1 eliminates packet bundling lag (target: 40-50Hz wheel odom, 90-100Hz IMU)

### Command
```bash
# Test 1: Wheel Odometry Frequency
echo "=== WHEEL ODOMETRY RATE ==="
timeout 10 rostopic hz /wheel_odom 2>&1 | grep "average rate"

# Test 2: IMU Data Frequency
echo "=== IMU DATA RATE ==="
timeout 10 rostopic hz /imu/data 2>&1 | grep "average rate"

# Test 3: Filtered Odometry Frequency
echo "=== EKF FILTERED ODOMETRY RATE ==="
timeout 10 rostopic hz /odometry/filtered 2>&1 | grep "average rate"

# Test 4: Latency Check (Robot Must Be Moving)
timeout 5 rostopic pub -r 10 /cmd_vel geometry_msgs/Twist "linear: {x: 0.1}" &
sleep 1
rostopic echo /wheel_odom/header/stamp --noarr -n 5
# Check timestamps are sequential and smooth
```

### Expected Result
- **Wheel Odom:** 40-50 Hz (target: 50Hz from ESP32)
- **IMU Data:** 90-100 Hz (target: 100Hz from MPU9250)
- **Filtered Odom:** 45-55 Hz (EKF rate: 50Hz)
- **No Burst Lag:** Timestamps increment smoothly by ~0.02s, not in bursts

### Pass/Fail Criteria
- ✅ **PASS:** All rates within ±10% of target, timestamps smooth
- ⚠️ **MARGINAL:** Rates 10-20% below target (WiFi congestion, acceptable)
- ❌ **FAIL:** Rates <30Hz or bursty timestamps → tcp_nodelay not applied or network issue

### Measured Result
```
Wheel Odom Rate: __________ Hz
IMU Data Rate: __________ Hz
Filtered Odom Rate: __________ Hz
Timestamp Pattern: [ ] Smooth  [ ] Occasional Bursts  [ ] Heavily Bursty
Status: [ ] PASS  [ ] MARGINAL  [ ] FAIL
Notes: _________________________________________________________________
```

---

## MASTER VALIDATION CHECKLIST

| Test # | Test Name | Status | Distance/Drift | Pass Criteria |
|--------|-----------|--------|----------------|---------------|
| 1 | Golden Meter (Linear) | [ ] PASS [ ] FAIL | _____ cm | 98-102cm |
| 2 | True North (Drift) | [ ] PASS [ ] FAIL | _____ ° | ≤0.05° |
| 3 | Full Circle (Angular) | [ ] PASS [ ] FAIL | _____ ° error | ±5° |
| 4 | Wall Snap (SLAM) | [ ] PASS [ ] FAIL | Visual | Sharp walls |
| 5 | Network Pulse (Latency) | [ ] PASS [ ] FAIL | _____ Hz | 40-50Hz |

### Overall System Status
- [ ] **ALL TESTS PASS** → System is production-ready for navigation
- [ ] **1-2 MARGINAL** → Acceptable, document limitations
- [ ] **ANY FAIL** → Debug required, see troubleshooting below

---

## TROUBLESHOOTING GUIDE

### If Test 1 Fails (Distance Wrong)

**Robot moves ~6cm instead of 1m:**
- Firmware not uploaded to ESP32
- Solution: Verify Arduino IDE upload, check Serial Monitor for "TICKS_PER_REV: 3960"

**Robot moves 80-90cm:**
- TICKS_PER_REV too high OR wheel slippage
- Check floor surface (carpet vs tile)
- Measure actual wheel diameter (should be 65mm)

**Robot moves 110-120cm:**
- TICKS_PER_REV too low
- Verify encoder wiring (all 4 channels connected)

### If Test 2 Fails (Drift Present)

**Drift > 0.20°:**
- EKF config not loaded
- Solution: `rosparam get /ekf_localization/imu0_config` → should show `[false, false, false, false, false, false, false, false, false, true, true, true, false, false, false]`
- Madgwick zeta not applied
- Solution: `rosparam get /imu_filter_node/zeta` → should return `0.01`

### If Test 3 Fails (Rotation Wrong)

**Under-rotation (<355°):**
- WHEEL_TRACK too large
- Measure physical track width (should be 0.26m)

**Over-rotation (>365°):**
- WHEEL_TRACK too small
- Check for wheel slippage on smooth floor

### If Test 4 Fails (Map Ghosting)

**Walls doubled/blurred:**
- Drift fix not working (recheck Test 2)
- minimumScore not applied
- Solution: `rosparam get /slam_gmapping/minimumScore` → should return `50`

### If Test 5 Fails (Low Rate)

**Wheel odom <30Hz:**
- WiFi congestion or rosserial disconnected
- Solution: Check `rostopic list | grep wheel_odom`, verify ESP32 connected

**tcp_nodelay not working:**
- Launch file not reloaded
- Solution: `rosnode kill -a`, restart bringup.launch

---

## POST-VALIDATION ACTIONS

### If ALL TESTS PASS:
1. Save Rviz map: `rosrun map_server map_saver -f ~/maps/validated_map`
2. Document results in system log
3. Proceed to autonomous navigation testing
4. Run `boss_level_test.sh` for automated validation suite

### If MARGINAL/FAIL:
1. Record all measurements in this document
2. Check firmware upload status: `grep "TICKS_PER_REV" elderly_bot_esp32_wifi.ino`
3. Verify launch files loaded: `rosparam dump /tmp/params.yaml`
4. Consult DRIFT_FIX_APPLIED.md and KINEMATIC_FIX_APPLIED.md
5. Re-upload firmware and restart ROS system

---

## VALIDATION SIGN-OFF

**Tester Name:** _______________________________  
**Date:** _______________________________  
**System Version:** TICKS_PER_REV=3960, EKF Yaw Fusion, Madgwick Bias Correction  
**Result:** [ ] PRODUCTION READY  [ ] NEEDS ADJUSTMENT  

**Notes:**
___________________________________________________________________________
___________________________________________________________________________
___________________________________________________________________________
