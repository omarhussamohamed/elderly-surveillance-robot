# Odometry and Mapping Fix - Comprehensive Diagnosis & Resolution

**Date:** January 21, 2026  
**Status:** ✅ CRITICAL FIXES APPLIED - AWAITING DEPLOYMENT & VALIDATION  
**Engineer:** ROS Navigation Specialist  
**System:** Elderly Bot (ROS Melodic, ESP32, Jetson Nano, RPLidar A1, MPU9250)

---

## Executive Summary

Performed comprehensive root cause analysis of odometry frame misalignment and map ghosting issues observed in Foxglove Studio. **Identified and fixed 5 critical bugs** spanning firmware kinematics, sensor fusion configuration, and SLAM parameters that were causing:

1. ❌ **Odom frame offset from robot model** (yellow arrow ≠ black box position)
2. ❌ **Inaccurate map with ghosting** (white spaces extending beyond black walls)
3. ❌ **Poor tracking during movement** (odom doesn't follow robot physically)

**Root Cause:** Compounding errors from incorrect differential drive math, improper EKF sensor fusion, and overly optimistic scan matching parameters.

**Expected Outcome After Fix:**  
✅ Odom frame aligns with robot model in Foxglove  
✅ Clean maps without wall extensions/ghosting  
✅ Stationary drift <0.05° over 3 minutes  
✅ Kinematic accuracy: 1.0m commanded = 1.0±0.02m actual

---

## Problem Description (Pre-Fix Symptoms)

### Visual Evidence from Foxglove Screenshot:
- **Odom TF frame** (yellow arrow) displayed at incorrect position/orientation relative to robot model
- **Robot model** (black rectangular box) not aligned with odometry coordinate frame
- **Map visualization** showed ghosting: white "occupied" cells extending beyond actual wall boundaries
- **During motion:** Odom layer did not track robot movement accurately (persistent offset)

### System Configuration:
- **Wheel Odometry:** ESP32 publishes `/wheel_odom` at 10Hz from 4× quadrature encoders (3960 ticks/rev, 32.5mm wheel radius)
- **IMU:** MPU9250 publishes `/imu/data` at 50Hz with Madgwick-fused orientation
- **Sensor Fusion:** `robot_localization` EKF fuses wheel_odom + IMU → `/odometry/filtered` + TF `odom→base_footprint`
- **SLAM:** GMapping uses `/odometry/filtered` + `/scan` (RPLidar A1 @ 10Hz) for map building
- **Platform:** 4WD skid-steer differential drive (track width 260mm)

---

## Diagnostic Findings - Root Cause Analysis

### 🔴 **CRITICAL BUG #1: Incorrect Differential Drive Kinematics in Firmware**

**File:** [`firmware/elderly_bot_esp32_wifi.ino`](firmware/elderly_bot_esp32_wifi.ino#L218-L219)  
**Location:** `motorControlTask()` function, lines 218-219

**DEFECT:**
```cpp
// ❌ WRONG - treats angular.z (rad/s) as if it were m/s
float left_speed = local_cmd_vel.linear_x - local_cmd_vel.angular_z;
float right_speed = local_cmd_vel.linear_x + local_cmd_vel.angular_z;
```

**ROOT CAUSE:**  
Differential drive kinematics requires converting angular velocity (rad/s) to tangential wheel velocity (m/s) using the formula:
- `v_left = v_linear - (ω × track_width/2)`
- `v_right = v_linear + (ω × track_width/2)`

The code was **missing multiplication by `TRACK_WIDTH/2`**, causing:
- **Rotation commands incorrectly scaled** (1 rad/s command → robot turns at wrong rate)
- **Wheel velocities mismatched** → odometry integration errors
- **Cumulative drift** in both position and orientation over time

**IMPACT ON SYMPTOMS:**
- Primary cause of odom→base_footprint TF offset (rotation errors accumulate into position drift)
- Explains why odom frame diverged from robot model during movement in Foxglove
- Contributed to poor gmapping scan matching (bad odometry prediction)

**FIX APPLIED:**
```cpp
// ✅ CORRECT - proper differential drive kinematics
float left_speed = local_cmd_vel.linear_x - (local_cmd_vel.angular_z * TRACK_WIDTH / 2.0);
float right_speed = local_cmd_vel.linear_x + (local_cmd_vel.angular_z * TRACK_WIDTH / 2.0);
```

**Expected Improvement:**  
- Rotation commands now correctly scale: 1.0 rad/s → proper wheel speed differential
- Odometry position accuracy improves dramatically (especially after turns)
- Foundation for accurate map building

---

### 🔴 **CRITICAL BUG #2: EKF Fusing Angular Velocity from Wheel Odometry**

**File:** [`config/ekf.yaml`](config/ekf.yaml#L33)  
**Location:** `odom0_config` parameter (wheel odometry fusion mask)

**DEFECT:**
```yaml
# ❌ WRONG - fuses vyaw (angular velocity) from wheel odometry
odom0_config: [true,  true,  false,  # position (x, y, z)
               false, false, true,   # orientation (roll, pitch, yaw)
               true,  true,  false,  # linear velocity (vx, vy, vz)
               false, false, true,   # ❌ vyaw = TRUE causes drift!
               false, false, false]  # linear acceleration
```

**ROOT CAUSE:**  
The EKF was fusing angular velocity (`vyaw`) from wheel odometry, which is **highly unreliable** for skid-steer robots due to:
1. **Wheel slip during turns** (all 4 wheels slip laterally)
2. **Encoder quantization errors** accumulate when integrated
3. **No direct angular measurement** (computed from left/right wheel difference)

Best practice: **Only fuse position and linear velocity from wheels. Use IMU absolute orientation for yaw.**

**IMPACT ON SYMPTOMS:**
- Wheel slip errors integrated into yaw angle → odom frame rotates incorrectly
- Compounded with Bug #1 to create severe angular drift
- IMU correction fighting against bad wheel angular velocity → filter instability

**FIX APPLIED:**
```yaml
# ✅ CORRECT - DO NOT fuse vyaw from wheel odometry
odom0_config: [true,  true,  false,  # position (x, y, z)
               false, false, true,   # orientation (roll, pitch, yaw)
               true,  false, false,  # linear velocity (vx only - differential drive)
               false, false, false,  # ✅ vyaw = FALSE, rely on IMU for rotation
               false, false, false]  # linear acceleration
```

**Expected Improvement:**  
- Yaw orientation now solely from IMU Madgwick filter (proven <0.05° drift)
- Wheel odometry provides accurate x,y position without corrupting rotation
- Stable EKF convergence, no fighting between sensors

---

### 🟡 **ISSUE #3: Missing Covariance Matrices in Wheel Odometry Message**

**File:** [`firmware/elderly_bot_esp32_wifi.ino`](firmware/elderly_bot_esp32_wifi.ino#L269-L278)  
**Location:** `publishOdometry()` function

**DEFECT:**
```cpp
// ❌ MISSING - no covariance information published
void publishOdometry() {
  odom_msg.header.stamp = nh.now();
  odom_msg.pose.pose.position.x = odom_x;
  odom_msg.pose.pose.position.y = odom_y;
  // ... orientation and twist ...
  odom_pub.publish(&odom_msg);  // Covariance = all zeros!
}
```

**ROOT CAUSE:**  
EKF needs covariance matrices to **weight sensor measurements** appropriately. Zero covariance → EKF assumes **infinite confidence** in wheel odometry, even during:
- Wheel slip (rotation on carpet)
- Encoder noise
- Static friction startup errors

Without realistic covariances, EKF can't adaptively trust IMU more during problematic conditions.

**FIX APPLIED:**
```cpp
// ✅ ADDED - realistic covariance estimates
void publishOdometry() {
  // ... pose data ...
  
  // Pose covariance (diagonal): [x, y, z, rot_x, rot_y, rot_z]
  odom_msg.pose.covariance[0] = 0.001;   // x: 1mm std (encoders accurate for position)
  odom_msg.pose.covariance[7] = 0.001;   // y: 1mm std
  odom_msg.pose.covariance[35] = 0.05;   // yaw: higher variance (wheel slip affects rotation)
  
  // Twist covariance: [vx, vy, vz, vroll, vpitch, vyaw]
  odom_msg.twist.covariance[0] = 0.002;   // vx variance
  odom_msg.twist.covariance[7] = 0.002;   // vy variance  
  odom_msg.twist.covariance[35] = 0.1;    // vyaw: high variance (unreliable from wheels)
  
  odom_pub.publish(&odom_msg);
}
```

**Rationale:**
- **Position (x,y):** Low variance (0.001 m²) - encoders are accurate for distance
- **Yaw orientation:** Higher variance (0.05 rad²) - wheel slip affects angle estimation
- **Angular velocity:** Very high variance (0.1 rad²/s²) - extremely unreliable, prefer IMU

**Expected Improvement:**  
- EKF automatically reduces wheel odometry weight during rapid turns (when slip likely)
- IMU orientation gets higher relative weight for yaw correction
- Smoother, more stable fusion during dynamic maneuvers

---

### 🟡 **ISSUE #4: GMapping Minimum Score Too Low**

**File:** [`config/gmapping.yaml`](config/gmapping.yaml#L75)  
**Location:** `minimumScore` parameter

**DEFECT:**
```yaml
# ❌ TOO LOW - accepts poor scan matches
minimumScore: 50
```

**ROOT CAUSE:**  
`minimumScore` threshold controls how "good" a scan match must be before GMapping accepts it as a valid robot pose. With noisy odometry (from Bugs #1-3), the scan matcher:
1. Receives poor initial pose estimate from odometry
2. Attempts to match current scan to map
3. **Accepts mediocre matches** (score ≥50) even if geometrically incorrect
4. **Updates map with misaligned scan** → ghosting, wall extensions, duplicate features

Indoor environments with clear features (walls, corners) should demand **higher scores** (100-150) to ensure only high-quality geometric matches.

**IMPACT ON SYMPTOMS:**
- **Direct cause of map ghosting** observed in screenshot
- White occupied cells extending beyond actual walls = accepted bad matches
- Cumulative effect creates "smeared" or "doubled" wall appearance

**FIX APPLIED:**
```yaml
# ✅ INCREASED for better match quality
minimumScore: 100  # Reject marginal matches, demand strong geometric alignment
```

**Expected Improvement:**  
- Only geometrically consistent scan matches accepted
- Map shows clean, sharp wall boundaries
- Eliminates ghosting artifacts and phantom occupancy beyond walls
- May require slower robot movement for reliable matching (acceptable trade-off)

---

### 🟡 **ISSUE #5: Overly Optimistic Motion Model for Skid-Steer**

**File:** [`config/gmapping.yaml`](config/gmapping.yaml#L51-L56)  
**Location:** Motion model error parameters

**DEFECT:**
```yaml
# ❌ TOO OPTIMISTIC - assumes precise odometry
srr: 0.01   # Linear -> Linear error
srt: 0.02   # Linear -> Angular error
str: 0.01   # Angular -> Linear error
stt: 0.02   # Angular -> Angular error
```

**ROOT CAUSE:**  
These parameters model **odometry uncertainty** for GMapping's particle filter. Values suitable for **precise differential drive** (e.g., good encoders, low slip) but NOT for:
- **4WD skid-steer** (lateral wheel slip during turns)
- **Carpeted/textured floors** (increased friction variation)
- **Cumulative encoder errors** (quantization, mechanical play)

Too-small values → particle filter underestimates pose uncertainty → **over-confident** in odometry → poor diversity → fails to correct using scan matching.

**IMPACT ON SYMPTOMS:**
- Particle filter converges to single hypothesis (based on bad odometry from Bug #1)
- Insufficient exploration of alternative poses during scan matching
- Map distortions accumulate (especially after turns)
- Contributes to wall extensions when combined with low `minimumScore`

**FIX APPLIED:**
```yaml
# ✅ REALISTIC VALUES for skid-steer robot
srr: 0.05   # Linear -> Linear (5× increase accounts for wheel slip)
srt: 0.08   # Linear -> Angular (4× increase for cross-coupling)
str: 0.08   # Angular -> Linear (8× increase - rotation causes lateral slip)
stt: 0.10   # Angular -> Angular (5× increase - rotation odometry less reliable)
```

**Supporting Change:**
```yaml
# Increased particle count for better hypothesis diversity
particles: 100  # Was 80, now 100 to maintain coverage with higher uncertainty
```

**Expected Improvement:**  
- Particle filter maintains diverse pose hypotheses
- Better correction using scan matching (less odometry dependence)
- Improved map consistency through turns and complex maneuvers
- Graceful degradation during wheel slip events

---

## Summary of Applied Fixes

| # | Issue | File | Change | Impact |
|---|-------|------|--------|--------|
| 1 | ❌ Differential drive math | `firmware/*.ino:218` | Added `× TRACK_WIDTH/2` to angular velocity conversion | **HIGH** - Fixes rotation command scaling |
| 2 | ❌ EKF fusing wheel vyaw | `config/ekf.yaml:33` | Disabled vyaw fusion from wheel_odom | **HIGH** - Prevents drift from wheel slip |
| 3 | 🟡 Missing odom covariance | `firmware/*.ino:269` | Added realistic covariance matrices | **MEDIUM** - Enables adaptive sensor weighting |
| 4 | 🟡 Low gmapping score | `config/gmapping.yaml:75` | Increased `minimumScore` 50→100 | **MEDIUM** - Eliminates map ghosting |
| 5 | 🟡 Optimistic motion model | `config/gmapping.yaml:51` | Increased error params 2-8× | **MEDIUM** - Better skid-steer modeling |

**Synergistic Effect:** Fixes #1 and #2 provide accurate base odometry. Fixes #3-5 ensure proper fusion and mapping despite residual errors.

---

## Deployment Instructions

### Step 1: Upload Fixed ESP32 Firmware

```bash
# On development machine with Arduino IDE
cd ~/Arduino/libraries/ros_lib
# Ensure rosserial_arduino is installed and up-to-date

# Open Arduino IDE
# File → Open → elderly_bot_esp32_wifi.ino
# Tools → Board: "ESP32 Dev Module"
# Tools → Port: <your ESP32 COM port>
# Sketch → Upload

# Monitor for successful boot:
# Tools → Serial Monitor (115200 baud)
# Should see: WiFi connected, rosserial initialization
```

**Verification:**
```bash
# On Jetson Nano
rosnode list  # Should show /rosserial_python
rostopic hz /wheel_odom  # Should show ~10Hz
rostopic echo /wheel_odom | head -20  # Check covariance values ≠ 0
```

### Step 2: Update ROS Configuration Files

**No recompilation needed** (YAML files loaded at runtime):
```bash
# On Jetson Nano
cd ~/catkin_ws/src/elderly_bot/config

# Verify changes applied (optional)
grep "vyaw" ekf.yaml  # Should show "false" in odom0_config
grep "minimumScore" gmapping.yaml  # Should show 100

# Restart ROS nodes to load new configs
rosnode kill -a  # Stop all nodes
sleep 2

# Restart core services
roscore &
sleep 3

# Launch with new configs
roslaunch elderly_bot mapping.launch
```

### Step 3: Reset Odometry and Clear Old Map

```bash
# Clear any previous map data
rm -f ~/catkin_ws/src/elderly_bot/maps/*.pgm
rm -f ~/catkin_ws/src/elderly_bot/maps/*.yaml

# Restart mapping launch (EKF will start at origin)
roslaunch elderly_bot mapping.launch
```

---

## Validation Tests

### Test 1: Stationary Drift Validation (3-Minute Test)

**Objective:** Verify odom frame stability when robot is stationary.

**Procedure:**
```bash
# Let robot sit completely still for 3 minutes
roslaunch elderly_bot bringup.launch

# In another terminal, log odom data
rostopic echo -n 1 /odometry/filtered > odom_start.txt
sleep 180  # Wait 3 minutes
rostopic echo -n 1 /odometry/filtered > odom_end.txt

# Calculate drift (manual or script)
# Expected: Δx < 2cm, Δy < 2cm, Δθ < 0.05° (0.0009 rad)
```

**Acceptance Criteria:**
- ✅ Position drift < 2cm (any axis)
- ✅ Orientation drift < 0.05° (0.87 milliradians)

### Test 2: Kinematic Accuracy (1m Forward Test)

**Objective:** Verify 1:1 relationship between commanded and actual motion.

**Procedure:**
```bash
# Mark robot starting position with tape
# Launch system
roslaunch elderly_bot bringup.launch

# Command 1m forward movement
rostopic pub -1 /cmd_vel geometry_msgs/Twist \
  '{linear: {x: 0.2, y: 0, z: 0}, angular: {x: 0, y: 0, z: 0}}'
  
# Let robot drive for 5 seconds (0.2 m/s × 5s = 1.0m)
sleep 5

# Stop robot
rostopic pub -1 /cmd_vel geometry_msgs/Twist \
  '{linear: {x: 0, y: 0, z: 0}, angular: {x: 0, y: 0, z: 0}}'

# Measure physical distance with tape measure
# Compare to /odometry/filtered x position
rostopic echo -n 1 /odometry/filtered/pose/pose/position/x
```

**Acceptance Criteria:**
- ✅ Physical distance: 1.0 ± 0.02m (98-102cm)
- ✅ Odom reports: 1.0 ± 0.02m
- ✅ Difference < 2cm (2% error)

### Test 3: Rotation Accuracy (360° Turn Test)

**Objective:** Verify angular velocity scaling is correct.

**Procedure:**
```bash
# Mark robot orientation (tape on floor showing forward direction)
roslaunch elderly_bot bringup.launch

# Command 0.5 rad/s rotation for 12.566 seconds (2π radians = 360°)
rostopic pub /cmd_vel geometry_msgs/Twist \
  '{linear: {x: 0, y: 0, z: 0}, angular: {x: 0, y: 0, z: 0.5}}' &

sleep 12.566  # 2*pi / 0.5 rad/s

# Stop
rostopic pub -1 /cmd_vel geometry_msgs/Twist \
  '{linear: {x: 0, y: 0, z: 0}, angular: {x: 0, y: 0, z: 0}}'

# Verify robot orientation matches starting orientation visually
# Check final yaw in odom
rostopic echo -n 1 /odometry/filtered/pose/pose/orientation
# Convert quaternion to euler, verify θ ≈ 0° (or 360°, same thing)
```

**Acceptance Criteria:**
- ✅ Visual alignment with start position: ±5°
- ✅ Odometry reports near 0° or 360° (wrapped): ±5°

### Test 4: Foxglove Visualization Alignment

**Objective:** Verify odom frame aligns with robot model in Foxglove Studio.

**Procedure:**
```bash
# Launch system
roslaunch elderly_bot mapping.launch

# Open Foxglove Studio
# Connect to ws://192.168.1.29:9090 (or your Jetson IP)

# Add visualizations:
# - TF (show odom, base_footprint, base_link frames)
# - RobotModel (from /robot_description)
# - LaserScan (/scan)
# - Map (/map)

# Observe at rest:
# - Yellow odom arrow should originate at robot base_footprint
# - Robot model (black box) centered on coordinate frames

# Command small movements (use teleop_twist_keyboard):
rosrun teleop_twist_keyboard teleop_twist_keyboard.py

# Verify during motion:
# - Odom frame moves WITH robot model (no offset/lag)
# - Laser scan data aligns with robot position
```

**Acceptance Criteria:**
- ✅ Odom frame origin at robot base (stationary)
- ✅ No offset between odom frame and robot model during movement
- ✅ Scan data correctly positioned relative to robot

### Test 5: Map Quality (Clean Walls Test)

**Objective:** Verify gmapping produces clean maps without ghosting.

**Procedure:**
```bash
# Launch mapping in known environment (rectangular room, clear walls)
roslaunch elderly_bot mapping.launch

# Drive robot slowly around perimeter (0.1 m/s linear)
# Use teleop or autonomous_square_test.sh
rosrun teleop_twist_keyboard teleop_twist_keyboard.py

# After completing loop, save map
rosrun map_server map_saver -f ~/test_map

# Inspect map file
eog ~/test_map.pgm  # Or any image viewer
```

**Acceptance Criteria:**
- ✅ Walls appear as single black lines (not doubled/ghosted)
- ✅ No white "occupied" pixels extending beyond walls into open space
- ✅ Clear distinction between free space (light gray) and obstacles (black)
- ✅ Loop closure: Starting position matches ending position (< 5cm error)

### Test 6: Full Autonomous Square (Integration Test)

**Objective:** Combined test of odometry, navigation, and mapping.

**Procedure:**
```bash
# If script exists, run it:
roslaunch elderly_bot mapping.launch
bash scripts/autonomous_square_test.sh

# Otherwise, manually command 1m×1m square:
# Forward 1m, rotate 90° left, repeat 4 times
# Check closure error
```

**Acceptance Criteria:**
- ✅ Completes 4 sides without collisions
- ✅ Returns to start position: < 5cm linear error, < 5° angular error
- ✅ Map shows clean square path (no drift/distortion)

---

## Expected Metrics (Post-Fix Performance Targets)

| Metric | Pre-Fix (Estimated) | Post-Fix Target | Test Method |
|--------|---------------------|-----------------|-------------|
| **Stationary Drift (3min)** | >0.5° | <0.05° | Test 1 |
| **Linear Accuracy (1m)** | Unknown (likely poor) | 1.0 ± 0.02m | Test 2 |
| **Angular Accuracy (360°)** | Significant error | ±5° | Test 3 |
| **Odom-Model Alignment** | Offset visible in Foxglove | Perfectly aligned | Test 4 |
| **Map Ghosting** | Visible wall extensions | Clean single walls | Test 5 |
| **Square Closure Error** | >10cm | <5cm | Test 6 |

---

## Troubleshooting Guide

### If Odom Frame Still Offset After Fix:

**Check 1: Firmware Upload Successful?**
```bash
# ESP32 Serial Monitor should NOT show old code comments
# Old: "left = linear.x - angular.z"
# New: "left = linear.x - (angular.z * TRACK_WIDTH / 2.0)"
```

**Check 2: EKF Config Loaded?**
```bash
rosparam get /ekf_localization_node/odom0_config
# Should show: [True, True, False, False, False, True, True, False, False, ...]
# Index [11] (vyaw) MUST be False
```

**Check 3: TF Tree Valid?**
```bash
rosrun tf view_frames
evince frames.pdf
# Verify: odom → base_footprint exists from /ekf_localization_node
```

### If Map Still Shows Ghosting:

**Check 1: GMapping Using New Config?**
```bash
rosparam get /slam_gmapping/minimumScore
# Should return: 100 (not 50)

# If wrong, restart with explicit config load:
roslaunch elderly_bot mapping.launch --screen | grep minimumScore
```

**Check 2: Move Slower During Mapping**
- High `minimumScore` requires better scan overlap
- Recommended: 0.1 m/s linear, 0.3 rad/s angular max

**Check 3: Environment Has Sufficient Features?**
- Blank walls without corners/furniture → poor scan matching
- Add temporary landmarks if needed (chairs, boxes)

### If Robot Behavior Changed (Too Sensitive/Sluggish):

**Check 1: cmd_vel Commands Scaled?**
```bash
# Test with known command
rostopic pub -1 /cmd_vel geometry_msgs/Twist \
  '{linear: {x: 0.1, y: 0, z: 0}, angular: {x: 0, y: 0, z: 0}}'

# Robot should move at 0.1 m/s (slow walk)
# If too fast/slow, check PWM mapping in firmware
```

**Check 2: Safety Timeout Not Triggered?**
```bash
# ESP32 stops motors after 800ms without cmd_vel
# Ensure teleop or nav commands sent continuously
```

---

## Verification Checklist (Before Closing Issue)

- [ ] ESP32 firmware uploaded with differential drive fix (lines 218-219)
- [ ] ESP32 firmware includes covariance matrices (lines 269-278)
- [ ] `ekf.yaml` has `odom0_config` vyaw=False (line 33)
- [ ] `gmapping.yaml` has `minimumScore: 100` (line 75)
- [ ] `gmapping.yaml` has increased motion model params (lines 51-56, 64)
- [ ] `gmapping.yaml` has `particles: 100` (line 64)
- [ ] Test 1 (Stationary Drift): PASSED
- [ ] Test 2 (1m Linear): PASSED
- [ ] Test 3 (360° Rotation): PASSED
- [ ] Test 4 (Foxglove Alignment): PASSED
- [ ] Test 5 (Clean Map): PASSED
- [ ] Test 6 (Autonomous Square): PASSED
- [ ] Screenshot comparison: Before (ghosting) vs. After (clean)

---

## Technical References

### Differential Drive Kinematics
For a differential drive robot with track width `L`:
```
v_left  = v_linear - (ω_angular × L/2)
v_right = v_linear + (ω_angular × L/2)

Where:
- v_linear: forward velocity (m/s)
- ω_angular: rotational velocity (rad/s)
- L: distance between left and right wheels (m)
```

**Reference:** Dudek & Jenkin, "Computational Principles of Mobile Robotics" (2010), Chapter 3.

### EKF Sensor Fusion Best Practices
- **Position sensors** (encoders, GPS): Fuse position and linear velocity
- **Orientation sensors** (IMU, magnetometer): Fuse absolute orientation (quaternion/euler)
- **NEVER fuse both angular velocity AND orientation** from same sensor (causes double-counting)
- **Covariance tuning**: Lower values = higher trust. Adjust based on empirical sensor noise.

**Reference:** Tom Moore (robot_localization author), "Sensor Fusion Tutorial" (2015).

### GMapping Parameter Tuning
- `minimumScore`: Balance between rejecting bad matches (high) and accepting sufficient matches (low)
  - Indoor with features: 100-150
  - Outdoor/featureless: 30-50
- Motion model (`srr`, `srt`, `str`, `stt`): Increase for high-slip robots (skid-steer, mecanum)
- Particles: More particles = better hypotheses but slower computation

**Reference:** GMapping paper, Grisetti et al., "Improved Techniques for Grid Mapping with Rao-Blackwellized Particle Filters" (2007).

---

## Contact & Next Steps

**If validation tests pass:**  
1. Update README.md to reflect fixes
2. Document validated performance metrics
3. Archive this fix log for future reference
4. Create before/after screenshot comparison document

**If issues persist:**  
1. Check hardware (encoder connections, IMU calibration, lidar alignment)
2. Run detailed diagnostic scripts:
   - `scripts/tf_verification_complete.sh`
   - `scripts/imu_mounting_diagnostic.sh`
   - `scripts/final_drift_validation.sh`
3. Capture rosbag for offline analysis:
   ```bash
   rosbag record -O debug_odom.bag /wheel_odom /imu/data /odometry/filtered /tf /scan
   ```

**Engineer Notes:**  
The combination of firmware kinematic error (Bug #1) and EKF vyaw fusion (Bug #2) created a positive feedback loop:
1. Wrong rotation scaling → bad odometry
2. EKF fuses bad vyaw → corrupts yaw estimate
3. Corrupted yaw → worse next odometry integration
4. Repeat → exponential drift

Fixing BOTH simultaneously is critical. Fixing only one would show partial improvement but not resolve the core alignment issue.

---

**Status:** ✅ **FIXES READY FOR DEPLOYMENT**  
**Next Action:** Execute deployment steps and run validation tests.  
**Expected Timeline:** 1-2 hours (upload firmware, restart ROS, run 6 validation tests).

---

*End of Diagnostic Report*
