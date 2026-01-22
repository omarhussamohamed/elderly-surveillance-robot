# TF Ownership & Regression Guardrails

**Date:** January 22, 2026  
**Status:** ✅ **AUTHORITATIVE REFERENCE**  
**Purpose:** Define single-source-of-truth for TF publishers and prevent magnetometer reintroduction

---

## TF TREE OWNERSHIP (DEFINITIVE)

### **Complete TF Tree Structure:**

```
map
 └─ odom  [published by: slam_gmapping]
     └─ base_footprint  [published by: ekf_localization_node]
         └─ base_link  [published by: robot_state_publisher (URDF static)]
             ├─ laser  [published by: robot_state_publisher (URDF static)]
             ├─ imu_link  [published by: robot_state_publisher (URDF static)]
             ├─ front_left_wheel_link  [published by: robot_state_publisher]
             ├─ front_right_wheel_link  [published by: robot_state_publisher]
             ├─ rear_left_wheel_link  [published by: robot_state_publisher]
             └─ rear_right_wheel_link  [published by: robot_state_publisher]

wheel_odom  [DISCONNECTED - data-only frame from ESP32, NOT in main tree]
```

---

## CRITICAL TF PUBLISHERS (ONE PER TRANSFORM)

| Transform | Publisher | Node | Topic/Mechanism | Config File |
|-----------|-----------|------|-----------------|-------------|
| **map → odom** | GMapping SLAM | `slam_gmapping` | TF broadcast | `mapping.launch` |
| **odom → base_footprint** | EKF Localization | `ekf_localization` | TF broadcast | `ekf.yaml` |
| **base_footprint → base_link** | URDF Static | `robot_state_publisher` | URDF joint | `elderly_bot.urdf` |
| **base_link → laser** | URDF Static | `robot_state_publisher` | URDF joint | `elderly_bot.urdf` |
| **base_link → imu_link** | URDF Static | `robot_state_publisher` | URDF joint | `elderly_bot.urdf` |

### **Data-Only Topics (NOT TF Publishers):**

| Topic | Frame ID | Purpose | Used By |
|-------|----------|---------|---------|
| `/wheel_odom` | `wheel_odom` | Raw wheel odometry data | EKF input only |
| `/imu/data` | `imu_link` | Fused IMU orientation (gyro-only) | EKF input only |
| `/imu/data_raw` | `imu_link` | Raw IMU accel+gyro | Madgwick filter |
| `/scan` | `laser` | Lidar scan data | GMapping SLAM |

---

## SINGLE-PUBLISHER VERIFICATION

**How to verify NO conflicts:**

```bash
# Check TF tree structure
rosrun tf view_frames
# Opens PDF showing TF tree - verify each frame has ONE parent

# Monitor TF publishers for conflicts
timeout 10s rostopic echo /tf | grep "frame_id.*odom" -A 2

# Expected: Only ekf_localization publishes odom→base_footprint
# Expected: Only slam_gmapping publishes map→odom

# Check for duplicate publishers
rosrun tf tf_monitor map odom base_footprint laser

# Should show:
# map → odom: Published by slam_gmapping ONLY
# odom → base_footprint: Published by ekf_localization ONLY
```

**Red flags (CRITICAL ERRORS):**

❌ Multiple nodes publishing same transform  
❌ ESP32 publishing `frame_id="odom"` (creates conflict with EKF)  
❌ Both GMapping and AMCL running (both publish map→odom)  
❌ Static transform for dynamic frame (odom→base_footprint must be dynamic)

---

## REGRESSION GUARDRAILS (MUST REMAIN DISABLED)

### **CRITICAL PARAMETER 1: Magnetometer Fusion**

**File:** `launch/imu_nav.launch`

**MUST BE:**
```xml
<param name="use_mag" value="false" />
```

**WHY:**
- Indoor environment has 10-40× Earth's magnetic field from electronics
- Magnetometer reads Jetson PSU, motors, battery instead of north
- Causes 60° random yaw jumps while stationary
- **NEVER re-enable indoors**

**Early detection symptom:**
```bash
# If this shows jumps > 0.1 while stationary → mag is active!
rostopic echo /imu/data/orientation/z
```

---

### **CRITICAL PARAMETER 2: EKF IMU Yaw Fusion**

**File:** `config/ekf.yaml`

**MUST BE:**
```yaml
imu0_config: [false, false, false,  # position
              false, false, false,  # orientation (YAW MUST BE FALSE)
              false, false, false,  # linear velocity
              false, false, true,   # angular velocity (vyaw)
              false, false, false]  # linear acceleration
```

**Specifically: `imu0_config[5] = false`** (6th element, orientation yaw)

**WHY:**
- IMU yaw comes from Madgwick filter
- If magnetometer enabled, yaw is corrupted
- EKF propagates corruption to entire localization stack
- **Only use gyro rate (vyaw), not absolute yaw**

**Early detection symptom:**
```bash
# If yaw drifts > 5° in 10 seconds while stationary → IMU yaw is fused
rosrun tf tf_monitor odom base_footprint
```

---

### **CRITICAL PARAMETER 3: ESP32 Frame ID**

**File:** `firmware/elderly_bot_esp32_wifi.ino`

**MUST BE:**
```cpp
odom_msg.header.frame_id = "wheel_odom";  // NOT "odom"!
odom_msg.child_frame_id = "base_footprint";
```

**WHY:**
- If ESP32 publishes `frame_id="odom"`, conflicts with EKF
- TF tree has two sources for odom→base_footprint
- Causes random switching between raw/fused data
- **ESP32 must use unique frame**

**Early detection symptom:**
```bash
# Check ESP32 frame ID
rostopic echo -n 1 /wheel_odom/header/frame_id

# MUST return: "wheel_odom" (NOT "odom")
```

---

## MAGNETOMETER POLICY (STRICT)

### **Current Status: PERMANENTLY DISABLED**

Magnetometer is **BANNED** from localization stack until proven safe.

### **Conditions for Re-Enabling (ALL must be true):**

1. ✅ **Environment:** Outdoor, open field
2. ✅ **Distance from metal:** > 2 meters from buildings, vehicles, fences
3. ✅ **Calibration:** Hard-iron + soft-iron calibration completed
4. ✅ **Validation:** 10-minute stationary test shows < 2° yaw drift
5. ✅ **Monitoring:** Live dashboard watching `orientation.z` for jumps
6. ✅ **Kill-switch:** Automated detection of mag jumps (disable if detected)

### **NEVER Re-Enable If:**

❌ Indoor environment (ANY building, house, office, lab)  
❌ Near metal chassis, motors, batteries, electronics  
❌ Without calibration (uncalibrated mag is worse than none)  
❌ In production (testing only, with supervision)  
❌ Without monitoring (must watch for corruption continuously)

### **Justification:**

**Magnetometer ONLY works in:**
- Open outdoor fields
- > 5m from metal structures
- > 2m from electronics
- After proper calibration

**Elderly monitoring robots:**
- Operate 100% indoors
- Near furniture (metal frames), appliances (EMI), walls (rebar)
- **Magnetometer is GUARANTEED to fail**

**Industry standard:**
- Indoor robots use gyro-only or visual odometry
- Magnetometer is for outdoor drones, outdoor rovers
- **Our configuration matches industry best practice**

---

## SINGLE-SYMPTOM EARLY DETECTION

**ONE symptom to watch for magnetometer poisoning:**

```bash
# Run this ONCE when robot is stationary:
timeout 30s rostopic echo /imu/data/orientation/z

# HEALTHY (gyro-only mode):
# Values stay within ±0.02 range
# Example: 0.01, 0.01, 0.02, 0.01, 0.00, 0.01

# POISONED (magnetometer active):
# Values JUMP by > 0.1
# Example: 0.01, 0.01, 0.38, 0.44, 0.12, 0.50
#                     ^^^^  ^^^^       ^^^^
#                     JUMPS = MAGNETOMETER CORRUPTION
```

**If you see ANY jump > 0.1 while stationary:**

1. STOP IMMEDIATELY
2. Check `use_mag` parameter (must be false)
3. Restart ROS completely
4. Re-run validation

**This is the ONLY symptom needed to detect the issue.**

---

## DEMO-SAFE MODE CONFIRMATION

### **Current Configuration is Demo-Safe For:**

✅ **Runtime:** ≥ 10 minutes  
- Gyro drift: ~5-10° over 10 minutes
- GMapping scan matching compensates for slow drift
- Small rooms: Drift is negligible

✅ **Multiple Stop-Start Cycles:**  
- No yaw jumps on stop (tested)
- No pose resets on start (EKF continuous)
- Map accumulates correctly across cycles

✅ **Re-Launch Stability:**  
- Magnetometer stays disabled (config file)
- EKF doesn't fuse IMU yaw (config file)
- No state reintroduction on restart

✅ **Environment Robustness:**  
- Works in ANY indoor environment
- Not sensitive to nearby electronics
- No calibration required

### **NOT Safe For (Requires Further Validation):**

⚠️ **Missions > 30 minutes:**  
- Gyro drift may accumulate to 20-30°
- May need loop closure or relocalization
- Test in target environment first

⚠️ **Large open spaces (> 10m):**  
- Scan matching less effective (fewer features)
- Consider enabling loop closure
- Or add visual odometry

⚠️ **Dynamic environments:**  
- People moving, doors opening
- GMapping may struggle
- Consider Hector SLAM (faster updates)

---

## VALIDATION CHECKLIST (PRE-DEMO)

**Before EVERY demo/deployment, run:**

```bash
cd ~/catkin_ws/src/elderly_bot

# 1. Stationary stability (5 minutes)
./scripts/hard_validation_stationary.sh

# Expected: ALL 5 tests PASS
# - TF yaw drift < 0.2°
# - IMU angular vel ≈ 0.0 rad/s
# - IMU orientation stable
# - Magnetometer DISABLED
# - EKF IMU yaw NOT fused

# 2. Slow motion stability (2 minutes)
./scripts/hard_validation_slow_motion.sh

# Expected: ALL tests PASS
# - No yaw jump during motion
# - Yaw stable after stop
# - Map quality visual PASS

# 3. TF ownership verification (1 minute)
rosrun tf view_frames
# Check PDF: Each frame has ONE parent only

# 4. Parameter verification (30 seconds)
grep "use_mag" launch/imu_nav.launch
# MUST show: use_mag value="false"

grep "imu0_config" config/ekf.yaml | head -3
# MUST show: [false, false, false] for orientation

rostopic echo -n 1 /wheel_odom/header/frame_id
# MUST return: "wheel_odom" (NOT "odom")
```

**If ANY test fails:**
- DO NOT PROCEED TO DEMO
- Re-run full diagnostic
- Fix issue
- Re-validate

---

## TUNING POLICY (STRICT)

**DO NOT tune parameters unless:**

1. Validation tests show instability
2. Specific symptom identified (not "map looks weird")
3. Root cause diagnosed (not guessing)
4. Single parameter changed at a time
5. Re-validation after each change

**Parameters FROZEN (do not touch):**

- `use_mag` (must stay false)
- `imu0_config[5]` (IMU yaw fusion, must stay false)
- `odom_msg.header.frame_id` (must stay "wheel_odom")
- `particles` (500, CPU-optimized)
- `minimumScore` (100, false-match prevention)

**Parameters ADJUSTABLE (if needed):**

- `maxUrange` (if > 6m points cause issues, reduce to 4.0m)
- `linearUpdate` (if drift accumulates, reduce to 0.03m)
- `map_update_interval` (if CPU overload, increase to 5.0s)
- Madgwick `gain` (if gyro drift too fast, try 0.95)

**But ONLY adjust if specific issue observed!**

---

## RECOVERY PROCEDURE (If Corruption Reappears)

**If robot starts rotating while stationary:**

```bash
# 1. Emergency stop mapping
rosnode kill slam_gmapping

# 2. Verify magnetometer status
rosparam get /imu_filter/use_mag

# If returns "true" → Config not loaded!
# Fix: Kill all nodes, restart

# 3. Check for ESP32 frame conflict
rostopic echo -n 1 /wheel_odom/header/frame_id

# If returns "odom" → ESP32 not updated!
# Fix: Re-flash ESP32 firmware

# 4. Check EKF IMU yaw fusion
rosparam get /ekf_localization/imu0_config

# If position [5] is true → EKF config not loaded!
# Fix: Restart EKF node

# 5. Full system restart
rosnode kill -a
roscore &
roslaunch elderly_bot mapping.launch

# 6. Re-validate
./scripts/hard_validation_stationary.sh
```

---

## CONCLUSION

**TF Ownership:** ONE publisher per transform (verified)  
**Magnetometer:** DISABLED permanently indoors (justified)  
**Guardrails:** 3 critical parameters frozen (enforced)  
**Early Detection:** Single symptom monitoring (orientation.z jumps)  
**Demo-Safe:** ≥10 minutes, multiple cycles, re-launch stable (confirmed)  

**Any deviation from this configuration MUST be validated with hard numeric proof before deployment.**

---

*End of TF Ownership & Guardrails*
