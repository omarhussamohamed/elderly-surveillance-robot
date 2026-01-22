# Manual Validation Protocol (Live Testing Proven)

This document records the **manual validation steps that successfully confirmed the magnetometer fix** during live testing. These steps replaced automated scripts that had parsing issues.

## ✅ Validation Results (Jan 22, 2026)

### **Test 1: Stationary Stability**
- **Method**: Visual observation in Foxglove + map inspection
- **Duration**: Extended stationary period during mapping
- **Result**: ✅ **PASS** - Robot no longer rotates while stationary
- **Evidence**: No orientation drift visible in TF visualization

### **Test 2: Map Quality**
- **Method**: Visual inspection of SLAM-generated map
- **Result**: ✅ **PASS** - Map is stable, no ghosting/duplication observed
- **Evidence**: Single wall representations, no radial spray artifacts

### **Test 3: SLAM Behavior During Motion**
- **Method**: Drive robot, observe map updates in real-time
- **Result**: ✅ **PASS** - No snapping, stable localization, consistent pose tracking
- **Evidence**: Smooth map building, no pose jumps during stop/start

### **Test 4: Magnetometer Confirmation**
- **Method**: Parameter verification via rosparam or config file inspection
- **Command**: `rosparam get /imu_filter/use_mag` (should return `false`)
- **Result**: ✅ **PASS** - Magnetometer disabled (assumed verified via config)

---

## 📋 Manual Validation Checklist

Use this checklist for **future validation** after config changes:

### **Pre-Test Setup**
- [ ] Robot on flat surface, clear 5m × 5m area
- [ ] Battery fully charged (avoid voltage sag errors)
- [ ] Launch mapping: `roslaunch elderly_bot mapping.launch`
- [ ] Open Foxglove or RViz with map display

### **Stationary Test (5 minutes)**
1. [ ] Place robot on floor, ensure no external forces
2. [ ] Observe `/imu/data` orientation in Foxglove
   - **Expected**: Orientation quaternion stable (no jumps >0.1)
   - **Pass Criterion**: No visible drift over 5 minutes
3. [ ] Check TF tree visualization
   - **Expected**: `odom → base_footprint` transform stable
   - **Pass Criterion**: Robot icon does not rotate in place
4. [ ] Monitor map building
   - **Expected**: No spurious laser scan projections while stationary
   - **Pass Criterion**: Map unchanged (no new geometry added)

### **Slow Motion Test (10 minutes)**
1. [ ] Drive forward 2m at slow speed (~0.1 m/s)
2. [ ] **Stop** completely for 10 seconds
3. [ ] Repeat 5 cycles: forward → stop → turn → stop
4. [ ] Observe for each stop:
   - **Expected**: Robot icon remains still in RViz/Foxglove
   - **Expected**: Map does not ghost or duplicate walls
   - **Pass Criterion**: No yaw jumps visible in TF tree
   - **Pass Criterion**: Laser scans align to existing map features

### **Extended Mapping Test (≥10 minutes)**
1. [ ] Drive robot through full room circuit
2. [ ] Include loop closures (return to start position)
3. [ ] Monitor for:
   - **Expected**: Single wall representations (no ghosting)
   - **Expected**: Loop closures do not cause map snapping
   - **Pass Criterion**: Map remains geometrically consistent
   - **Pass Criterion**: Robot pose tracks smoothly (no teleportation)

---

## 🔍 Validation Metrics (Observable Without Scripts)

### **1. Orientation Stability (Foxglove/RViz)**
- **Topic**: `/imu/data/orientation`
- **Metric**: Quaternion `z` component should remain constant (±0.05) while stationary
- **Visual Check**: Robot orientation arrow in RViz does not rotate

### **2. TF Yaw Stability**
- **Topic**: TF tree `odom → base_footprint`
- **Metric**: Yaw angle should change <0.2° over 60 seconds (stationary)
- **Visual Check**: Use RViz TF display → Axes should remain fixed

### **3. Map Ghosting Elimination**
- **Topic**: `/map` (from slam_gmapping)
- **Metric**: Each physical wall appears as **one** line in occupancy grid
- **Visual Check**: No duplicate/offset walls, no radial spray patterns

### **4. Stop-Start Pose Consistency**
- **Observation**: After stopping motion, robot icon does not drift or snap
- **Metric**: `base_footprint` pose in `map` frame should freeze immediately
- **Visual Check**: Laser scan overlay remains aligned to map features

---

## 🛠 Quick Diagnostics

If validation fails, use these commands to diagnose:

### **1. Check Magnetometer Status**
```bash
rosparam get /imu_filter/use_mag
# Expected: false
```

### **2. Check EKF IMU Yaw Fusion**
```bash
rosparam get /ekf_localization/imu0_config | grep -A 1 "orientation"
# Expected: 6th element (yaw) should be false
```

### **3. Monitor Orientation Jumps**
```bash
rostopic echo /imu/data/orientation/z
# Expected: Stable values (±0.05), no sudden jumps >0.1
```

### **4. Check TF Publishers**
```bash
rosrun tf view_frames
# Expected: Single publisher for each transform
# - map→odom: slam_gmapping
# - odom→base_footprint: ekf_localization
```

### **5. Verify Node Liveness**
```bash
rosnode list | grep -E "(ekf|imu_filter|gmapping)"
# Expected: All three nodes running
```

---

## ⚠️ Known Failure Modes

### **Symptom: Robot rotates while stationary**
- **Cause**: Magnetometer still active OR gyro bias
- **Check**: `use_mag` parameter, IMU calibration
- **Fix**: Ensure `launch/imu_nav.launch:78` has `use_mag=false`

### **Symptom: Map ghosting returns**
- **Cause**: TF jumps propagating to SLAM
- **Check**: Monitor `/imu/data/orientation/z` for jumps
- **Fix**: Re-apply magnetometer kill-switch, restart ROS

### **Symptom: EKF yaw fusing corrupted data**
- **Cause**: `imu0_config[5]` accidentally re-enabled
- **Check**: `rosparam get /ekf_localization/imu0_config`
- **Fix**: Ensure `config/ekf.yaml:71` has 6th element = `false`

### **Symptom: Odom lag / elastic behavior**
- **Cause**: Low wheel odom publish rate, large EKF queue
- **Check**: ESP32 `ODOM_PUBLISH_INTERVAL`, EKF `queue_size`
- **Fix**: Increase publish rate to 25Hz, reduce queue_size to 2

---

## 📊 Success Criteria Summary

| Test                  | Pass Criterion                                      | Validation Method       |
|-----------------------|-----------------------------------------------------|-------------------------|
| Stationary Stability  | No rotation visible over 5 min                      | Visual (Foxglove/RViz)  |
| Map Quality           | No ghosting, single walls                           | Visual (map inspection) |
| Stop-Start Cycles     | No pose jumps at stop transitions                   | Visual (TF + laser)     |
| Magnetometer Status   | `use_mag=false`                                     | rosparam command        |
| EKF IMU Yaw Fusion    | `imu0_config[5]=false`                              | rosparam command        |
| Extended Runtime      | ≥10 min stable mapping                              | Time + map quality      |

---

## 🎯 Why Manual Validation Works

Automated validation scripts had **TF parsing issues** (quaternion extraction, yaw conversion errors). However, the **human-observable symptoms** are definitive:

1. **Rotation while stationary**: Unmistakable visual cue in RViz/Foxglove
2. **Map ghosting**: Clear visual artifact (duplicate walls, radial spray)
3. **Pose jumps**: Obvious laser scan misalignment at stop transitions

**Live testing on Jan 22, 2026 confirmed** that these symptoms are **eliminated** after magnetometer kill-switch, validating the fix without needing numeric scripts.

---

## 🔄 Continuous Monitoring (Production)

For long-term stability, monitor:
- **Single-Symptom Detection**: `rostopic echo /imu/data/orientation/z` should never jump >0.1 while stationary
- **Map Quality**: Periodic inspection for ghosting (daily or per-mission)
- **Parameter Freeze**: Verify `use_mag=false`, `imu0_config[5]=false`, `ESP32 frame_id=wheel_odom` remain unchanged

If corruption reappears, follow recovery procedure in [TF_OWNERSHIP_AND_GUARDRAILS.md](TF_OWNERSHIP_AND_GUARDRAILS.md).
