# COMPLETE TF FIX DEPLOYMENT PROCEDURE

## Step-by-Step Deployment on Jetson Nano

**Date:** January 18, 2026  
**Status:** Ready for deployment and verification

---

## PHASE 1: TRANSFER UPDATED FILES

### From Windows PowerShell:

```powershell
cd "c:\Users\omarh\Desktop\Graduation Project\Elderly_Robot_Project\catkin_ws\src\elderly_bot"

# Transfer updated configuration files
scp urdf\elderly_bot.urdf omar@192.168.1.29:~/catkin_ws/src/elderly_bot/urdf/
scp launch\bringup.launch omar@192.168.1.29:~/catkin_ws/src/elderly_bot/launch/
scp launch\imu_nav.launch omar@192.168.1.29:~/catkin_ws/src/elderly_bot/launch/

# Transfer diagnostic and validation scripts
scp scripts\tf_verification_complete.sh omar@192.168.1.29:~/catkin_ws/src/elderly_bot/scripts/
scp scripts\imu_mounting_diagnostic.sh omar@192.168.1.29:~/catkin_ws/src/elderly_bot/scripts/
scp scripts\master_validator.sh omar@192.168.1.29:~/catkin_ws/src/elderly_bot/scripts/

# Transfer documentation
scp TF_FRAME_ALIGNMENT_FIX.md omar@192.168.1.29:~/catkin_ws/src/elderly_bot/
scp IMU_MOUNTING_FIX_REFERENCE.md omar@192.168.1.29:~/catkin_ws/src/elderly_bot/
```

---

## PHASE 2: RESTART ROS SYSTEM

### SSH to Jetson:

```bash
ssh omar@192.168.1.29
```

### Kill existing ROS processes:

```bash
rosnode kill -a
killall -9 rosmaster roscore
sleep 3
```

### Start fresh ROS system:

```bash
roscore &
sleep 3
roslaunch elderly_bot bringup.launch
```

**Expected console output:**
- `[INFO] Starting robot_state_publisher` ← NEW, confirms fix applied
- `[INFO] Starting rosserial...`
- `[INFO] Starting RPLidar node...`
- `[INFO] Starting IMU pipeline...`
- No TF warnings about duplicate publishers

---

## PHASE 3: RUN TF VERIFICATION

### Open new SSH session:

```bash
ssh omar@192.168.1.29
cd ~/catkin_ws/src/elderly_bot
chmod +x scripts/tf_verification_complete.sh
bash scripts/tf_verification_complete.sh
```

### Expected Output - LASER SECTION:

```
PART 2: LASER ROTATION VERIFICATION
------------------------------------
Checking base_link → laser transform...

Translation: [0.000, 0.000, 0.230]
Rotation: in Quaternion [0.000, 0.000, 1.000, 0.000]
         in RPY (radian) [0.000, 0.000, 3.142]
         in RPY (degree) [0.000, 0.000, 180.000]

LASER ROTATION ANALYSIS:
  Roll:  0.0000 rad = 0.00°
  Pitch: 0.0000 rad = 0.00°
  Yaw:   3.1416 rad = 180.00°

✓ PASS: Laser has 180° YAW rotation (correct for backward-facing lidar)
        Roll is near zero (not using wrong axis)
        FIX SUCCESSFULLY APPLIED!
```

**If you see this:** ✅ Lidar fix confirmed working!

**If you see 180° ROLL instead:** ❌ URDF not loaded, check file transfer and restart

### Expected Output - IMU SECTION:

```
PART 3: IMU LINK ROTATION VERIFICATION
---------------------------------------
Checking base_link → imu_link transform...

Translation: [0.000, 0.000, 0.000]
Rotation: in Quaternion [0.000, 0.000, 0.000, 1.000]
         in RPY (radian) [0.000, 0.000, 0.000]
         in RPY (degree) [0.000, 0.000, 0.000]

IMU LINK ROTATION ANALYSIS:
  Roll:  0.0000 rad = 0.00°
  Pitch: 0.0000 rad = 0.00°
  Yaw:   0.0000 rad = 0.00°

✓ IMU mounted standard: No rotation needed
```

**If you see this:** ✅ IMU axes aligned, no fix needed

**If you see non-zero angles:** ⚠️ IMU mounted rotated, proceed to Phase 4

### Expected Output - GRAVITY SECTION:

```
PART 4: IMU PHYSICAL MOUNTING DIAGNOSTIC
-----------------------------------------
Sampling accelerometer...

Accelerometer readings (stationary):
  X: 0.12 m/s²
  Y: -0.08 m/s²
  Z: 9.81 m/s²

GRAVITY DETECTION:
  Axis with gravity: IMU Z-axis = 9.81 m/s²
✓ IMU Z-axis points UP (+9.8) - CORRECT orientation
```

**If you see this:** ✅ IMU physically correct, no URDF fix needed

**If gravity on X or Y axis:** ❌ IMU mounted on side, needs rotation fix

---

## PHASE 4: APPLY IMU FIX (IF NEEDED)

### Only if Phase 3 diagnostic showed IMU rotation issue

**Based on diagnostic output, identify your scenario from IMU_MOUNTING_FIX_REFERENCE.md**

### Example: IMU Upside-Down (Z-axis = -9.8)

```bash
nano ~/catkin_ws/src/elderly_bot/urdf/elderly_bot.urdf
```

Find line 199 and change:
```xml
<!-- BEFORE -->
<origin xyz="0 0 0" rpy="0 0 0"/>

<!-- AFTER -->
<origin xyz="0 0 0" rpy="3.14159265359 0 0"/>
```

Save (Ctrl+O, Enter, Ctrl+X)

### Restart ROS:

```bash
rosnode kill -a
killall -9 rosmaster
sleep 3
roscore &
sleep 3
roslaunch elderly_bot bringup.launch
```

### Re-verify:

```bash
bash scripts/tf_verification_complete.sh
# Check IMU section now shows roll=0, pitch=0, yaw=0
# Check gravity section now shows Z-axis = +9.8
```

---

## PHASE 5: RVIZ VISUAL VERIFICATION

### Start RViz:

```bash
rviz
```

### Configure TF Display:

1. Click "Add" (bottom left)
2. Select "TF"
3. Click "OK"
4. In TF settings:
   - ✅ Check "Show Axes"
   - Set "Marker Scale" = 0.3
   - ✅ Check "Show Names" (optional, helpful for debugging)
5. Set "Fixed Frame" dropdown = "base_footprint"

### Expected Visual Result:

```
Frame Axes (RGB = XYZ):
├─ base_footprint
│   ├─ RED arrow: points forward ✓
│   ├─ GREEN arrow: points left ✓
│   └─ BLUE arrow: points up ✓
│
├─ base_link (slightly above base_footprint)
│   ├─ RED arrow: points forward (parallel to base_footprint) ✓
│   ├─ GREEN arrow: points left (parallel) ✓
│   └─ BLUE arrow: points up (parallel) ✓
│
├─ laser (on top of base_link)
│   ├─ RED arrow: points BACKWARD (180° from base_link) ✓ EXPECTED
│   ├─ GREEN arrow: points RIGHT (180° from base_link) ✓ EXPECTED
│   └─ BLUE arrow: points up (parallel to base_link) ✓
│
└─ imu_link (same position as base_link)
    ├─ RED arrow: points forward (parallel to base_link) ✓
    ├─ GREEN arrow: points left (parallel to base_link) ✓
    └─ BLUE arrow: points up (parallel to base_link) ✓
```

**KEY CHECKS:**
- ✅ base_link and imu_link: All arrows same direction
- ✅ laser: Red/green opposite to base_link (180° yaw), blue same
- ✅ All blue (Z) arrows point up
- ❌ If any arrows misaligned: Re-check URDF and restart

---

## PHASE 6: MASTER SYSTEM VALIDATION

### Run complete validation suite:

```bash
cd ~/catkin_ws/src/elderly_bot
bash scripts/master_validator.sh
```

### Expected Results:

```
STAGE 1: Communication & Sync Audit
  Status: ✓ PASS
  /wheel_odom: 45.2 Hz (target: >8 Hz)
  /odometry/filtered: 48.7 Hz (target: >40 Hz)

STAGE 2: 3-Minute Stationary Drift Test
  Status: ✓ PASS
  Total Drift: 0.03° (target: <0.1°)

STAGE 3: Golden Meter Linear Scaling Test
  Status: ✓ PASS
  Odometry Distance: 1.02 m
  Physical Distance: 101 cm
  Error: 1.0 cm (target: ±2cm)

STAGE 4: 360° Compass Rotation Test
  Status: ✓ PASS
  Rotation Complete: Yes
  
OVERALL SYSTEM STATUS:
★ ALL TESTS PASSED ★
System is PRODUCTION READY for autonomous navigation
```

**All 4 stages must PASS** ← Confirms TF fixes didn't break previous work

---

## PHASE 7: MAPPING QUALITY TEST (FINAL PROOF)

### Start mapping session:

```bash
roslaunch elderly_bot mapping.launch
```

### In new terminal, open RViz with mapping config:

```bash
rviz -d ~/catkin_ws/src/elderly_bot/rviz/mapping.rviz
```

### Drive robot manually:

**Option A: Simple square pattern (command-line):**
```bash
# Forward 2m
timeout 20 rostopic pub -r 10 /cmd_vel geometry_msgs/Twist "linear: {x: 0.1}"

# Rotate 90° left
timeout 7 rostopic pub -r 10 /cmd_vel geometry_msgs/Twist "angular: {z: 0.25}"

# Forward 2m
timeout 20 rostopic pub -r 10 /cmd_vel geometry_msgs/Twist "linear: {x: 0.1}"

# Rotate 90° left
timeout 7 rostopic pub -r 10 /cmd_vel geometry_msgs/Twist "angular: {z: 0.25}"

# Complete square...
```

**Option B: Interactive teleop:**
```bash
sudo apt install ros-melodic-teleop-twist-keyboard
rosrun teleop_twist_keyboard teleop_twist_keyboard.py
```

### Observe Map Quality in RViz:

**GOOD MAP (TF fix working):**
- ✅ Walls are single sharp lines (1-2 pixels wide)
- ✅ Corners form precise 90° angles
- ✅ When revisiting same area, scans "snap" to existing map
- ✅ No "ghosting" or doubled geometry
- ✅ Robot pose (red arrow) moves smoothly

**BAD MAP (TF still broken):**
- ❌ Walls have blur/ghosting (4-5 pixels wide)
- ❌ Corners are rounded or doubled
- ❌ Revisiting area creates new parallel walls
- ❌ Robot pose jumps or jitters

### Save successful map:

```bash
rosrun map_server map_saver -f ~/maps/tf_verified_map
```

---

## TROUBLESHOOTING GUIDE

### Issue: "robot_state_publisher not found"

**Cause:** Package not installed

**Fix:**
```bash
sudo apt-get update
sudo apt-get install ros-melodic-robot-state-publisher
```

### Issue: Laser still shows 180° roll in tf_echo

**Cause:** URDF file not transferred or ROS not restarted

**Fix:**
```bash
# Verify file has correct content
grep -A 2 "base_link_to_laser" ~/catkin_ws/src/elderly_bot/urdf/elderly_bot.urdf
# Should show: rpy="0 0 3.14159265359"

# If wrong, re-transfer:
# (On Windows) scp urdf\elderly_bot.urdf omar@192.168.1.29:~/catkin_ws/src/elderly_bot/urdf/

# Restart ROS completely:
rosnode kill -a
killall -9 rosmaster roscore
roscore &
roslaunch elderly_bot bringup.launch
```

### Issue: IMU gravity on X or Y axis

**Cause:** Physical IMU mounted on side or upside-down

**Fix:** See IMU_MOUNTING_FIX_REFERENCE.md, apply appropriate rpy rotation to URDF

### Issue: Master validator Stage 2 fails (drift increased)

**Cause:** Wrong IMU rotation applied (axes inverted)

**Fix:**
- Revert IMU rotation in URDF to rpy="0 0 0"
- Re-run tf_verification_complete.sh
- Carefully apply correct rotation based on gravity test

### Issue: Map ghosting still present

**Cause:** IMU axes inverted (sign error in rotation)

**Fix:**
- Check if rotation needs negative sign (e.g., -1.5708 instead of +1.5708)
- Verify gravity points UP (+9.8 on Z) not DOWN (-9.8)

---

## SUCCESS CHECKLIST

Before proceeding to autonomous navigation:

- [ ] tf_verification_complete.sh shows laser with 180° yaw ✓
- [ ] tf_verification_complete.sh shows IMU with 0° rotation (or known offset) ✓
- [ ] Gravity detected on Z-axis = +9.8 m/s² ✓
- [ ] RViz TF display: all axes aligned correctly ✓
- [ ] Master validator: ALL 4 STAGES PASS ✓
- [ ] Mapping test: sharp walls, no ghosting ✓
- [ ] No TF warnings in console ✓

**When all checked:** System is ready for autonomous navigation and patrol missions!

---

## DOCUMENTATION REFERENCE

- **TF_FRAME_ALIGNMENT_FIX.md** - Complete technical explanation of fixes
- **IMU_MOUNTING_FIX_REFERENCE.md** - IMU rotation scenarios and solutions
- **TF_FIX_DEPLOYMENT_CHECKLIST.md** - Quick deployment commands
- **PHYSICAL_VALIDATION_PROTOCOL.md** - Manual physical testing procedures

---

**Deployment Date:** _______________  
**Verified By:** _______________  
**Final Status:** [ ] PASS [ ] NEEDS ADJUSTMENT  
**Notes:** _______________________________________________________________
