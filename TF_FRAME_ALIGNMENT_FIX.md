# TF FRAME ALIGNMENT FIX - APPLIED JANUARY 18, 2026

## PROBLEM STATEMENT

**Issue:** RViz TF display showed misaligned coordinate frame axes:
- `laser` frame: Standard orientation (red=X forward, green=Y left, blue=Z up)
- `imu_link` frame: Axes rotated/inverted relative to laser
- `base_link`, `base_footprint`: Axes not parallel to laser

**Impact:**
- Incorrect sensor fusion in robot_localization EKF
- Potential sign flips in velocities/orientations
- Contributed to mapping ghosting if orientations inconsistent
- Violated ROS REP-105 standard coordinate conventions

## ROOT CAUSES IDENTIFIED

### 1. LIDAR FRAME ROTATION ERROR (CRITICAL)

**Location:** urdf/elderly_bot.urdf line 169 & launch/bringup.launch line 80

**Problem:**
```xml
<!-- BEFORE (WRONG) -->
<origin xyz="0 0 0.23" rpy="3.14159265359 0 0"/>
```

- Used 180° **roll** (rotation around X-axis) instead of 180° **yaw** (rotation around Z-axis)
- This INVERTED the laser's X and Y axes backward
- Comment in launch file says "Lidar motor facing backward = 180 degree rotation around Z" but URDF used roll

**Fix Applied:**
```xml
<!-- AFTER (CORRECT) -->
<origin xyz="0 0 0.23" rpy="0 0 3.14159265359"/>
```

- Changed to 180° yaw rotation (around Z-axis)
- Laser X-axis now points forward (aligned with base_link)
- Laser Y-axis now points left (aligned with base_link)

**Static Transform Also Fixed:**
```bash
# BEFORE
args="0 0 0.23 3.14159 0 0 base_link laser 100"

# AFTER
args="0 0 0.23 0 0 3.14159 base_link laser 100"
```

### 2. MISSING ROBOT_STATE_PUBLISHER (ARCHITECTURAL)

**Problem:**
- URDF defined all robot links and joints
- But no `robot_state_publisher` node to broadcast transforms from URDF
- Instead, manual `static_transform_publisher` nodes duplicated URDF
- Made URDF mostly decorative, easy to have inconsistencies

**Fix Applied:**
Added proper robot_state_publisher to bringup.launch:
```xml
<node name="robot_state_publisher" 
      pkg="robot_state_publisher" 
      type="robot_state_publisher"
      output="screen">
  <param name="publish_frequency" value="50.0" />
</node>
```

**Benefits:**
- Single source of truth (URDF)
- Automatic broadcast of all fixed joints
- Consistent with standard ROS architecture
- Easier maintenance

### 3. DUPLICATE STATIC TRANSFORM PUBLISHERS (REMOVED)

**Problem:**
- `static_transform_publisher` nodes for base_footprint→base_link and base_link→laser
- Redundant with URDF definitions
- Could cause TF conflicts or inconsistencies

**Fix Applied:**
- Removed `base_footprint_to_base_link` static publisher from bringup.launch
- Removed `base_link_to_laser` static publisher from bringup.launch
- Removed `base_link_to_imu_link` static publisher from imu_nav.launch
- All transforms now published by robot_state_publisher from URDF

### 4. IMU AXIS ORIENTATION (PENDING VERIFICATION)

**Status:** Requires physical testing

**Concern:**
- MPU9250 driver publishes raw chip axes without transformation
- Physical IMU mounting orientation unknown
- May require axis remapping if chip not aligned with REP-105

**Diagnostic Created:**
- `scripts/imu_mounting_diagnostic.sh` - guides user through physical tests
- Tests gravity detection (Z-axis), forward motion (X-axis), left motion (Y-axis)
- Provides specific URDF rotation fixes based on results

## CHANGES MADE

### File: urdf/elderly_bot.urdf

**Line 169 - Laser Joint Rotation:**
```diff
- <origin xyz="0 0 0.23" rpy="3.14159265359 0 0"/>
+ <origin xyz="0 0 0.23" rpy="0 0 3.14159265359"/>
```

### File: launch/bringup.launch

**Added robot_state_publisher (after line 31):**
```xml
<node name="robot_state_publisher" 
      pkg="robot_state_publisher" 
      type="robot_state_publisher"
      output="screen">
  <param name="publish_frequency" value="50.0" />
</node>
```

**Removed redundant static transforms (lines 69-81):**
- Deleted `base_footprint_to_base_link` static publisher
- Deleted `base_link_to_laser` static publisher
- Added comment explaining robot_state_publisher now handles these

### File: launch/imu_nav.launch

**Removed redundant static transform (lines 112-116):**
- Deleted `base_link_to_imu_link` static publisher
- Added comment directing users to edit URDF instead

### File: scripts/imu_mounting_diagnostic.sh (NEW)

**Purpose:** Interactive physical testing to determine IMU chip orientation

**Tests Performed:**
1. Stationary gravity detection → identifies which axis points up/down
2. Forward motion test → identifies which axis points forward
3. Left motion test → identifies which axis points left
4. Provides specific URDF rotation fixes based on results

## VERIFICATION STEPS

### 1. Transfer Scripts to Jetson

```bash
# From Windows PowerShell
scp "scripts/imu_mounting_diagnostic.sh" omar@192.168.1.29:~/catkin_ws/src/elderly_bot/scripts/
scp "scripts/master_validator.sh" omar@192.168.1.29:~/catkin_ws/src/elderly_bot/scripts/
```

### 2. Restart ROS System

```bash
ssh omar@192.168.1.29
rosnode kill -a
killall -9 rosmaster roscore
sleep 2
roscore &
sleep 3
roslaunch elderly_bot bringup.launch
```

### 3. Check TF Tree

```bash
# Generate TF tree diagram
rosrun tf view_frames
evince frames.pdf

# Check specific transforms
rosrun tf tf_echo base_footprint laser
rosrun tf tf_echo base_footprint imu_link

# Expected output for laser (with 180° yaw):
# - Translation: (0, 0, 0.30)
# - Rotation: quaternion (0, 0, 1, 0) = yaw 180°
# - RPY: (0, 0, 3.14159)
```

### 4. Verify in RViz

```bash
rviz
```

**Enable TF Display:**
1. Add → TF
2. Check "Show Axes"
3. Set "Marker Scale" to 0.3 for visibility

**Expected Result:**
- All red arrows (X-axes) point same direction (forward)
- All green arrows (Y-axes) point same direction (left)
- All blue arrows (Z-axes) point same direction (up)
- No misalignment between base_footprint, base_link, laser, imu_link

### 5. Run IMU Mounting Diagnostic

```bash
chmod +x ~/catkin_ws/src/elderly_bot/scripts/imu_mounting_diagnostic.sh
bash ~/catkin_ws/src/elderly_bot/scripts/imu_mounting_diagnostic.sh
```

**Follow script instructions:**
- Place robot level and stationary
- Push robot forward when prompted
- Push robot left when prompted
- Record which IMU axes respond to each motion

**If IMU axes misaligned, edit urdf/elderly_bot.urdf line 199:**

```xml
<!-- Example: If IMU upside-down (Z points down) -->
<origin xyz="0 0 0" rpy="3.14159 0 0"/>

<!-- Example: If IMU rotated 180° horizontal (X backward) -->
<origin xyz="0 0 0" rpy="0 0 3.14159"/>

<!-- Example: If IMU rotated 90° (X→Y swap) -->
<origin xyz="0 0 0" rpy="0 0 1.5708"/>
```

### 6. Run Master Validation Suite

```bash
chmod +x ~/catkin_ws/src/elderly_bot/scripts/master_validator.sh
bash ~/catkin_ws/src/elderly_bot/scripts/master_validator.sh
```

**Confirm all previous fixes still working:**
- Stage 1: Communication >8Hz wheel, >40Hz filtered
- Stage 2: Drift <0.1° over 3 minutes
- Stage 3: 1.0m commanded = 1.0m traveled (±2cm)
- Stage 4: 360° rotation returns to start

## EXPECTED OUTCOMES

### RViz TF Display (AFTER FIX)

```
base_footprint → base_link → laser
                          ↘ imu_link

ALL frames aligned:
✓ Red (X) arrows parallel, pointing forward
✓ Green (Y) arrows parallel, pointing left
✓ Blue (Z) arrows parallel, pointing up
```

### TF Echo Output

```bash
$ rosrun tf tf_echo base_footprint laser
Translation: [0.000, 0.000, 0.300]
Rotation: in Quaternion [0.000, 0.000, 1.000, 0.000]
         in RPY (radian) [0.000, 0.000, 3.142]
         in RPY (degree) [0.000, 0.000, 180.000]
```

### Robot Localization EKF

- No warning: "Imu orientation is identity quaternion"
- No sudden jumps in /odometry/filtered
- Yaw from IMU correctly fused with wheel odometry
- Drift remains <0.1° over 3 minutes

### Gmapping SLAM

- Sharp single-line walls (no ghosting)
- Corners at precise 90° angles
- Loop closure snaps to existing map features
- No doubled geometry from frame misalignment

## TROUBLESHOOTING

### Issue: "No transform from base_footprint to laser"

**Cause:** robot_state_publisher not running or URDF not loaded

**Fix:**
```bash
# Check robot_state_publisher running
rosnode list | grep robot_state

# Check URDF loaded
rosparam get /robot_description

# Restart if needed
rosnode kill /robot_state_publisher
roslaunch elderly_bot bringup.launch
```

### Issue: "Multiple TF publishers for base_link"

**Cause:** Old static_transform_publisher still running

**Fix:**
```bash
rosnode list | grep static_transform
rosnode kill /base_footprint_to_base_link
rosnode kill /base_link_to_laser
rosnode kill /base_link_to_imu_link
```

### Issue: Laser frame still inverted in RViz

**Cause:** Old RViz config cached or URDF not reloaded

**Fix:**
```bash
rosnode kill -a
killall -9 roscore rosmaster
# Wait 5 seconds
roscore &
roslaunch elderly_bot bringup.launch
```

### Issue: IMU axes still misaligned after diagnostic

**Cause:** Physical IMU mounting requires rotation transform

**Fix:**
1. Run `imu_mounting_diagnostic.sh` to identify exact rotation needed
2. Edit `urdf/elderly_bot.urdf` line 199 `<origin>` tag with correct rpy
3. Restart ROS system
4. Re-verify in RViz

## COMPATIBILITY WITH PREVIOUS FIXES

### Drift Fix (PRESERVED)
✓ EKF yaw fusion from IMU absolute orientation unchanged
✓ Madgwick filter zeta=0.01 bias correction intact
✓ IMU frame_id still 'imu_link'
✓ No changes to ekf.yaml fusion config

### Kinematic Scaling (PRESERVED)
✓ TICKS_PER_REV=3960 in firmware unchanged
✓ WHEEL_RADIUS=0.0325m unchanged
✓ Float math in ticks_to_m calculation unchanged
✓ No impact on odometry calculations

### Communication Fixes (PRESERVED)
✓ tcp_nodelay=1 for rosserial unchanged
✓ Topic frequencies unaffected
✓ Network optimizations intact

### Gmapping Config (PRESERVED)
✓ minimumScore=50 unchanged
✓ SLAM configuration intact
✓ Will benefit from corrected frame alignment

## TECHNICAL NOTES

### Static Transform Publisher Args Format

```bash
args="x y z yaw pitch roll parent_frame child_frame period_ms"
```

**Order matters:** yaw-pitch-roll (NOT roll-pitch-yaw!)

**Rotation around axes:**
- yaw = rotation around Z (horizontal spin)
- pitch = rotation around Y (tilt forward/back)
- roll = rotation around X (tilt left/right)

### URDF Origin Tag RPY Order

```xml
<origin xyz="x y z" rpy="roll pitch yaw"/>
```

**Order matters:** roll-pitch-yaw (NOT yaw-pitch-roll!)

**Confusing difference:** Launch file args use YPR order, URDF uses RPY order!

### REP-105 Coordinate Conventions

**Right-Hand Rule:**
- X forward (robot front)
- Y left
- Z up
- Rotation: counter-clockwise positive when looking along positive axis

**Common Mistakes:**
- Lidar inverted: use yaw rotation (Z), not roll/pitch
- IMU axes: must match sensor datasheet physical orientation
- Static transforms: remember YPR order in args

## SIGN-OFF

**Changes Applied:** January 18, 2026  
**System Status:** TF frames corrected, IMU orientation pending physical verification  
**Next Action:** Run imu_mounting_diagnostic.sh on Jetson to verify/correct IMU axes  
**Validation:** Run master_validator.sh after IMU fix to confirm all systems operational  

**Files Modified:**
1. urdf/elderly_bot.urdf - Corrected laser rotation (180° yaw)
2. launch/bringup.launch - Added robot_state_publisher, removed duplicate transforms
3. launch/imu_nav.launch - Removed duplicate imu_link transform
4. scripts/imu_mounting_diagnostic.sh - NEW diagnostic tool
5. TF_FRAME_ALIGNMENT_FIX.md - THIS DOCUMENT
