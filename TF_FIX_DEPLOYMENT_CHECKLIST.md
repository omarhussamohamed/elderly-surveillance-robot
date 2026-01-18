# TF FRAME ALIGNMENT - DEPLOYMENT CHECKLIST

## CRITICAL FIX APPLIED: LIDAR ROTATION CORRECTED

**Problem:** Lidar frame had 180° **roll** (wrong axis) instead of 180° **yaw**  
**Result:** Red/green axes pointed backward, violating REP-105  
**Fix:** Changed `rpy="3.14159 0 0"` → `rpy="0 0 3.14159"`  

---

## DEPLOYMENT STEPS (Execute on Jetson)

### Step 1: Transfer Files from Windows
```powershell
# Run in Windows PowerShell
scp "c:\Users\omarh\Desktop\Graduation Project\Elderly_Robot_Project\catkin_ws\src\elderly_bot\urdf\elderly_bot.urdf" omar@192.168.1.29:~/catkin_ws/src/elderly_bot/urdf/
scp "c:\Users\omarh\Desktop\Graduation Project\Elderly_Robot_Project\catkin_ws\src\elderly_bot\launch\bringup.launch" omar@192.168.1.29:~/catkin_ws/src/elderly_bot/launch/
scp "c:\Users\omarh\Desktop\Graduation Project\Elderly_Robot_Project\catkin_ws\src\elderly_bot\launch\imu_nav.launch" omar@192.168.1.29:~/catkin_ws/src/elderly_bot/launch/
scp "c:\Users\omarh\Desktop\Graduation Project\Elderly_Robot_Project\catkin_ws\src\elderly_bot\scripts\imu_mounting_diagnostic.sh" omar@192.168.1.29:~/catkin_ws/src/elderly_bot/scripts/
scp "c:\Users\omarh\Desktop\Graduation Project\Elderly_Robot_Project\catkin_ws\src\elderly_bot\scripts\master_validator.sh" omar@192.168.1.29:~/catkin_ws/src/elderly_bot/scripts/
```

### Step 2: Restart ROS System
```bash
ssh omar@192.168.1.29
rosnode kill -a
killall -9 rosmaster roscore
sleep 3
roscore &
sleep 3
roslaunch elderly_bot bringup.launch
```

### Step 3: Quick TF Verification
```bash
# New SSH session
rosrun tf tf_echo base_footprint laser
# Expected: Translation (0, 0, 0.30), RPY (0, 0, 180°)

rosrun tf tf_echo base_footprint imu_link
# Expected: Translation (0, 0, 0.07), RPY (0, 0, 0°)
```

### Step 4: RViz Visual Check
```bash
rviz
```

**Setup:**
1. Add → TF
2. Check "Show Axes"
3. Set "Marker Scale" = 0.3
4. Fixed Frame = "base_footprint"

**Expected:**
- ✅ All RED arrows (X) point forward, parallel
- ✅ All GREEN arrows (Y) point left, parallel
- ✅ All BLUE arrows (Z) point up, parallel

**If IMU axes still misaligned → proceed to Step 5**

### Step 5: IMU Mounting Diagnostic (IF NEEDED)
```bash
chmod +x ~/catkin_ws/src/elderly_bot/scripts/imu_mounting_diagnostic.sh
bash ~/catkin_ws/src/elderly_bot/scripts/imu_mounting_diagnostic.sh
```

**Follow prompts:**
- Test 1: Gravity detection (Z-axis)
- Test 2: Forward motion (X-axis)
- Test 3: Left motion (Y-axis)

**If IMU axes wrong, script will tell you exact URDF fix needed**

### Step 6: Final System Validation
```bash
chmod +x ~/catkin_ws/src/elderly_bot/scripts/master_validator.sh
bash ~/catkin_ws/src/elderly_bot/scripts/master_validator.sh
```

**Must pass:**
- ✅ Stage 1: Communication >8Hz wheel, >40Hz filtered
- ✅ Stage 2: Drift <0.1° over 3 minutes
- ✅ Stage 3: 1.0m commanded = 1.0m traveled ±2cm
- ✅ Stage 4: 360° rotation returns to start

---

## WHAT WAS CHANGED

### urdf/elderly_bot.urdf
```diff
Line 169:
- <origin xyz="0 0 0.23" rpy="3.14159265359 0 0"/>
+ <origin xyz="0 0 0.23" rpy="0 0 3.14159265359"/>
```

### launch/bringup.launch
- ✅ Added `robot_state_publisher` node
- ✅ Removed redundant `base_footprint_to_base_link` static publisher
- ✅ Removed redundant `base_link_to_laser` static publisher

### launch/imu_nav.launch
- ✅ Removed redundant `base_link_to_imu_link` static publisher

---

## QUICK REFERENCE: ROTATION CONVENTIONS

### Static Transform Args (launch files)
```bash
args="x y z YAW PITCH ROLL parent child period"
# Order: YAW-PITCH-ROLL (confusing!)
```

### URDF Origin Tag
```xml
<origin xyz="x y z" rpy="ROLL PITCH YAW"/>
# Order: ROLL-PITCH-YAW (opposite!)
```

### REP-105 Coordinate Frame
```
X = Forward (red arrow)
Y = Left (green arrow)
Z = Up (blue arrow)
```

### Common Rotations
```
180° yaw (horizontal flip):     rpy="0 0 3.14159"
180° roll (upside-down flip):   rpy="3.14159 0 0"
90° yaw (left turn):            rpy="0 0 1.5708"
```

---

## TROUBLESHOOTING

**"No transform from base_footprint to laser"**
→ robot_state_publisher not running, check rosnode list

**"Laser axes still backward"**
→ URDF not reloaded, restart ROS completely (kill roscore)

**"IMU axes misaligned"**
→ Run imu_mounting_diagnostic.sh, apply fix to URDF line 199

**"Drift increased after TF fix"**
→ Unlikely, but re-check ekf.yaml imu0_config hasn't changed

---

## SUCCESS CRITERIA

✅ **RViz TF Display:** All same-color arrows parallel  
✅ **tf_echo laser:** Shows yaw=180°, not roll=180°  
✅ **Master Validator:** All 4 stages PASS  
✅ **Mapping:** Sharp walls, no ghosting from frame issues  
✅ **Navigation:** Smooth, no sudden pose jumps  

---

**Date:** January 18, 2026  
**Status:** FIX APPLIED - DEPLOYMENT PENDING  
**Next:** Transfer files, restart ROS, verify in RViz
