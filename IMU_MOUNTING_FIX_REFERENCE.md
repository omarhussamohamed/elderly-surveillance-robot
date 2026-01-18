# IMU MOUNTING FIX REFERENCE

## Common MPU9250 Mounting Scenarios & URDF Fixes

Based on tf_verification_complete.sh diagnostic output, apply the appropriate fix to `urdf/elderly_bot.urdf` line 199.

---

### SCENARIO 1: IMU Mounted Standard (IDEAL)

**Diagnostic Shows:**
- Z-axis: +9.8 m/s² (gravity pointing up) ✓
- Forward motion → X-axis ✓
- Left motion → Y-axis ✓

**TF Rotation:** Roll=0°, Pitch=0°, Yaw=0°

**URDF Fix:** None needed - already correct
```xml
<joint name="base_link_to_imu_link" type="fixed">
  <parent link="base_link"/>
  <child link="imu_link"/>
  <origin xyz="0 0 0" rpy="0 0 0"/>
</joint>
```

---

### SCENARIO 2: IMU Upside-Down (180° Roll)

**Diagnostic Shows:**
- Z-axis: -9.8 m/s² (gravity pointing down) ✗
- Chip mounted with components facing down

**TF Rotation:** Roll=180°, Pitch=0°, Yaw=0°

**URDF Fix:** Add 180° roll rotation
```xml
<joint name="base_link_to_imu_link" type="fixed">
  <parent link="base_link"/>
  <child link="imu_link"/>
  <origin xyz="0 0 0" rpy="3.14159265359 0 0"/>
</joint>
```

---

### SCENARIO 3: IMU Rotated 180° Horizontal (180° Yaw)

**Diagnostic Shows:**
- Z-axis: +9.8 m/s² (up) ✓
- Forward motion → X-axis negative (backward) ✗
- Left motion → Y-axis negative (right) ✗

**TF Rotation:** Roll=0°, Pitch=0°, Yaw=180°

**URDF Fix:** Add 180° yaw rotation
```xml
<joint name="base_link_to_imu_link" type="fixed">
  <parent link="base_link"/>
  <child link="imu_link"/>
  <origin xyz="0 0 0" rpy="0 0 3.14159265359"/>
</joint>
```

---

### SCENARIO 4: IMU Rotated 90° Right (90° Yaw)

**Diagnostic Shows:**
- Z-axis: +9.8 m/s² (up) ✓
- Forward motion → Y-axis positive ✗
- Left motion → X-axis negative ✗

**TF Rotation:** Roll=0°, Pitch=0°, Yaw=90°

**URDF Fix:** Add 90° yaw rotation
```xml
<joint name="base_link_to_imu_link" type="fixed">
  <parent link="base_link"/>
  <child link="imu_link"/>
  <origin xyz="0 0 0" rpy="0 0 1.5707963268"/>
</joint>
```

---

### SCENARIO 5: IMU Rotated 90° Left (-90° Yaw)

**Diagnostic Shows:**
- Z-axis: +9.8 m/s² (up) ✓
- Forward motion → Y-axis negative ✗
- Left motion → X-axis positive ✗

**TF Rotation:** Roll=0°, Pitch=0°, Yaw=-90°

**URDF Fix:** Add -90° yaw rotation
```xml
<joint name="base_link_to_imu_link" type="fixed">
  <parent link="base_link"/>
  <child link="imu_link"/>
  <origin xyz="0 0 0" rpy="0 0 -1.5707963268"/>
</joint>
```

---

### SCENARIO 6: IMU On Side - X-axis Up (90° Pitch)

**Diagnostic Shows:**
- X-axis: +9.8 m/s² (gravity) ✗
- Chip mounted vertically, components facing forward

**TF Rotation:** Roll=0°, Pitch=90°, Yaw=0°

**URDF Fix:** Add 90° pitch rotation
```xml
<joint name="base_link_to_imu_link" type="fixed">
  <parent link="base_link"/>
  <child link="imu_link"/>
  <origin xyz="0 0 0" rpy="0 1.5707963268 0"/>
</joint>
```

---

### SCENARIO 7: IMU On Side - Y-axis Up (90° Roll)

**Diagnostic Shows:**
- Y-axis: +9.8 m/s² (gravity) ✗
- Chip mounted vertically, components facing left

**TF Rotation:** Roll=90°, Pitch=0°, Yaw=0°

**URDF Fix:** Add 90° roll rotation
```xml
<joint name="base_link_to_imu_link" type="fixed">
  <parent link="base_link"/>
  <child link="imu_link"/>
  <origin xyz="0 0 0" rpy="1.5707963268 0 0"/>
</joint>
```

---

## How to Apply Fix

### 1. Identify Your Scenario
Run the diagnostic script:
```bash
bash ~/catkin_ws/src/elderly_bot/scripts/tf_verification_complete.sh
```

Look for the diagnostic output indicating which axis detects gravity and which sees motion.

### 2. Edit URDF
```bash
nano ~/catkin_ws/src/elderly_bot/urdf/elderly_bot.urdf
```

Find line 199 (base_link_to_imu_link joint) and update the `<origin>` tag with the appropriate `rpy` values from the scenario above.

### 3. Restart ROS
```bash
rosnode kill -a
killall -9 roscore rosmaster
sleep 3
roscore &
sleep 3
roslaunch elderly_bot bringup.launch
```

### 4. Verify Fix
```bash
# Check TF rotation
rosrun tf tf_echo base_link imu_link
# Should now show roll=0, pitch=0, yaw=0 (or close to zero)

# Visual verification in RViz
rviz
# Add TF display, check axes - imu_link arrows should align with base_link
```

### 5. Confirm System Still Works
```bash
bash ~/catkin_ws/src/elderly_bot/scripts/master_validator.sh
# All 4 stages should still PASS
```

---

## Rotation Angle Reference

| Degrees | Radians | Use Case |
|---------|---------|----------|
| 0° | 0 | No rotation (standard mounting) |
| 90° | 1.5707963268 | Quarter turn |
| -90° | -1.5707963268 | Quarter turn opposite |
| 180° | 3.14159265359 | Half turn (flip) |
| 270° | 4.71238898038 | Three-quarter turn |

---

## RPY Order Reminder

**URDF uses: rpy="ROLL PITCH YAW"**
- Roll = rotation around X-axis (tilt left/right)
- Pitch = rotation around Y-axis (tilt forward/back)
- Yaw = rotation around Z-axis (spin horizontal)

**Common mistake:** Confusing with static_transform_publisher which uses YPR order!

---

## Troubleshooting

**"IMU orientation still wrong after fix"**
→ URDF not reloaded, restart ROS completely (kill roscore)

**"Drift increased after rotation fix"**
→ Wrong rotation applied, verify gravity axis points up after fix

**"Map ghosting worse"**
→ IMU axes inverted, double-check sign of rotation (+ vs -)

**"Robot moves crooked"**
→ IMU axes swapped X↔Y, check yaw rotation (90° off)

---

**Quick Test After Any Fix:**
```bash
# Gravity should be on Z-axis, ~9.8 m/s²
rostopic echo -n 1 /imu/data_raw/linear_acceleration

# Expected output:
# x: ~0.0
# y: ~0.0
# z: ~9.8
```
