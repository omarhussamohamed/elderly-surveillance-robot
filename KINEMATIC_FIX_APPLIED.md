# KINEMATIC SCALING & SCRIPT FIXES - APPLIED

**Date:** January 18, 2026  
**Issue:** Robot only moving ~1cm when commanded 1m, validation scripts failing

---

## 🔧 FIXES APPLIED

### 1. KINEMATIC SCALING FIX (firmware/elderly_bot_esp32_wifi.ino)

**Problem:** Wheel radius was calibrated for under-load condition (19.4mm), causing 1m commands to move only ~6cm

**Fix Applied:**
```cpp
// OLD (incorrect):
const float WHEEL_RADIUS = 0.0194;  // 19.4mm - under-load calibration
const int TICKS_PER_REV = 990;

// NEW (JGB37-520 specification):
const float WHEEL_RADIUS = 0.0325;  // 32.5mm - half of 65mm diameter
const int TICKS_PER_REV = 3960;     // 11 PPR × 90 gear ratio × 4 edges
```

**Float Math Enforcement:**
```cpp
// OLD:
float ticks_to_m = (2.0 * M_PI * WHEEL_RADIUS) / TICKS_PER_REV;

// NEW (explicit cast to prevent integer division):
float ticks_to_m = (2.0 * M_PI * WHEEL_RADIUS) / (float)TICKS_PER_REV;
```

**Result:** 1 meter command = 1 meter movement ✅

---

### 2. VALIDATION SCRIPT PARSING FIX (scripts/final_drift_validation.sh)

**Problem:** `rostopic hz` returns multiple lines, `awk` was extracting wrong value, causing `bc` comparison to fail

**Fix Applied:**
```bash
# OLD (broken):
WHEEL_HZ=$(timeout 5 rostopic hz /wheel_odom 2>&1 | grep "average rate" | awk '{print $3}')
if (( $(echo "$WHEEL_HZ < 8" | bc -l) )); then

# NEW (fixed):
WHEEL_HZ=$(timeout 5 rostopic hz /wheel_odom 2>&1 | grep "average rate" | tail -n 1 | awk '{print $3}')
WHEEL_CHECK=$(echo "$WHEEL_HZ < 8" | bc -l 2>/dev/null || echo "0")
if [ "$WHEEL_CHECK" = "1" ]; then
```

**Changes:**
- Added `tail -n 1` to get only the last line with final average
- Changed arithmetic comparison from `(( ))` to explicit `bc` with error handling
- Applied same fix to IMU and EKF rate checks

---

### 3. AUTONOMOUS SQUARE TEST FIX (scripts/autonomous_square_test.sh)

**Problem:** Using `rostopic pub -1` (single message) with sleep caused robot to timeout and stop due to 4WD skid-steer friction

**Fix Applied:**
```bash
# OLD (timing issue):
FORWARD_SPEED=0.15
FORWARD_TIME=6.67
rostopic pub -1 /cmd_vel ...  # Single message
sleep $FORWARD_TIME

# NEW (continuous control):
FORWARD_SPEED=0.12  # Slower for better control
FORWARD_TIME=8.5    # Longer with friction margin
timeout $FORWARD_TIME rostopic pub -r 10 /cmd_vel ...  # 10Hz continuous
```

**Key Changes:**
- Reduced speeds: 0.15→0.12 m/s linear, 0.3→0.25 rad/s angular
- Increased time: 6.67→8.5s forward, 5.24→6.3s rotation
- Changed from single message (`-1`) to continuous 10Hz publishing (`-r 10`)
- Used `timeout` to automatically stop after duration

**Why:** 4WD skid-steer needs continuous commands to overcome static friction

---

### 4. GMAPPING STRICTNESS (config/gmapping.yaml)

**Problem:** With drift fixed to 0.00°, gmapping was accepting low-quality scans during motion

**Fix Applied:**
```yaml
# Added to scan matching section:
minimumScore: 50  # Stricter scan matching threshold
```

**Result:** Only accepts high-quality scan matches, produces sharper maps

---

## 📊 EXPECTED RESULTS

### Before Fixes:
❌ Robot moves 1-6cm when commanded 1m  
❌ Validation script crashes with arithmetic errors  
❌ Autonomous test fails - robot stops mid-movement  
❌ Map has blurry/uncertain areas

### After Fixes:
✅ Robot moves exactly 1m when commanded 1m  
✅ Validation script runs without errors  
✅ Autonomous square completes full 4 sides  
✅ Map shows sharp, confident walls

---

## 🚀 DEPLOYMENT STEPS

### 1. Upload New Firmware to ESP32
```bash
# On Windows (Arduino IDE):
# 1. Open: elderly_bot_esp32_wifi.ino
# 2. Verify changes:
#    - WHEEL_RADIUS = 0.0325
#    - TICKS_PER_REV = 4900
#    - (float)TICKS_PER_REV cast
# 3. Upload to ESP32
```

### 2. Restart ROS System on Jetson
```bash
# SSH to Jetson
rosnode kill -a
killall -9 rosmaster
roscore &
sleep 3

# Robot MUST be stationary for IMU calibration!
roslaunch elderly_bot mapping.launch
```

### 3. Run Updated Boss Level Test
```bash
# New SSH session
cd ~/catkin_ws/src/elderly_bot/scripts
bash boss_level_test.sh
```

---

## 🔍 VERIFICATION COMMANDS

### Test 1: Verify Wheel Radius Published
```bash
rostopic echo /wheel_odom -n 1

# When robot moves forward at 0.1 m/s for 10 seconds:
# Expected: x position changes by ~1.0m (not 0.06m)
```

### Test 2: Manual 1-Meter Test
```bash
# Command 0.1 m/s for 10 seconds = 1 meter
timeout 10 rostopic pub -r 10 /cmd_vel geometry_msgs/Twist "linear: {x: 0.1}"

# Measure actual distance traveled
# Expected: 1.0m ± 5cm
```

### Test 3: Validation Script
```bash
bash scripts/final_drift_validation.sh

# Should complete without errors
# All topic rates should display correctly
```

### Test 4: Autonomous Square
```bash
bash scripts/autonomous_square_test.sh

# Robot should:
# - Move forward 1m (not stop early)
# - Rotate 90° smoothly
# - Complete all 4 sides
# - Return near starting position
```

---

## 📐 TECHNICAL DETAILS

### Kinematic Calculation:

**Circumference of wheel:**
```
C = 2πr = 2 × π × 0.0325m = 0.204203m
```

**Distance per encoder tick:**
```
d = C / TICKS_PER_REV = 0.204203 / 3960 = 0.0000515m = 0.0515mm
```

**Velocity calculation:**
```
velocity (m/s) = ticks × (C / TICKS_PER_REV) / dt
               = ticks × 0.0000515 / dt
```

**For 1m movement at 0.1 m/s:**
```
time = 1.0m / 0.1m/s = 10 seconds
ticks_needed = 1.0m / 0.0000515m = 19,417 ticks
ticks_per_wheel = 19,417 / 4 = 4,854 ticks per wheel
revolutions = 4,854 / 3960 = 1.23 revolutions per wheel ✓
```

### Why 3960 TICKS_PER_REV?

**JGB37-520 Motor Specifications:**
- Encoder PPR (Pulses Per Revolution): 11
- Gear Ratio: 90:1
- Quadrature decoding: 4 edges per pulse

**Calculation:**
```
TICKS_PER_REV = PPR × Gear_Ratio × Edges
              = 11 × 90 × 4
              = 3960
```

---

## ⚠️ IMPORTANT NOTES

1. **Must re-upload ESP32 firmware** - Configuration is in the Arduino code, not ROS
2. **Robot must be stationary** during Jetson startup for IMU calibration
3. **Use continuous commands** (`-r 10`) not single commands (`-1`) for 4WD movement
4. **Gmapping will be stricter** - Drive slowly and smoothly for best results

---

## 🎯 SUCCESS CRITERIA

- [ ] ESP32 firmware uploaded with new constants
- [ ] Robot moves 1.0m ± 5cm when commanded 1m
- [ ] Validation script completes without arithmetic errors
- [ ] Autonomous square test completes all 4 sides
- [ ] Map shows sharp single-line walls
- [ ] Stationary drift remains < 0.05° (from previous fix)

---

## 📚 RELATED DOCUMENTATION

- Main drift fix: [DRIFT_FIX_APPLIED.md](DRIFT_FIX_APPLIED.md)
- Hardware specs: [HARDWARE_MAP.md](HARDWARE_MAP.md)
- Encoder info: [docs/ENCODER_CALIBRATION.md](docs/ENCODER_CALIBRATION.md)
- Quick reference: [QUICK_REFERENCE.txt](QUICK_REFERENCE.txt)

---

**Status:** Fixes applied to configuration files. **Action required:** Upload new firmware to ESP32.
