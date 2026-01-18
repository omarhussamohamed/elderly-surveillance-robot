#!/bin/bash

################################################################################
# IMU PHYSICAL MOUNTING DIAGNOSTIC SCRIPT
# Purpose: Determine IMU chip orientation relative to robot coordinate frame
# Date: January 18, 2026
# Usage: Place robot on level surface, run script, follow instructions
################################################################################

set -e

RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
CYAN='\033[0;36m'
NC='\033[0m'
BOLD='\033[1m'

print_header() {
    echo -e "${CYAN}${BOLD}"
    echo "════════════════════════════════════════════════════════════════"
    echo "  $1"
    echo "════════════════════════════════════════════════════════════════"
    echo -e "${NC}"
}

print_info() {
    echo -e "${CYAN}ℹ INFO:${NC} $1"
}

print_warn() {
    echo -e "${YELLOW}⚠ WARNING:${NC} $1"
}

print_pass() {
    echo -e "${GREEN}✓${NC} $1"
}

clear
print_header "IMU PHYSICAL MOUNTING DIAGNOSTIC"

echo ""
echo "This script will help determine your MPU9250 IMU chip orientation"
echo "relative to the robot's coordinate frame (REP-105)."
echo ""
echo -e "${BOLD}Robot Coordinate Convention (REP-105):${NC}"
echo "  X-axis: Points FORWARD (robot front)"
echo "  Y-axis: Points LEFT"
echo "  Z-axis: Points UP (vertical)"
echo ""
echo "We will test each axis by moving/tilting the robot and observing"
echo "which IMU axis reports the motion."
echo ""
print_warn "Prerequisites:"
echo "  • Robot on flat, level surface"
echo "  • roslaunch elderly_bot bringup.launch running"
echo "  • IMU publishing to /imu/data_raw"
echo ""
echo -e "${YELLOW}Press ENTER to begin diagnostic...${NC}"
read -r

# Check ROS is running
if ! rostopic list &> /dev/null; then
    echo -e "${RED}ERROR: ROS master not running!${NC}"
    echo "Start with: roslaunch elderly_bot bringup.launch"
    exit 1
fi

# Check IMU topic exists
if ! rostopic info /imu/data_raw &> /dev/null; then
    echo -e "${RED}ERROR: /imu/data_raw topic not found!${NC}"
    echo "Check that mpu9250_node.py is running"
    exit 1
fi

print_pass "ROS system ready"
echo ""

################################################################################
# TEST 1: STATIONARY BASELINE (GRAVITY DETECTION)
################################################################################
print_header "TEST 1: STATIONARY BASELINE - GRAVITY AXIS DETECTION"

echo "Robot should be FLAT and STATIONARY on level surface."
echo "We will detect which IMU axis sees gravity (-9.8 m/s²)."
echo ""
echo -e "${YELLOW}Press ENTER when robot is level and stationary...${NC}"
read -r

print_info "Sampling IMU accelerometer for 3 seconds..."
ACCEL_DATA=$(timeout 3 rostopic echo -n 10 /imu/data_raw/linear_acceleration 2>&1)

# Extract average values
ACCEL_X=$(echo "$ACCEL_DATA" | grep "x:" | awk '{sum+=$2; count++} END {print sum/count}')
ACCEL_Y=$(echo "$ACCEL_DATA" | grep "y:" | awk '{sum+=$2; count++} END {print sum/count}')
ACCEL_Z=$(echo "$ACCEL_DATA" | grep "z:" | awk '{sum+=$2; count++} END {print sum/count}')

echo ""
echo "Accelerometer readings (stationary, level):"
echo "  X: ${ACCEL_X} m/s²"
echo "  Y: ${ACCEL_Y} m/s²"
echo "  Z: ${ACCEL_Z} m/s²"
echo ""

# Determine which axis has -9.8 (gravity)
python3 << EOF
import math

accel_x = $ACCEL_X
accel_y = $ACCEL_Y
accel_z = $ACCEL_Z

# Find axis closest to -9.8 or +9.8 (gravity)
axes = [('X', abs(accel_x - 9.8), abs(accel_x + 9.8)),
        ('Y', abs(accel_y - 9.8), abs(accel_y + 9.8)),
        ('Z', abs(accel_z - 9.8), abs(accel_z + 9.8))]

# Find which axis is closest to ±9.8
min_diff = 999
gravity_axis = 'UNKNOWN'
gravity_sign = '?'

for axis, diff_pos, diff_neg in axes:
    if diff_pos < min_diff:
        min_diff = diff_pos
        gravity_axis = axis
        gravity_sign = '+'
    if diff_neg < min_diff:
        min_diff = diff_neg
        gravity_axis = axis
        gravity_sign = '-'

print(f"${BLUE}RESULT: IMU chip ${gravity_axis}-axis points {'UP' if gravity_sign == '+' else 'DOWN'} (detected {gravity_sign}9.8 m/s²)${NC}")
print(f"${CYAN}Expected: Z-axis should point UP (+9.8 m/s²) in REP-105${NC}")

if gravity_axis == 'Z' and gravity_sign == '+':
    print(f"${GREEN}✓ Z-axis CORRECT - IMU mounted with chip Z pointing up${NC}")
elif gravity_axis == 'Z' and gravity_sign == '-':
    print(f"${RED}✗ Z-axis INVERTED - IMU mounted upside-down${NC}")
elif gravity_axis == 'X':
    print(f"${RED}✗ X-axis detecting gravity - IMU mounted on its side (90° pitch rotation)${NC}")
elif gravity_axis == 'Y':
    print(f"${RED}✗ Y-axis detecting gravity - IMU mounted on its side (90° roll rotation)${NC}")
else:
    print(f"${RED}✗ Cannot detect gravity - check IMU readings${NC}")
EOF

echo ""
sleep 2

################################################################################
# TEST 2: FORWARD MOTION - X-AXIS DETECTION
################################################################################
print_header "TEST 2: FORWARD MOTION - ROBOT X-AXIS DETECTION"

echo "Now we will push the robot FORWARD (robot's +X direction)"
echo "and observe which IMU axis shows acceleration."
echo ""
echo -e "${BOLD}Instructions:${NC}"
echo "  1. Prepare to push robot forward smoothly (~0.5m)"
echo "  2. Press ENTER to start recording"
echo "  3. Push robot forward immediately after countdown"
echo "  4. We will detect which IMU axis sees the acceleration"
echo ""
echo -e "${YELLOW}Press ENTER when ready to record...${NC}"
read -r

print_info "Recording IMU for 5 seconds - PUSH ROBOT FORWARD NOW!"
for i in 3 2 1; do
    echo -ne "${CYAN}${i}...${NC}\r"
    sleep 1
done
echo -e "${GREEN}GO - PUSH FORWARD!${NC}"

ACCEL_FWD=$(timeout 5 rostopic echo -n 20 /imu/data_raw/linear_acceleration 2>&1)

# Find peak acceleration in each axis
ACCEL_X_MAX=$(echo "$ACCEL_FWD" | grep "x:" | awk '{print $2}' | sort -n | tail -n 1)
ACCEL_Y_MAX=$(echo "$ACCEL_FWD" | grep "y:" | awk '{print $2}' | sort -n | tail -n 1)
ACCEL_Z_MAX=$(echo "$ACCEL_FWD" | grep "z:" | awk '{print $2}' | sort -n | tail -n 1)

ACCEL_X_MIN=$(echo "$ACCEL_FWD" | grep "x:" | awk '{print $2}' | sort -n | head -n 1)
ACCEL_Y_MIN=$(echo "$ACCEL_FWD" | grep "y:" | awk '{print $2}' | sort -n | head -n 1)
ACCEL_Z_MIN=$(echo "$ACCEL_FWD" | grep "z:" | awk '{print $2}' | sort -n | head -n 1)

echo ""
echo "Peak accelerations during forward motion:"
echo "  X: max=${ACCEL_X_MAX}, min=${ACCEL_X_MIN}"
echo "  Y: max=${ACCEL_Y_MAX}, min=${ACCEL_Y_MIN}"
echo "  Z: max=${ACCEL_Z_MAX}, min=${ACCEL_Z_MIN}"
echo ""

python3 << EOF
import math

# Calculate range (motion magnitude) for each axis
x_range = abs($ACCEL_X_MAX - $ACCEL_X_MIN)
y_range = abs($ACCEL_Y_MAX - $ACCEL_Y_MIN)
z_range = abs($ACCEL_Z_MAX - $ACCEL_Z_MIN)

print(f"Acceleration ranges: X={x_range:.2f}, Y={y_range:.2f}, Z={z_range:.2f}")
print("")

# Determine which axis saw the most motion
max_axis = 'X' if x_range >= y_range and x_range >= z_range else ('Y' if y_range >= z_range else 'Z')
max_range = max(x_range, y_range, z_range)

# Determine sign (forward motion should be positive acceleration)
if max_axis == 'X':
    sign = '+' if $ACCEL_X_MAX > abs($ACCEL_X_MIN) else '-'
elif max_axis == 'Y':
    sign = '+' if $ACCEL_Y_MAX > abs($ACCEL_Y_MIN) else '-'
else:
    sign = '+' if $ACCEL_Z_MAX > abs($ACCEL_Z_MIN) else '-'

print(f"${BLUE}RESULT: Forward motion detected on IMU {max_axis}-axis ({sign}{max_range:.2f} m/s²)${NC}")
print(f"${CYAN}Expected: X-axis should show forward motion in REP-105${NC}")

if max_axis == 'X' and sign == '+':
    print(f"${GREEN}✓ X-axis CORRECT - IMU chip X points forward${NC}")
elif max_axis == 'X' and sign == '-':
    print(f"${RED}✗ X-axis INVERTED - IMU chip X points backward (180° yaw rotation)${NC}")
elif max_axis == 'Y':
    print(f"${RED}✗ Y-axis sees forward motion - IMU rotated 90° (chip axes swapped with robot axes)${NC}")
elif max_axis == 'Z':
    print(f"${RED}✗ Z-axis sees forward motion - IMU mounted vertically (90° pitch or roll)${NC}")
else:
    print(f"${YELLOW}⚠ Motion unclear - try pushing harder or check IMU connection${NC}")
EOF

echo ""
sleep 2

################################################################################
# TEST 3: LEFT MOTION - Y-AXIS DETECTION
################################################################################
print_header "TEST 3: LEFT MOTION - ROBOT Y-AXIS DETECTION"

echo "Now we will push the robot LEFT (robot's +Y direction)"
echo "and observe which IMU axis shows acceleration."
echo ""
echo -e "${BOLD}Instructions:${NC}"
echo "  1. Prepare to push robot to the LEFT smoothly (~0.3m)"
echo "  2. Press ENTER to start recording"
echo "  3. Push robot LEFT immediately after countdown"
echo ""
echo -e "${YELLOW}Press ENTER when ready to record...${NC}"
read -r

print_info "Recording IMU for 5 seconds - PUSH ROBOT LEFT NOW!"
for i in 3 2 1; do
    echo -ne "${CYAN}${i}...${NC}\r"
    sleep 1
done
echo -e "${GREEN}GO - PUSH LEFT!${NC}"

ACCEL_LEFT=$(timeout 5 rostopic echo -n 20 /imu/data_raw/linear_acceleration 2>&1)

ACCEL_X_MAX_L=$(echo "$ACCEL_LEFT" | grep "x:" | awk '{print $2}' | sort -n | tail -n 1)
ACCEL_Y_MAX_L=$(echo "$ACCEL_LEFT" | grep "y:" | awk '{print $2}' | sort -n | tail -n 1)
ACCEL_Z_MAX_L=$(echo "$ACCEL_LEFT" | grep "z:" | awk '{print $2}' | sort -n | tail -n 1)

ACCEL_X_MIN_L=$(echo "$ACCEL_LEFT" | grep "x:" | awk '{print $2}' | sort -n | head -n 1)
ACCEL_Y_MIN_L=$(echo "$ACCEL_LEFT" | grep "y:" | awk '{print $2}' | sort -n | head -n 1)
ACCEL_Z_MIN_L=$(echo "$ACCEL_LEFT" | grep "z:" | awk '{print $2}' | sort -n | head -n 1)

echo ""
echo "Peak accelerations during left motion:"
echo "  X: max=${ACCEL_X_MAX_L}, min=${ACCEL_X_MIN_L}"
echo "  Y: max=${ACCEL_Y_MAX_L}, min=${ACCEL_Y_MIN_L}"
echo "  Z: max=${ACCEL_Z_MAX_L}, min=${ACCEL_Z_MIN_L}"
echo ""

python3 << EOF
import math

x_range = abs($ACCEL_X_MAX_L - $ACCEL_X_MIN_L)
y_range = abs($ACCEL_Y_MAX_L - $ACCEL_Y_MIN_L)
z_range = abs($ACCEL_Z_MAX_L - $ACCEL_Z_MIN_L)

print(f"Acceleration ranges: X={x_range:.2f}, Y={y_range:.2f}, Z={z_range:.2f}")
print("")

max_axis = 'X' if x_range >= y_range and x_range >= z_range else ('Y' if y_range >= z_range else 'Z')

if max_axis == 'X':
    sign = '+' if $ACCEL_X_MAX_L > abs($ACCEL_X_MIN_L) else '-'
elif max_axis == 'Y':
    sign = '+' if $ACCEL_Y_MAX_L > abs($ACCEL_Y_MIN_L) else '-'
else:
    sign = '+' if $ACCEL_Z_MAX_L > abs($ACCEL_Z_MIN_L) else '-'

print(f"${BLUE}RESULT: Left motion detected on IMU {max_axis}-axis ({sign})${NC}")
print(f"${CYAN}Expected: Y-axis should show left motion in REP-105${NC}")

if max_axis == 'Y' and sign == '+':
    print(f"${GREEN}✓ Y-axis CORRECT - IMU chip Y points left${NC}")
elif max_axis == 'Y' and sign == '-':
    print(f"${RED}✗ Y-axis INVERTED - IMU chip Y points right (180° yaw or roll)${NC}")
elif max_axis == 'X':
    print(f"${RED}✗ X-axis sees left motion - IMU rotated 90° (axes swapped)${NC}")
else:
    print(f"${YELLOW}⚠ Result unclear - check IMU or try again${NC}")
EOF

echo ""
sleep 2

################################################################################
# FINAL SUMMARY
################################################################################
print_header "DIAGNOSTIC COMPLETE - SUMMARY & RECOMMENDATIONS"

echo ""
echo -e "${BOLD}Based on the tests above, determine your IMU mounting:${NC}"
echo ""
echo "1. If ALL axes correct (X forward, Y left, Z up):"
echo "   → No changes needed, IMU is perfectly aligned"
echo ""
echo "2. If Z-axis inverted (pointing down instead of up):"
echo "   → IMU is upside-down, apply 180° roll rotation"
echo "   → Add to urdf/elderly_bot.urdf:"
echo "   → <origin xyz=\"0 0 0\" rpy=\"3.14159 0 0\"/>"
echo ""
echo "3. If X-axis inverted (backward instead of forward):"
echo "   → IMU rotated 180° horizontally, apply 180° yaw"
echo "   → Add to urdf/elderly_bot.urdf:"
echo "   → <origin xyz=\"0 0 0\" rpy=\"0 0 3.14159\"/>"
echo ""
echo "4. If Y-axis inverted (right instead of left):"
echo "   → Unusual, may need 180° pitch or combined rotations"
echo ""
echo "5. If axes completely swapped (X→Y, Y→X, etc):"
echo "   → IMU mounted at 90° angle, apply appropriate rotation"
echo "   → 90° yaw: rpy=\"0 0 1.5708\""
echo "   → 90° pitch: rpy=\"0 1.5708 0\""
echo "   → 90° roll: rpy=\"1.5708 0 0\""
echo ""
echo -e "${CYAN}Next Steps:${NC}"
echo "  1. Record the test results above"
echo "  2. Update urdf/elderly_bot.urdf base_link_to_imu_link joint"
echo "  3. Restart ROS: rosnode kill -a && roslaunch elderly_bot bringup.launch"
echo "  4. Verify in RViz: all frame axes should align (red-red, green-green, blue-blue)"
echo "  5. Run master_validator.sh to confirm drift fixes still work"
echo ""
echo "Results saved to: /tmp/imu_diagnostic_$(date +%Y%m%d_%H%M%S).txt"
echo ""
