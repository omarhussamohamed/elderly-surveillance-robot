#!/bin/bash

################################################################################
# COMPLETE TF VERIFICATION SCRIPT
# Purpose: Verify laser rotation fix and diagnose IMU mounting orientation
# Date: January 18, 2026
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

print_pass() {
    echo -e "${GREEN}✓ PASS:${NC} $1"
}

print_fail() {
    echo -e "${RED}✗ FAIL:${NC} $1"
}

print_info() {
    echo -e "${CYAN}ℹ INFO:${NC} $1"
}

print_warn() {
    echo -e "${YELLOW}⚠ WARNING:${NC} $1"
}

# Check prerequisites
check_prerequisites() {
    print_header "CHECKING PREREQUISITES"
    
    if ! rostopic list &> /dev/null; then
        print_fail "ROS master not running!"
        echo "Start with: roslaunch elderly_bot bringup.launch"
        exit 1
    fi
    print_pass "ROS master running"
    
    if ! rosnode list | grep -q robot_state_publisher; then
        print_fail "robot_state_publisher not running!"
        echo "Check bringup.launch includes robot_state_publisher node"
        exit 1
    fi
    print_pass "robot_state_publisher running"
    
    if ! rostopic info /imu/data_raw &> /dev/null; then
        print_warn "IMU topic /imu/data_raw not found (may be OK if using /imu/data)"
    else
        print_pass "IMU topic available"
    fi
    
    echo ""
}

################################################################################
# PART 1: TF TREE STRUCTURE VERIFICATION
################################################################################
verify_tf_tree() {
    print_header "PART 1: TF TREE STRUCTURE VERIFICATION"
    
    print_info "Checking TF tree structure..."
    
    # Check base_footprint exists
    if timeout 2 rosrun tf tf_echo base_footprint base_link &> /dev/null; then
        print_pass "base_footprint → base_link transform exists"
    else
        print_fail "base_footprint → base_link transform missing!"
    fi
    
    # Check laser exists
    if timeout 2 rosrun tf tf_echo base_link laser &> /dev/null; then
        print_pass "base_link → laser transform exists"
    else
        print_fail "base_link → laser transform missing!"
    fi
    
    # Check imu_link exists
    if timeout 2 rosrun tf tf_echo base_link imu_link &> /dev/null; then
        print_pass "base_link → imu_link transform exists"
    else
        print_fail "base_link → imu_link transform missing!"
    fi
    
    echo ""
}

################################################################################
# PART 2: LASER ROTATION VERIFICATION
################################################################################
verify_laser_rotation() {
    print_header "PART 2: LASER ROTATION VERIFICATION (LIDAR FIX CHECK)"
    
    print_info "Checking base_link → laser transform..."
    echo ""
    
    TF_OUTPUT=$(timeout 3 rosrun tf tf_echo base_link laser 2>&1 | head -n 20)
    
    echo "$TF_OUTPUT"
    echo ""
    
    # Extract rotation values
    ROLL=$(echo "$TF_OUTPUT" | grep "in RPY (radian)" | awk '{print $5}' | tr -d '[]')
    PITCH=$(echo "$TF_OUTPUT" | grep "in RPY (radian)" | awk '{print $6}' | tr -d '[]')
    YAW=$(echo "$TF_OUTPUT" | grep "in RPY (radian)" | awk '{print $7}' | tr -d '[]')
    
    if [[ -z "$YAW" ]]; then
        print_fail "Could not extract rotation values from tf_echo"
        echo ""
        return
    fi
    
    print_info "Extracted RPY: Roll=$ROLL, Pitch=$PITCH, Yaw=$YAW"
    echo ""
    
    # Check if yaw is approximately ±180° (±3.14159 rad)
    python3 << EOF
import math

roll = float("$ROLL")
pitch = float("$PITCH")
yaw = float("$YAW")

print("${BOLD}LASER ROTATION ANALYSIS:${NC}")
print(f"  Roll:  {roll:.4f} rad = {math.degrees(roll):.2f}°")
print(f"  Pitch: {pitch:.4f} rad = {math.degrees(pitch):.2f}°")
print(f"  Yaw:   {yaw:.4f} rad = {math.degrees(yaw):.2f}°")
print("")

# Check if yaw is approximately 180° (3.14159) or -180° (-3.14159)
yaw_deg = math.degrees(yaw)
yaw_180_error = min(abs(yaw_deg - 180), abs(yaw_deg + 180))

# Check if roll is near zero (should not be 180°)
roll_deg_abs = abs(math.degrees(roll))

if yaw_180_error < 5.0 and roll_deg_abs < 10.0:
    print("${GREEN}✓ PASS: Laser has 180° YAW rotation (correct for backward-facing lidar)${NC}")
    print("${GREEN}        Roll is near zero (not using wrong axis)${NC}")
    print("${GREEN}        FIX SUCCESSFULLY APPLIED!${NC}")
elif roll_deg_abs > 170:
    print("${RED}✗ FAIL: Laser still has ~180° ROLL rotation (wrong axis!)${NC}")
    print("${RED}        Should be YAW rotation, not ROLL${NC}")
    print("${RED}        URDF fix not applied or not loaded${NC}")
elif yaw_180_error > 15.0:
    print("${YELLOW}⚠ WARNING: Yaw is not 180° (found {yaw_deg:.1f}°)${NC}")
    print("${YELLOW}           Expected ~180° for backward-facing lidar${NC}")
else:
    print("${YELLOW}⚠ MARGINAL: Close to correct but check values above${NC}")

EOF
    
    echo ""
}

################################################################################
# PART 3: IMU LINK ROTATION VERIFICATION
################################################################################
verify_imu_rotation() {
    print_header "PART 3: IMU LINK ROTATION VERIFICATION"
    
    print_info "Checking base_link → imu_link transform..."
    echo ""
    
    TF_OUTPUT=$(timeout 3 rosrun tf tf_echo base_link imu_link 2>&1 | head -n 20)
    
    echo "$TF_OUTPUT"
    echo ""
    
    # Extract rotation values
    ROLL=$(echo "$TF_OUTPUT" | grep "in RPY (radian)" | awk '{print $5}' | tr -d '[]')
    PITCH=$(echo "$TF_OUTPUT" | grep "in RPY (radian)" | awk '{print $6}' | tr -d '[]')
    YAW=$(echo "$TF_OUTPUT" | grep "in RPY (radian)" | awk '{print $7}' | tr -d '[]')
    
    if [[ -z "$YAW" ]]; then
        print_fail "Could not extract rotation values from tf_echo"
        echo ""
        return
    fi
    
    print_info "Extracted RPY: Roll=$ROLL, Pitch=$PITCH, Yaw=$YAW"
    echo ""
    
    python3 << EOF
import math

roll = float("$ROLL")
pitch = float("$PITCH")
yaw = float("$YAW")

print("${BOLD}IMU LINK ROTATION ANALYSIS:${NC}")
print(f"  Roll:  {roll:.4f} rad = {math.degrees(roll):.2f}°")
print(f"  Pitch: {pitch:.4f} rad = {math.degrees(pitch):.2f}°")
print(f"  Yaw:   {yaw:.4f} rad = {math.degrees(yaw):.2f}°")
print("")

roll_deg = math.degrees(roll)
pitch_deg = math.degrees(pitch)
yaw_deg = math.degrees(yaw)

# Check if near identity (no rotation)
if abs(roll_deg) < 5 and abs(pitch_deg) < 5 and abs(yaw_deg) < 5:
    print("${GREEN}✓ IMU mounted standard: No rotation needed${NC}")
    print("${CYAN}  IMU chip axes should align with robot: X=forward, Y=left, Z=up${NC}")
elif abs(abs(roll_deg) - 180) < 5:
    print("${RED}✗ IMU has ~180° ROLL: Mounted upside-down${NC}")
    print("${YELLOW}  Fix needed: Add rpy=\"3.14159 0 0\" to URDF base_link_to_imu_link joint${NC}")
elif abs(abs(pitch_deg) - 180) < 5:
    print("${RED}✗ IMU has ~180° PITCH: Mounted backward tilt${NC}")
    print("${YELLOW}  Fix needed: Add rpy=\"0 3.14159 0\" to URDF${NC}")
elif abs(abs(yaw_deg) - 180) < 5:
    print("${RED}✗ IMU has ~180° YAW: Rotated 180° horizontal${NC}")
    print("${YELLOW}  Fix needed: Add rpy=\"0 0 3.14159\" to URDF${NC}")
elif abs(abs(yaw_deg) - 90) < 10:
    print("${YELLOW}⚠ IMU has ~90° YAW: Rotated quarter-turn${NC}")
    print("${YELLOW}  Fix needed: Add rpy=\"0 0 1.5708\" or \"0 0 -1.5708\" to URDF${NC}")
elif abs(roll_deg) > 10 or abs(pitch_deg) > 10 or abs(yaw_deg) > 10:
    print("${YELLOW}⚠ IMU has non-standard rotation:${NC}")
    print(f"${YELLOW}  Roll={roll_deg:.1f}°, Pitch={pitch_deg:.1f}°, Yaw={yaw_deg:.1f}°${NC}")
    print("${CYAN}  Proceed to IMU mounting diagnostic for detailed analysis${NC}")
else:
    print("${CYAN}ℹ Near-zero rotation, but proceed to physical diagnostic to confirm${NC}")

EOF
    
    echo ""
}

################################################################################
# PART 4: IMU PHYSICAL MOUNTING DIAGNOSTIC (ABBREVIATED)
################################################################################
imu_physical_diagnostic() {
    print_header "PART 4: IMU PHYSICAL MOUNTING DIAGNOSTIC"
    
    # Check if IMU topic exists
    if ! rostopic info /imu/data_raw &> /dev/null && ! rostopic info /imu/data &> /dev/null; then
        print_warn "No IMU topics found - skipping physical diagnostic"
        echo "Run full imu_mounting_diagnostic.sh when IMU is available"
        echo ""
        return
    fi
    
    IMU_TOPIC="/imu/data_raw"
    if ! rostopic info /imu/data_raw &> /dev/null; then
        IMU_TOPIC="/imu/data"
    fi
    
    print_info "Using IMU topic: $IMU_TOPIC"
    echo ""
    
    # Test 1: Stationary gravity detection
    print_info "TEST 1: Stationary Gravity Detection (5 seconds)"
    print_warn "Robot must be FLAT and STATIONARY!"
    echo ""
    
    echo -e "${YELLOW}Sampling accelerometer...${NC}"
    ACCEL_DATA=$(timeout 5 rostopic echo -n 20 $IMU_TOPIC/linear_acceleration 2>&1)
    
    if [[ -z "$ACCEL_DATA" ]]; then
        print_fail "Could not read IMU accelerometer data"
        echo ""
        return
    fi
    
    ACCEL_X=$(echo "$ACCEL_DATA" | grep "x:" | awk '{sum+=$2; count++} END {print sum/count}')
    ACCEL_Y=$(echo "$ACCEL_DATA" | grep "y:" | awk '{sum+=$2; count++} END {print sum/count}')
    ACCEL_Z=$(echo "$ACCEL_DATA" | grep "z:" | awk '{sum+=$2; count++} END {print sum/count}')
    
    echo "Accelerometer readings (stationary):"
    echo "  X: ${ACCEL_X} m/s²"
    echo "  Y: ${ACCEL_Y} m/s²"
    echo "  Z: ${ACCEL_Z} m/s²"
    echo ""
    
    python3 << EOF
import math

accel_x = float("$ACCEL_X")
accel_y = float("$ACCEL_Y")
accel_z = float("$ACCEL_Z")

# Find which axis has gravity (closest to ±9.8 m/s²)
axes = [('X', accel_x), ('Y', accel_y), ('Z', accel_z)]
gravity_axis = max(axes, key=lambda x: abs(abs(x[1]) - 9.8))

print("${BOLD}GRAVITY DETECTION:${NC}")
print(f"  Axis with gravity: IMU {gravity_axis[0]}-axis = {gravity_axis[1]:.2f} m/s²")

if gravity_axis[0] == 'Z' and gravity_axis[1] > 8.0:
    print("${GREEN}✓ IMU Z-axis points UP (+9.8) - CORRECT orientation${NC}")
elif gravity_axis[0] == 'Z' and gravity_axis[1] < -8.0:
    print("${RED}✗ IMU Z-axis points DOWN (-9.8) - UPSIDE-DOWN${NC}")
    print("${YELLOW}  Fix: Add rpy=\"3.14159 0 0\" (180° roll) to URDF imu_link joint${NC}")
elif gravity_axis[0] == 'X':
    print("${RED}✗ IMU X-axis detects gravity - Mounted on SIDE (90° pitch)${NC}")
    if gravity_axis[1] > 0:
        print("${YELLOW}  Fix: Add rpy=\"0 -1.5708 0\" (-90° pitch) to URDF${NC}")
    else:
        print("${YELLOW}  Fix: Add rpy=\"0 1.5708 0\" (+90° pitch) to URDF${NC}")
elif gravity_axis[0] == 'Y':
    print("${RED}✗ IMU Y-axis detects gravity - Mounted on SIDE (90° roll)${NC}")
    if gravity_axis[1] > 0:
        print("${YELLOW}  Fix: Add rpy=\"-1.5708 0 0\" (-90° roll) to URDF${NC}")
    else:
        print("${YELLOW}  Fix: Add rpy=\"1.5708 0 0\" (+90° roll) to URDF${NC}")

print("")
print("${CYAN}ℹ For detailed forward/left motion tests, run:${NC}")
print("${CYAN}  bash ~/catkin_ws/src/elderly_bot/scripts/imu_mounting_diagnostic.sh${NC}")

EOF
    
    echo ""
}

################################################################################
# PART 5: SUMMARY & RECOMMENDATIONS
################################################################################
print_summary() {
    print_header "VERIFICATION SUMMARY & RECOMMENDATIONS"
    
    echo -e "${BOLD}TF TREE STATUS:${NC}"
    echo "  1. Laser rotation: Check output above for 180° yaw (correct) vs 180° roll (wrong)"
    echo "  2. IMU rotation: Check if identity (no rotation) or has unexpected angles"
    echo ""
    
    echo -e "${BOLD}NEXT STEPS:${NC}"
    echo ""
    echo "IF LASER SHOWS 180° YAW:"
    echo "  ${GREEN}✓ Lidar fix successfully applied${NC}"
    echo "  → Proceed to IMU fix if needed"
    echo ""
    echo "IF LASER STILL SHOWS 180° ROLL:"
    echo "  ${RED}✗ URDF changes not loaded${NC}"
    echo "  → Check urdf/elderly_bot.urdf line 169 shows: rpy=\"0 0 3.14159\""
    echo "  → Restart: rosnode kill -a && roslaunch elderly_bot bringup.launch"
    echo ""
    echo "IF IMU HAS NON-ZERO ROTATION:"
    echo "  ${YELLOW}⚠ IMU mounted at angle${NC}"
    echo "  → Edit urdf/elderly_bot.urdf line 199 (base_link_to_imu_link joint)"
    echo "  → Add/update: <origin xyz=\"0 0 0\" rpy=\"ROLL PITCH YAW\"/>"
    echo "  → Use rotation values from diagnostic output above"
    echo "  → Restart ROS after editing"
    echo ""
    echo "IF ALL ROTATIONS CORRECT:"
    echo "  ${GREEN}✓ All TF frames aligned${NC}"
    echo "  → Verify in RViz: TF display, all same-color arrows parallel"
    echo "  → Run master_validator.sh to confirm system functionality"
    echo "  → Proceed to mapping test"
    echo ""
    
    echo -e "${BOLD}QUICK RVIZ VERIFICATION:${NC}"
    echo "  1. Open RViz: rviz"
    echo "  2. Add → TF, check 'Show Axes', set Marker Scale = 0.3"
    echo "  3. Expected:"
    echo "     • base_link RED arrow → forward"
    echo "     • laser RED arrow → backward (opposite, 180° from base_link)"
    echo "     • imu_link RED arrow → same direction as base_link"
    echo "     • All GREEN arrows parallel (Y-axis, left)"
    echo "     • All BLUE arrows parallel (Z-axis, up)"
    echo ""
}

################################################################################
# MAIN EXECUTION
################################################################################
main() {
    clear
    print_header "COMPLETE TF VERIFICATION - LASER & IMU ALIGNMENT"
    echo ""
    echo "This script verifies:"
    echo "  1. TF tree structure (robot_state_publisher working)"
    echo "  2. Laser rotation fix (180° yaw, not roll)"
    echo "  3. IMU mounting orientation (detect if rotated)"
    echo "  4. Physical IMU axis alignment with robot frame"
    echo ""
    echo "Estimated time: 30 seconds"
    echo ""
    
    check_prerequisites
    verify_tf_tree
    verify_laser_rotation
    verify_imu_rotation
    imu_physical_diagnostic
    print_summary
    
    echo "Verification complete. Results saved to /tmp/tf_verification_$(date +%Y%m%d_%H%M%S).log"
    echo ""
}

# Run main function and log output
main 2>&1 | tee /tmp/tf_verification_$(date +%Y%m%d_%H%M%S).log
