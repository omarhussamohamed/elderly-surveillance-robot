#!/bin/bash

################################################################################
# MASTER SYSTEM VALIDATION SCRIPT
# Purpose: Automated sequential battery test for drift fixes, kinematic scaling,
#          and communication stability after TICKS_PER_REV=3960 correction
# Date: January 18, 2026
# Prerequisites: roslaunch elderly_bot bringup.launch (running)
#                Robot on flat floor with 2m clearance
################################################################################

set -e  # Exit on error

# Color codes for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
CYAN='\033[0;36m'
NC='\033[0m' # No Color
BOLD='\033[1m'

# Test results storage
STAGE1_RESULT="PENDING"
STAGE2_RESULT="PENDING"
STAGE3_RESULT="PENDING"
STAGE4_RESULT="PENDING"

WHEEL_ODOM_HZ=0
FILTERED_ODOM_HZ=0
IMU_DATA_HZ=0
DRIFT_DEGREES=0
ODOM_DISTANCE=0
ROTATION_COMPLETE=0

# Utility functions
print_header() {
    echo -e "${CYAN}${BOLD}"
    echo "════════════════════════════════════════════════════════════════"
    echo "  $1"
    echo "════════════════════════════════════════════════════════════════"
    echo -e "${NC}"
}

print_stage() {
    echo -e "${BLUE}${BOLD}[$1]${NC} $2"
}

print_pass() {
    echo -e "${GREEN}✓ PASS:${NC} $1"
}

print_fail() {
    echo -e "${RED}✗ FAIL:${NC} $1"
}

print_warn() {
    echo -e "${YELLOW}⚠ WARNING:${NC} $1"
}

print_info() {
    echo -e "${CYAN}ℹ INFO:${NC} $1"
}

# Quaternion to Yaw conversion
calculate_yaw() {
    local x=$1
    local y=$2
    local z=$3
    local w=$4
    
    python3 << EOF
import math
x, y, z, w = $x, $y, $z, $w
yaw_rad = math.atan2(2.0*(w*z + x*y), 1.0 - 2.0*(y*y + z*z))
yaw_deg = math.degrees(yaw_rad)
print(f"{yaw_deg:.6f}")
EOF
}

# Check if ROS is running
check_ros_running() {
    print_info "Checking ROS system status..."
    if ! rostopic list &> /dev/null; then
        print_fail "ROS master not running! Start with: roslaunch elderly_bot bringup.launch"
        exit 1
    fi
    print_pass "ROS master is running"
}

# Cleanup function
cleanup() {
    print_info "Stopping robot..."
    rostopic pub -1 /cmd_vel geometry_msgs/Twist "linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}" &> /dev/null || true
    sleep 1
}

trap cleanup EXIT

################################################################################
# STAGE 1: COMMUNICATION & SYNC AUDIT
################################################################################
stage1_communication_audit() {
    print_header "STAGE 1: COMMUNICATION & SYNC AUDIT"
    print_stage "1/4" "Monitoring topic frequencies for 10 seconds..."
    
    # Test /wheel_odom
    print_info "Testing /wheel_odom frequency..."
    WHEEL_OUTPUT=$(timeout 10 rostopic hz /wheel_odom 2>&1 | grep "average rate" | tail -n 1 || echo "ERROR")
    if [[ "$WHEEL_OUTPUT" == *"ERROR"* ]] || [[ -z "$WHEEL_OUTPUT" ]]; then
        print_fail "/wheel_odom topic not publishing"
        WHEEL_ODOM_HZ=0
        STAGE1_RESULT="FAIL"
    else
        WHEEL_ODOM_HZ=$(echo "$WHEEL_OUTPUT" | awk '{print $3}')
        if (( $(echo "$WHEEL_ODOM_HZ >= 8.0" | bc -l) )); then
            print_pass "/wheel_odom: ${WHEEL_ODOM_HZ} Hz"
        else
            print_fail "/wheel_odom: ${WHEEL_ODOM_HZ} Hz (expected >8 Hz)"
            STAGE1_RESULT="FAIL"
        fi
    fi
    
    # Test /odometry/filtered
    print_info "Testing /odometry/filtered frequency..."
    FILTERED_OUTPUT=$(timeout 10 rostopic hz /odometry/filtered 2>&1 | grep "average rate" | tail -n 1 || echo "ERROR")
    if [[ "$FILTERED_OUTPUT" == *"ERROR"* ]] || [[ -z "$FILTERED_OUTPUT" ]]; then
        print_fail "/odometry/filtered topic not publishing"
        FILTERED_ODOM_HZ=0
        STAGE1_RESULT="FAIL"
    else
        FILTERED_ODOM_HZ=$(echo "$FILTERED_OUTPUT" | awk '{print $3}')
        if (( $(echo "$FILTERED_ODOM_HZ >= 40.0" | bc -l) )); then
            print_pass "/odometry/filtered: ${FILTERED_ODOM_HZ} Hz"
        else
            print_fail "/odometry/filtered: ${FILTERED_ODOM_HZ} Hz (expected >40 Hz)"
            STAGE1_RESULT="FAIL"
        fi
    fi
    
    # Test /imu/data
    print_info "Testing /imu/data frequency..."
    IMU_OUTPUT=$(timeout 10 rostopic hz /imu/data 2>&1 | grep "average rate" | tail -n 1 || echo "ERROR")
    if [[ "$IMU_OUTPUT" == *"ERROR"* ]] || [[ -z "$IMU_OUTPUT" ]]; then
        print_warn "/imu/data topic not publishing (optional)"
        IMU_DATA_HZ=0
    else
        IMU_DATA_HZ=$(echo "$IMU_OUTPUT" | awk '{print $3}')
        print_pass "/imu/data: ${IMU_DATA_HZ} Hz"
    fi
    
    # Final Stage 1 verdict
    if [[ "$STAGE1_RESULT" != "FAIL" ]]; then
        STAGE1_RESULT="PASS"
        print_pass "Stage 1 Complete: Communication channels healthy"
    else
        print_fail "Stage 1 Failed: Check WiFi/rosserial connection"
    fi
    
    echo ""
    sleep 2
}

################################################################################
# STAGE 2: 3-MINUTE STATIONARY DRIFT STRESS TEST
################################################################################
stage2_drift_test() {
    print_header "STAGE 2: 3-MINUTE STATIONARY DRIFT STRESS TEST"
    print_stage "2/4" "Robot MUST remain completely stationary for 180 seconds"
    
    print_warn "DO NOT TOUCH THE ROBOT for the next 3 minutes!"
    echo -e "${YELLOW}Press ENTER when robot is stationary and ready...${NC}"
    read -r
    
    print_info "Recording starting yaw orientation..."
    
    # Get starting orientation
    START_QUAT=$(timeout 5 rostopic echo -n 1 /odometry/filtered/pose/pose/orientation 2>&1)
    if [[ -z "$START_QUAT" ]]; then
        print_fail "Could not read /odometry/filtered"
        STAGE2_RESULT="FAIL"
        DRIFT_DEGREES=999
        echo ""
        return
    fi
    
    START_X=$(echo "$START_QUAT" | grep "x:" | awk '{print $2}')
    START_Y=$(echo "$START_QUAT" | grep "y:" | awk '{print $2}')
    START_Z=$(echo "$START_QUAT" | grep "z:" | awk '{print $2}')
    START_W=$(echo "$START_QUAT" | grep "w:" | awk '{print $2}')
    
    START_YAW=$(calculate_yaw "$START_X" "$START_Y" "$START_Z" "$START_W")
    print_pass "Starting Yaw: ${START_YAW}°"
    
    # Wait 180 seconds with countdown
    print_info "Monitoring drift for 180 seconds..."
    for ((i=180; i>0; i-=10)); do
        echo -ne "${CYAN}Time remaining: ${i}s...${NC}\r"
        sleep 10
    done
    echo -ne '\n'
    
    # Get ending orientation
    print_info "Recording final yaw orientation..."
    END_QUAT=$(timeout 5 rostopic echo -n 1 /odometry/filtered/pose/pose/orientation 2>&1)
    if [[ -z "$END_QUAT" ]]; then
        print_fail "Could not read /odometry/filtered at end"
        STAGE2_RESULT="FAIL"
        DRIFT_DEGREES=999
        echo ""
        return
    fi
    
    END_X=$(echo "$END_QUAT" | grep "x:" | awk '{print $2}')
    END_Y=$(echo "$END_QUAT" | grep "y:" | awk '{print $2}')
    END_Z=$(echo "$END_QUAT" | grep "z:" | awk '{print $2}')
    END_W=$(echo "$END_QUAT" | grep "w:" | awk '{print $2}')
    
    END_YAW=$(calculate_yaw "$END_X" "$END_Y" "$END_Z" "$END_W")
    print_pass "Ending Yaw: ${END_YAW}°"
    
    # Calculate drift
    DRIFT_DEGREES=$(python3 << EOF
start = $START_YAW
end = $END_YAW
drift = end - start
print(f"{drift:.6f}")
EOF
    )
    
    DRIFT_ABS=$(python3 << EOF
import math
drift = abs($DRIFT_DEGREES)
print(f"{drift:.6f}")
EOF
    )
    
    echo ""
    print_info "Total Drift: ${DRIFT_DEGREES}° (absolute: ${DRIFT_ABS}°)"
    
    # Pass/Fail criteria: < 0.1°
    if (( $(echo "$DRIFT_ABS < 0.1" | bc -l) )); then
        STAGE2_RESULT="PASS"
        print_pass "Stage 2 Complete: Drift within acceptable limits (<0.1°)"
    else
        STAGE2_RESULT="FAIL"
        print_fail "Stage 2 Failed: Excessive drift (${DRIFT_ABS}° >= 0.1°)"
        print_info "Check: EKF yaw fusion, Madgwick zeta=0.01, IMU calibration"
    fi
    
    echo ""
    sleep 2
}

################################################################################
# STAGE 3: GOLDEN METER LINEAR SCALING TEST
################################################################################
stage3_linear_test() {
    print_header "STAGE 3: GOLDEN METER LINEAR SCALING TEST"
    print_stage "3/4" "Commanding robot to move exactly 1.0 meter forward"
    
    print_warn "SETUP REQUIRED:"
    echo "  1. Mark robot's current position with tape"
    echo "  2. Ensure 2m clear path ahead"
    echo "  3. Have tape measure ready"
    echo -e "${YELLOW}Press ENTER when ready...${NC}"
    read -r
    
    # Calculate duration for 1.0m at 0.12 m/s = 8.33 seconds
    DURATION=8.33
    VELOCITY=0.12
    EXPECTED_DISTANCE=1.0
    
    print_info "Commanding: linear.x = ${VELOCITY} m/s for ${DURATION} seconds"
    print_info "Expected distance: ${EXPECTED_DISTANCE} m"
    
    # Record starting odometry
    START_ODOM=$(timeout 5 rostopic echo -n 1 /odometry/filtered/pose/pose/position 2>&1)
    if [[ -z "$START_ODOM" ]]; then
        print_fail "Could not read /odometry/filtered"
        STAGE3_RESULT="FAIL"
        echo ""
        return
    fi
    
    START_X=$(echo "$START_ODOM" | grep "x:" | awk '{print $2}')
    START_Y=$(echo "$START_ODOM" | grep "y:" | awk '{print $2}')
    
    # Execute movement
    print_info "Robot moving NOW..."
    timeout $(echo "$DURATION + 1" | bc) rostopic pub -r 10 /cmd_vel geometry_msgs/Twist \
        "linear: {x: ${VELOCITY}, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}" &> /dev/null
    
    # Stop robot explicitly
    rostopic pub -1 /cmd_vel geometry_msgs/Twist \
        "linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}" &> /dev/null
    
    sleep 2
    
    # Record ending odometry
    END_ODOM=$(timeout 5 rostopic echo -n 1 /odometry/filtered/pose/pose/position 2>&1)
    if [[ -z "$END_ODOM" ]]; then
        print_fail "Could not read /odometry/filtered at end"
        STAGE3_RESULT="FAIL"
        echo ""
        return
    fi
    
    END_X=$(echo "$END_ODOM" | grep "x:" | awk '{print $2}')
    END_Y=$(echo "$END_ODOM" | grep "y:" | awk '{print $2}')
    
    # Calculate odometry distance
    ODOM_DISTANCE=$(python3 << EOF
import math
dx = $END_X - $START_X
dy = $END_Y - $START_Y
distance = math.sqrt(dx*dx + dy*dy)
print(f"{distance:.4f}")
EOF
    )
    
    ODOM_DISTANCE_CM=$(python3 << EOF
distance_m = $ODOM_DISTANCE
distance_cm = distance_m * 100
print(f"{distance_cm:.1f}")
EOF
    )
    
    print_pass "Odometry reported distance: ${ODOM_DISTANCE} m (${ODOM_DISTANCE_CM} cm)"
    
    # Get user measurement
    echo ""
    print_warn "MANUAL MEASUREMENT REQUIRED:"
    echo -e "${YELLOW}Measure the physical distance from start tape to robot center (in cm):${NC}"
    read -r PHYSICAL_DISTANCE_CM
    
    if [[ ! "$PHYSICAL_DISTANCE_CM" =~ ^[0-9]+\.?[0-9]*$ ]]; then
        print_fail "Invalid input. Please enter a number."
        STAGE3_RESULT="FAIL"
        echo ""
        return
    fi
    
    PHYSICAL_DISTANCE_M=$(python3 << EOF
cm = $PHYSICAL_DISTANCE_CM
meters = cm / 100.0
print(f"{meters:.4f}")
EOF
    )
    
    ERROR_CM=$(python3 << EOF
import math
odom = $ODOM_DISTANCE * 100
physical = $PHYSICAL_DISTANCE_CM
error = abs(odom - physical)
print(f"{error:.1f}")
EOF
    )
    
    ERROR_PERCENT=$(python3 << EOF
import math
odom = $ODOM_DISTANCE
physical = $PHYSICAL_DISTANCE_M
if physical > 0:
    error_pct = abs(odom - physical) / physical * 100
    print(f"{error_pct:.1f}")
else:
    print("0.0")
EOF
    )
    
    print_info "Physical distance: ${PHYSICAL_DISTANCE_CM} cm (${PHYSICAL_DISTANCE_M} m)"
    print_info "Odometry vs Physical error: ${ERROR_CM} cm (${ERROR_PERCENT}%)"
    
    # Pass criteria: within 2cm (98-102cm for 1m command)
    if (( $(echo "$ERROR_CM <= 2.0" | bc -l) )); then
        STAGE3_RESULT="PASS"
        print_pass "Stage 3 Complete: Linear scaling accurate (±2cm)"
    elif (( $(echo "$ERROR_CM <= 5.0" | bc -l) )); then
        STAGE3_RESULT="WARN"
        print_warn "Stage 3 Marginal: Error ${ERROR_CM}cm acceptable but not ideal"
    else
        STAGE3_RESULT="FAIL"
        print_fail "Stage 3 Failed: Excessive error (${ERROR_CM}cm > 5cm)"
        
        # Diagnostic help
        if (( $(echo "$PHYSICAL_DISTANCE_CM < 20.0" | bc -l) )); then
            print_info "Robot moved ~${PHYSICAL_DISTANCE_CM}cm: Firmware likely not uploaded (TICKS_PER_REV still wrong)"
        elif (( $(echo "$PHYSICAL_DISTANCE_CM < 90.0" | bc -l) )); then
            print_info "Robot under-traveled: TICKS_PER_REV may be too high or wheel slippage"
        elif (( $(echo "$PHYSICAL_DISTANCE_CM > 110.0" | bc -l) )); then
            print_info "Robot over-traveled: TICKS_PER_REV may be too low"
        fi
    fi
    
    echo ""
    sleep 2
}

################################################################################
# STAGE 4: 360° COMPASS ROTATION TEST
################################################################################
stage4_rotation_test() {
    print_header "STAGE 4: 360° COMPASS ROTATION TEST"
    print_stage "4/4" "Commanding robot to rotate exactly 360°"
    
    print_warn "SETUP REQUIRED:"
    echo "  1. Mark robot's front orientation with tape arrow"
    echo "  2. Ensure 1m clearance around robot"
    echo "  3. Prepare to observe if it returns to starting orientation"
    echo -e "${YELLOW}Press ENTER when ready...${NC}"
    read -r
    
    # Record starting orientation
    print_info "Recording starting orientation..."
    START_QUAT=$(timeout 5 rostopic echo -n 1 /odometry/filtered/pose/pose/orientation 2>&1)
    if [[ -z "$START_QUAT" ]]; then
        print_fail "Could not read /odometry/filtered"
        STAGE4_RESULT="FAIL"
        echo ""
        return
    fi
    
    START_X=$(echo "$START_QUAT" | grep "x:" | awk '{print $2}')
    START_Y=$(echo "$START_QUAT" | grep "y:" | awk '{print $2}')
    START_Z=$(echo "$START_QUAT" | grep "z:" | awk '{print $2}')
    START_W=$(echo "$START_QUAT" | grep "w:" | awk '{print $2}')
    
    START_YAW=$(calculate_yaw "$START_X" "$START_Y" "$START_Z" "$START_W")
    print_pass "Starting Yaw: ${START_YAW}°"
    
    # Calculate rotation duration
    # 360° = 2π radians at 0.25 rad/s = 25.13 seconds
    ANGULAR_VEL=0.25
    TARGET_ROTATION=6.283185307  # 2π radians
    DURATION=25.13
    
    print_info "Commanding: angular.z = ${ANGULAR_VEL} rad/s for ${DURATION} seconds"
    print_info "Target rotation: 360° (${TARGET_ROTATION} radians)"
    
    # Execute rotation
    print_info "Robot rotating NOW..."
    timeout $(echo "$DURATION + 1" | bc) rostopic pub -r 10 /cmd_vel geometry_msgs/Twist \
        "linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: ${ANGULAR_VEL}}" &> /dev/null
    
    # Stop robot explicitly
    rostopic pub -1 /cmd_vel geometry_msgs/Twist \
        "linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}" &> /dev/null
    
    sleep 2
    
    # Record ending orientation
    print_info "Recording final orientation..."
    END_QUAT=$(timeout 5 rostopic echo -n 1 /odometry/filtered/pose/pose/orientation 2>&1)
    if [[ -z "$END_QUAT" ]]; then
        print_fail "Could not read /odometry/filtered at end"
        STAGE4_RESULT="FAIL"
        echo ""
        return
    fi
    
    END_X=$(echo "$END_QUAT" | grep "x:" | awk '{print $2}')
    END_Y=$(echo "$END_QUAT" | grep "y:" | awk '{print $2}')
    END_Z=$(echo "$END_QUAT" | grep "z:" | awk '{print $2}')
    END_W=$(echo "$END_QUAT" | grep "w:" | awk '{print $2}')
    
    END_YAW=$(calculate_yaw "$END_X" "$END_Y" "$END_Z" "$END_W")
    print_pass "Ending Yaw: ${END_YAW}°"
    
    # Calculate total rotation
    ROTATION_DIFF=$(python3 << EOF
import math
start = $START_YAW
end = $END_YAW
diff = end - start
# Normalize to -180 to 180
while diff > 180:
    diff -= 360
while diff < -180:
    diff += 360
print(f"{diff:.4f}")
EOF
    )
    
    ROTATION_ERROR=$(python3 << EOF
import math
diff = $ROTATION_DIFF
# Expected is 0° (full circle back to start)
error = abs(diff)
print(f"{error:.4f}")
EOF
    )
    
    print_info "Orientation change: ${ROTATION_DIFF}° (error from 0°: ${ROTATION_ERROR}°)"
    
    # Get user visual confirmation
    echo ""
    print_warn "VISUAL CONFIRMATION:"
    echo -e "${YELLOW}Does the robot face its ORIGINAL direction? (y/n):${NC}"
    read -r USER_CONFIRM
    
    # Pass criteria: < 5° error AND user confirms
    if [[ "$USER_CONFIRM" =~ ^[Yy]$ ]] && (( $(echo "$ROTATION_ERROR <= 5.0" | bc -l) )); then
        STAGE4_RESULT="PASS"
        print_pass "Stage 4 Complete: Angular scaling accurate (±5°)"
        ROTATION_COMPLETE=1
    elif [[ "$USER_CONFIRM" =~ ^[Yy]$ ]] && (( $(echo "$ROTATION_ERROR <= 15.0" | bc -l) )); then
        STAGE4_RESULT="WARN"
        print_warn "Stage 4 Marginal: Visual OK but odometry error ${ROTATION_ERROR}°"
        ROTATION_COMPLETE=1
    else
        STAGE4_RESULT="FAIL"
        print_fail "Stage 4 Failed: Rotation error ${ROTATION_ERROR}° or visual mismatch"
        ROTATION_COMPLETE=0
        
        # Diagnostic help
        if (( $(echo "$ROTATION_ERROR > 20.0" | bc -l) )); then
            print_info "Large rotation error: Check WHEEL_TRACK calibration (currently 0.26m)"
        fi
    fi
    
    echo ""
    sleep 2
}

################################################################################
# FINAL DASHBOARD
################################################################################
print_dashboard() {
    print_header "MASTER VALIDATION DASHBOARD"
    
    # Stage 1
    echo -e "${BOLD}STAGE 1: Communication & Sync Audit${NC}"
    if [[ "$STAGE1_RESULT" == "PASS" ]]; then
        echo -e "  Status: ${GREEN}✓ PASS${NC}"
    else
        echo -e "  Status: ${RED}✗ FAIL${NC}"
    fi
    echo "  /wheel_odom: ${WHEEL_ODOM_HZ} Hz (target: >8 Hz)"
    echo "  /odometry/filtered: ${FILTERED_ODOM_HZ} Hz (target: >40 Hz)"
    echo "  /imu/data: ${IMU_DATA_HZ} Hz (informational)"
    echo ""
    
    # Stage 2
    echo -e "${BOLD}STAGE 2: 3-Minute Stationary Drift Test${NC}"
    if [[ "$STAGE2_RESULT" == "PASS" ]]; then
        echo -e "  Status: ${GREEN}✓ PASS${NC}"
    else
        echo -e "  Status: ${RED}✗ FAIL${NC}"
    fi
    echo "  Total Drift: ${DRIFT_DEGREES}° (target: <0.1°)"
    echo ""
    
    # Stage 3
    echo -e "${BOLD}STAGE 3: Golden Meter Linear Scaling Test${NC}"
    if [[ "$STAGE3_RESULT" == "PASS" ]]; then
        echo -e "  Status: ${GREEN}✓ PASS${NC}"
    elif [[ "$STAGE3_RESULT" == "WARN" ]]; then
        echo -e "  Status: ${YELLOW}⚠ MARGINAL${NC}"
    else
        echo -e "  Status: ${RED}✗ FAIL${NC}"
    fi
    echo "  Odometry Distance: ${ODOM_DISTANCE} m"
    echo ""
    
    # Stage 4
    echo -e "${BOLD}STAGE 4: 360° Compass Rotation Test${NC}"
    if [[ "$STAGE4_RESULT" == "PASS" ]]; then
        echo -e "  Status: ${GREEN}✓ PASS${NC}"
    elif [[ "$STAGE4_RESULT" == "WARN" ]]; then
        echo -e "  Status: ${YELLOW}⚠ MARGINAL${NC}"
    else
        echo -e "  Status: ${RED}✗ FAIL${NC}"
    fi
    if [[ "$ROTATION_COMPLETE" == "1" ]]; then
        echo "  Rotation Complete: Yes"
    else
        echo "  Rotation Complete: No"
    fi
    echo ""
    
    # Overall Status
    echo "════════════════════════════════════════════════════════════════"
    echo -e "${BOLD}OVERALL SYSTEM STATUS:${NC}"
    
    PASS_COUNT=0
    WARN_COUNT=0
    FAIL_COUNT=0
    
    for result in "$STAGE1_RESULT" "$STAGE2_RESULT" "$STAGE3_RESULT" "$STAGE4_RESULT"; do
        if [[ "$result" == "PASS" ]]; then
            ((PASS_COUNT++))
        elif [[ "$result" == "WARN" ]]; then
            ((WARN_COUNT++))
        elif [[ "$result" == "FAIL" ]]; then
            ((FAIL_COUNT++))
        fi
    done
    
    if [[ $FAIL_COUNT -eq 0 ]] && [[ $WARN_COUNT -eq 0 ]]; then
        echo -e "${GREEN}${BOLD}★ ALL TESTS PASSED ★${NC}"
        echo -e "${GREEN}System is PRODUCTION READY for autonomous navigation${NC}"
    elif [[ $FAIL_COUNT -eq 0 ]] && [[ $WARN_COUNT -gt 0 ]]; then
        echo -e "${YELLOW}${BOLD}⚠ MARGINAL - ACCEPTABLE${NC}"
        echo -e "${YELLOW}System functional but has ${WARN_COUNT} marginal result(s)${NC}"
    else
        echo -e "${RED}${BOLD}✗ VALIDATION FAILED${NC}"
        echo -e "${RED}${FAIL_COUNT} test(s) failed. Review logs and troubleshoot.${NC}"
        echo ""
        echo "Common Issues:"
        echo "  - Stage 1 fail: Check WiFi, rosserial connection, tcp_nodelay"
        echo "  - Stage 2 fail: Check EKF yaw fusion, Madgwick zeta=0.01"
        echo "  - Stage 3 fail: Verify firmware uploaded with TICKS_PER_REV=3960"
        echo "  - Stage 4 fail: Check WHEEL_TRACK calibration (0.26m)"
    fi
    
    echo "════════════════════════════════════════════════════════════════"
    echo ""
    echo "Test completed at: $(date)"
    echo "Results saved to: /tmp/master_validator_results.txt"
    
    # Save results to file
    {
        echo "Master Validation Results - $(date)"
        echo "=========================================="
        echo "Stage 1: $STAGE1_RESULT - Wheel: ${WHEEL_ODOM_HZ}Hz, Filtered: ${FILTERED_ODOM_HZ}Hz"
        echo "Stage 2: $STAGE2_RESULT - Drift: ${DRIFT_DEGREES}°"
        echo "Stage 3: $STAGE3_RESULT - Odom Distance: ${ODOM_DISTANCE}m"
        echo "Stage 4: $STAGE4_RESULT - Rotation: ${ROTATION_COMPLETE}"
        echo "Overall: PASS=$PASS_COUNT, WARN=$WARN_COUNT, FAIL=$FAIL_COUNT"
    } > /tmp/master_validator_results.txt
}

################################################################################
# MAIN EXECUTION
################################################################################
main() {
    clear
    print_header "MASTER SYSTEM VALIDATOR v1.0"
    echo "This script will perform 4 sequential validation stages:"
    echo "  1. Communication & Sync Audit (10s)"
    echo "  2. 3-Minute Stationary Drift Test (180s)"
    echo "  3. Golden Meter Linear Scaling Test (manual measurement)"
    echo "  4. 360° Compass Rotation Test (visual confirmation)"
    echo ""
    echo "Total estimated time: ~8 minutes"
    echo ""
    print_warn "PREREQUISITES:"
    echo "  • roslaunch elderly_bot bringup.launch is running"
    echo "  • Robot on flat floor with 2m clearance"
    echo "  • Tape measure ready"
    echo "  • Tape for marking positions"
    echo ""
    echo -e "${YELLOW}Press ENTER to begin validation...${NC}"
    read -r
    
    check_ros_running
    
    stage1_communication_audit
    stage2_drift_test
    stage3_linear_test
    stage4_rotation_test
    
    print_dashboard
}

# Run main function
main
