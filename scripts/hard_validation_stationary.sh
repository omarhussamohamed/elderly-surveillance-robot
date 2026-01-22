#!/bin/bash
# HARD VALIDATION: Numeric Proof of Stationary Stability
# Provides quantitative evidence that magnetometer kill-switch worked

echo "========================================"
echo "HARD VALIDATION - STATIONARY STABILITY"
echo "========================================"
echo ""
echo "This test provides NUMERIC PROOF that yaw is stable."
echo "Robot MUST be stationary for 60 seconds."
echo ""
read -p "Place robot on floor, ensure stationary, press ENTER..."

# Colors
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m'

PASS_COUNT=0
FAIL_COUNT=0

echo ""
echo "=========================================="
echo "TEST 1: TF Yaw Stability (60 seconds)"
echo "=========================================="
echo ""
echo "Measuring: odom → base_footprint yaw change"
echo "Threshold: < ±0.2° over 60 seconds"
echo ""

# Get initial TF
echo "Sampling initial TF..."
INITIAL_TF=$(timeout 2s rosrun tf tf_echo odom base_footprint 2>&1 | grep -A 1 "Rotation: axis")

if echo "$INITIAL_TF" | grep -q "Exception"; then
    echo -e "${RED}✗ CRITICAL: TF transform odom → base_footprint NOT AVAILABLE${NC}"
    echo "  Check: Is EKF running? rosnode list | grep ekf"
    exit 1
fi

# Extract initial quaternion (yaw component)
INITIAL_QZ=$(echo "$INITIAL_TF" | grep "Rotation:" | awk '{print $5}' | tr -d ',')
INITIAL_QW=$(echo "$INITIAL_TF" | grep "Rotation:" | awk '{print $6}' | tr -d ')')

if [ -z "$INITIAL_QZ" ] || [ -z "$INITIAL_QW" ]; then
    echo -e "${RED}✗ FAILURE: Could not parse initial TF quaternion${NC}"
    exit 1
fi

# Convert to yaw (degrees)
INITIAL_YAW=$(python3 -c "import math; qz=$INITIAL_QZ; qw=$INITIAL_QW; yaw=2*math.atan2(qz,qw)*180/math.pi; print(f'{yaw:.3f}')")

echo "Initial yaw: ${INITIAL_YAW}°"
echo ""
echo "Waiting 60 seconds (monitoring live)..."

# Monitor for 60 seconds (sample every 5 seconds)
for i in {1..12}; do
    sleep 5
    CURRENT_TF=$(timeout 2s rosrun tf tf_echo odom base_footprint 2>&1 | grep "Rotation:")
    CURRENT_QZ=$(echo "$CURRENT_TF" | awk '{print $5}' | tr -d ',')
    CURRENT_QW=$(echo "$CURRENT_TF" | awk '{print $6}' | tr -d ')')
    CURRENT_YAW=$(python3 -c "import math; qz=$CURRENT_QZ; qw=$CURRENT_QW; yaw=2*math.atan2(qz,qw)*180/math.pi; print(f'{yaw:.3f}')")
    echo "[$((i*5))s] Current yaw: ${CURRENT_YAW}°"
done

# Get final TF
echo ""
echo "Sampling final TF..."
FINAL_TF=$(timeout 2s rosrun tf tf_echo odom base_footprint 2>&1 | grep -A 1 "Rotation: axis")
FINAL_QZ=$(echo "$FINAL_TF" | grep "Rotation:" | awk '{print $5}' | tr -d ',')
FINAL_QW=$(echo "$FINAL_TF" | grep "Rotation:" | awk '{print $6}' | tr -d ')')
FINAL_YAW=$(python3 -c "import math; qz=$FINAL_QZ; qw=$FINAL_QW; yaw=2*math.atan2(qz,qw)*180/math.pi; print(f'{yaw:.3f}')")

echo "Final yaw: ${FINAL_YAW}°"
echo ""

# Calculate drift
YAW_DRIFT=$(python3 -c "drift=abs($FINAL_YAW - $INITIAL_YAW); print(f'{drift:.3f}')")

echo "=========================================="
echo "RESULT: TF Yaw Drift = ${YAW_DRIFT}°"
echo "=========================================="

THRESHOLD_PASS=$(python3 -c "print(1 if $YAW_DRIFT < 0.2 else 0)")

if [ "$THRESHOLD_PASS" -eq 1 ]; then
    echo -e "${GREEN}✓ PASS: Yaw drift ${YAW_DRIFT}° < 0.2° threshold${NC}"
    echo "  TF transform is STABLE"
    PASS_COUNT=$((PASS_COUNT + 1))
else
    echo -e "${RED}✗ FAIL: Yaw drift ${YAW_DRIFT}° > 0.2° threshold${NC}"
    echo "  TF transform is UNSTABLE"
    FAIL_COUNT=$((FAIL_COUNT + 1))
fi

echo ""
echo "=========================================="
echo "TEST 2: IMU Angular Velocity (60 seconds)"
echo "=========================================="
echo ""
echo "Measuring: /imu/data angular_velocity.z"
echo "Threshold: mean ≈ 0.0 rad/s (±0.01)"
echo ""

# Collect 60 seconds of angular velocity data
echo "Collecting 60 seconds of IMU data..."
timeout 60s rostopic echo /imu/data/angular_velocity/z 2>/dev/null | grep -E "^[-0-9]" > /tmp/imu_angvel_z.txt &
IMU_PID=$!

# Live monitor
for i in {1..12}; do
    sleep 5
    CURRENT_VEL=$(rostopic echo -n 1 /imu/data/angular_velocity/z 2>/dev/null || echo "N/A")
    echo "[$((i*5))s] angular_velocity.z = $CURRENT_VEL rad/s"
done

wait $IMU_PID

if [ ! -s /tmp/imu_angvel_z.txt ]; then
    echo -e "${RED}✗ FAILURE: No IMU data received${NC}"
    echo "  Check: rostopic hz /imu/data"
    FAIL_COUNT=$((FAIL_COUNT + 1))
else
    # Compute statistics
    SAMPLE_COUNT=$(wc -l < /tmp/imu_angvel_z.txt)
    MEAN_VEL=$(awk '{sum+=$1} END {printf "%.6f", sum/NR}' /tmp/imu_angvel_z.txt)
    MIN_VEL=$(sort -n /tmp/imu_angvel_z.txt | head -1)
    MAX_VEL=$(sort -n /tmp/imu_angvel_z.txt | tail -1)
    RANGE_VEL=$(python3 -c "print(f'{abs($MAX_VEL - $MIN_VEL):.6f}')")
    
    echo ""
    echo "=========================================="
    echo "RESULT: IMU Angular Velocity"
    echo "=========================================="
    echo "  Samples: $SAMPLE_COUNT"
    echo "  Mean:    $MEAN_VEL rad/s"
    echo "  Min:     $MIN_VEL rad/s"
    echo "  Max:     $MAX_VEL rad/s"
    echo "  Range:   $RANGE_VEL rad/s"
    echo ""
    
    # Check threshold
    MEAN_ABS=$(python3 -c "print(abs($MEAN_VEL))")
    MEAN_PASS=$(python3 -c "print(1 if $MEAN_ABS < 0.01 else 0)")
    
    if [ "$MEAN_PASS" -eq 1 ]; then
        echo -e "${GREEN}✓ PASS: Mean angular velocity ${MEAN_VEL} rad/s ≈ 0.0${NC}"
        echo "  Robot is NOT rotating while stationary"
        PASS_COUNT=$((PASS_COUNT + 1))
    else
        echo -e "${RED}✗ FAIL: Mean angular velocity ${MEAN_VEL} rad/s > 0.01 rad/s${NC}"
        echo "  Robot thinks it's rotating (gyro bias or hardware issue)"
        FAIL_COUNT=$((FAIL_COUNT + 1))
    fi
fi

echo ""
echo "=========================================="
echo "TEST 3: IMU Orientation Quaternion (60s)"
echo "=========================================="
echo ""
echo "Measuring: /imu/data orientation.z (was jumping -0.02 to 0.50)"
echo "Threshold: range < 0.05 (no jumps)"
echo ""

# Collect 60 seconds of orientation data
echo "Collecting 60 seconds of IMU orientation..."
timeout 60s rostopic echo /imu/data/orientation/z 2>/dev/null | grep -E "^[-0-9]" > /tmp/imu_quat_z.txt &
QUAT_PID=$!

for i in {1..12}; do
    sleep 5
    CURRENT_QZ=$(rostopic echo -n 1 /imu/data/orientation/z 2>/dev/null || echo "N/A")
    echo "[$((i*5))s] orientation.z = $CURRENT_QZ"
done

wait $QUAT_PID

if [ ! -s /tmp/imu_quat_z.txt ]; then
    echo -e "${RED}✗ FAILURE: No IMU orientation data${NC}"
    FAIL_COUNT=$((FAIL_COUNT + 1))
else
    MIN_QZ=$(sort -n /tmp/imu_quat_z.txt | head -1)
    MAX_QZ=$(sort -n /tmp/imu_quat_z.txt | tail -1)
    RANGE_QZ=$(python3 -c "print(f'{abs($MAX_QZ - $MIN_QZ):.6f}')")
    
    echo ""
    echo "=========================================="
    echo "RESULT: IMU Orientation Stability"
    echo "=========================================="
    echo "  Min orientation.z: $MIN_QZ"
    echo "  Max orientation.z: $MAX_QZ"
    echo "  Range:             $RANGE_QZ"
    echo ""
    
    RANGE_PASS=$(python3 -c "print(1 if $RANGE_QZ < 0.05 else 0)")
    
    if [ "$RANGE_PASS" -eq 1 ]; then
        echo -e "${GREEN}✓ PASS: Orientation range ${RANGE_QZ} < 0.05 threshold${NC}"
        echo "  NO magnetometer jumps detected"
        PASS_COUNT=$((PASS_COUNT + 1))
    else
        echo -e "${RED}✗ FAIL: Orientation range ${RANGE_QZ} > 0.05 threshold${NC}"
        echo "  Magnetometer may still be active or gyro drifting severely"
        FAIL_COUNT=$((FAIL_COUNT + 1))
    fi
fi

echo ""
echo "=========================================="
echo "TEST 4: Magnetometer Status Verification"
echo "=========================================="
echo ""

# Check if magnetometer is disabled
USE_MAG=$(rosparam get /imu_filter/use_mag 2>/dev/null || echo "UNKNOWN")

echo "Madgwick use_mag parameter: $USE_MAG"

if [ "$USE_MAG" = "false" ] || [ "$USE_MAG" = "False" ]; then
    echo -e "${GREEN}✓ PASS: Magnetometer is DISABLED${NC}"
    PASS_COUNT=$((PASS_COUNT + 1))
else
    echo -e "${RED}✗ FAIL: Magnetometer is ENABLED (use_mag=$USE_MAG)${NC}"
    echo "  CRITICAL: Magnetometer must be disabled indoors"
    FAIL_COUNT=$((FAIL_COUNT + 1))
fi

# Check EKF IMU yaw fusion
IMU_YAW_FUSED=$(rosparam get /ekf_localization/imu0_config 2>/dev/null | sed -n '2p' | awk '{print $3}' | tr -d ',')

echo "EKF IMU yaw fusion (position [5]): $IMU_YAW_FUSED"

if [ "$IMU_YAW_FUSED" = "false" ] || [ "$IMU_YAW_FUSED" = "False" ]; then
    echo -e "${GREEN}✓ PASS: EKF is NOT fusing corrupted IMU yaw${NC}"
    PASS_COUNT=$((PASS_COUNT + 1))
else
    echo -e "${RED}✗ FAIL: EKF is fusing IMU yaw (corrupted by magnetometer)${NC}"
    FAIL_COUNT=$((FAIL_COUNT + 1))
fi

echo ""
echo "=========================================="
echo "FINAL VALIDATION SUMMARY"
echo "=========================================="
echo ""
echo "Tests Passed: $PASS_COUNT / 5"
echo "Tests Failed: $FAIL_COUNT / 5"
echo ""

if [ "$FAIL_COUNT" -eq 0 ]; then
    echo -e "${GREEN}========================================${NC}"
    echo -e "${GREEN}✓✓✓ ALL VALIDATION TESTS PASSED ✓✓✓${NC}"
    echo -e "${GREEN}========================================${NC}"
    echo ""
    echo "NUMERIC PROOF OF STABILITY:"
    echo "  • TF yaw drift:        ${YAW_DRIFT}° < 0.2° ✓"
    echo "  • IMU angular vel:     ${MEAN_VEL} rad/s ≈ 0.0 ✓"
    echo "  • IMU orientation:     range ${RANGE_QZ} < 0.05 ✓"
    echo "  • Magnetometer:        DISABLED ✓"
    echo "  • EKF IMU yaw fusion:  DISABLED ✓"
    echo ""
    echo "SYSTEM IS STABLE - READY FOR MOTION VALIDATION"
    echo ""
    echo "Next step: Run slow motion validation"
    echo "  ./scripts/hard_validation_slow_motion.sh"
    echo ""
    exit 0
else
    echo -e "${RED}========================================${NC}"
    echo -e "${RED}✗✗✗ VALIDATION FAILED ✗✗✗${NC}"
    echo -e "${RED}========================================${NC}"
    echo ""
    echo "SYSTEM IS NOT STABLE"
    echo ""
    echo "Failed tests indicate:"
    if [ "$YAW_DRIFT" != "" ]; then
        DRIFT_FAIL=$(python3 -c "print(1 if $YAW_DRIFT > 0.2 else 0)")
        if [ "$DRIFT_FAIL" -eq 1 ]; then
            echo "  • TF yaw drifting > 0.2° (${YAW_DRIFT}°)"
        fi
    fi
    
    if [ "$MEAN_VEL" != "" ]; then
        VEL_FAIL=$(python3 -c "print(1 if abs($MEAN_VEL) > 0.01 else 0)")
        if [ "$VEL_FAIL" -eq 1 ]; then
            echo "  • IMU reports rotation while stationary"
        fi
    fi
    
    if [ "$USE_MAG" != "false" ] && [ "$USE_MAG" != "False" ]; then
        echo "  • Magnetometer still ENABLED (poisoning data)"
    fi
    
    echo ""
    echo "DO NOT PROCEED TO MAPPING"
    echo ""
    echo "Debug checklist:"
    echo "  1. Restart ROS: rosnode kill -a; roslaunch elderly_bot mapping.launch"
    echo "  2. Verify config: grep 'use_mag' launch/imu_nav.launch"
    echo "  3. Check hardware: MPU9250 connection, I2C errors"
    echo "  4. Re-run this test"
    echo ""
    exit 1
fi
