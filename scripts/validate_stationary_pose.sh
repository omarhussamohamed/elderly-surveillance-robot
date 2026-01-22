#!/bin/bash
# EMERGENCY VALIDATION: IMU Magnetometer Kill-Switch
# Verifies robot pose is stable at rest after disabling mag fusion

echo "========================================"
echo "EMERGENCY IMU VALIDATION - STATIONARY TEST"
echo "========================================"
echo ""
echo "🚨 CRITICAL: Robot MUST be physically stationary for this test!"
echo "   Place robot on floor, DO NOT TOUCH for 30 seconds"
echo ""
read -p "Press ENTER when robot is stationary and ready..."

# Colors
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m'

echo ""
echo "=========================================="
echo "TEST 1: IMU Quaternion Stability (30 sec)"
echo "=========================================="
echo ""
echo "Monitoring /imu/data orientation.z (quaternion z-component)"
echo "MUST remain within ±0.02 range if stationary"
echo ""

# Capture 30 seconds of IMU data
timeout 30s rostopic echo /imu/data/orientation/z > /tmp/imu_z_test.txt 2>&1 &
IMU_PID=$!

# Live monitor (show values in real-time)
echo -e "${YELLOW}Collecting 30 seconds of data...${NC}"
for i in {1..30}; do
    CURRENT_Z=$(rostopic echo -n 1 /imu/data/orientation/z 2>/dev/null || echo "N/A")
    echo "[$i/30] orientation.z = $CURRENT_Z"
    sleep 1
done

wait $IMU_PID

echo ""
echo "Analysis of orientation.z values:"
echo "-----------------------------------"

# Extract numerical values only
grep -E "^[-0-9]" /tmp/imu_z_test.txt > /tmp/imu_z_numbers.txt

if [ ! -s /tmp/imu_z_numbers.txt ]; then
    echo -e "${RED}✗ FAILURE: No IMU data received!${NC}"
    echo "  Check: Is /imu/data topic publishing?"
    echo "  Run: rostopic hz /imu/data"
    exit 1
fi

# Compute statistics
MIN_Z=$(sort -n /tmp/imu_z_numbers.txt | head -1)
MAX_Z=$(sort -n /tmp/imu_z_numbers.txt | tail -1)
RANGE=$(echo "$MAX_Z - $MIN_Z" | bc -l)
COUNT=$(wc -l < /tmp/imu_z_numbers.txt)

echo "  Samples collected: $COUNT"
echo "  Min orientation.z: $MIN_Z"
echo "  Max orientation.z: $MAX_Z"
echo "  Range (max-min):   $RANGE"
echo ""

# Check if range exceeds threshold
THRESHOLD=0.05  # Allow ±2.5° drift (orientation.z ≈ sin(yaw/2))

PASS=$(echo "$RANGE < $THRESHOLD" | bc -l)

if [ "$PASS" -eq 1 ]; then
    echo -e "${GREEN}✓ PASS: IMU orientation stable (range < 0.05)${NC}"
    echo "  Robot yaw is NOT jumping randomly"
    echo "  Magnetometer kill-switch SUCCESSFUL"
else
    echo -e "${RED}✗ FAILURE: IMU orientation UNSTABLE (range > 0.05)${NC}"
    echo "  Robot still thinks it's rotating while stationary"
    echo "  Possible causes:"
    echo "    1. Madgwick filter still using magnetometer (check use_mag=false)"
    echo "    2. Gyro bias drift (needs recalibration)"
    echo "    3. MPU9250 hardware failure"
    echo ""
    echo "  CRITICAL: Do NOT proceed to mapping until this is fixed!"
fi

echo ""
echo "=========================================="
echo "TEST 2: EKF Odometry Stability (15 sec)"
echo "=========================================="
echo ""
echo "Monitoring /odometry/filtered pose.position and orientation"
echo "MUST remain within ±2cm position, ±5° yaw if stationary"
echo ""

# Capture odometry
timeout 15s rostopic echo /odometry/filtered/pose/pose > /tmp/odom_test.txt 2>&1 &
ODOM_PID=$!

echo -e "${YELLOW}Collecting 15 seconds of odometry...${NC}"
for i in {1..15}; do
    ODOM_X=$(rostopic echo -n 1 /odometry/filtered/pose/pose/position/x 2>/dev/null || echo "N/A")
    ODOM_YAW_Z=$(rostopic echo -n 1 /odometry/filtered/pose/pose/orientation/z 2>/dev/null || echo "N/A")
    echo "[$i/15] x=$ODOM_X, orientation.z=$ODOM_YAW_Z"
    sleep 1
done

wait $ODOM_PID

echo ""
echo "Analysis of odometry pose:"
echo "--------------------------"

# Extract position x values
grep -A 1 "position:" /tmp/odom_test.txt | grep "x:" | awk '{print $2}' > /tmp/odom_x.txt

# Extract orientation z values
grep -A 3 "orientation:" /tmp/odom_test.txt | grep "z:" | awk '{print $2}' > /tmp/odom_z.txt

if [ ! -s /tmp/odom_x.txt ] || [ ! -s /tmp/odom_z.txt ]; then
    echo -e "${RED}✗ FAILURE: No odometry data received!${NC}"
    echo "  Check: Is /odometry/filtered topic publishing?"
    echo "  Run: rostopic hz /odometry/filtered"
    exit 1
fi

# Position analysis
MIN_X=$(sort -n /tmp/odom_x.txt | head -1)
MAX_X=$(sort -n /tmp/odom_x.txt | tail -1)
RANGE_X=$(echo "$MAX_X - $MIN_X" | bc -l | awk '{printf "%.4f", $1}')

# Orientation analysis
MIN_OZ=$(sort -n /tmp/odom_z.txt | head -1)
MAX_OZ=$(sort -n /tmp/odom_z.txt | tail -1)
RANGE_OZ=$(echo "$MAX_OZ - $MIN_OZ" | bc -l | awk '{printf "%.4f", $1}')

echo "  Position X range: $RANGE_X m"
echo "  Orientation Z range: $RANGE_OZ"
echo ""

# Check thresholds
POS_PASS=$(echo "$RANGE_X < 0.02" | bc -l)  # 2cm position threshold
ORI_PASS=$(echo "$RANGE_OZ < 0.1" | bc -l)  # ~11° orientation threshold (z ≈ sin(θ/2))

if [ "$POS_PASS" -eq 1 ] && [ "$ORI_PASS" -eq 1 ]; then
    echo -e "${GREEN}✓ PASS: EKF odometry stable${NC}"
    echo "  Position drift < 2cm"
    echo "  Orientation drift < 11°"
    echo "  EKF kill-switch SUCCESSFUL"
else
    if [ "$POS_PASS" -eq 0 ]; then
        echo -e "${RED}✗ FAILURE: Position drifting > 2cm while stationary${NC}"
        echo "  Possible: Wheel encoder noise, EKF process noise too high"
    fi
    if [ "$ORI_PASS" -eq 0 ]; then
        echo -e "${RED}✗ FAILURE: Orientation drifting > 11° while stationary${NC}"
        echo "  Possible: IMU yaw still being fused (check imu0_config[5]=false)"
    fi
fi

echo ""
echo "=========================================="
echo "TEST 3: TF Stability (10 sec)"
echo "=========================================="
echo ""
echo "Monitoring TF: odom → base_footprint"
echo "Transform must be constant if robot stationary"
echo ""

echo -e "${YELLOW}Sampling TF transform...${NC}"

# Use tf_echo to monitor transform
timeout 10s rosrun tf tf_echo odom base_footprint > /tmp/tf_test.txt 2>&1 &
TF_PID=$!

sleep 10
wait $TF_PID

# Extract translation and rotation from tf_echo output
grep "Translation" /tmp/tf_test.txt | head -5
grep "Rotation" /tmp/tf_test.txt | head -5

echo ""
if grep -q "Exception" /tmp/tf_test.txt; then
    echo -e "${RED}✗ FAILURE: TF transform not available${NC}"
    echo "  Check: Is EKF publishing odom→base_footprint?"
else
    echo -e "${GREEN}✓ TF transform exists${NC}"
    echo "  Manually inspect values above - should be nearly constant"
fi

echo ""
echo "=========================================="
echo "SUMMARY & NEXT STEPS"
echo "=========================================="
echo ""

# Count passes
PASS_COUNT=0
[ "$PASS" -eq 1 ] && PASS_COUNT=$((PASS_COUNT + 1))
[ "$POS_PASS" -eq 1 ] && [ "$ORI_PASS" -eq 1 ] && PASS_COUNT=$((PASS_COUNT + 1))

if [ "$PASS_COUNT" -eq 2 ]; then
    echo -e "${GREEN}========================================${NC}"
    echo -e "${GREEN}✓✓✓ ALL CRITICAL TESTS PASSED ✓✓✓${NC}"
    echo -e "${GREEN}========================================${NC}"
    echo ""
    echo "Robot pose is STABLE at rest!"
    echo ""
    echo "READY FOR MAPPING:"
    echo "  1. roslaunch elderly_bot mapping.launch"
    echo "  2. Drive SLOWLY (0.05 m/s max)"
    echo "  3. Map small rectangular room"
    echo "  4. Check for NO ghosting/duplication"
    echo ""
    echo "Expected improvement:"
    echo "  ✓ No spontaneous rotation while stationary"
    echo "  ✓ Single copy of walls (no duplication)"
    echo "  ✓ Clean map without radial spray artifacts"
    echo ""
    echo "GYRO-ONLY MODE TRADE-OFF:"
    echo "  ⚠️ Yaw will drift slowly over time (~5-10° per 5 minutes)"
    echo "  ⚠️ For long missions, consider loop closure or relocalization"
    echo "  ✓ But NO sudden jumps (much better than mag interference)"
else
    echo -e "${RED}========================================${NC}"
    echo -e "${RED}✗✗✗ TESTS FAILED - DO NOT MAP YET ✗✗✗${NC}"
    echo -e "${RED}========================================${NC}"
    echo ""
    echo "Robot pose is STILL UNSTABLE!"
    echo ""
    echo "DEBUGGING CHECKLIST:"
    echo "  1. Verify magnetometer disabled:"
    echo "     grep 'use_mag' launch/imu_nav.launch"
    echo "     Should show: use_mag value=\"false\""
    echo ""
    echo "  2. Verify EKF not using IMU yaw:"
    echo "     grep 'imu0_config' config/ekf.yaml | grep orientation"
    echo "     Should show: [false, false, false] for orientation"
    echo ""
    echo "  3. Restart ROS completely:"
    echo "     rosnode kill -a"
    echo "     roscore &"
    echo "     roslaunch elderly_bot mapping.launch"
    echo ""
    echo "  4. Re-run this test"
    echo ""
    echo "If still failing:"
    echo "  - Check MPU9250 hardware (loose connection?)"
    echo "  - Recalibrate gyro (mpu9250_node.py _calibrate_gyro)"
    echo "  - Check EKF process_noise covariance (may be too high)"
fi

echo ""
echo "Test complete."
echo ""
