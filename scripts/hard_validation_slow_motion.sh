#!/bin/bash
# HARD VALIDATION: Slow Motion Test
# Validates no yaw jumps during start/stop cycles

echo "========================================"
echo "HARD VALIDATION - SLOW MOTION TEST"
echo "========================================"
echo ""
echo "This test validates:"
echo "  • No yaw jumps on stop"
echo "  • Map walls remain aligned"
echo "  • TF stability during motion"
echo ""
echo "Requirements:"
echo "  • Robot on floor with 3m clear space ahead"
echo "  • mapping.launch already running"
echo "  • Teleop ready (or autonomous drive)"
echo ""
read -p "Press ENTER when ready..."

RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m'

echo ""
echo "=========================================="
echo "PHASE 1: Pre-Motion Baseline"
echo "=========================================="
echo ""

# Get baseline yaw
BASELINE_YAW=$(timeout 2s rosrun tf tf_echo odom base_footprint 2>&1 | grep "Rotation:" | awk '{print $5}' | tr -d ',')
BASELINE_QW=$(timeout 2s rosrun tf tf_echo odom base_footprint 2>&1 | grep "Rotation:" | awk '{print $6}' | tr -d ')')
BASELINE_YAW_DEG=$(python3 -c "import math; qz=$BASELINE_YAW; qw=$BASELINE_QW; print(f'{2*math.atan2(qz,qw)*180/math.pi:.2f}')")

echo "Baseline yaw: ${BASELINE_YAW_DEG}°"
echo ""

echo "=========================================="
echo "PHASE 2: Forward Motion (2 meters)"
echo "=========================================="
echo ""
echo -e "${YELLOW}ACTION REQUIRED:${NC}"
echo "  Drive robot forward 2 meters at 0.1 m/s"
echo "  (Use teleop or publish to /cmd_vel)"
echo ""
echo "Command example:"
echo "  rostopic pub -r 10 /cmd_vel geometry_msgs/Twist '{linear: {x: 0.1}}'"
echo ""
read -p "Press ENTER when robot has moved 2 meters and STOPPED..."

echo ""
echo "Measuring post-motion yaw..."
sleep 2  # Wait for robot to fully stop

POST_MOVE_YAW=$(timeout 2s rosrun tf tf_echo odom base_footprint 2>&1 | grep "Rotation:" | awk '{print $5}' | tr -d ',')
POST_MOVE_QW=$(timeout 2s rosrun tf tf_echo odom base_footprint 2>&1 | grep "Rotation:" | awk '{print $6}' | tr -d ')')
POST_MOVE_YAW_DEG=$(python3 -c "import math; qz=$POST_MOVE_YAW; qw=$POST_MOVE_QW; print(f'{2*math.atan2(qz,qw)*180/math.pi:.2f}')")

YAW_CHANGE=$(python3 -c "print(f'{abs($POST_MOVE_YAW_DEG - $BASELINE_YAW_DEG):.2f}')")

echo "Post-motion yaw: ${POST_MOVE_YAW_DEG}°"
echo "Yaw change: ${YAW_CHANGE}°"
echo ""

# Check if yaw jumped unexpectedly (should be ~0° for straight line)
YAW_JUMP_THRESHOLD=5.0
YAW_JUMP=$(python3 -c "print(1 if $YAW_CHANGE > $YAW_JUMP_THRESHOLD else 0)")

if [ "$YAW_JUMP" -eq 0 ]; then
    echo -e "${GREEN}✓ PASS: No unexpected yaw jump (${YAW_CHANGE}° < 5°)${NC}"
    echo "  Robot maintained heading during straight-line motion"
else
    echo -e "${RED}✗ FAIL: Unexpected yaw jump (${YAW_CHANGE}° > 5°)${NC}"
    echo "  Robot rotated during straight-line drive"
    echo "  Possible causes: wheel slip, IMU drift, EKF instability"
fi

echo ""
echo "=========================================="
echo "PHASE 3: Stop Stability (10 seconds)"
echo "=========================================="
echo ""
echo "Monitoring yaw stability after stop..."

# Sample yaw for 10 seconds after stop
YAW_SAMPLES=()
for i in {1..5}; do
    sleep 2
    CURRENT_YAW=$(timeout 2s rosrun tf tf_echo odom base_footprint 2>&1 | grep "Rotation:" | awk '{print $5}' | tr -d ',')
    CURRENT_QW=$(timeout 2s rosrun tf tf_echo odom base_footprint 2>&1 | grep "Rotation:" | awk '{print $6}' | tr -d ')')
    CURRENT_YAW_DEG=$(python3 -c "import math; qz=$CURRENT_YAW; qw=$CURRENT_QW; print(f'{2*math.atan2(qz,qw)*180/math.pi:.3f}')")
    echo "[$((i*2))s] Yaw: ${CURRENT_YAW_DEG}°"
    YAW_SAMPLES+=($CURRENT_YAW_DEG)
done

# Calculate yaw stability (range of samples)
MIN_STOPPED_YAW=$(printf '%s\n' "${YAW_SAMPLES[@]}" | sort -n | head -1)
MAX_STOPPED_YAW=$(printf '%s\n' "${YAW_SAMPLES[@]}" | sort -n | tail -1)
STOPPED_YAW_RANGE=$(python3 -c "print(f'{abs($MAX_STOPPED_YAW - $MIN_STOPPED_YAW):.3f}')")

echo ""
echo "Yaw range while stopped: ${STOPPED_YAW_RANGE}°"

STOPPED_STABLE=$(python3 -c "print(1 if $STOPPED_YAW_RANGE < 0.5 else 0)")

if [ "$STOPPED_STABLE" -eq 1 ]; then
    echo -e "${GREEN}✓ PASS: Yaw stable after stop (${STOPPED_YAW_RANGE}° < 0.5°)${NC}"
    echo "  No spontaneous rotation after stopping"
else
    echo -e "${RED}✗ FAIL: Yaw drifting after stop (${STOPPED_YAW_RANGE}° > 0.5°)${NC}"
    echo "  Robot still thinks it's rotating"
fi

echo ""
echo "=========================================="
echo "PHASE 4: Map Quality Check"
echo "=========================================="
echo ""
echo -e "${YELLOW}MANUAL INSPECTION REQUIRED:${NC}"
echo ""
echo "Open Foxglove/RViz and inspect /map topic:"
echo ""
echo "PASS criteria (check visually):"
echo "  ✓ Single copy of walls (no duplication)"
echo "  ✓ Straight walls (not smeared or wavy)"
echo "  ✓ Sharp corners (not blurred)"
echo "  ✓ Lidar points align with map borders"
echo ""
echo "FAIL criteria (check visually):"
echo "  ✗ Multiple wall copies"
echo "  ✗ Radial spray artifacts"
echo "  ✗ Walls smeared or duplicated"
echo "  ✗ Lidar points offset from map"
echo ""
read -p "Inspect map in Foxglove. Does map PASS visual inspection? (y/n): " MAP_PASS

echo ""
echo "=========================================="
echo "SLOW MOTION VALIDATION SUMMARY"
echo "=========================================="
echo ""

if [ "$YAW_JUMP" -eq 0 ] && [ "$STOPPED_STABLE" -eq 1 ] && [ "$MAP_PASS" = "y" ]; then
    echo -e "${GREEN}✓✓✓ SLOW MOTION TEST PASSED ✓✓✓${NC}"
    echo ""
    echo "NUMERIC PROOF:"
    echo "  • Yaw change during motion: ${YAW_CHANGE}° < 5° ✓"
    echo "  • Yaw stability after stop:  ${STOPPED_YAW_RANGE}° < 0.5° ✓"
    echo "  • Map quality:               PASS (visual) ✓"
    echo ""
    echo "SYSTEM IS STABLE DURING MOTION"
    echo ""
    echo "Ready for full mapping mission"
    exit 0
else
    echo -e "${RED}✗✗✗ SLOW MOTION TEST FAILED ✗✗✗${NC}"
    echo ""
    echo "Issues detected:"
    [ "$YAW_JUMP" -eq 1 ] && echo "  ✗ Unexpected yaw jump during motion"
    [ "$STOPPED_STABLE" -eq 0 ] && echo "  ✗ Yaw unstable after stop"
    [ "$MAP_PASS" != "y" ] && echo "  ✗ Map quality issues"
    echo ""
    echo "DO NOT PROCEED TO FULL MAPPING"
    echo ""
    echo "Further diagnosis required."
    exit 1
fi
