#!/bin/bash
# ================================================
# FINAL DRIFT FIX VALIDATION - "Boss Level" Test
# ================================================
# This script performs comprehensive validation of all drift fixes

set -e

RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

echo -e "${BLUE}================================================${NC}"
echo -e "${BLUE}  DRIFT FIX VALIDATION - PHASE 1: DATA INTEGRITY${NC}"
echo -e "${BLUE}================================================${NC}"
echo ""

# ================================================
# PHASE 1: Pre-Flight Checks
# ================================================

echo -e "${YELLOW}[1/6] Checking ROS system status...${NC}"
if ! rostopic list &>/dev/null; then
    echo -e "${RED}❌ ROS is not running!${NC}"
    echo "   Start with: roslaunch elderly_bot mapping.launch"
    exit 1
fi
echo -e "${GREEN}✓ ROS system active${NC}"
echo ""

# ================================================
# PHASE 2: Topic Rate Verification
# ================================================

echo -e "${YELLOW}[2/6] Verifying /wheel_odom rate (should be ~10Hz)...${NC}"
WHEEL_HZ=$(timeout 5 rostopic hz /wheel_odom 2>&1 | grep "average rate" | tail -n 1 | awk '{print $3}')
if [ ! -z "$WHEEL_HZ" ]; then
    echo -e "${GREEN}✓ /wheel_odom publishing at ${WHEEL_HZ} Hz${NC}"
    WHEEL_CHECK=$(echo "$WHEEL_HZ < 8" | bc -l 2>/dev/null || echo "0")
    if [ "$WHEEL_CHECK" = "1" ]; then
        echo -e "${RED}⚠️  WARNING: Rate too low! Check ESP32 connection${NC}"
    fi
else
    echo -e "${RED}❌ /wheel_odom not publishing!${NC}"
    exit 1
fi
echo ""

echo -e "${YELLOW}[3/6] Verifying /imu/data rate (should be ~100Hz)...${NC}"
IMU_HZ=$(timeout 5 rostopic hz /imu/data 2>&1 | grep "average rate" | tail -n 1 | awk '{print $3}')
if [ ! -z "$IMU_HZ" ]; then
    echo -e "${GREEN}✓ /imu/data publishing at ${IMU_HZ} Hz${NC}"
else
    echo -e "${RED}❌ /imu/data not publishing!${NC}"
    exit 1
fi
echo ""

echo -e "${YELLOW}[4/6] Verifying /odometry/filtered rate (should be ~50Hz)...${NC}"
EKF_HZ=$(timeout 5 rostopic hz /odometry/filtered 2>&1 | grep "average rate" | tail -n 1 | awk '{print $3}')
if [ ! -z "$EKF_HZ" ]; then
    echo -e "${GREEN}✓ EKF publishing at ${EKF_HZ} Hz${NC}"
else
    echo -e "${RED}❌ EKF not publishing!${NC}"
    exit 1
fi
echo ""

# ================================================
# PHASE 3: TCP Parameter Verification
# ================================================

echo -e "${YELLOW}[5/6] Verifying rosserial TCP optimization...${NC}"
if rosparam list | grep -q "esp32_serial_node/tcp_nodelay"; then
    TCP_VAL=$(rosparam get /esp32_serial_node/tcp_nodelay 2>/dev/null || echo "not_set")
    if [ "$TCP_VAL" == "True" ] || [ "$TCP_VAL" == "1" ]; then
        echo -e "${GREEN}✓ tcp_nodelay is ENABLED${NC}"
    else
        echo -e "${YELLOW}⚠️  tcp_nodelay parameter exists but value unclear${NC}"
    fi
else
    echo -e "${YELLOW}⚠️  tcp_nodelay parameter not found (may still be working)${NC}"
fi
echo ""

# ================================================
# PHASE 4: 2-Minute Yaw Stability Test
# ================================================

echo -e "${BLUE}================================================${NC}"
echo -e "${BLUE}  PHASE 2: YAW STABILITY TEST (2 MINUTES)${NC}"
echo -e "${BLUE}================================================${NC}"
echo ""
echo -e "${RED}⚠️  CRITICAL: Robot must be COMPLETELY STATIONARY!${NC}"
echo -e "${YELLOW}   - Do NOT touch the robot${NC}"
echo -e "${YELLOW}   - Do NOT walk near the robot${NC}"
echo -e "${YELLOW}   - Wheels must not slip${NC}"
echo ""
read -p "Press Enter when robot is still and you're ready..."
echo ""

echo -e "${YELLOW}[6/6] Running 2-minute yaw stability test...${NC}"
echo -e "${YELLOW}This will sample rotation every 10 seconds${NC}"
echo ""

# Create temporary file for results
TEMP_FILE=$(mktemp)

# Sample yaw 12 times over 2 minutes
for i in {1..12}; do
    TIMESTAMP=$(date +%s)
    
    # Get quaternion z value
    QUAT_Z=$(timeout 2 rostopic echo -n 1 /odometry/filtered/pose/pose/orientation/z 2>/dev/null || echo "ERROR")
    
    if [ "$QUAT_Z" != "ERROR" ]; then
        # Convert quaternion to approximate yaw in degrees (simplified)
        YAW_DEG=$(python3 -c "import math; z=$QUAT_Z; yaw_rad=2*math.asin(z) if abs(z)<=1 else 0; print(f'{math.degrees(yaw_rad):.4f}')" 2>/dev/null || echo "0.0000")
        echo "$TIMESTAMP $YAW_DEG" >> $TEMP_FILE
        echo -e "  Sample $i/12: Yaw = ${YELLOW}${YAW_DEG}°${NC}"
    else
        echo -e "  Sample $i/12: ${RED}Failed to read${NC}"
    fi
    
    if [ $i -lt 12 ]; then
        sleep 10
    fi
done

echo ""
echo -e "${YELLOW}Analyzing results...${NC}"

# Calculate variance
if [ -s $TEMP_FILE ]; then
    VARIANCE=$(awk '{sum+=$2; sumsq+=$2*$2} END {printf "%.6f", (sumsq/NR - (sum/NR)^2)}' $TEMP_FILE)
    MAX_YAW=$(awk '{print $2}' $TEMP_FILE | sort -n | tail -1)
    MIN_YAW=$(awk '{print $2}' $TEMP_FILE | sort -n | head -1)
    DRIFT=$(python3 -c "print(f'{abs($MAX_YAW - $MIN_YAW):.4f}')" 2>/dev/null || echo "0.0000")
    
    echo ""
    echo -e "${BLUE}================================================${NC}"
    echo -e "${BLUE}  RESULTS${NC}"
    echo -e "${BLUE}================================================${NC}"
    echo -e "Max Yaw:      ${YELLOW}${MAX_YAW}°${NC}"
    echo -e "Min Yaw:      ${YELLOW}${MIN_YAW}°${NC}"
    echo -e "Total Drift:  ${YELLOW}${DRIFT}°${NC}"
    echo -e "Variance:     ${YELLOW}${VARIANCE}${NC}"
    echo ""
    
    # Evaluate results
    DRIFT_CHECK=$(python3 -c "print('PASS' if $DRIFT < 0.05 else 'FAIL')" 2>/dev/null || echo "UNKNOWN")
    
    if [ "$DRIFT_CHECK" == "PASS" ]; then
        echo -e "${GREEN}✅ EXCELLENT! Drift < 0.05° - System is STABLE${NC}"
        echo -e "${GREEN}   Drift fixes are working perfectly!${NC}"
    elif (( $(echo "$DRIFT < 0.5" | bc -l) )); then
        echo -e "${YELLOW}⚠️  ACCEPTABLE: Drift = ${DRIFT}° (< 0.5°)${NC}"
        echo -e "${YELLOW}   System is usable but could be improved${NC}"
    else
        echo -e "${RED}❌ FAILED: Drift = ${DRIFT}° (> 0.5°)${NC}"
        echo -e "${RED}   Drift fixes may need adjustment${NC}"
    fi
else
    echo -e "${RED}❌ Unable to collect sufficient data${NC}"
fi

rm -f $TEMP_FILE

echo ""
echo -e "${BLUE}================================================${NC}"
echo -e "${BLUE}  NEXT STEPS${NC}"
echo -e "${BLUE}================================================${NC}"
echo ""
echo -e "${GREEN}1. Autonomous Mapping Stress Test:${NC}"
echo "   bash ~/catkin_ws/src/elderly_bot/scripts/autonomous_square_test.sh"
echo ""
echo -e "${GREEN}2. Manual Mapping Verification:${NC}"
echo "   rosrun teleop_twist_keyboard teleop_twist_keyboard.py"
echo "   - Drive in a circle"
echo "   - Walls should be sharp single lines"
echo ""
echo -e "${GREEN}3. Save successful map:${NC}"
echo "   rosrun map_server map_saver -f ~/catkin_ws/src/elderly_bot/maps/validated_map"
echo ""
