#!/bin/bash
# SLAM TF and Odometry Diagnostic Script
# Diagnoses ghosting/duplication/radial artifacts in SLAM maps

echo "=========================================="
echo "SLAM DIAGNOSTIC TOOL - TF & ODOMETRY"
echo "=========================================="
echo ""

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

echo "STEP 1: Check if ROS is running..."
if ! rostopic list &> /dev/null; then
    echo -e "${RED}ERROR: ROS is not running! Start roscore first.${NC}"
    exit 1
fi
echo -e "${GREEN}✓ ROS is running${NC}"
echo ""

echo "STEP 2: TF Tree Structure Analysis"
echo "-----------------------------------"

# Check for TF publishers
echo "Who is publishing TF transforms?"
rostopic info /tf 2>/dev/null | grep "Publishers:" -A 10

echo ""
echo "Checking for CRITICAL TF tree issues..."

# Issue 1: Multiple map->odom publishers
echo ""
echo -e "${YELLOW}TEST 1: Multiple map->odom publishers (COMMON CAUSE OF GHOSTING)${NC}"
MAP_ODOM_PUBLISHERS=$(rostopic echo -n 1 /tf 2>/dev/null | grep -c "map.*odom\|odom.*map" || echo "0")
if [ "$MAP_ODOM_PUBLISHERS" -gt 1 ]; then
    echo -e "${RED}⚠ WARNING: Multiple nodes may be publishing map->odom transform!${NC}"
    echo "   This causes: Map jumps, ghosting, duplication"
    echo "   Suspects: gmapping + amcl both running, or multiple SLAM nodes"
else
    echo -e "${GREEN}✓ No obvious multiple map->odom publishers${NC}"
fi

# Issue 2: Check frame IDs
echo ""
echo -e "${YELLOW}TEST 2: Frame ID Consistency${NC}"

# Get laser frame
LASER_FRAME=$(rostopic echo -n 1 /scan/header/frame_id 2>/dev/null | tr -d '"' || echo "UNKNOWN")
echo "Laser scan frame_id: $LASER_FRAME"

# Get odom frame
ODOM_FRAME=$(rostopic echo -n 1 /wheel_odom/header/frame_id 2>/dev/null | tr -d '"' || echo "UNKNOWN")
echo "Wheel odometry frame_id: $ODOM_FRAME"

# Get filtered odom frame
FILTERED_ODOM_FRAME=$(rostopic echo -n 1 /odometry/filtered/header/frame_id 2>/dev/null | tr -d '"' || echo "UNKNOWN")
echo "EKF filtered odometry frame_id: $FILTERED_ODOM_FRAME"

# Get IMU frame
IMU_FRAME=$(rostopic echo -n 1 /imu/data/header/frame_id 2>/dev/null | tr -d '"' || echo "UNKNOWN")
echo "IMU frame_id: $IMU_FRAME"

echo ""
echo -e "${YELLOW}TEST 3: TF Connectivity${NC}"
echo "Checking if critical frames are connected..."

# Check laser->base_footprint
if rosrun tf tf_echo base_footprint laser 2>&1 | grep -q "Exception"; then
    echo -e "${RED}✗ CRITICAL: laser frame NOT connected to base_footprint!${NC}"
    echo "   This causes: Lidar scans appear in wrong location, ghosting"
else
    echo -e "${GREEN}✓ laser -> base_footprint connected${NC}"
fi

# Check odom->base_footprint
if rosrun tf tf_echo odom base_footprint 2>&1 | grep -q "Exception"; then
    echo -e "${RED}✗ CRITICAL: odom frame NOT connected to base_footprint!${NC}"
    echo "   This causes: SLAM cannot track robot motion, massive drift"
else
    echo -e "${GREEN}✓ odom -> base_footprint connected${NC}"
fi

# Check map->odom (should exist if SLAM is running)
if rosrun tf tf_echo map odom 2>&1 | grep -q "Exception"; then
    echo -e "${YELLOW}⚠ map -> odom NOT published (SLAM may not be running)${NC}"
else
    echo -e "${GREEN}✓ map -> odom connected (SLAM active)${NC}"
fi

echo ""
echo -e "${YELLOW}TEST 4: TF Timing Analysis (CRITICAL FOR GHOSTING)${NC}"
echo "Monitoring TF publish rates and delays..."

# Monitor TF for 5 seconds
timeout 5s rostopic echo /tf 2>/dev/null > /tmp/tf_dump.txt

# Check if odom->base_footprint is updating
ODOM_TF_COUNT=$(grep -c "odom.*base" /tmp/tf_dump.txt || echo "0")
echo "odom -> base_footprint updates in 5sec: $ODOM_TF_COUNT"

if [ "$ODOM_TF_COUNT" -lt 10 ]; then
    echo -e "${RED}✗ CRITICAL: odom->base_footprint NOT updating frequently!${NC}"
    echo "   Expected: >200 updates/5sec (~40Hz)"
    echo "   Actual: $ODOM_TF_COUNT updates"
    echo "   This causes: Lidar scans use stale TF, ghosting/smearing"
else
    echo -e "${GREEN}✓ odom->base_footprint updating${NC}"
fi

# Check map->odom updates
MAP_TF_COUNT=$(grep -c "map.*odom" /tmp/tf_dump.txt || echo "0")
echo "map -> odom updates in 5sec: $MAP_TF_COUNT"

if [ "$MAP_TF_COUNT" -gt 50 ]; then
    echo -e "${RED}✗ WARNING: map->odom updating TOO FREQUENTLY ($MAP_TF_COUNT/5sec)${NC}"
    echo "   Normal: 25-100 updates/5sec (5-20Hz)"
    echo "   Too frequent suggests: SLAM is resetting/jumping constantly"
elif [ "$MAP_TF_COUNT" -lt 10 ]; then
    echo -e "${YELLOW}⚠ map->odom updating slowly ($MAP_TF_COUNT/5sec)${NC}"
else
    echo -e "${GREEN}✓ map->odom update rate normal${NC}"
fi

echo ""
echo "STEP 3: Odometry Quality Check"
echo "-------------------------------"

echo "Checking odometry data quality..."

# Get 5 seconds of wheel odometry
timeout 5s rostopic echo /wheel_odom 2>/dev/null > /tmp/wheel_odom_dump.txt

# Check if odometry is publishing
if [ ! -s /tmp/wheel_odom_dump.txt ]; then
    echo -e "${RED}✗ CRITICAL: /wheel_odom NOT publishing!${NC}"
    echo "   This causes: SLAM has no motion estimate, map explodes"
else
    echo -e "${GREEN}✓ /wheel_odom is publishing${NC}"
    
    # Check for suspicious covariance (all zeros = untrustworthy)
    ZERO_COV=$(grep -c "covariance.*\[0.0, 0.0, 0.0" /tmp/wheel_odom_dump.txt || echo "0")
    if [ "$ZERO_COV" -gt 0 ]; then
        echo -e "${RED}✗ WARNING: Wheel odometry has ZERO covariance!${NC}"
        echo "   This causes: EKF trusts bad odometry completely"
    else
        echo -e "${GREEN}✓ Wheel odometry has proper covariance${NC}"
    fi
fi

# Check filtered odometry
timeout 5s rostopic echo /odometry/filtered 2>/dev/null > /tmp/filtered_odom_dump.txt

if [ ! -s /tmp/filtered_odom_dump.txt ]; then
    echo -e "${RED}✗ CRITICAL: /odometry/filtered NOT publishing!${NC}"
    echo "   This causes: GMapping using raw wheel odom (no IMU fusion)"
else
    echo -e "${GREEN}✓ /odometry/filtered is publishing (EKF active)${NC}"
fi

echo ""
echo "STEP 4: Scan Timing Check"
echo "-------------------------"

# Check laser scan timing
SCAN_RATE=$(rostopic hz /scan 2>&1 | grep "average rate" | awk '{print $3}')
echo "Laser scan rate: $SCAN_RATE Hz"

if [ -z "$SCAN_RATE" ]; then
    echo -e "${RED}✗ CRITICAL: /scan NOT publishing!${NC}"
else
    SCAN_RATE_INT=$(echo "$SCAN_RATE" | cut -d. -f1)
    if [ "$SCAN_RATE_INT" -lt 5 ]; then
        echo -e "${YELLOW}⚠ WARNING: Scan rate too low (<5Hz)${NC}"
        echo "   This causes: Jerky motion in map, temporal artifacts"
    else
        echo -e "${GREEN}✓ Scan rate acceptable${NC}"
    fi
fi

# Check scan timing vs TF timing
echo ""
echo "Checking TF-Scan timestamp synchronization..."
SCAN_TIME=$(rostopic echo -n 1 /scan/header/stamp 2>/dev/null | grep "secs" | awk '{print $2}')
TF_TIME=$(rostopic echo -n 1 /tf/transforms[0]/header/stamp 2>/dev/null | grep "secs" | awk '{print $2}')

if [ ! -z "$SCAN_TIME" ] && [ ! -z "$TF_TIME" ]; then
    TIME_DIFF=$((SCAN_TIME - TF_TIME))
    TIME_DIFF_ABS=${TIME_DIFF#-}  # Absolute value
    
    if [ "$TIME_DIFF_ABS" -gt 1 ]; then
        echo -e "${RED}✗ CRITICAL: Scan and TF timestamps differ by >1 second!${NC}"
        echo "   Scan timestamp: $SCAN_TIME"
        echo "   TF timestamp: $TF_TIME"
        echo "   This causes: Scans use WRONG robot pose, severe ghosting"
    else
        echo -e "${GREEN}✓ Scan and TF timestamps synchronized${NC}"
    fi
fi

echo ""
echo "STEP 5: GMapping Status"
echo "-----------------------"

# Check if GMapping is running
if rosnode list 2>/dev/null | grep -q "slam_gmapping"; then
    echo -e "${GREEN}✓ GMapping is running${NC}"
    
    # Check GMapping parameters
    echo ""
    echo "GMapping Configuration:"
    echo "  odom_frame: $(rosparam get /slam_gmapping/odom_frame 2>/dev/null || echo 'NOT SET')"
    echo "  base_frame: $(rosparam get /slam_gmapping/base_frame 2>/dev/null || echo 'NOT SET')"
    echo "  map_frame: $(rosparam get /slam_gmapping/map_frame 2>/dev/null || echo 'NOT SET')"
    echo "  particles: $(rosparam get /slam_gmapping/particles 2>/dev/null || echo 'NOT SET')"
    echo "  laser_model_type: $(rosparam get /slam_gmapping/laser_model_type 2>/dev/null || echo 'NOT SET')"
    
    # Critical: Check if using correct odometry topic
    GMAPPING_TOPIC=$(rosparam get /slam_gmapping/scan_topic 2>/dev/null || echo "/scan")
    echo "  scan_topic: $GMAPPING_TOPIC"
    
else
    echo -e "${YELLOW}⚠ GMapping is NOT running${NC}"
fi

echo ""
echo "=========================================="
echo "DIAGNOSTIC SUMMARY"
echo "=========================================="
echo ""
echo "If you see RED errors above, those are CRITICAL issues causing ghosting."
echo ""
echo "MOST COMMON CAUSES OF GHOSTING/DUPLICATION:"
echo "1. Multiple nodes publishing map->odom (gmapping + amcl conflict)"
echo "2. TF timestamps out of sync (WiFi lag in rosserial)"
echo "3. odom->base_footprint not updating (odometry frozen)"
echo "4. Wheel odometry with zero/bad covariance (EKF trusts garbage)"
echo "5. IMU orientation drift (yaw accumulation)"
echo ""
echo "NEXT STEPS:"
echo "1. Review RED errors above"
echo "2. Run: rosrun tf view_frames  (generates TF tree PDF)"
echo "3. Run: rosrun rqt_tf_tree rqt_tf_tree  (visual TF inspector)"
echo "4. Monitor: rostopic echo /diagnostics  (hardware health)"
echo ""
echo "Diagnostic complete."
