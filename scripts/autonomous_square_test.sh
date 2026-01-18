#!/bin/bash
# ================================================
# AUTONOMOUS 1-METER SQUARE MAPPING TEST
# ================================================
# "Boss Level" stress test for drift fixes
# Robot will autonomously drive in a 1m square

set -e

RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m'

echo -e "${BLUE}================================================${NC}"
echo -e "${BLUE}  AUTONOMOUS SQUARE MAPPING TEST${NC}"
echo -e "${BLUE}================================================${NC}"
echo ""
echo -e "${YELLOW}This test will command the robot to:${NC}"
echo "  1. Move forward 1 meter"
echo "  2. Rotate 90° left"
echo "  3. Repeat 4 times (complete square)"
echo ""
echo -e "${RED}⚠️  SAFETY WARNINGS:${NC}"
echo "  - Clear 2m x 2m area around robot"
echo "  - Be ready to press Ctrl+C to stop"
echo "  - Watch for obstacles"
echo ""
read -p "Press Enter to start, or Ctrl+C to cancel..."
echo ""

# Check ROS
if ! rostopic list &>/dev/null; then
    echo -e "${RED}❌ ROS not running!${NC}"
    exit 1
fi

# Parameters (adjust based on your robot's calibration)
FORWARD_SPEED=0.15  # m/s
ROTATE_SPEED=0.3    # rad/s
FORWARD_TIME=6.67   # seconds to travel 1m at 0.15 m/s
ROTATE_TIME=5.24    # seconds to rotate 90° at 0.3 rad/s

echo -e "${GREEN}Starting autonomous square...${NC}"
echo ""

for side in {1..4}; do
    echo -e "${YELLOW}Side $side/4: Moving forward 1 meter...${NC}"
    
    # Move forward
    rostopic pub -1 /cmd_vel geometry_msgs/Twist "linear:
  x: $FORWARD_SPEED
  y: 0.0
  z: 0.0
angular:
  x: 0.0
  y: 0.0
  z: 0.0" &
  
    sleep $FORWARD_TIME
    
    # Stop
    rostopic pub -1 /cmd_vel geometry_msgs/Twist "linear:
  x: 0.0
  y: 0.0
  z: 0.0
angular:
  x: 0.0
  y: 0.0
  z: 0.0"
  
    sleep 1
    
    if [ $side -lt 4 ]; then
        echo -e "${YELLOW}Rotating 90° left...${NC}"
        
        # Rotate
        rostopic pub -1 /cmd_vel geometry_msgs/Twist "linear:
  x: 0.0
  y: 0.0
  z: 0.0
angular:
  x: 0.0
  y: 0.0
  z: $ROTATE_SPEED" &
  
        sleep $ROTATE_TIME
        
        # Stop
        rostopic pub -1 /cmd_vel geometry_msgs/Twist "linear:
  x: 0.0
  y: 0.0
  z: 0.0
angular:
  x: 0.0
  y: 0.0
  z: 0.0"
  
        sleep 1
    fi
done

echo ""
echo -e "${GREEN}✓ Square complete!${NC}"
echo ""
echo -e "${BLUE}================================================${NC}"
echo -e "${BLUE}  VALIDATION CHECKLIST${NC}"
echo -e "${BLUE}================================================${NC}"
echo ""
echo -e "${YELLOW}Check in Foxglove/RViz:${NC}"
echo "  [ ] Walls are sharp SINGLE lines (no ghosting)"
echo "  [ ] Robot path forms a closed square"
echo "  [ ] No jumps or teleportation"
echo "  [ ] Map is stable and consistent"
echo ""
echo -e "${YELLOW}If all checks pass:${NC}"
echo -e "${GREEN}  ✅ DRIFT FIXES VALIDATED - System ready for production!${NC}"
echo ""
echo -e "${YELLOW}If checks fail:${NC}"
echo "  - Review DRIFT_FIX_APPLIED.md troubleshooting section"
echo "  - Check wheel encoder calibration in HARDWARE_MAP.md"
echo "  - Verify IMU calibration on startup"
echo ""
