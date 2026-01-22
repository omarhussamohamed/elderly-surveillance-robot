#!/bin/bash
# Elderly Bot - Complete System Setup, Update, and Launch Script
# Handles environment, permissions, authentication, updates, and launches the robot

set -e  # Exit on error

echo "=========================================="
echo "  Elderly Bot - System Manager"
echo "=========================================="
echo ""

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

WORKSPACE_DIR="$HOME/catkin_ws"
ROBOT_DIR="$WORKSPACE_DIR/src/elderly_bot"

# ============================================
# STEP 0: Cleanup Redundant Files (First Run)
# ============================================
echo -e "${BLUE}[0/8] Cleaning redundant files...${NC}"

cd "$ROBOT_DIR"

# Remove redundant scripts silently
rm -f clean_rebuild.sh deploy_scripts.sh cleanup_package.sh 2>/dev/null
rm -f scripts/fix_gpio_permissions.sh scripts/diagnose_tf_slam.sh scripts/validate_stationary_pose.sh 2>/dev/null
rm -f scripts/*.bak scripts/*.py.bak scripts/*.old 2>/dev/null
rm -f CLEANUP_AUDIT.md FINAL_PACKAGE_STRUCTURE.md 2>/dev/null

echo "✓ Package cleaned"
echo ""

# ============================================
# STEP 1: Environment Setup
# ============================================
echo -e "${BLUE}[1/8] Configuring environment...${NC}"

# Add workspace to bashrc if not already there
SETUP_LINE="source $WORKSPACE_DIR/devel/setup.bash"
if ! grep -q "$SETUP_LINE" ~/.bashrc; then
    echo "" >> ~/.bashrc
    echo "# ROS Workspace - Added by update_robot.sh" >> ~/.bashrc
    echo "$SETUP_LINE" >> ~/.bashrc
    echo -e "${GREEN}✓ Added workspace to ~/.bashrc${NC}"
else
    echo "✓ Workspace already in ~/.bashrc"
fi

# Source for current session
source /opt/ros/melodic/setup.bash 2>/dev/null || echo "⚠ ROS Melodic not found"
echo ""

# ============================================
# STEP 2: Fix Jetson Stats Authentication
# ============================================
echo -e "${BLUE}[2/8] Fixing Jetson stats permissions...${NC}"

# Upgrade jetson-stats to latest version (fixes float parsing bugs)
if command -v jtop &> /dev/null; then
    echo "Upgrading jetson-stats to latest version..."
    sudo -H pip install -U jetson-stats || echo "⚠ jetson-stats upgrade skipped"
else
    echo "Installing jetson-stats..."
    sudo -H pip install jetson-stats || echo "⚠ jetson-stats install skipped"
fi

# Check if user is in jtop group
if ! groups | grep -q "jtop"; then
    echo "Adding user to jtop group..."
    sudo usermod -aG jtop $USER
    echo -e "${YELLOW}⚠ User added to jtop group. You may need to logout/login for this to take effect.${NC}"
else
    echo "✓ User already in jtop group"
fi

# Restart jtop service if running
if systemctl is-active --quiet jtop.service 2>/dev/null; then
    sudo systemctl restart jtop.service
    echo "✓ Restarted jtop service"
else
    echo "✓ jtop service not running (will start on demand)"
fi
echo ""

# ============================================
# STEP 3: GPIO Permissions
# ============================================
echo -e "${BLUE}[3/8] Setting up GPIO permissions...${NC}"

# Add user to gpio group if exists
if getent group gpio > /dev/null 2>&1; then
    if ! groups | grep -q "gpio"; then
        sudo usermod -aG gpio $USER
        echo "✓ Added user to gpio group"
    else
        echo "✓ User already in gpio group"
    fi
fi

# Setup GPIO export permissions
sudo chmod 666 /sys/class/gpio/export 2>/dev/null || true
sudo chmod 666 /sys/class/gpio/unexport 2>/dev/null || true
echo "✓ GPIO export permissions configured"
echo ""

# ============================================
# STEP 4: Update from Git
# ============================================
echo -e "${BLUE}[4/8] Updating from GitHub...${NC}"

if [ -d "$ROBOT_DIR/.git" ]; then
    cd "$ROBOT_DIR"
    
    # Check for local changes
    if ! git diff-index --quiet HEAD -- 2>/dev/null; then
        echo -e "${YELLOW}⚠ Local changes detected, stashing...${NC}"
        git stash save "Auto-stash before update on $(date)"
    fi
    
    # Fetch and pull
    git fetch origin main
    git pull origin main || {
        echo -e "${RED}✗ Git pull failed, resetting to origin/main...${NC}"
        git reset --hard origin/main
        git clean -fd
    }
    
    echo -e "${GREEN}✓ Code updated${NC}"
    echo "  Latest commit: $(git log -1 --oneline)"
else
    echo "⚠ Not a git repository, skipping update"
fi
echo ""

# ============================================
# STEP 5: Make Scripts Executable
# ============================================
echo -e "${BLUE}[5/8] Setting script permissions...${NC}"

cd "$ROBOT_DIR"
chmod +x scripts/*.py 2>/dev/null || true
chmod +x scripts/*.sh 2>/dev/null || true
chmod +x *.sh 2>/dev/null || true
echo "✓ All scripts are executable"
echo ""

# ============================================
# STEP 6: Build Workspace
# ============================================
echo -e "${BLUE}[6/8] Building workspace...${NC}"

cd "$WORKSPACE_DIR"
catkin_make
echo -e "${GREEN}✓ Build complete${NC}"
echo ""

# ============================================
# STEP 7: Deploy Scripts to Devel Space
# ============================================
echo -e "${BLUE}[7/8] Deploying scripts...${NC}"

SOURCE_SCRIPT="$ROBOT_DIR/scripts/sensors_actuators_node.py"
DEVEL_DIR="$WORKSPACE_DIR/devel/lib/elderly_bot"
DEVEL_SCRIPT="$DEVEL_DIR/sensors_actuators_node.py"

if [ -f "$SOURCE_SCRIPT" ]; then
    mkdir -p "$DEVEL_DIR"
    cp "$SOURCE_SCRIPT" "$DEVEL_SCRIPT"
    chmod +x "$DEVEL_SCRIPT"
    echo "✓ sensors_actuators_node.py deployed to devel space"
else
    echo -e "${YELLOW}⚠ Source script not found: $SOURCE_SCRIPT${NC}"
fi

# Deploy other scripts
for script in patrol_client.py mpu9250_node.py cloud_bridge_node.py; do
    if [ -f "$ROBOT_DIR/scripts/$script" ]; then
        cp "$ROBOT_DIR/scripts/$script" "$DEVEL_DIR/"
        chmod +x "$DEVEL_DIR/$script"
        echo "✓ $script deployed"
    fi
done
echo ""

# ============================================
# STEP 8: Source and Launch
# ============================================
echo -e "${BLUE}[8/8] Launching robot...${NC}"

# Source workspace for current session
source "$WORKSPACE_DIR/devel/setup.bash"
echo "✓ Workspace sourced"
echo ""

echo -e "${GREEN}=========================================="
echo "  ✓ System Ready!"
echo -e "==========================================${NC}"
echo ""
echo "Environment configured:"
echo "  • ROS workspace: sourced"
echo "  • GPIO permissions: configured"
echo "  • Jetson stats: configured"
echo "  • Scripts: deployed and executable"
echo ""
echo "Buzzer control (continuous beeping):"
echo "  • Start: rostopic pub /buzzer_command std_msgs/Bool \"data: true\""
echo "  • Stop:  rostopic pub /buzzer_command std_msgs/Bool \"data: false\""
echo ""
echo -e "${BLUE}Launching elderly_bot...${NC}"
echo ""

# Launch the robot
roslaunch elderly_bot bringup.launch
