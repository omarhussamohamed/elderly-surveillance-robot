#!/bin/bash
# Elderly Bot Update Script
# Updates code from Git, rebuilds workspace, and handles errors automatically

set -e  # Exit on error

echo "=========================================="
echo "  Elderly Bot Update Script"
echo "=========================================="
echo ""

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

# Function to handle errors
handle_error() {
    echo ""
    echo -e "${RED}=========================================="
    echo "  ERROR: Update failed!"
    echo -e "==========================================${NC}"
    echo ""
    echo -e "${YELLOW}Attempting recovery: resetting to origin/main...${NC}"
    
    cd ~/catkin_ws/src/elderly_bot
    
    # Save local changes (just in case)
    git stash save "Auto-stash before reset on $(date)"
    
    # Hard reset to remote
    git reset --hard origin/main
    git clean -fd
    
    echo ""
    echo -e "${GREEN}✓ Repository reset to clean state${NC}"
    echo ""
    echo "Retrying build..."
    
    # Retry build
    cd ~/catkin_ws
    catkin_make install
    source install/setup.bash
    
    echo ""
    echo -e "${GREEN}=========================================="
    echo "  Recovery successful!"
    echo -e "==========================================${NC}"
    cd
}

# Trap errors and call handle_error
trap 'handle_error' ERR

echo "Step 1/6: Navigating to repository..."
cd ~/catkin_ws/src/elderly_bot

echo "Step 2/6: Fetching updates from GitHub..."
git fetch origin main

echo "Step 3/6: Pulling latest code..."
git pull origin main

echo "Step 4/6: Setting script permissions..."
chmod +x scripts/*.py
chmod +x scripts/*.sh 2>/dev/null || true

echo "Step 5/6: Building workspace..."
cd ~/catkin_ws
catkin_make install

echo "Step 6/6: Sourcing setup..."
source install/setup.bash

cd

echo ""
echo -e "${GREEN}=========================================="
echo "  ✓ Update completed successfully!"
echo -e "==========================================${NC}"
echo ""
echo "Changes applied:"
git -C ~/catkin_ws/src/elderly_bot log -1 --oneline
echo ""
echo "To apply changes to current terminal:"
echo "  source ~/catkin_ws/install/setup.bash"
echo ""
echo "To restart robot:"
echo "  roslaunch elderly_bot bringup.launch"
echo ""
