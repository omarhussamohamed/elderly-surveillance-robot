#!/usr/bin/env bash

###############################################################################
# FAST UPDATE - Elderly Bot Quick Update & Launch Script
# Optimized for <30 second runtime (typically <15s when clean)
#
# Target: Ubuntu 18.04, ROS Melodic, Jetson Nano
# Safe to run multiple times (idempotent)
#
# Usage:
#   ./update_robot.sh              # Fast mode (no launch)
#   ./update_robot.sh --launch     # Fast mode + auto launch
#   ./update_robot.sh --force-git  # Force git pull
#   ./update_robot.sh --upgrade-jtop  # Reinstall jetson-stats
#   ./update_robot.sh --launch --force-git  # Combine flags
###############################################################################

set -euo pipefail  # Exit on error, undefined vars, pipe failures

# === CONFIGURATION ===
WORKSPACE_DIR="${HOME}/catkin_ws"
ROBOT_DIR="${WORKSPACE_DIR}/src/elderly_bot"
AUTO_LAUNCH=false
FORCE_GIT=false
UPGRADE_JTOP=false

# Parse arguments
for arg in "$@"; do
    case $arg in
        --launch) AUTO_LAUNCH=true ;;
        --force-git) FORCE_GIT=true ;;
        --upgrade-jtop) UPGRADE_JTOP=true ;;
        *) echo "Unknown option: $arg"; exit 1 ;;
    esac
done

# === HELPER FUNCTIONS ===
progress() {
    echo -e "\n\033[1;36m[$1/$2]\033[0m $3"
}

needs_catkin_rebuild() {
    # Check if devel/setup.bash exists and is newer than all source files
    if [ ! -f "${WORKSPACE_DIR}/devel/setup.bash" ]; then
        return 0  # Need build (no prior build)
    fi
    
    # Find any .py, .cpp, .h, .launch, .urdf, .yaml modified after last build
    local newer_files
    newer_files=$(find "${ROBOT_DIR}" \
        \( -name "*.py" -o -name "*.cpp" -o -name "*.h" -o -name "*.launch" -o -name "*.urdf" -o -name "*.yaml" \) \
        -newer "${WORKSPACE_DIR}/devel/setup.bash" 2>/dev/null | wc -l)
    
    [ "$newer_files" -gt 0 ]
}

# === START ===
echo -e "\n\033[1;32m⚡ FAST-UPDATE: Elderly Bot Quick Update\033[0m"
echo "Mode: $([ "$AUTO_LAUNCH" = true ] && echo "UPDATE + LAUNCH" || echo "UPDATE ONLY")"

# === [1/6] VALIDATE ENVIRONMENT ===
progress 1 6 "Validating environment..."

if [ ! -f /opt/ros/melodic/setup.bash ]; then
    echo "ERROR: ROS Melodic not found!"
    exit 1
fi

if [ ! -d "$ROBOT_DIR" ]; then
    echo "ERROR: elderly_bot package not found at $ROBOT_DIR"
    exit 1
fi

source /opt/ros/melodic/setup.bash &>/dev/null

# === [2/6] CLEAN PYTHON CACHE ===
progress 2 6 "Cleaning Python cache..."

find "${ROBOT_DIR}" -type d -name __pycache__ -exec rm -rf {} + 2>/dev/null || true
find "${ROBOT_DIR}" -type f -name "*.pyc" -delete 2>/dev/null || true

# === [3/6] GIT UPDATE (CONDITIONAL) ===
progress 3 6 "Checking for code updates..."

if [ -d "$ROBOT_DIR/.git" ]; then
    cd "$ROBOT_DIR"
    
    # Check if git pull needed
    git fetch origin main --quiet 2>/dev/null || true
    LOCAL=$(git rev-parse HEAD 2>/dev/null || echo "none")
    REMOTE=$(git rev-parse origin/main 2>/dev/null || echo "none")
    
    if [ "$LOCAL" != "$REMOTE" ] || [ "$FORCE_GIT" = true ]; then
        echo "  → Pulling latest changes..."
        
        # Stash local changes if any
        if ! git diff-index --quiet HEAD -- 2>/dev/null; then
            git stash save "Auto-stash $(date +%s)" --quiet
            echo "  → Stashed local changes"
        fi
        
        # Pull quietly
        git pull origin main --quiet 2>&1 | grep -v "Already up to date" || {
            echo "  ⚠ Git pull failed, resetting..."
            git reset --hard origin/main --quiet
            git clean -fd --quiet
        }
        echo "  ✓ Code updated: $(git log -1 --oneline 2>/dev/null || echo 'latest')"
    else
        echo "  ✓ Already up to date (skip pull)"
    fi
else
    echo "  ✓ Not a git repo (skip update)"
fi

# === [4/6] PERMISSIONS & ENVIRONMENT (BATCH SUDO) ===
progress 4 6 "Configuring permissions..."

# Check what needs doing
JTOP_MISSING=false
if ! python3 -c "import Jetson.GPIO" &>/dev/null; then
    JTOP_MISSING=true
fi

NEEDS_JTOP_GROUP=false
if ! groups | grep -qE "jtop|jetson_stats"; then
    NEEDS_JTOP_GROUP=true
fi

NEEDS_GPIO_GROUP=false
if getent group gpio &>/dev/null && ! groups | grep -q "gpio"; then
    NEEDS_GPIO_GROUP=true
fi

NEEDS_DIALOUT_GROUP=false
if ! groups | grep -q "dialout"; then
    NEEDS_DIALOUT_GROUP=true
fi

# Single sudo block (one password prompt)
if [ "$JTOP_MISSING" = true ] || [ "$UPGRADE_JTOP" = true ] || [ "$NEEDS_JTOP_GROUP" = true ] || [ "$NEEDS_GPIO_GROUP" = true ] || [ "$NEEDS_DIALOUT_GROUP" = true ]; then
    echo "  → Applying system permissions..."
    
    sudo bash -c "
        set -e
        
        # Jetson.GPIO (only if missing or requested)
        if [ '$JTOP_MISSING' = true ] || [ '$UPGRADE_JTOP' = true ]; then
            pip3 install -q --upgrade Jetson.GPIO jetson-stats 2>&1 | grep -v 'Requirement already satisfied' || true
        fi
        
        # User groups (idempotent)
        if [ '$NEEDS_JTOP_GROUP' = true ]; then
            usermod -aG jetson_stats $USER 2>/dev/null || usermod -aG jtop $USER 2>/dev/null || true
        fi
        
        if [ '$NEEDS_GPIO_GROUP' = true ]; then
            usermod -aG gpio $USER
        fi
        
        if [ '$NEEDS_DIALOUT_GROUP' = true ]; then
            usermod -aG dialout $USER
        fi
        
        # GPIO permissions
        chmod 666 /sys/class/gpio/export /sys/class/gpio/unexport 2>/dev/null || true
        
        # Restart jtop if running
        if systemctl is-active --quiet jtop.service 2>/dev/null; then
            systemctl restart jtop.service 2>/dev/null || true
        fi
    "
    echo "  ✓ Permissions configured"
else
    echo "  ✓ All permissions already set (skip)"
fi

# Ensure bashrc has workspace sourcing (idempotent)
if ! grep -q "source ${WORKSPACE_DIR}/devel/setup.bash" ~/.bashrc 2>/dev/null; then
    echo -e "\n# Elderly Bot workspace - $(date +%Y-%m-%d)" >> ~/.bashrc
    echo "source /opt/ros/melodic/setup.bash" >> ~/.bashrc
    echo "source ${WORKSPACE_DIR}/devel/setup.bash" >> ~/.bashrc
    echo "  → Added workspace to ~/.bashrc"
fi

# === [5/6] CONDITIONAL BUILD & DEPLOY ===
progress 5 6 "Checking workspace..."

if needs_catkin_rebuild; then
    echo "  → Source changes detected, rebuilding..."
    cd "${WORKSPACE_DIR}"
    catkin_make -DCMAKE_BUILD_TYPE=Release --quiet 2>&1 | tail -n 5
    echo "  ✓ Workspace rebuilt"
else
    echo "  ✓ No source changes (skip build)"
fi

# Source workspace
source "${WORKSPACE_DIR}/devel/setup.bash" &>/dev/null

# Deploy scripts to devel (fast copy)
DEVEL_DIR="${WORKSPACE_DIR}/devel/lib/elderly_bot"
mkdir -p "$DEVEL_DIR"

DEPLOY_COUNT=0
for script in sensors_actuators_node.py patrol_client.py mpu9250_node.py cloud_bridge_node.py; do
    if [ -f "${ROBOT_DIR}/scripts/$script" ]; then
        cp "${ROBOT_DIR}/scripts/$script" "$DEVEL_DIR/" 2>/dev/null || true
        chmod +x "$DEVEL_DIR/$script" 2>/dev/null || true
        DEPLOY_COUNT=$((DEPLOY_COUNT + 1))
    fi
done

if [ $DEPLOY_COUNT -gt 0 ]; then
    echo "  ✓ Deployed $DEPLOY_COUNT scripts to devel"
fi

# Make all scripts executable (fast)
chmod +x "${ROBOT_DIR}/scripts"/*.py "${ROBOT_DIR}"/*.sh 2>/dev/null || true

# === [6/6] FINALIZE ===
progress 6 6 "Finalizing..."

# Success message
echo -e "\n\033[1;32m✓ UPDATE COMPLETE\033[0m"
echo ""
echo "📊 System Status:"
echo "   • Workspace: sourced and ready"
echo "   • Scripts: deployed to devel/lib/elderly_bot"
echo "   • Permissions: configured (may need logout/login for groups)"
echo ""
echo "🔧 Gas Sensor Reminder:"
echo "   If MQ-6 detected=TRUE in clean air:"
echo "   → Turn potentiometer CLOCKWISE (10-15 turns)"
echo "   → Watch for 'RAW PIN STUCK LOW' warning (>30s)"
echo ""

# Auto launch if requested
if [ "$AUTO_LAUNCH" = true ]; then
    echo "🚀 Launching elderly_bot..."
    echo ""
    exec roslaunch elderly_bot bringup.launch
else
    echo "🚀 Ready to launch:"
    echo "   roslaunch elderly_bot bringup.launch"
    echo ""
    echo "⚡ Script runtime: ~$SECONDS seconds"
fi
