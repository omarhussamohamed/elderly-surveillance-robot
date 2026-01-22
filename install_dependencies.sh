#!/usr/bin/env bash

###############################################################################
# FAST FIX - Elderly Bot Quick Update/Fix Script
# Optimized for <30 second runtime (typically <15s when clean)
#
# Target: Ubuntu 18.04, ROS Melodic, Jetson Nano
# Safe to run multiple times (idempotent)
#
# Usage:
#   ./install_dependencies.sh              # Fast mode (default)
#   ./install_dependencies.sh --force-upgrade  # Include system apt upgrade
#   ./install_dependencies.sh --upgrade-jtop   # Reinstall jetson-stats
###############################################################################

set -euo pipefail  # Exit on error, undefined vars, pipe failures

# === CONFIGURATION ===
WORKSPACE_ROOT="${HOME}/catkin_ws"
ELDERLY_BOT_PATH="${WORKSPACE_ROOT}/src/elderly_bot"
FORCE_UPGRADE=false
UPGRADE_JTOP=false

# Parse arguments
for arg in "$@"; do
    case $arg in
        --force-upgrade) FORCE_UPGRADE=true ;;
        --upgrade-jtop) UPGRADE_JTOP=true ;;
        *) echo "Unknown option: $arg"; exit 1 ;;
    esac
done

# === HELPER FUNCTIONS ===
progress() {
    echo -e "\n\033[1;36m[$1/$2]\033[0m $3"
}

check_package_installed() {
    dpkg-query -W -f='${Status}' "$1" 2>/dev/null | grep -q "ok installed"
}

needs_catkin_rebuild() {
    # Check if devel/setup.bash exists and is newer than all source files
    if [ ! -f "${WORKSPACE_ROOT}/devel/setup.bash" ]; then
        return 0  # Need build (no prior build)
    fi
    
    # Find any .py, .cpp, .h, .launch, .urdf, .yaml modified after last build
    local newer_files
    newer_files=$(find "${ELDERLY_BOT_PATH}" \
        \( -name "*.py" -o -name "*.cpp" -o -name "*.h" -o -name "*.launch" -o -name "*.urdf" -o -name "*.yaml" \) \
        -newer "${WORKSPACE_ROOT}/devel/setup.bash" 2>/dev/null | wc -l)
    
    [ "$newer_files" -gt 0 ]
}

# === START ===
echo -e "\n\033[1;32m⚡ FAST-FIX: Elderly Bot Quick Update\033[0m"
echo "Mode: $([ "$FORCE_UPGRADE" = true ] && echo "FULL UPGRADE" || echo "FAST (skip apt)")"

# === [1/5] VALIDATE ENVIRONMENT ===
progress 1 5 "Validating environment..."

if [ ! -f /opt/ros/melodic/setup.bash ]; then
    echo "ERROR: ROS Melodic not found!"
    exit 1
fi

if [ ! -d "$ELDERLY_BOT_PATH" ]; then
    echo "ERROR: elderly_bot package not found at $ELDERLY_BOT_PATH"
    exit 1
fi

source /opt/ros/melodic/setup.bash &>/dev/null

# === [2/5] CLEAN PYTHON CACHE (FAST) ===
progress 2 5 "Cleaning Python cache..."

find "${ELDERLY_BOT_PATH}" -type d -name __pycache__ -exec rm -rf {} + 2>/dev/null || true
find "${ELDERLY_BOT_PATH}" -type f -name "*.pyc" -delete 2>/dev/null || true

# === [3/5] ENSURE DEPENDENCIES (BATCH SUDO) ===
progress 3 5 "Checking dependencies..."

# Build list of missing packages
MISSING_PKGS=()
REQUIRED_PKGS=(
    "python-pip"
    "python3-pip"
    "ros-melodic-robot-localization"
    "ros-melodic-gmapping"
    "ros-melodic-navigation"
    "ros-melodic-rplidar-ros"
    "ros-melodic-rosserial-python"
)

for pkg in "${REQUIRED_PKGS[@]}"; do
    if ! check_package_installed "$pkg"; then
        MISSING_PKGS+=("$pkg")
    fi
done

# Check Python GPIO library
if ! python3 -c "import Jetson.GPIO" &>/dev/null; then
    JTOP_MISSING=true
else
    JTOP_MISSING=false
fi

# Single sudo block (one password prompt max)
if [ ${#MISSING_PKGS[@]} -gt 0 ] || [ "$FORCE_UPGRADE" = true ] || [ "$UPGRADE_JTOP" = true ] || [ "$JTOP_MISSING" = true ]; then
    echo "  → Installing: ${MISSING_PKGS[*]:-none} + Python deps..."
    
    sudo bash -c "
        set -e
        
        # Fast apt update (only if needed)
        if [ '$FORCE_UPGRADE' = true ] || [ ${#MISSING_PKGS[@]} -gt 0 ]; then
            apt-get update -qq
        fi
        
        # Install missing packages (quiet, no recommends)
        if [ ${#MISSING_PKGS[@]} -gt 0 ]; then
            DEBIAN_FRONTEND=noninteractive apt-get install -qq -y --no-install-recommends ${MISSING_PKGS[*]}
        fi
        
        # Optional full upgrade
        if [ '$FORCE_UPGRADE' = true ]; then
            DEBIAN_FRONTEND=noninteractive apt-get upgrade -qq -y
        fi
        
        # Jetson.GPIO + jetson-stats (only if missing or requested)
        if [ '$JTOP_MISSING' = true ] || [ '$UPGRADE_JTOP' = true ]; then
            pip3 install -q --upgrade Jetson.GPIO jetson-stats 2>&1 | grep -v 'Requirement already satisfied' || true
        fi
        
        # Serial packages (idempotent)
        pip install -q --user pyserial pyyaml 2>&1 | grep -v 'Requirement already satisfied' || true
        pip3 install -q --user pyserial pyyaml 2>&1 | grep -v 'Requirement already satisfied' || true
        
        # Add user to dialout group (idempotent)
        usermod -a -G dialout $USER 2>/dev/null || true
        
        # Setup udev rules (idempotent)
        cat > /etc/udev/rules.d/99-rplidar.rules << 'UDEVEOF'
# RPLidar A1
KERNEL==\"ttyUSB*\", ATTRS{idVendor}==\"10c4\", ATTRS{idProduct}==\"ea60\", MODE:=\"0666\", GROUP:=\"dialout\", SYMLINK+=\"rplidar\"
UDEVEOF
        
        cat > /etc/udev/rules.d/99-esp32.rules << 'UDEVEOF'
# ESP32 Development Board
KERNEL==\"ttyUSB*\", ATTRS{idVendor}==\"1a86\", ATTRS{idProduct}==\"7523\", MODE:=\"0666\", GROUP:=\"dialout\", SYMLINK+=\"esp32\"
KERNEL==\"ttyUSB*\", ATTRS{idVendor}==\"10c4\", ATTRS{idProduct}==\"ea60\", MODE:=\"0666\", GROUP:=\"dialout\"
UDEVEOF
        
        udevadm control --reload-rules
        udevadm trigger
    "
    echo "  ✓ Dependencies resolved"
else
    echo "  ✓ All packages present (skip install)"
fi

# === [4/5] CONDITIONAL CATKIN BUILD ===
progress 4 5 "Checking workspace..."

if needs_catkin_rebuild; then
    echo "  → Source changes detected, rebuilding..."
    cd "${WORKSPACE_ROOT}"
    catkin_make -DCMAKE_BUILD_TYPE=Release --quiet 2>&1 | tail -n 5
    echo "  ✓ Workspace rebuilt"
else
    echo "  ✓ No source changes (skip build)"
fi

# Source workspace
source "${WORKSPACE_ROOT}/devel/setup.bash" &>/dev/null

# === [5/5] BASHRC + FINAL CHECKS ===
progress 5 5 "Finalizing setup..."

# Ensure workspace sourcing in bashrc (idempotent)
if ! grep -q "source ${WORKSPACE_ROOT}/devel/setup.bash" ~/.bashrc 2>/dev/null; then
    echo -e "\n# Elderly Bot workspace" >> ~/.bashrc
    echo "source /opt/ros/melodic/setup.bash" >> ~/.bashrc
    echo "source ${WORKSPACE_ROOT}/devel/setup.bash" >> ~/.bashrc
    echo "  → Added workspace to ~/.bashrc"
fi

# Quick GPIO permissions check
if [ -e /sys/class/gpio ]; then
    if ! groups | grep -q dialout; then
        echo "  ⚠ Not in dialout group (run: sudo usermod -a -G dialout \$USER && logout)"
    fi
fi

# === SUCCESS ===
echo -e "\n\033[1;32m✓ FAST-FIX COMPLETE\033[0m"
echo ""
echo "🚀 Quick Test:"
echo "   roslaunch elderly_bot bringup.launch"
echo ""
echo "🔧 Gas Sensor Reminder:"
echo "   If MQ-6 detected=TRUE in clean air:"
echo "   → Turn potentiometer CLOCKWISE (10-15 turns)"
echo "   → Watch for 'RAW PIN STUCK LOW' warning (>30s)"
echo "   → Goal: RAW HIGH in clean air, LOW only with gas+LED"
echo ""
echo "📖 Full Setup Guide:"
echo "   See README.md for Arduino IDE, ESP32 firmware, hardware setup"
echo ""
echo "⚡ Script runtime: ~$SECONDS seconds"


