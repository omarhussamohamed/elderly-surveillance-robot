#!/usr/bin/env bash
###############################################################################
# FAST UPDATE - Elderly Bot Quick Update & Launch Script (Ultra-Optimized)
# Target runtime: <15s clean, <30s with build
###############################################################################

set -euo pipefail

# === CONFIG ===
WORKSPACE_DIR="${HOME}/catkin_ws"
ROBOT_DIR="${WORKSPACE_DIR}/src/elderly_bot"
AUTO_LAUNCH=false
FORCE_GIT=false
UPGRADE_JTOP=false

# Parse flags (fast)
while [[ $# -gt 0 ]]; do
    case $1 in
        --launch)     AUTO_LAUNCH=true; shift ;;
        --force-git)  FORCE_GIT=true;  shift ;;
        --upgrade-jtop) UPGRADE_JTOP=true; shift ;;
        *) echo "Unknown option: $1"; exit 1 ;;
    esac
done

progress() { echo -e "\n\033[1;36m[$1/6]\033[0m $2"; }

# === START ===
echo -e "\n\033[1;32m⚡ FAST-UPDATE: Elderly Bot\033[0m"

# [1/6] Validate (very fast checks)
progress 1 6 "Validating..."
[ -f /opt/ros/melodic/setup.bash ] || { echo "ERROR: ROS Melodic missing"; exit 1; }
[ -d "$ROBOT_DIR" ] || { echo "ERROR: elderly_bot missing"; exit 1; }
source /opt/ros/melodic/setup.bash &>/dev/null

# [2/6] Clean cache (parallel & fast)
progress 2 6 "Cleaning cache..."
find "$ROBOT_DIR" \( -name __pycache__ -o -name "*.pyc" \) -delete 2>/dev/null &

# [3/6] Git (only if needed — very quiet)
progress 3 6 "Git check..."
if [ -d "$ROBOT_DIR/.git" ]; then
    cd "$ROBOT_DIR"
    git fetch --quiet origin main 2>/dev/null || true
    LOCAL=$(git rev-parse HEAD 2>/dev/null)
    REMOTE=$(git rev-parse origin/main 2>/dev/null)
    if [ "$LOCAL" != "$REMOTE" ] || [ "$FORCE_GIT" = true ]; then
        echo "  → Updating..."
        git stash --quiet 2>/dev/null || true
        git pull --quiet --rebase origin main || { git reset --hard origin/main --quiet; git clean -fd --quiet; }
        echo "  ✓ Updated"
    else
        echo "  ✓ Up to date"
    fi
else
    echo "  ✓ No git repo"
fi
wait  # wait for background clean

# [4/6] Permissions & deps (single sudo block, only if needed)
progress 4 6 "Permissions..."

NEEDS_SUDO=false
if ! python3 -c "import Jetson.GPIO" &>/dev/null || [ "$UPGRADE_JTOP" = true ]; then NEEDS_SUDO=true; fi
if ! groups | grep -q "dialout"; then NEEDS_SUDO=true; fi

if [ "$NEEDS_SUDO" = true ]; then
    sudo bash -c "
        set -e
        if [ '$UPGRADE_JTOP' = true ] || ! python3 -c 'import Jetson.GPIO' &>/dev/null; then
            pip3 install --quiet --upgrade Jetson.GPIO jetson-stats || true
        fi
        usermod -aG dialout \$USER 2>/dev/null || true
        chmod 666 /sys/class/gpio/export /sys/class/gpio/unexport 2>/dev/null || true
    "
    echo "  ✓ Permissions set (logout/login may be needed for groups)"
else
    echo "  ✓ Permissions OK"
fi

# Fast sourcing check/add
if ! grep -q "source ${WORKSPACE_DIR}/devel/setup.bash" ~/.bashrc 2>/dev/null; then
    echo "source ${WORKSPACE_DIR}/devel/setup.bash" >> ~/.bashrc
    echo "  → Sourcing added to ~/.bashrc"
fi

# [5/6] Build (fastest possible check)
progress 5 6 "Workspace check..."

BUILD_NEEDED=false
LAST_BUILD="${WORKSPACE_DIR}/.last_build_time"
if [ ! -f "${WORKSPACE_DIR}/devel/setup.bash" ] || [ ! -f "$LAST_BUILD" ]; then
    BUILD_NEEDED=true
else
    # Faster than full find: check only key dirs, use timestamp file
    if find "$ROBOT_DIR" -type f \( -name "*.py" -o -name "*.cpp" -o -name "*.h" -o -name "*.launch" \) -newer "$LAST_BUILD" -print -quit | grep -q .; then
        BUILD_NEEDED=true
    fi
fi

if [ "$BUILD_NEEDED" = true ]; then
    echo "  → Changes detected → building..."
    cd "$WORKSPACE_DIR"
    catkin_make -j$(nproc) --quiet
    touch "$LAST_BUILD"  # update timestamp
    echo "  ✓ Built"
else
    echo "  ✓ No changes"
fi
source "${WORKSPACE_DIR}/devel/setup.bash" &>/dev/null

# [6/6] Deploy & finalize
progress 6 6 "Finalizing..."
mkdir -p "${WORKSPACE_DIR}/devel/lib/elderly_bot"
cp -u "${ROBOT_DIR}/scripts/"*.py "${WORKSPACE_DIR}/devel/lib/elderly_bot/" 2>/dev/null || true
chmod +x "${WORKSPACE_DIR}/devel/lib/elderly_bot/"*.py 2>/dev/null || true

echo -e "\n\033[1;32m✓ DONE in $SECONDS seconds\033[0m"
echo "   Gas reminder: If always detected → turn pot CLOCKWISE until RAW=HIGH in clean air"
if [ "$AUTO_LAUNCH" = true ]; then
    echo "Launching..."
    exec roslaunch elderly_bot bringup.launch
else
    echo "Run: roslaunch elderly_bot bringup.launch"
fi