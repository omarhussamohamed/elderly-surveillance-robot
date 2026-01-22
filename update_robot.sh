#!/usr/bin/env bash
###############################################################################
# FAST UPDATE - Elderly Bot (Optimized & Permissions-Hardened)
# Runtime: <15s clean, <30s with build
###############################################################################

set -euo pipefail

# === CONFIG ===
WORKSPACE_DIR="${HOME}/catkin_ws"
ROBOT_DIR="${WORKSPACE_DIR}/src/elderly_bot"
AUTO_LAUNCH=false
FORCE_GIT=false
UPGRADE_JTOP=false
DRY_RUN=false

# Parse flags
while [[ $# -gt 0 ]]; do
    case $1 in
        --launch)       AUTO_LAUNCH=true; shift ;;
        --force-git)    FORCE_GIT=true;  shift ;;
        --upgrade-jtop) UPGRADE_JTOP=true; shift ;;
        --dry-run)      DRY_RUN=true; shift ;;
        *) echo "Unknown option: $1"; exit 1 ;;
    esac
done

progress() { echo -e "\n\033[1;36m[$1/6]\033[0m $2"; }

changed() { echo -e "  \033[1;33m→\033[0m $1"; }

# === START ===
echo -e "\n\033[1;32m⚡ Elderly Bot Fast Update\033[0m"
[[ $DRY_RUN = true ]] && echo "  (DRY RUN - no changes applied)"

# [1/6] Validate
progress 1 6 "Validating..."
[ -f /opt/ros/melodic/setup.bash ] || { echo "ERROR: ROS Melodic missing"; exit 1; }
[ -d "$ROBOT_DIR" ] || { echo "ERROR: elderly_bot missing"; exit 1; }
source /opt/ros/melodic/setup.bash &>/dev/null

# [2/6] Clean cache
progress 2 6 "Cleaning cache..."
find "$ROBOT_DIR" \( -name __pycache__ -o -name "*.pyc" \) -delete 2>/dev/null || true

# [3/6] Git
progress 3 6 "Git update..."
if [ -d "$ROBOT_DIR/.git" ]; then
    cd "$ROBOT_DIR"
    git fetch --quiet origin main 2>/dev/null || true
    LOCAL=$(git rev-parse HEAD 2>/dev/null)
    REMOTE=$(git rev-parse origin/main 2>/dev/null)
    if [ "$LOCAL" != "$REMOTE" ] || [ "$FORCE_GIT" = true ]; then
        changed "Code changed or forced → pulling"
        git stash --quiet 2>/dev/null || true
        git pull --quiet origin main || {
            changed "Pull failed → force reset"
            git reset --hard origin/main --quiet
            git clean -fd --quiet
        }
    else
        echo "  ✓ Up to date"
    fi
else
    echo "  ✓ No git repo"
fi

# [4/6] Permissions & Groups (single sudo block)
progress 4 6 "Permissions..."

# Pre-checks (non-sudo)
CURRENT_GROUPS=$(id -nG)
NEEDS_SUDO=false
CHANGES_SUMMARY=""

if ! python3 -c "import Jetson.GPIO" &>/dev/null || [ "$UPGRADE_JTOP" = true ]; then
    NEEDS_SUDO=true
    CHANGES_SUMMARY+="Jetson.GPIO/jtop "
fi

for g in dialout gpio i2c jetson_stats jtop; do
    if ! echo "$CURRENT_GROUPS" | grep -qw "$g"; then
        if [[ "$g" = "i2c" && ! -e /dev/i2c-* ]]; then continue; fi
        if [[ "$g" = "gpio" && ! -e /sys/class/gpio ]]; then continue; fi
        NEEDS_SUDO=true
        CHANGES_SUMMARY+="$g "
    fi
done

if [ -e /sys/class/gpio/export ] && [ ! -w /sys/class/gpio/export ]; then
    NEEDS_SUDO=true
    CHANGES_SUMMARY+="GPIO sysfs "
fi
if ls /dev/i2c-* &>/dev/null && [ ! -w /dev/i2c-1 ]; then
    NEEDS_SUDO=true
    CHANGES_SUMMARY+="I2C "
fi

if [ "$NEEDS_SUDO" = true ]; then
    if [ "$DRY_RUN" = true ]; then
        changed "Would apply: $CHANGES_SUMMARY"
    else
        echo "  → Applying changes..."
        sudo bash -c "
            set -e
            CHANGED=false

            # Packages
            if [ '$UPGRADE_JTOP' = true ] || ! python3 -c 'import Jetson.GPIO' &>/dev/null; then
                export PYTHONWARNINGS=ignore
                pip3 install --quiet --upgrade Jetson.GPIO jetson-stats
                CHANGED=true
            fi

            # Groups
            for g in dialout gpio i2c jetson_stats jtop; do
                if ! id -nG $USER | grep -qw \$g; then
                    if [[ \$g = i2c && ! -e /dev/i2c-* ]] || [[ \$g = gpio && ! -e /sys/class/gpio ]]; then continue; fi
                    usermod -aG \$g $USER
                    CHANGED=true
                fi
            done

            # GPIO sysfs
            chmod 666 /sys/class/gpio/export /sys/class/gpio/unexport 2>/dev/null || true
            chmod -R 666 /sys/class/gpio/gpio*/direction /sys/class/gpio/gpio*/value 2>/dev/null || true

            # I2C
            if ls /dev/i2c-* &>/dev/null; then
                if getent group i2c >/dev/null; then
                    chgrp i2c /dev/i2c-* 2>/dev/null || true
                    chmod 660 /dev/i2c-* 2>/dev/null || true
                else
                    chmod 666 /dev/i2c-* 2>/dev/null || true
                fi
            fi

            [ \"\$CHANGED\" = true ] && echo '    ⚠ Group changes → logout/login needed'
        "
    fi
else
    echo "  ✓ All permissions OK"
fi

# Executable bits (always safe & fast)
chmod +x "${ROBOT_DIR}/scripts/"*.{py,sh} 2>/dev/null || true

# [5/6] Build
progress 5 6 "Workspace..."
LAST_BUILD="${WORKSPACE_DIR}/.last_build"
if [ ! -f "${WORKSPACE_DIR}/devel/setup.bash" ] || find "$ROBOT_DIR" -type f -newer "$LAST_BUILD" -print -quit 2>/dev/null | grep -q .; then
    changed "Building..."
    cd "$WORKSPACE_DIR"
    catkin_make -j$(nproc) --quiet 2>&1 | tail -n 5
    touch "$LAST_BUILD"
else
    echo "  ✓ No changes"
fi
source "${WORKSPACE_DIR}/devel/setup.bash" &>/dev/null

# [6/6] Deploy
progress 6 6 "Deploying..."
DEVEL_LIB="${WORKSPACE_DIR}/devel/lib/elderly_bot"
mkdir -p "$DEVEL_LIB"
cp -u "${ROBOT_DIR}/scripts/"*.py "$DEVEL_LIB/" 2>/dev/null || true
chmod +x "$DEVEL_LIB/"*.py 2>/dev/null || true

# Final
echo -e "\n\033[1;32m✓ COMPLETE in $SECONDS seconds\033[0m"
echo "   Gas tip: always detected? → turn pot CLOCKWISE"
if [ "$AUTO_LAUNCH" = true ]; then
    exec roslaunch elderly_bot bringup.launch
else
    echo "   Run: roslaunch elderly_bot bringup.launch"
fi