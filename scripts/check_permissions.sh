#!/bin/bash
# AWS IoT Integration - Permissions Check and Repair Script
# ==========================================================
# Ensures all Python nodes are executable and certificates have correct permissions.
# Run this script on the Jetson Nano before launching ROS with cloud enabled.

echo "=========================================="
echo "AWS IoT Integration - Permissions Check"
echo "=========================================="
echo ""

# Get script directory
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
PKG_ROOT="$( cd "$SCRIPT_DIR/.." && pwd )"

echo "Package root: $PKG_ROOT"
echo ""

# === STEP 1: Python Nodes ===
echo "=== [1/3] Setting Python Script Permissions ==="
echo ""

PYTHON_SCRIPTS=(
    "$PKG_ROOT/scripts/cloud_bridge_node.py"
    "$PKG_ROOT/scripts/aws_connection_test.py"
    "$PKG_ROOT/scripts/final_handshake.py"
    "$PKG_ROOT/scripts/mpu9250_node.py"
    "$PKG_ROOT/scripts/patrol_client.py"
    "$PKG_ROOT/scripts/sensors_actuators_node.py"
)

SUCCESS_COUNT=0
FAIL_COUNT=0

for script in "${PYTHON_SCRIPTS[@]}"; do
    if [ -f "$script" ]; then
        chmod +x "$script"
        if [ $? -eq 0 ]; then
            echo "✓ $(basename $script) - executable"
            SUCCESS_COUNT=$((SUCCESS_COUNT + 1))
        else
            echo "✗ $(basename $script) - chmod failed"
            FAIL_COUNT=$((FAIL_COUNT + 1))
        fi
    else
        echo "⚠ $(basename $script) - not found (skipped)"
    fi
done

echo ""
echo "Python scripts: $SUCCESS_COUNT set, $FAIL_COUNT failed"
echo ""

# === STEP 2: Certificate Permissions ===
echo "=== [2/3] Setting Certificate Permissions ==="
echo ""

CERT_DIR="$PKG_ROOT/aws_certs"

if [ ! -d "$CERT_DIR" ]; then
    echo "✗ Certificate directory not found: $CERT_DIR"
    exit 1
fi

# Private key MUST be 600 (owner read/write only)
PRIVATE_KEY="$CERT_DIR/private.pem.key"
if [ -f "$PRIVATE_KEY" ]; then
    chmod 600 "$PRIVATE_KEY"
    PERMS=$(stat -c %a "$PRIVATE_KEY" 2>/dev/null || stat -f %A "$PRIVATE_KEY")
    if [ "$PERMS" = "600" ]; then
        echo "✓ private.pem.key - permissions 600 (owner read/write only)"
    else
        echo "✗ private.pem.key - permissions $PERMS (expected 600)"
        FAIL_COUNT=$((FAIL_COUNT + 1))
    fi
else
    echo "✗ private.pem.key - not found"
    FAIL_COUNT=$((FAIL_COUNT + 1))
fi

# Device certificate can be 644 (world readable)
DEVICE_CERT="$CERT_DIR/certificate.pem.crt"
if [ -f "$DEVICE_CERT" ]; then
    chmod 644 "$DEVICE_CERT"
    echo "✓ certificate.pem.crt - permissions 644"
else
    echo "✗ certificate.pem.crt - not found"
    FAIL_COUNT=$((FAIL_COUNT + 1))
fi

# Root CA can be 644 (world readable)
ROOT_CA="$CERT_DIR/AmazonRootCA1.pem"
if [ -f "$ROOT_CA" ]; then
    chmod 644 "$ROOT_CA"
    echo "✓ AmazonRootCA1.pem - permissions 644"
else
    echo "✗ AmazonRootCA1.pem - not found"
    FAIL_COUNT=$((FAIL_COUNT + 1))
fi

echo ""

# === STEP 3: Ownership Check ===
echo "=== [3/3] Checking File Ownership ==="
echo ""

CURRENT_USER=$(whoami)
echo "Current user: $CURRENT_USER"
echo ""

# Check if current user owns the certificate directory
CERT_OWNER=$(stat -c %U "$CERT_DIR" 2>/dev/null || stat -f %Su "$CERT_DIR")
if [ "$CERT_OWNER" = "$CURRENT_USER" ]; then
    echo "✓ Certificate directory owned by: $CURRENT_USER"
else
    echo "⚠ Certificate directory owned by: $CERT_OWNER (current: $CURRENT_USER)"
    echo "  You may need to run: sudo chown -R $CURRENT_USER:$CURRENT_USER $CERT_DIR"
fi

echo ""

# === SUMMARY ===
echo "=========================================="
echo "Summary"
echo "=========================================="
echo ""

if [ $FAIL_COUNT -eq 0 ]; then
    echo "✅ ALL CHECKS PASSED"
    echo ""
    echo "Permissions are correctly configured."
    echo "You can now run:"
    echo "  python scripts/final_handshake.py"
    echo "  roslaunch elderly_bot bringup.launch enable_cloud:=true"
    echo ""
    exit 0
else
    echo "⚠ $FAIL_COUNT ISSUE(S) FOUND"
    echo ""
    echo "Please fix the issues above before launching."
    echo ""
    exit 1
fi
