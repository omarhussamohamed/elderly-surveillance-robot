#!/bin/bash
# AWS IoT Core Setup and Deployment Guide
# ========================================
# Complete setup script for AWS IoT Core integration

set -e

echo "=========================================="
echo "AWS IoT Core Setup for Elderly Bot"
echo "=========================================="

# Get script directory
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PKG_ROOT="$(dirname "$SCRIPT_DIR")"

echo ""
echo "Package root: $PKG_ROOT"
echo ""

# === STEP 1: Check Dependencies ===
echo "Step 1: Checking dependencies..."

# Check Python version
if command -v python3 &> /dev/null; then
    PYTHON_VERSION=$(python3 --version)
    echo "✓ Python3 found: $PYTHON_VERSION"
else
    echo "❌ Python3 not found"
    exit 1
fi

# Check pip3
if command -v pip3 &> /dev/null; then
    echo "✓ pip3 found"
else
    echo "❌ pip3 not found"
    echo "Install with: sudo apt install python3-pip"
    exit 1
fi

# Check if AWS SDK is installed
if python3 -c "import AWSIoTPythonSDK" 2>/dev/null; then
    echo "✓ AWSIoTPythonSDK already installed"
else
    echo "⚠ AWSIoTPythonSDK not installed"
    read -p "Install AWS IoT SDK now? (y/n) " -n 1 -r
    echo
    if [[ $REPLY =~ ^[Yy]$ ]]; then
        echo "Installing AWSIoTPythonSDK..."
        pip3 install AWSIoTPythonSDK
        echo "✓ Installation complete"
    else
        echo "❌ AWS SDK required. Install with: pip3 install AWSIoTPythonSDK"
        exit 1
    fi
fi

# Check YAML library
if python3 -c "import yaml" 2>/dev/null; then
    echo "✓ PyYAML installed"
else
    echo "Installing PyYAML..."
    pip3 install pyyaml
fi

echo ""

# === STEP 2: Check Certificates ===
echo "Step 2: Checking certificate files..."

CERT_DIR="$PKG_ROOT/aws_certs"
ROOT_CA="$CERT_DIR/AmazonRootCA1.pem"
DEVICE_CERT="$CERT_DIR/certificate.pem.crt"
PRIVATE_KEY="$CERT_DIR/private.pem.key"

if [ ! -f "$ROOT_CA" ]; then
    echo "❌ Root CA not found: $ROOT_CA"
    echo "   Download from: https://www.amazontrust.com/repository/AmazonRootCA1.pem"
    exit 1
fi

if [ ! -f "$DEVICE_CERT" ]; then
    echo "❌ Device certificate not found: $DEVICE_CERT"
    echo "   Download from AWS IoT Core console"
    exit 1
fi

if [ ! -f "$PRIVATE_KEY" ]; then
    echo "❌ Private key not found: $PRIVATE_KEY"
    echo "   Download from AWS IoT Core console"
    exit 1
fi

echo "✓ All certificate files present"
echo ""

# === STEP 3: Set Permissions ===
echo "Step 3: Setting certificate permissions..."

chmod 644 "$ROOT_CA"
chmod 644 "$DEVICE_CERT"
chmod 600 "$PRIVATE_KEY"

echo "✓ Root CA: 644 (readable)"
echo "✓ Device cert: 644 (readable)"
echo "✓ Private key: 600 (secure)"
echo ""

# === STEP 4: Validate Certificates ===
echo "Step 4: Validating certificates..."

if [ -f "$SCRIPT_DIR/validate_aws_certs.sh" ]; then
    bash "$SCRIPT_DIR/validate_aws_certs.sh"
else
    echo "⚠ Certificate validation script not found, skipping..."
fi

echo ""

# === STEP 5: Set Script Permissions ===
echo "Step 5: Setting script permissions..."

chmod +x "$SCRIPT_DIR/cloud_bridge_node.py"
chmod +x "$SCRIPT_DIR/aws_connection_test.py"
chmod +x "$SCRIPT_DIR/validate_aws_certs.sh"

echo "✓ All scripts executable"
echo ""

# === STEP 6: Configuration Instructions ===
echo "Step 6: Configuration instructions..."
echo ""
echo "=========================================="
echo "NEXT STEPS - Manual Configuration Required"
echo "=========================================="
echo ""
echo "1. Get your AWS IoT Core endpoint:"
echo "   • Go to AWS IoT Core Console"
echo "   • Click 'Settings' in the left menu"
echo "   • Copy 'Device data endpoint'"
echo "   • Example: a1b2c3d4e5f6g7-ats.iot.us-east-1.amazonaws.com"
echo ""
echo "2. Edit configuration file:"
echo "   nano $PKG_ROOT/config/cloud_config.yaml"
echo ""
echo "3. Update these fields:"
echo "   • aws_endpoint: [paste your endpoint]"
echo "   • client_id: elderly_bot_01 (or your Thing name)"
echo ""
echo "4. Test connection (without ROS):"
echo "   python3 $SCRIPT_DIR/aws_connection_test.py"
echo ""
echo "5. If test passes, enable cloud in config:"
echo "   • Set enable_cloud: true in cloud_config.yaml"
echo ""
echo "6. Launch with ROS:"
echo "   roslaunch elderly_bot bringup.launch enable_cloud:=true"
echo ""
echo "=========================================="
echo "✅ Setup Complete - Ready for Configuration"
echo "=========================================="
echo ""
