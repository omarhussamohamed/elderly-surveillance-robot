#!/bin/bash
###############################################################################
# AWS IoT Core Certificates Setup Script
#
# This script helps set up the AWS IoT certificates on Jetson Nano
#
# Usage:
#   1. Transfer your certificates to the Jetson
#   2. Run this script: ./setup_certificates.sh
#   3. Follow the prompts
###############################################################################

set -e

echo "=========================================="
echo "AWS IoT Certificates Setup"
echo "=========================================="
echo ""

# Check if running on Jetson
if [ ! -f /etc/nv_tegra_release ]; then
    echo "WARNING: This doesn't appear to be a Jetson device"
    read -p "Continue anyway? (y/n) " -n 1 -r
    echo
    if [[ ! $REPLY =~ ^[Yy]$ ]]; then
        exit 1
    fi
fi

# Define directories
CERT_DIR="$HOME/aws_certs"
PROJECT_CERT_DIR="$HOME/catkin_ws/src/elderly_bot/aws_certs"

echo "Certificate directory: $CERT_DIR"
echo ""

# Create directory if it doesn't exist
if [ ! -d "$CERT_DIR" ]; then
    echo "Creating certificate directory..."
    mkdir -p "$CERT_DIR"
    chmod 700 "$CERT_DIR"
    echo "✓ Directory created: $CERT_DIR"
else
    echo "✓ Certificate directory exists"
fi

echo ""
echo "=========================================="
echo "Step 1: Download Amazon Root CA"
echo "=========================================="
echo ""

if [ -f "$CERT_DIR/AmazonRootCA1.pem" ]; then
    echo "✓ AmazonRootCA1.pem already exists"
else
    echo "Downloading Amazon Root CA certificate..."
    wget -q https://www.amazontrust.com/repository/AmazonRootCA1.pem \
         -O "$CERT_DIR/AmazonRootCA1.pem"
    
    if [ $? -eq 0 ]; then
        chmod 644 "$CERT_DIR/AmazonRootCA1.pem"
        echo "✓ Downloaded AmazonRootCA1.pem"
    else
        echo "✗ Failed to download Root CA"
        echo "  Download manually from: https://www.amazontrust.com/repository/AmazonRootCA1.pem"
    fi
fi

echo ""
echo "=========================================="
echo "Step 2: Device Certificate"
echo "=========================================="
echo ""

if [ -f "$CERT_DIR/device-certificate.pem.crt" ]; then
    echo "✓ device-certificate.pem.crt already exists"
else
    echo "Device certificate not found."
    echo ""
    echo "Please copy your device certificate from AWS IoT Console:"
    echo "  1. On your development machine, download the certificate"
    echo "  2. Transfer it to Jetson using one of these methods:"
    echo ""
    echo "     Method A - SCP (from Windows/Linux):"
    echo "     scp /path/to/your-cert.pem.crt omar@192.168.1.29:~/aws_certs/device-certificate.pem.crt"
    echo ""
    echo "     Method B - USB drive:"
    echo "     cp /media/usb/your-cert.pem.crt ~/aws_certs/device-certificate.pem.crt"
    echo ""
    read -p "Press Enter when the certificate file is in place..."
    
    if [ -f "$CERT_DIR/device-certificate.pem.crt" ]; then
        chmod 644 "$CERT_DIR/device-certificate.pem.crt"
        echo "✓ Certificate file found and permissions set"
    else
        echo "✗ Certificate file still not found at: $CERT_DIR/device-certificate.pem.crt"
    fi
fi

echo ""
echo "=========================================="
echo "Step 3: Private Key"
echo "=========================================="
echo ""

if [ -f "$CERT_DIR/device-private.pem.key" ]; then
    echo "✓ device-private.pem.key already exists"
else
    echo "Private key not found."
    echo ""
    echo "⚠️  CRITICAL: Handle with care - this authenticates your device!"
    echo ""
    echo "Please copy your private key from AWS IoT Console:"
    echo "  1. On your development machine, download the private key"
    echo "  2. Transfer it to Jetson using one of these methods:"
    echo ""
    echo "     Method A - SCP (from Windows/Linux):"
    echo "     scp /path/to/your-private.pem.key omar@192.168.1.29:~/aws_certs/device-private.pem.key"
    echo ""
    echo "     Method B - USB drive:"
    echo "     cp /media/usb/your-private.pem.key ~/aws_certs/device-private.pem.key"
    echo ""
    read -p "Press Enter when the private key file is in place..."
    
    if [ -f "$CERT_DIR/device-private.pem.key" ]; then
        chmod 600 "$CERT_DIR/device-private.pem.key"
        echo "✓ Private key found and secured (permissions: 600)"
    else
        echo "✗ Private key still not found at: $CERT_DIR/device-private.pem.key"
    fi
fi

echo ""
echo "=========================================="
echo "Verification"
echo "=========================================="
echo ""

echo "Checking certificate directory contents..."
ls -lh "$CERT_DIR"

echo ""
echo "File status:"

if [ -f "$CERT_DIR/AmazonRootCA1.pem" ]; then
    echo "  ✓ Root CA certificate"
else
    echo "  ✗ Root CA certificate MISSING"
fi

if [ -f "$CERT_DIR/device-certificate.pem.crt" ]; then
    echo "  ✓ Device certificate"
else
    echo "  ✗ Device certificate MISSING"
fi

if [ -f "$CERT_DIR/device-private.pem.key" ]; then
    echo "  ✓ Private key"
    
    # Check permissions
    PERMS=$(stat -c "%a" "$CERT_DIR/device-private.pem.key" 2>/dev/null || stat -f "%A" "$CERT_DIR/device-private.pem.key" 2>/dev/null)
    if [ "$PERMS" = "600" ]; then
        echo "    Permissions: OK (600)"
    else
        echo "    WARNING: Permissions should be 600, currently $PERMS"
        echo "    Fix with: chmod 600 $CERT_DIR/device-private.pem.key"
    fi
else
    echo "  ✗ Private key MISSING"
fi

echo ""
echo "=========================================="
echo "Next Steps"
echo "=========================================="
echo ""

if [ -f "$CERT_DIR/AmazonRootCA1.pem" ] && \
   [ -f "$CERT_DIR/device-certificate.pem.crt" ] && \
   [ -f "$CERT_DIR/device-private.pem.key" ]; then
    echo "✓ All certificates are in place!"
    echo ""
    echo "1. Update config/cloud_config.yaml with your AWS IoT endpoint"
    echo "   Example: a1b2c3d4e5f6g7-ats.iot.us-east-1.amazonaws.com"
    echo ""
    echo "2. Set enable_cloud: true in cloud_config.yaml"
    echo ""
    echo "3. Launch with cloud enabled:"
    echo "   roslaunch elderly_bot bringup.launch enable_cloud:=true"
    echo ""
    echo "4. Monitor connection:"
    echo "   rostopic echo /cloud_bridge/status"
else
    echo "✗ Some certificates are missing. Please complete the setup."
    echo ""
    echo "Missing files should be placed in: $CERT_DIR"
fi

echo ""
echo "=========================================="
echo "Setup Complete"
echo "=========================================="
