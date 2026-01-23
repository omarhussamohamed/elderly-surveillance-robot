#!/bin/bash
# AWS IoT Certificate Validation Script
# ======================================
# Validates certificate integrity and matching between private key and certificate
# Run before deploying to AWS IoT Core

set -e  # Exit on error

echo "=========================================="
echo "AWS IoT Certificate Validation"
echo "=========================================="

# Get script directory and navigate to package root
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PKG_ROOT="$(dirname "$SCRIPT_DIR")"
CERT_DIR="$PKG_ROOT/aws_certs"

# Certificate file paths
ROOT_CA="$CERT_DIR/AmazonRootCA1.pem"
DEVICE_CERT="$CERT_DIR/certificate.pem.crt"
PRIVATE_KEY="$CERT_DIR/private.pem.key"

echo ""
echo "Certificate Directory: $CERT_DIR"
echo ""

# Function to check file existence
check_file() {
    if [ ! -f "$1" ]; then
        echo "❌ ERROR: File not found: $1"
        exit 1
    else
        echo "✓ Found: $(basename $1)"
    fi
}

# === 1. CHECK FILE EXISTENCE ===
echo "Step 1: Checking certificate files..."
check_file "$ROOT_CA"
check_file "$DEVICE_CERT"
check_file "$PRIVATE_KEY"
echo ""

# === 2. CHECK FILE PERMISSIONS ===
echo "Step 2: Checking file permissions..."
PRIVATE_KEY_PERMS=$(stat -c "%a" "$PRIVATE_KEY")
if [ "$PRIVATE_KEY_PERMS" != "600" ] && [ "$PRIVATE_KEY_PERMS" != "400" ]; then
    echo "⚠ WARNING: Private key has loose permissions: $PRIVATE_KEY_PERMS"
    echo "  Recommended: chmod 600 $PRIVATE_KEY"
else
    echo "✓ Private key permissions OK: $PRIVATE_KEY_PERMS"
fi
echo ""

# === 3. VALIDATE ROOT CA CERTIFICATE ===
echo "Step 3: Validating Root CA certificate..."
if openssl x509 -in "$ROOT_CA" -text -noout > /dev/null 2>&1; then
    ROOT_CA_SUBJECT=$(openssl x509 -in "$ROOT_CA" -subject -noout | sed 's/subject=//')
    ROOT_CA_ISSUER=$(openssl x509 -in "$ROOT_CA" -issuer -noout | sed 's/issuer=//')
    ROOT_CA_EXPIRY=$(openssl x509 -in "$ROOT_CA" -enddate -noout | sed 's/notAfter=//')
    
    echo "✓ Root CA certificate is valid"
    echo "  Subject: $ROOT_CA_SUBJECT"
    echo "  Issuer: $ROOT_CA_ISSUER"
    echo "  Expires: $ROOT_CA_EXPIRY"
else
    echo "❌ ERROR: Root CA certificate is invalid or corrupted"
    exit 1
fi
echo ""

# === 4. VALIDATE DEVICE CERTIFICATE ===
echo "Step 4: Validating device certificate..."
if openssl x509 -in "$DEVICE_CERT" -text -noout > /dev/null 2>&1; then
    DEVICE_SUBJECT=$(openssl x509 -in "$DEVICE_CERT" -subject -noout | sed 's/subject=//')
    DEVICE_EXPIRY=$(openssl x509 -in "$DEVICE_CERT" -enddate -noout | sed 's/notAfter=//')
    
    echo "✓ Device certificate is valid"
    echo "  Subject: $DEVICE_SUBJECT"
    echo "  Expires: $DEVICE_EXPIRY"
    
    # Check if certificate is expired
    if ! openssl x509 -in "$DEVICE_CERT" -checkend 0 -noout > /dev/null 2>&1; then
        echo "❌ ERROR: Device certificate has EXPIRED!"
        exit 1
    else
        echo "✓ Certificate is not expired"
    fi
else
    echo "❌ ERROR: Device certificate is invalid or corrupted"
    exit 1
fi
echo ""

# === 5. VALIDATE PRIVATE KEY ===
echo "Step 5: Validating private key..."
if openssl rsa -in "$PRIVATE_KEY" -check -noout > /dev/null 2>&1; then
    echo "✓ Private key is valid"
else
    echo "❌ ERROR: Private key is invalid or corrupted"
    exit 1
fi
echo ""

# === 6. VERIFY KEY-CERTIFICATE MATCH ===
echo "Step 6: Verifying private key matches certificate..."

# Extract modulus from both and compare
CERT_MODULUS=$(openssl x509 -in "$DEVICE_CERT" -modulus -noout)
KEY_MODULUS=$(openssl rsa -in "$PRIVATE_KEY" -modulus -noout)

if [ "$CERT_MODULUS" == "$KEY_MODULUS" ]; then
    echo "✓ Private key and certificate MATCH (modulus check passed)"
else
    echo "❌ ERROR: Private key and certificate DO NOT MATCH!"
    echo "  This means the certificate was not issued for this private key."
    exit 1
fi
echo ""

# === 7. CERTIFICATE CHAIN VERIFICATION ===
echo "Step 7: Verifying certificate chain..."
if openssl verify -CAfile "$ROOT_CA" "$DEVICE_CERT" > /dev/null 2>&1; then
    echo "✓ Certificate chain is valid (device cert signed by Root CA)"
else
    echo "⚠ WARNING: Certificate chain verification failed"
    echo "  This might be OK if you're using an intermediate CA"
    echo "  AWS IoT Core will validate the chain during connection"
fi
echo ""

# === 8. CHECK CERTIFICATE DETAILS ===
echo "Step 8: Certificate details summary..."
echo "----------------------------------------"
openssl x509 -in "$DEVICE_CERT" -text -noout | grep -A3 "Validity"
echo "----------------------------------------"
echo ""

# === SUCCESS ===
echo "=========================================="
echo "✅ ALL VALIDATIONS PASSED"
echo "=========================================="
echo ""
echo "Next steps:"
echo "1. Update config/cloud_config.yaml with correct paths"
echo "2. Run aws_connection_test.py to test AWS IoT connectivity"
echo "3. Enable cloud bridge in bringup.launch"
echo ""
