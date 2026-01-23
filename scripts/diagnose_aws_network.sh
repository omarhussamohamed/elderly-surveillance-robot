#!/bin/bash
# AWS IoT Core Network Connectivity Diagnostics
# ==============================================
# Run this script on Jetson Nano to diagnose connection issues

echo "=========================================="
echo "AWS IoT Core Network Diagnostics"
echo "=========================================="
echo ""

# Your AWS endpoint
AWS_ENDPOINT="a1k8itxfx77i0w-ats.iot.eu-north-1.amazonaws.com"
PORT_8883=8883
PORT_443=443

echo "Testing connectivity to: $AWS_ENDPOINT"
echo ""

# === TEST 1: DNS Resolution ===
echo "=== Test 1: DNS Resolution ==="
if host $AWS_ENDPOINT > /dev/null 2>&1; then
    echo "✓ DNS resolution successful"
    host $AWS_ENDPOINT | grep "has address"
else
    echo "✗ DNS resolution FAILED"
    echo "  Check your DNS settings or internet connection"
    exit 1
fi
echo ""

# === TEST 2: Ping Test ===
echo "=== Test 2: Ping Test ==="
if ping -c 3 -W 5 $AWS_ENDPOINT > /dev/null 2>&1; then
    echo "✓ Ping successful (host is reachable)"
else
    echo "⚠ Ping failed (might be blocked by firewall, but not critical)"
fi
echo ""

# === TEST 3: Port 8883 Connectivity ===
echo "=== Test 3: MQTT Port 8883 Connectivity ==="
echo "Testing TCP connection to $AWS_ENDPOINT:$PORT_8883..."

if command -v nc > /dev/null 2>&1; then
    # Using netcat with 5 second timeout
    if timeout 5 nc -zv $AWS_ENDPOINT $PORT_8883 2>&1 | grep -q "succeeded\|open"; then
        echo "✓ Port 8883 is OPEN and reachable"
        echo "  Your network allows MQTT over TLS on port 8883"
    else
        echo "✗ Port 8883 is BLOCKED or unreachable"
        echo "  This is likely why your connection times out"
        echo "  Solution: Use port 443 instead (see below)"
    fi
else
    # Fallback: try with telnet or timeout
    if timeout 5 bash -c "echo -n > /dev/tcp/$AWS_ENDPOINT/$PORT_8883" 2>/dev/null; then
        echo "✓ Port 8883 is OPEN and reachable"
    else
        echo "✗ Port 8883 appears to be BLOCKED"
        echo "  Your router/ISP may be blocking this port"
    fi
fi
echo ""

# === TEST 4: Port 443 Connectivity ===
echo "=== Test 4: HTTPS Port 443 Connectivity ==="
echo "Testing TCP connection to $AWS_ENDPOINT:$PORT_443..."

if command -v nc > /dev/null 2>&1; then
    if timeout 5 nc -zv $AWS_ENDPOINT $PORT_443 2>&1 | grep -q "succeeded\|open"; then
        echo "✓ Port 443 is OPEN and reachable"
        echo "  You can use MQTT over WebSockets on port 443 as alternative"
    else
        echo "✗ Port 443 is also blocked"
        echo "  This is unusual - check your firewall settings"
    fi
else
    if timeout 5 bash -c "echo -n > /dev/tcp/$AWS_ENDPOINT/$PORT_443" 2>/dev/null; then
        echo "✓ Port 443 is OPEN"
    else
        echo "✗ Port 443 is blocked"
    fi
fi
echo ""

# === TEST 5: TLS Handshake Test ===
echo "=== Test 5: TLS/SSL Handshake Test ==="
if command -v openssl > /dev/null 2>&1; then
    echo "Testing TLS handshake on port 8883..."
    if timeout 10 openssl s_client -connect $AWS_ENDPOINT:$PORT_8883 -CAfile ~/catkin_ws/src/elderly_bot/aws_certs/AmazonRootCA1.pem </dev/null 2>&1 | grep -q "Verify return code: 0"; then
        echo "✓ TLS handshake successful on port 8883"
        echo "  Certificates are valid and trusted"
    else
        echo "⚠ TLS handshake had issues on port 8883"
        echo "  Trying port 443..."
        
        if timeout 10 openssl s_client -connect $AWS_ENDPOINT:$PORT_443 -servername $AWS_ENDPOINT </dev/null 2>&1 | grep -q "Verify return code: 0"; then
            echo "✓ TLS handshake successful on port 443"
            echo "  Port 443 with ALPN should work"
        fi
    fi
else
    echo "⚠ OpenSSL not found, skipping TLS test"
fi
echo ""

# === SUMMARY ===
echo "=========================================="
echo "DIAGNOSTIC SUMMARY"
echo "=========================================="
echo ""
echo "If port 8883 is BLOCKED:"
echo "  1. Update cloud_config.yaml to use port 443"
echo "  2. Enable ALPN in your connection script"
echo "  3. See: aws_connection_test_port443.py"
echo ""
echo "If TLS handshake fails:"
echo "  1. Check AWS IoT policy attached to certificate"
echo "  2. Verify certificate is ACTIVATED in AWS console"
echo "  3. Check system time is correct: date"
echo ""
echo "Next steps:"
echo "  - Run: python aws_connection_test.py (for verbose logs)"
echo "  - Check AWS IoT Console → Monitor → Logs"
echo ""
