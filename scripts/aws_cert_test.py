#!/usr/bin/env python2
"""
aws_cert_test.py - Test AWS IoT certificates and connection
"""
import ssl
import socket
import os
import sys

# Your paths
ENDPOINT = "a1k8itxfx77i0w-ats.iot.us-east-1.amazonaws.com"
PORT = 8883
ROOT_CA = "/home/omar/catkin_ws/src/elderly_bot/aws_certs/AmazonRootCA1.pem"
CERT = "/home/omar/catkin_ws/src/elderly_bot/aws_certs/certificate.pem.crt"
KEY = "/home/omar/catkin_ws/src/elderly_bot/aws_certs/private.pem.key"

print("=== AWS IoT Certificate Test ===")

# 1. Check if files exist
print("1. Checking certificate files...")
for path, name in [(ROOT_CA, "Root CA"), (CERT, "Certificate"), (KEY, "Private Key")]:
    if os.path.exists(path):
        print("   ✓ %s: %s" % (name, path))
    else:
        print("   ✗ %s NOT FOUND: %s" % (name, path))
        sys.exit(1)

# 2. Try to create SSL context
print("\n2. Testing SSL context...")
try:
    context = ssl.create_default_context(ssl.Purpose.SERVER_AUTH)
    context.load_verify_locations(ROOT_CA)
    context.load_cert_chain(CERT, KEY)
    context.check_hostname = False
    context.verify_mode = ssl.CERT_REQUIRED
    print("   ✓ SSL context created successfully")
except Exception as e:
    print("   ✗ SSL context failed: %s" % e)
    sys.exit(1)

# 3. Try TCP connection
print("\n3. Testing TCP connection to %s:%s..." % (ENDPOINT, PORT))
try:
    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    sock.settimeout(10)
    sock.connect((ENDPOINT, PORT))
    print("   ✓ TCP connection successful")
    sock.close()
except Exception as e:
    print("   ✗ TCP connection failed: %s" % e)
    print("   Check: Internet connection, firewall, AWS IoT endpoint")
    sys.exit(1)

# 4. Try TLS connection
print("\n4. Testing TLS handshake...")
try:
    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    sock.settimeout(10)
    tls_sock = context.wrap_socket(sock, server_hostname=ENDPOINT)
    tls_sock.connect((ENDPOINT, PORT))
    print("   ✓ TLS handshake successful")
    print("   ✓ Connected to AWS IoT Core!")
    tls_sock.close()
except Exception as e:
    print("   ✗ TLS handshake failed: %s" % e)
    print("   Check: Certificate expiration, AWS IoT policy, Thing status")
    sys.exit(1)

print("\n=== ALL TESTS PASSED ===")
print("Your AWS IoT certificates and connection are working correctly!")
print("\nNow try running the cloud bridge again.")