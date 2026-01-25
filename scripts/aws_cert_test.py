#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""
aws_cert_test.py - Test AWS IoT certificates and connection
"""
import ssl
import socket
import os
import sys

# Your paths - UPDATE THESE to match your actual paths
ENDPOINT = "a1k8itxfx77i0w-ats.iot.us-east-1.amazonaws.com"
PORT = 8883

# Check which path has your certificates
CERTS_DIR = None
possible_paths = [
    "/home/omar/catkin_ws/src/elderly_bot/aws_certs",
    "/home/omar/aws_certs"
]

for path in possible_paths:
    if os.path.exists(os.path.join(path, "AmazonRootCA1.pem")):
        CERTS_DIR = path
        break

if not CERTS_DIR:
    print("ERROR: No certificates found in any known location!")
    print("Checked:")
    for path in possible_paths:
        print("  - %s" % path)
    sys.exit(1)

ROOT_CA = os.path.join(CERTS_DIR, "AmazonRootCA1.pem")
CERT = os.path.join(CERTS_DIR, "certificate.pem.crt")
KEY = os.path.join(CERTS_DIR, "private.pem.key")

print("=== AWS IoT Certificate Test ===")

# 1. Check if files exist
print("1. Checking certificate files in %s..." % CERTS_DIR)
for path, name in [(ROOT_CA, "Root CA"), (CERT, "Certificate"), (KEY, "Private Key")]:
    if os.path.exists(path):
        print("   OK %s: %s" % (name, path))
    else:
        print("   ERROR %s NOT FOUND: %s" % (name, path))
        sys.exit(1)

# 2. Try to create SSL context
print("\n2. Testing SSL context...")
try:
    context = ssl.create_default_context(ssl.Purpose.SERVER_AUTH)
    context.load_verify_locations(ROOT_CA)
    context.load_cert_chain(CERT, KEY)
    context.check_hostname = False
    context.verify_mode = ssl.CERT_REQUIRED
    print("   OK SSL context created successfully")
except Exception as e:
    print("   ERROR SSL context failed: %s" % e)
    sys.exit(1)

# 3. Try TCP connection
print("\n3. Testing TCP connection to %s:%s..." % (ENDPOINT, PORT))
try:
    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    sock.settimeout(10)
    sock.connect((ENDPOINT, PORT))
    print("   OK TCP connection successful")
    sock.close()
except Exception as e:
    print("   ERROR TCP connection failed: %s" % e)
    print("   Check: Internet connection, firewall, AWS IoT endpoint")
    sys.exit(1)

# 4. Try TLS connection
print("\n4. Testing TLS handshake...")
try:
    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    sock.settimeout(10)
    tls_sock = context.wrap_socket(sock, server_hostname=ENDPOINT)
    tls_sock.connect((ENDPOINT, PORT))
    print("   OK TLS handshake successful")
    print("   OK Connected to AWS IoT Core!")
    tls_sock.close()
except Exception as e:
    print("   ERROR TLS handshake failed: %s" % e)
    print("   Check: Certificate expiration, AWS IoT policy, Thing status")
    sys.exit(1)

print("\n=== ALL TESTS PASSED ===")
print("Your AWS IoT certificates and connection are working correctly!")
print("\nNow try running the cloud bridge again.")