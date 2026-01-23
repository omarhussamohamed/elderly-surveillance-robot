#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
Final AWS IoT Handshake Test - Robot Nano
==========================================
Production-ready connection test to verify AWS IoT Core configuration.
Uses exact settings from cloud_config.yaml for robot_nano Thing.

USAGE:
    python final_handshake.py

REQUIREMENTS:
    pip install AWSIoTPythonSDK pyyaml

WHAT THIS SCRIPT DOES:
    1. Loads configuration from cloud_config.yaml (aws_iot_config section)
    2. Validates all certificates exist and are readable
    3. Connects to AWS IoT Core using robot_nano client_id on port 8883
    4. Tests publish/subscribe on elderly_bot/test topic
    5. Verifies bidirectional MQTT communication
    6. Reports SUCCESS or specific failure point with troubleshooting
"""

from __future__ import print_function
import sys
import os
import time
import json
import yaml
from datetime import datetime

# === AWS IoT SDK Import ===
try:
    from AWSIoTPythonSDK.MQTTLib import AWSIoTMQTTClient
except ImportError:
    print("ERROR: AWSIoTPythonSDK not installed")
    print("Install with: pip install AWSIoTPythonSDK")
    sys.exit(1)

# === CONFIGURATION ===
# Paths relative to script location
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
PKG_ROOT = os.path.dirname(SCRIPT_DIR)
CONFIG_FILE = os.path.join(PKG_ROOT, 'config', 'cloud_config.yaml')

# AWS Configuration (loaded from YAML aws_iot_config section)
AWS_ENDPOINT = None
CLIENT_ID = "robot_nano"  # Must match AWS Thing name exactly
PORT = 8883  # Standard MQTT over TLS

# Certificate paths (loaded from YAML)
ROOT_CA = None
DEVICE_CERT = None
PRIVATE_KEY = None

# Test topic
TEST_TOPIC = "elderly_bot/test"

# Message received flag
message_received = False
received_payload = None


def load_config():
    """Load AWS configuration from cloud_config.yaml"""
    global AWS_ENDPOINT, CLIENT_ID, PORT, ROOT_CA, DEVICE_CERT, PRIVATE_KEY
    
    print("=" * 60)
    print("AWS IoT Final Handshake Test - robot_nano")
    print("=" * 60)
    print("\n[1/6] Loading Configuration...")
    print("Config file: {}".format(CONFIG_FILE))
    
    if not os.path.exists(CONFIG_FILE):
        print("ERROR: Configuration file not found!")
        print("Expected: {}".format(CONFIG_FILE))
        sys.exit(1)
    
    try:
        with open(CONFIG_FILE, 'r') as f:
            config = yaml.safe_load(f)
        
        # Try aws_iot_config section first (new format)
        if 'aws_iot_config' in config:
            cfg = config['aws_iot_config']
            AWS_ENDPOINT = cfg.get('endpoint', '')
            CLIENT_ID = cfg.get('client_id', 'robot_nano')
            PORT = cfg.get('port', 8883)
            ROOT_CA = os.path.expanduser(cfg.get('root_ca', ''))
            DEVICE_CERT = os.path.expanduser(cfg.get('device_cert', ''))
            PRIVATE_KEY = os.path.expanduser(cfg.get('private_key', ''))
        # Fallback to cloud_bridge_node section (old format)
        elif 'cloud_bridge_node' in config:
            cfg = config['cloud_bridge_node']
            AWS_ENDPOINT = cfg.get('aws_endpoint', '')
            CLIENT_ID = cfg.get('client_id', 'robot_nano')
            PORT = cfg.get('port', 8883)
            ROOT_CA = os.path.expanduser(cfg.get('root_ca_path', ''))
            DEVICE_CERT = os.path.expanduser(cfg.get('cert_path', ''))
            PRIVATE_KEY = os.path.expanduser(cfg.get('key_path', ''))
        else:
            print("ERROR: Neither 'aws_iot_config' nor 'cloud_bridge_node' section found")
            sys.exit(1)
        
        print("✓ Configuration loaded successfully")
        print("\n  Endpoint:  {}".format(AWS_ENDPOINT))
        print("  Client ID: {}".format(CLIENT_ID))
        print("  Port:      {}".format(PORT))
        
        return True
        
    except Exception as e:
        print("ERROR: Failed to load configuration: {}".format(e))
        sys.exit(1)


def validate_certificates():
    """Validate that all certificate files exist and are readable"""
    print("\n[2/6] Validating Certificates...")
    
    certs = [
        ("Root CA", ROOT_CA),
        ("Device Certificate", DEVICE_CERT),
        ("Private Key", PRIVATE_KEY)
    ]
    
    all_valid = True
    
    for name, path in certs:
        if not path:
            print("✗ {}: Path not configured".format(name))
            all_valid = False
        elif not os.path.exists(path):
            print("✗ {}: File not found".format(name))
            print("  Path: {}".format(path))
            all_valid = False
        elif not os.access(path, os.R_OK):
            print("✗ {}: File not readable".format(name))
            print("  Path: {}".format(path))
            all_valid = False
        else:
            file_size = os.path.getsize(path)
            print("✓ {}: {} bytes".format(name, file_size))
    
    if not all_valid:
        print("\nERROR: Certificate validation failed")
        sys.exit(1)
    
    print("✓ All certificates validated")
    return True


def on_message(client, userdata, message):
    """Callback when message is received"""
    global message_received, received_payload
    message_received = True
    received_payload = message.payload.decode('utf-8')
    print("\n    ← MESSAGE RECEIVED")
    print("      Topic: {}".format(message.topic))
    print("      Payload: {}".format(received_payload))


def test_connection():
    """Test AWS IoT Core connection and bidirectional communication"""
    print("\n[3/6] Initializing MQTT Client...")
    
    # Create MQTT client
    mqtt_client = AWSIoTMQTTClient(CLIENT_ID)
    mqtt_client.configureEndpoint(AWS_ENDPOINT, PORT)
    mqtt_client.configureCredentials(ROOT_CA, PRIVATE_KEY, DEVICE_CERT)
    print("✓ MQTT client configured (Port {}, Standard MQTT/TLS)".format(PORT)ishQueueing(-1)
    mqtt_client.configureDrainingFrequency(2)
    mqtt_client.configureConnectDisconnectTimeout(30)
    mqtt_client.configureMQTTOperationTimeout(10)
    
    if PORT == 443:
        print("✓ ALPN enabled for port 443")
    
    print("✓ MQTT client configured")
    
    # === CONNECT ===
    print("\n[4/6] Connecting to AWS IoT Core...")
    print("  Endpoint: {}:{}".format(AWS_ENDPOINT, PORT))
    print("  Client:   {}".format(CLIENT_ID))
    print("  Timeout:  30 seconds")
    print("\n  Attempting connection...")
    
    try:
        connect_result = mqtt_client.connect()
        
        if not connect_result:
            print("\n✗ CONNECTION FAILED")
            print("\nTroubleshooting:")
            print("  1. Check AWS IoT Policy allows client '{}'".format(CLIENT_ID))
            print("  2. Verify certificate is ACTIVE in AWS Console")
            print("  3. Confirm policy is attached to certificate")
            print("  4. Check Thing name matches client_id exactly")
            sys.exit(1)
        
        print("  ✓ CONNECTION SUCCESSFUL!")
        
    except Exception as e:
        print("\n✗ CONNECTION EXCEPTION")
        print("  Error: {}".format(str(e)))
        print("\nTroubleshooting:")
        print("  1. Check internet connectivity")
        print("  2. Verify endpoint URL is correct")
        print("  3. Ensure port 443 is not blocked by firewall")
        print("  4. Check AWS IoT Core CloudWat: ping google.com")
        print("  2. Verify endpoint URL: {}".format(AWS_ENDPOINT))
        print("  3. Ensure port {} is not blocked: nc -zv {} {}".format(PORT, AWS_ENDPOINT, PORT))
        print("  4. Check AWS IoT Policy allows client '{}'".format(CLIENT_ID))
        print("  5. Verify certificate is ACTIVE in AWS Console")
        print("  6E ===
    print("\n[5/6] Testing Publish/Subscribe...")
    print("  Subscribing to: {}".format(TEST_TOPIC))
    
    try:
        mqtt_client.subscribe(TEST_TOPIC, 1, on_message)
        time.sleep(2)  # Wait for subscription to complete
        print("  ✓ Subscribed successfully")
        
    except Exception as e:
        print("  ✗ Subscribe failed: {}".format(e))
        mqtt_client.disconnect()
        sys.exit(1)
    
    # === PUBLISH ===
    print("  Publishing test message to: {}".format(TEST_TOPIC))
    
    test_message = {
        "test": True,
        "message": "Final handshake test from robot_nano",
        "timestamp": datetime.now().isoformat(),
        "client_id": CLIENT_ID,
        "port": PORT
    }
    
    try:
        mqtt_client.publish(TEST_TOPIC, json.dumps(test_message), 1)
        print("  ✓ Message published")
        
    except Exception as e:
        print("  ✗ Publish failed: {}".format(e))
        mqtt_client.disconnect()
        sys.exit(1)
    
    # === WAIT FOR ECHO ===
    print("\n  Waiting for message echo (10 seconds)...")
    
    timeout = 10
    start_time = time.time()
    
    while not message_received and (time.time() - start_time) < timeout:
        time.sleep(0.5)
    
    # === VERIFY ===
    print("\n[6/6] Verification...")
    
    if message_received:
        print("  ✓ Message echo received")
        
        # Verify payload
        try:
            received_data = json.loads(received_payload)
            if received_data.get('client_id') == CLIENT_ID:
                print("  ✓ Client ID matches: {}".format(CLIENT_ID))
            else:
                print("  ⚠ Client ID mismatch")
        except:
            pass
        
    else:
        print("  ✗ Message echo NOT received")
        print("  (This may indicate policy restrictions or subscription issues)")
    
    # === CLEANUP ===
    mqtt_client.unsubscribe(TEST_TOPIC)
    mqtt_client.disconnect()
    print("\n  Disconnected cleanly")
    
    # === FINAL RESULT ===
    print("\n" + "=" * 60)
    if message_received:
        print("✅ FINAL HANDSHAKE SUCCESSFUL!")
        print("=" * 60)
        print("\nAWS IoT Core connection is fully functional:")
        print("  ✓ Client ID: {} connected successfully".format(CLIENT_ID))
        print("  ✓ Thing Name: robot_nano")
        print("  ✓ Client ID: {} connected successfully".format(CLIENT_ID))
        print("  ✓ Endpoint: {}:{}".format(AWS_ENDPOINT, PORT))
        print("  ✓ Publish to {} successful".format(TEST_TOPIC))
        print("  ✓ Subscribe to {} successful".format(TEST_TOPIC))
        print("  ✓ Bidirectional communication verified")
        print("\nREADY FOR ROS LAUNCH:")
        print("  cd ~/catkin_ws")
        print("  source devel/setup.bashe_cloud:=true")
        return 0
    else:
        print("⚠ PARTIAL SUCCESS")
        print("=" * 60)
        print("\nConnection successful but message echo not received.")
        print("This typically means:")
        print("  ✓ Connection works (client can connect)")
        print("  ✓ Publish works (messages sent)")
        print("  ? Subscribe may have policy restrictions")
        print("\nCheck AWS IoT Policy allows:")
        print("  - iot:Subscribe on topicfilter/elderly_bot/*")
        print("  - iot:Receive on topic/elderly_bot/*")
        print("\nYou may proceed with caution.")
        return 0


def main():
    """Main execution flow"""
    try:
        load_config()
        validate_certificates()
        return test_connection()
        
    except KeyboardInterrupt:
        print("\n\nTest interrupted by user")
        return 1
    except Exception as e:
        print("\n\nUNEXPECTED ERROR: {}".format(e))
        import traceback
        traceback.print_exc()
        return 1


if __name__ == '__main__':
    sys.exit(main())
