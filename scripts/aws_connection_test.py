#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
AWS IoT Core Connection Test (Standalone)
==========================================
Tests AWS IoT Core connectivity BEFORE ROS integration.
This is a pure MQTT test without any ROS dependencies.

USAGE:
    python aws_connection_test.py

REQUIREMENTS:
    pip install AWSIoTPythonSDK

PREREQUISITES:
    1. Run validate_aws_certs.sh to verify certificate integrity
    2. Update cloud_config.yaml with correct AWS endpoint and cert paths
    3. Ensure certificates have correct permissions (chmod 600 private.pem.key)
"""

import sys
import os
import time
import json
import yaml
from datetime import datetime

try:
    from AWSIoTPythonSDK.MQTTLib import AWSIoTMQTTClient
except ImportError:
    print("ERROR: AWSIoTPythonSDK not installed")
    print("Install with: pip install AWSIoTPythonSDK")
    sys.exit(1)


def load_config():
    """Load AWS configuration from cloud_config.yaml"""
    # Get script directory and navigate to package root
    script_dir = os.path.dirname(os.path.abspath(__file__))
    pkg_root = os.path.dirname(script_dir)
    config_path = os.path.join(pkg_root, 'config', 'cloud_config.yaml')
    
    print("Loading configuration from: {}".format(config_path))
    
    if not os.path.exists(config_path):
        print("ERROR: Configuration file not found!")
        sys.exit(1)
    
    with open(config_path, 'r') as f:
        config = yaml.safe_load(f)
    
    # Extract cloud_bridge_node parameters
    if 'cloud_bridge_node' not in config:
        print("ERROR: 'cloud_bridge_node' section not found in config file")
        sys.exit(1)
    
    return config['cloud_bridge_node']


def validate_config(config):
    """Validate that all required parameters are set"""
    print("\n=== Configuration Validation ===")
    
    required_params = [
        'aws_endpoint',
        'client_id',
        'root_ca_path',
        'cert_path',
        'key_path'
    ]
    
    errors = []
    
    for param in required_params:
        if param not in config or not config[param] or config[param] == '':
            errors.append("Missing parameter: {}".format(param))
        elif param in ['root_ca_path', 'cert_path', 'key_path']:
            # Check if file exists
            path = os.path.expanduser(config[param])
            if not os.path.exists(path):
                errors.append("File not found: {} = {}".format(param, path))
            else:
                print("OK: {} exists".format(param))
    
    if errors:
        print("\nERROR: Configuration validation failed:")
        for error in errors:
            print("  - {}".format(error))
        print("\nPlease update config/cloud_config.yaml")
        sys.exit(1)
    
    print("OK: All required parameters present")
    return True


def on_connection_success(client, userdata, flags, rc):
    """Callback when connection succeeds"""
    print("\n=== CONNECTION SUCCESS ===")
    print("Connected to AWS IoT Core!")
    print("Return code: {}".format(rc))


def on_connection_failure(client, userdata, rc):
    """Callback when connection fails"""
    print("\n=== CONNECTION FAILED ===")
    print("Return code: {}".format(rc))
    print("\nCommon error codes:")
    print("  0  = Connection successful")
    print("  1  = Incorrect protocol version")
    print("  2  = Invalid client identifier")
    print("  3  = Server unavailable")
    print("  4  = Bad username or password")
    print("  5  = Not authorized (check AWS IoT policy)")
    print("  >5 = Other connection error")


def on_message_received(client, userdata, message):
    """Callback when message is received"""
    print("\n=== MESSAGE RECEIVED ===")
    print("Topic: {}".format(message.topic))
    print("Payload: {}".format(message.payload.decode('utf-8')))


def main():
    print("=" * 60)
    print("AWS IoT Core Connection Test")
    print("=" * 60)
    
    # Load and validate configuration
    config = load_config()
    validate_config(config)
    
    # Extract parameters
    endpoint = config['aws_endpoint']
    client_id = config['client_id'] + "_test"  # Add suffix to avoid conflicts
    port = config.get('port', 8883)
    root_ca = os.path.expanduser(config['root_ca_path'])
    cert = os.path.expanduser(config['cert_path'])
    key = os.path.expanduser(config['key_path'])
    
    print("\n=== Connection Parameters ===")
    print("Endpoint:  {}".format(endpoint))
    print("Client ID: {}".format(client_id))
    print("Port:      {}".format(port))
    print("Root CA:   {}".format(root_ca))
    print("Cert:      {}".format(cert))
    print("Key:       {}".format(key))
    
    # Initialize MQTT client
    print("\n=== Initializing MQTT Client ===")
    mqtt_client = AWSIoTMQTTClient(client_id)
    mqtt_client.configureEndpoint(endpoint, port)
    mqtt_client.configureCredentials(root_ca, key, cert)
    
    # Configure connection parameters
    mqtt_client.configureAutoReconnectBackoffTime(1, 32, 20)
    mqtt_client.configureOfflinePublishQueueing(-1)  # Infinite
    mqtt_client.configureDrainingFrequency(2)  # 2 Hz
    mqtt_client.configureConnectDisconnectTimeout(10)  # 10 seconds
    mqtt_client.configureMQTTOperationTimeout(5)  # 5 seconds
    
    print("MQTT client configured")
    
    # Attempt connection
    print("\n=== Connecting to AWS IoT Core ===")
    print("Connecting to: {}:{}".format(endpoint, port))
    print("This may take 10-15 seconds...")
    
    try:
        connect_result = mqtt_client.connect()
        
        if connect_result:
            print("\n✅ CONNECTION SUCCESSFUL!")
            
            # Test topic
            test_topic = "elderly_bot/test"
            
            # Subscribe to test topic
            print("\n=== Testing Publish/Subscribe ===")
            print("Subscribing to topic: {}".format(test_topic))
            mqtt_client.subscribe(test_topic, 1, on_message_received)
            time.sleep(2)  # Wait for subscription to complete
            
            # Publish test message
            test_message = {
                "test": True,
                "message": "Hello from Elderly Bot!",
                "timestamp": datetime.now().isoformat(),
                "client_id": client_id
            }
            
            print("Publishing test message to: {}".format(test_topic))
            print("Message: {}".format(json.dumps(test_message, indent=2)))
            
            mqtt_client.publish(test_topic, json.dumps(test_message), 1)
            
            print("\nWaiting for message echo (5 seconds)...")
            time.sleep(5)
            
            # Unsubscribe
            print("\nUnsubscribing from topic...")
            mqtt_client.unsubscribe(test_topic)
            
            # Disconnect
            print("\n=== Disconnecting ===")
            mqtt_client.disconnect()
            print("Disconnected successfully")
            
            print("\n" + "=" * 60)
            print("✅ AWS IoT CORE CONNECTION TEST PASSED")
            print("=" * 60)
            print("\nYour certificates and configuration are correct!")
            print("\nNext steps:")
            print("1. Update cloud_config.yaml: set enable_cloud: true")
            print("2. Launch ROS with cloud bridge:")
            print("   roslaunch elderly_bot bringup.launch enable_cloud:=true")
            print("3. Monitor AWS IoT Core console for robot telemetry")
            
            return 0
            
        else:
            print("\n❌ CONNECTION FAILED")
            print("Check AWS IoT Core console for policy/certificate issues")
            return 1
            
    except Exception as e:
        print("\n❌ CONNECTION EXCEPTION")
        print("Error: {}".format(str(e)))
        print("\nTroubleshooting:")
        print("1. Verify endpoint URL in cloud_config.yaml")
        print("2. Check certificate paths are absolute and correct")
        print("3. Verify internet connectivity")
        print("4. Check AWS IoT policy allows connect/publish/subscribe")
        print("5. Run validate_aws_certs.sh to check certificate integrity")
        return 1


if __name__ == '__main__':
    sys.exit(main())
