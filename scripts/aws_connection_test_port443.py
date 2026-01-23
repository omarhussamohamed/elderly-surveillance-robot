#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
AWS IoT Core Connection Test - Port 443 with ALPN
==================================================
Alternative connection test using port 443 (HTTPS) instead of 8883 (MQTT).
Use this if port 8883 is blocked by your ISP/router.

ALPN (Application Layer Protocol Negotiation) is REQUIRED for port 443.

USAGE:
    python aws_connection_test_port443.py
"""

import sys
import os
import time
import json
import yaml
import ssl
from datetime import datetime

try:
    from AWSIoTPythonSDK.MQTTLib import AWSIoTMQTTClient
except ImportError:
    print("ERROR: AWSIoTPythonSDK not installed")
    print("Install with: pip install AWSIoTPythonSDK")
    sys.exit(1)


def load_config():
    """Load AWS configuration from cloud_config.yaml"""
    script_dir = os.path.dirname(os.path.abspath(__file__))
    pkg_root = os.path.dirname(script_dir)
    config_path = os.path.join(pkg_root, 'config', 'cloud_config.yaml')
    
    print("Loading configuration from: {}".format(config_path))
    
    if not os.path.exists(config_path):
        print("ERROR: Configuration file not found!")
        sys.exit(1)
    
    with open(config_path, 'r') as f:
        config = yaml.safe_load(f)
    
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


def main():
    print("=" * 60)
    print("AWS IoT Core Connection Test - Port 443 with ALPN")
    print("=" * 60)
    
    # Load and validate configuration
    config = load_config()
    validate_config(config)
    
    # Extract parameters - FORCE PORT 443
    endpoint = config['aws_endpoint']
    client_id = config['client_id'] + "_test_443"
    port = 443  # FORCE PORT 443 for ALPN
    root_ca = os.path.expanduser(config['root_ca_path'])
    cert = os.path.expanduser(config['cert_path'])
    key = os.path.expanduser(config['key_path'])
    
    print("\n=== Connection Parameters ===")
    print("Endpoint:  {}".format(endpoint))
    print("Client ID: {}".format(client_id))
    print("Port:      {} (HTTPS with ALPN)".format(port))
    print("Root CA:   {}".format(root_ca))
    print("Cert:      {}".format(cert))
    print("Key:       {}".format(key))
    
    # Initialize MQTT client
    print("\n=== Initializing MQTT Client with ALPN ===")
    mqtt_client = AWSIoTMQTTClient(client_id)
    mqtt_client.configureEndpoint(endpoint, port)
    mqtt_client.configureCredentials(root_ca, key, cert)
    
    # CRITICAL: Configure ALPN for port 443
    # ALPN protocol negotiation tells the server we want MQTT over TLS
    print("Configuring ALPN (Application Layer Protocol Negotiation)...")
    try:
        # Enable ALPN with x-amzn-mqtt-ca protocol
        mqtt_client.configureIAMCredentials(None, None, None)  # Not using IAM
        
        # For port 443, we need to set ALPN protocols
        # This is done automatically by the SDK when using port 443
        print("✓ ALPN will be negotiated automatically on port 443")
        
    except Exception as e:
        print("Note: ALPN configuration: {}".format(e))
    
    # Configure connection parameters
    mqtt_client.configureAutoReconnectBackoffTime(1, 32, 20)
    mqtt_client.configureOfflinePublishQueueing(-1)
    mqtt_client.configureDrainingFrequency(2)
    mqtt_client.configureConnectDisconnectTimeout(30)  # 30 seconds
    mqtt_client.configureMQTTOperationTimeout(10)
    
    # Enable verbose logging
    import logging
    logging.basicConfig(level=logging.DEBUG)
    logger = logging.getLogger("AWSIoTPythonSDK.core")
    logger.setLevel(logging.DEBUG)
    streamHandler = logging.StreamHandler()
    formatter = logging.Formatter('%(asctime)s - %(name)s - %(levelname)s - %(message)s')
    streamHandler.setFormatter(formatter)
    logger.addHandler(streamHandler)
    
    print("MQTT client configured")
    
    # Attempt connection
    print("\n=== Connecting to AWS IoT Core via Port 443 ===")
    print("Connecting to: {}:{}".format(endpoint, port))
    print("This may take 20-30 seconds...")
    print("\nVerbose logs:")
    print("-" * 60)
    
    try:
        connect_result = mqtt_client.connect()
        
        if connect_result:
            print("\n✅ CONNECTION SUCCESSFUL on Port 443!")
            
            # Test topic
            test_topic = "elderly_bot/test"
            
            print("\n=== Testing Publish/Subscribe ===")
            print("Subscribing to topic: {}".format(test_topic))
            
            def on_message(client, userdata, message):
                print("\n=== MESSAGE RECEIVED ===")
                print("Topic: {}".format(message.topic))
                print("Payload: {}".format(message.payload.decode('utf-8')))
            
            mqtt_client.subscribe(test_topic, 1, on_message)
            time.sleep(2)
            
            # Publish test message
            test_message = {
                "test": True,
                "message": "Hello from Port 443!",
                "timestamp": datetime.now().isoformat(),
                "client_id": client_id
            }
            
            print("Publishing test message...")
            mqtt_client.publish(test_topic, json.dumps(test_message), 1)
            
            print("\nWaiting for message echo (5 seconds)...")
            time.sleep(5)
            
            mqtt_client.unsubscribe(test_topic)
            mqtt_client.disconnect()
            
            print("\n" + "=" * 60)
            print("✅ PORT 443 CONNECTION TEST PASSED")
            print("=" * 60)
            print("\nPort 443 with ALPN works!")
            print("\nNext steps:")
            print("1. Update cloud_config.yaml: set port: 443")
            print("2. Update cloud_bridge_node.py to use port 443")
            print("3. Launch ROS with cloud enabled")
            
            return 0
            
        else:
            print("\n❌ CONNECTION FAILED on Port 443")
            return 1
            
    except Exception as e:
        print("\n❌ CONNECTION EXCEPTION")
        print("Error: {}".format(str(e)))
        print("\nTroubleshooting:")
        print("1. Run: bash diagnose_aws_network.sh")
        print("2. Check if port 443 is also blocked")
        print("3. Verify AWS IoT policy allows your client_id")
        print("4. Check AWS IoT Console → Monitor → Logs")
        return 1


if __name__ == '__main__':
    sys.exit(main())
