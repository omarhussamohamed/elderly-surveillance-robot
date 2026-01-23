#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
AWS IoT Core Bridge Node for Elderly Bot
=========================================
Bridges local ROS topics with AWS IoT Core MQTT for cloud communication.

Python 2.7 compatible for ROS Melodic.

FEATURES:
- Publishes Jetson stats to elderly_bot/telemetry
- Publishes gas alerts to elderly_bot/alerts
- Receives commands from elderly_bot/commands
- Automatic reconnection on connection loss
- Clean shutdown with disconnect hook

DEPENDENCIES:
  pip install AWSIoTPythonSDK
  
CERTIFICATES REQUIRED:
  - Root CA, Device Certificate, Private Key (configured in cloud_config.yaml)
"""

from __future__ import print_function
import rospy
import json
import threading
import time
import os
from std_msgs.msg import Float32, Bool, UInt16
from sensor_msgs.msg import Temperature
from diagnostic_msgs.msg import DiagnosticStatus

# === AWS IoT SDK IMPORT (with graceful failure) ===
AWS_AVAILABLE = False
try:
    from AWSIoTPythonSDK.MQTTLib import AWSIoTMQTTClient
    AWS_AVAILABLE = True
except ImportError:
    AWSIoTMQTTClient = None
    # Will log warning in __init__ when enable_cloud=True

class CloudBridgeNode(object):
    """
    Main bridge node connecting ROS topics to AWS IoT Core via MQTT.
    """
    
    def __init__(self):
        rospy.init_node('cloud_bridge_node', anonymous=False)
        rospy.loginfo("=== Cloud Bridge Node Starting ===")
        
        # === PARAMETERS ===
        self.enable_cloud = rospy.get_param('~enable_cloud', False)
        
        if not self.enable_cloud:
            rospy.loginfo("Cloud bridge DISABLED by parameter. Set enable_cloud:=true to activate.")
            rospy.signal_shutdown("Cloud bridge disabled")
            return
        
        # AWS IoT Core connection parameters
        self.aws_endpoint = rospy.get_param('~aws_endpoint', '')
        self.client_id = rospy.get_param('~client_id', 'robot_nano')
        self.port = rospy.get_param('~port', 8883)
        
        # Certificate paths (absolute paths required)
        self.root_ca_path = rospy.get_param('~root_ca_path', '')
        self.cert_path = rospy.get_param('~cert_path', '')
        self.key_path = rospy.get_param('~key_path', '')
        
        # MQTT topics
        self.mqtt_topic_telemetry = rospy.get_param('~mqtt_topic_telemetry', 'elderly_bot/telemetry')
        self.mqtt_topic_alerts = rospy.get_param('~mqtt_topic_alerts', 'elderly_bot/alerts')
        self.mqtt_topic_commands = rospy.get_param('~mqtt_topic_commands', 'elderly_bot/commands')
        
        # Connection settings
        self.publish_rate = rospy.get_param('~publish_rate', 1.0)
        self.keepalive_interval = rospy.get_param('~keepalive_interval', 30)
        
        # === STATE ===
        self.cloud_connected = False
        self.mqtt_client = None
        self.connection_lock = threading.Lock()
        
        # Latest sensor data
        self.latest_temperature = 0.0
        self.latest_power = 0.0
        self.latest_gas_detected = False
        self.latest_jetson_stats = None
        self.data_lock = threading.Lock()
        
        # === VALIDATION ===
        if not self._validate_config():
            rospy.logwarn("Cloud bridge DISABLED: Configuration incomplete")
            rospy.signal_shutdown("Invalid configuration")
            return
        
        # === INITIALIZE AWS CONNECTION ===
        if not self._init_aws_client():
            rospy.logwarn("Cloud bridge DISABLED: AWS client initialization failed")
            rospy.signal_shutdown("AWS initialization failed")
            return
        
        # === ROS SUBSCRIBERS (Local → Cloud) ===
        rospy.Subscriber('/jetson_temperature', Temperature, self.temperature_callback, queue_size=10)
        rospy.Subscriber('/jetson_power', Float32, self.power_callback, queue_size=10)
        rospy.Subscriber('/gas_detected', Bool, self.gas_detected_callback, queue_size=10)
        
        # === ROS PUBLISHERS (Cloud → Local) ===
        self.pub_buzzer = rospy.Publisher('/buzzer_command', UInt16, queue_size=1, latch=True)
        
        rospy.loginfo("=== Cloud Bridge Node Initialized ===")
        rospy.loginfo("AWS Endpoint: {}".format(self.aws_endpoint))
        rospy.loginfo("Client ID: {}".format(self.client_id))
        rospy.loginfo("Cloud Connection: {}".format('ACTIVE' if self.cloud_connected else 'DISABLED'))
        rospy.loginfo("Telemetry Topic: {}".format(self.mqtt_topic_telemetry))
        rospy.loginfo("Alerts Topic: {}".format(self.mqtt_topic_alerts))
        rospy.loginfo("Commands Topic: {}".format(self.mqtt_topic_commands))
        
        # === SHUTDOWN HOOK ===
        rospy.on_shutdown(self.shutdown_hook)
    
    def _validate_config(self):
        """Validate all required configuration parameters."""
        if not AWS_AVAILABLE:
            rospy.logerr("AWS IoT SDK not available. Install: pip install AWSIoTPythonSDK")
            return False install AWSIoTPythonSDK")
            return False
        
        if not self.aws_endpoint:
            rospy.logerr("AWS endpoint not configured. Set aws_endpoint parameter.")
            return False
        
        if not self.root_ca_path or not self.cert_path or not self.key_path:
            rospy.logerr("Certificate paths not configured.")
            return False
        
        # Check if certificate files exist
        if not os.path.isfile(self.root_ca_path):
            rospy.logerr("Root CA not found: {}".format(self.root_ca_path))
            return False
        if not os.path.isfile(self.cert_path):
            rospy.logerr("Device certificate not found: {}".format(self.cert_path))
            return False
        if not os.path.isfile(self.key_path):
            rospy.logerr("Private key not found: {}".format(self.key_path))
            return False
        
        rospy.loginfo("Configuration validated")
        return True
    
    def _init_aws_client(self):
        """Initialize AWS IoT MQTT client with certificates."""
        try:
            # Create MQTT client
            self.mqtt_client = AWSIoTMQTTClient(self.client_id)
            
            # Configure endpoint and port
            self.mqtt_client.configureEndpoint(self.aws_endpoint, self.port)
            
            # Configure certificates
            self.mqtt_client.configureCredentials(
                self.root_ca_path,
                self.key_path,
                self.cert_path
            )
            
            # Configure MQTT client settings
            self.mqtt_client.configureAutoReconnectBackoffTime(1, 32, 20)
            self.mqtt_client.configureOfflinePublishQueueing(-1)  # Infinite queue
            self.mqtt_client.configureDrainingFrequency(2)  # 2 Hz
            self.mqtt_client.configureConnectDisconnectTimeout(30)  # 30 seconds
            self.mqtt_client.configureMQTTOperationTimeout(10)  # 10 seconds
            
            # Connect to AWS IoT Core
            rospy.loginfo("Connecting to AWS IoT Core...")
            rospy.loginfo("  Endpoint: {}:{}".format(self.aws_endpoint, self.port))
            rospy.loginfo("  Client ID: {}".format(self.client_id))
            
            try:
                connect_result = self.mqtt_client.connect(self.keepalive_interval)
            except Exception as e:
                rospy.logerr("Connection exception: {}".format(e))
                rospy.logerr("Check: Internet connection, endpoint URL, certificate paths")
                rospy.logerr("Check: Port {} is not blocked by firewall".format(self.port))
                return False
            
            if connect_result:
                rospy.loginfo("Connected to AWS IoT Core")
                
                # Subscribe to command topic
                self.mqtt_client.subscribe(self.mqtt_topic_commands, 1, self.mqtt_command_callback)
                rospy.loginfo("Subscribed to MQTT topic: {}".format(self.mqtt_topic_commands))
                
                self.cloud_connected = True
                return True
            else:
                rospy.logerr("Failed to connect to AWS IoT Core")
                return False
            
        except Exception as e:
            rospy.logerr("Failed to initialize AWS client: {}".format(e))
            rospy.logerr("Check: Internet connection, endpoint URL, certificate paths")
            return False
    
    # === ROS CALLBACKS (Local → Cloud) ===
    
    def temperature_callback(self, msg):
        """Cache temperature data for telemetry."""
        with self.data_lock:
            self.latest_temperature = msg.temperature
    
    def power_callback(self, msg):
        """Cache power data for telemetry."""
        with self.data_lock:
            self.latest_power = msg.data
    
    def gas_detected_callback(self, msg):
        """Handle gas detection and publish alert to cloud."""
        with self.data_lock:
            self.latest_gas_detected = msg.data
        
        # Publish alert immediately when gas is detected
        if msg.data:
            self.publish_alert("gas_detected", True)
    
    # === MQTT CALLBACK (Cloud → Local) ===
    
    def mqtt_command_callback(self, client, userdata, message):
        """
        Handle incoming MQTT messages from AWS IoT Core.
        Expected JSON format:
        {
            "command": "buzzer",
            "value": 1000 (frequency in Hz) or 0 (off)
        }
        """
        try:
            payload = json.loads(message.payload.decode('utf-8'))
            command = payload.get('command', '')
            value = payload.get('value')
            
            rospy.loginfo("Received cloud command: {} = {}".format(command, value))
            
            if command == 'buzzer':
                # Buzzer control command (UInt16 frequency)
                buzzer_freq = int(value) if value is not None else 0
                self.pub_buzzer.publish(UInt16(buzzer_freq))
                rospy.loginfo("Buzzer command sent: {} Hz".format(buzzer_freq))
                
            else:
                rospy.logwarn("Unknown command: {}".format(command))
                
        except (ValueError, TypeError) as e:
            rospy.logerr("Failed to parse MQTT message: {}".format(e))
        except Exception as e:
            rospy.logerr("Error handling MQTT command: {}".format(e))
    
    # === CLOUD PUBLISHING ===
    
    def publish_telemetry(self):
        """
        Publish Jetson telemetry data to AWS IoT Core.
        Topic: elderly_bot/telemetry
        """
        if not self.cloud_connected:
            return
        
        try:
            with self.data_lock:
                payload = {
                    'timestamp': rospy.Time.now().to_sec(),
                    'robot_id': self.client_id,
                    'telemetry': {
                        'temperature': self.latest_temperature,
                        'power': self.latest_power,
                        'gas_detected': self.latest_gas_detected
                    }
                }
            
            json_payload = json.dumps(payload)
            self.mqtt_client.publish(self.mqtt_topic_telemetry, json_payload, 1)
            
            rospy.logdebug("Published telemetry: {}".format(json_payload))
            
        except Exception as e:
            rospy.logwarn_throttle(10, "Failed to publish telemetry: {}".format(e))
    
    def publish_alert(self, alert_type, value):
        """
        Publish alert to AWS IoT Core.
        Topic: elderly_bot/alerts
        """
        if not self.cloud_connected:
            return
        
        try:
            payload = {
                'timestamp': rospy.Time.now().to_sec(),
                'robot_id': self.client_id,
                'alert_type': alert_type,
                'value': value
            }
            
            json_payload = json.dumps(payload)
            self.mqtt_client.publish(self.mqtt_topic_alerts, json_payload, 1)
            
            rospy.loginfo("Published alert: {}".format(json_payload))
            
        except Exception as e:
            rospy.logerr("Failed to publish alert: {}".format(e))
    
    def run(self):
        """Main loop: periodically publish telemetry to cloud."""
        if not self.cloud_connected:
            rospy.logwarn("Cloud not connected, node exiting")
            return
        
        rate = rospy.Rate(self.publish_rate)
        
        rospy.loginfo("Main loop starting at {} Hz".format(self.publish_rate))
        
        while not rospy.is_shutdown():
            try:
                self.publish_telemetry()
                rate.sleep()
                
            except rospy.ROSInterruptException:
                break
            except Exception as e:
                rospy.logerr("Error in main loop: {}".format(e))
                rospy.sleep(1.0)
    
    def shutdown_hook(self):
        """Clean shutdown: disconnect from AWS IoT Core."""
        rospy.loginfo("=== Shutting down Cloud Bridge Node ===")
        
        if self.cloud_connected and self.mqtt_client:
            try:
                self.mqtt_client.disconnect()
                rospy.loginfo("Disconnected from AWS IoT Core")
            except Exception as e:
                rospy.logwarn("Error during disconnect: {}".format(e))
        
        rospy.loginfo("Shutdown complete")


if __name__ == '__main__':
    try:
        node = CloudBridgeNode()
        if node.enable_cloud:
            node.run()
    except rospy.ROSInterruptException:
        pass
    except Exception as e:
        rospy.logerr("Fatal error: {}".format(e))
