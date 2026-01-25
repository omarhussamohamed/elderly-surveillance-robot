#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""
Cloud Bridge Node for AWS IoT Core - Python 2.7 Compatible
"""
from __future__ import print_function
import rospy
import json
import os
import sys
import time
from std_msgs.msg import Float32, Bool, UInt16
from sensor_msgs.msg import Temperature
from geometry_msgs.msg import Twist

# Try to import AWS SDK
try:
    from AWSIoTPythonSDK.MQTTLib import AWSIoTMQTTClient
    AWS_AVAILABLE = True
except ImportError:
    AWS_AVAILABLE = False
    rospy.logwarn("AWS IoT SDK not installed")

class CloudBridgeNode:
    def __init__(self):
        rospy.init_node('cloud_bridge_node', anonymous=False)
        rospy.loginfo("=== Cloud Bridge Node Starting ===")
        
        # Get parameters
        self.enable_cloud = rospy.get_param('~enable_cloud', False)
        
        if not self.enable_cloud:
            rospy.loginfo("Cloud bridge disabled by parameter")
            rospy.signal_shutdown("Cloud bridge disabled")
            return
        
        self.aws_endpoint = rospy.get_param('~aws_endpoint', '')
        self.client_id = rospy.get_param('~client_id', 'robot_nano')
        self.port = rospy.get_param('~port', 8883)
        
        # Certificate paths
        self.root_ca_path = rospy.get_param('~root_ca_path', '')
        self.cert_path = rospy.get_param('~cert_path', '')
        self.key_path = rospy.get_param('~key_path', '')
        
        # MQTT topics
        self.mqtt_topic_telemetry = rospy.get_param('~mqtt_topic_telemetry', 'elderly_bot/telemetry')
        self.mqtt_topic_commands = rospy.get_param('~mqtt_topic_commands', 'elderly_bot/commands')
        
        # State
        self.cloud_connected = False
        self.mqtt_client = None
        
        # Sensor data
        self.temperature = 0.0
        self.power = 0.0
        self.gas_detected = False
        
        # Validate
        if not self._validate_config():
            rospy.logwarn("Cloud bridge disabled: Invalid configuration")
            rospy.signal_shutdown("Invalid config")
            return
        
        # Initialize AWS connection
        self._init_aws()
        
        # ROS Subscribers
        rospy.Subscriber('/jetson_temperature', Temperature, self.temp_callback)
        rospy.Subscriber('/jetson_power', Float32, self.power_callback)
        rospy.Subscriber('/gas_detected', Bool, self.gas_callback)
        
        # ROS Publishers
        self.buzzer_pub = rospy.Publisher('/buzzer_command', UInt16, queue_size=1)
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        
        rospy.loginfo("Cloud Bridge initialized for AWS IoT Core")
        rospy.loginfo("AWS Endpoint: %s", self.aws_endpoint)
        
        # Start telemetry publishing
        self.publish_rate = rospy.get_param('~publish_rate', 2.0)
        self.timer = rospy.Timer(rospy.Duration(1.0/self.publish_rate), self.publish_telemetry)
        
        rospy.on_shutdown(self.shutdown)
    
    def _validate_config(self):
        """Validate configuration"""
        if not AWS_AVAILABLE:
            rospy.logerr("AWS IoT SDK not available. Install: pip install AWSIoTPythonSDK==1.4.9")
            return False
        
        # Check certificate files exist
        certs = [
            (self.root_ca_path, "Root CA"),
            (self.cert_path, "Device certificate"), 
            (self.key_path, "Private key")
        ]
        
        for cert_path, cert_name in certs:
            if not cert_path:
                rospy.logerr("%s path not configured", cert_name)
                return False
            if not os.path.isfile(cert_path):
                rospy.logerr("%s not found: %s", cert_name, cert_path)
                return False
        
        return True
    
    def _init_aws(self):
        """Initialize AWS IoT connection"""
        if not AWS_AVAILABLE:
            return
        
        try:
            self.mqtt_client = AWSIoTMQTTClient(self.client_id)
            
            # Configure endpoint
            self.mqtt_client.configureEndpoint(self.aws_endpoint, self.port)
            
            # Configure certificates
            self.mqtt_client.configureCredentials(
                self.root_ca_path,
                self.key_path,
                self.cert_path
            )
            
            # Configure MQTT settings
            self.mqtt_client.configureAutoReconnectBackoffTime(1, 32, 20)
            self.mqtt_client.configureOfflinePublishQueueing(-1)
            self.mqtt_client.configureDrainingFrequency(2)
            self.mqtt_client.configureConnectDisconnectTimeout(30)
            self.mqtt_client.configureMQTTOperationTimeout(10)
            
            # Connect
            rospy.loginfo("Connecting to AWS IoT Core...")
            if self.mqtt_client.connect():
                self.cloud_connected = True
                # Subscribe to commands
                self.mqtt_client.subscribe(self.mqtt_topic_commands, 1, self.mqtt_command_callback)
                rospy.loginfo("Connected to AWS IoT Core")
                rospy.loginfo("Subscribed to: %s", self.mqtt_topic_commands)
            else:
                rospy.logerr("Failed to connect to AWS IoT Core")
                
        except Exception as e:
            rospy.logerr("AWS IoT connection error: %s", str(e))
            rospy.logerr("Check: 1) Internet connection 2) Certificate validity 3) AWS IoT Policy")
    
    # ROS Callbacks
    def temp_callback(self, msg):
        self.temperature = msg.temperature
    
    def power_callback(self, msg):
        self.power = msg.data
    
    def gas_callback(self, msg):
        old_gas = self.gas_detected
        self.gas_detected = msg.data
        
        # Publish alert if gas detection changed
        if msg.data and not old_gas:
            self.publish_alert("gas_detected", True)
            rospy.logwarn("Gas detected! Alert sent to cloud")
    
    # MQTT Command Callback
    def mqtt_command_callback(self, client, userdata, message):
        """Handle commands from AWS IoT"""
        try:
            payload = json.loads(message.payload)
            command = payload.get('command', '')
            value = payload.get('value', '')
            
            rospy.loginfo("Received cloud command: %s = %s", command, value)
            
            if command == 'buzzer':
                # Control buzzer
                buzzer_freq = int(value) if value else 0
                self.buzzer_pub.publish(UInt16(buzzer_freq))
                rospy.loginfo("Buzzer set to %d Hz", buzzer_freq)
                
            elif command == 'move':
                # Move robot
                twist = Twist()
                if isinstance(value, dict):
                    twist.linear.x = float(value.get('linear_x', 0.0))
                    twist.angular.z = float(value.get('angular_z', 0.0))
                else:
                    twist.linear.x = 0.0
                    twist.angular.z = 0.0
                self.cmd_vel_pub.publish(twist)
                
            elif command == 'stop':
                # Emergency stop
                twist = Twist()
                twist.linear.x = 0.0
                twist.angular.z = 0.0
                self.cmd_vel_pub.publish(twist)
                rospy.logwarn("Emergency stop commanded from cloud")
                
            else:
                rospy.logwarn("Unknown command: %s", command)
                
        except Exception as e:
            rospy.logerr("Error processing MQTT command: %s", str(e))
    
    # Telemetry Publishing
    def publish_telemetry(self, event=None):
        """Publish telemetry to AWS IoT Core"""
        if not self.cloud_connected or not self.mqtt_client:
            return
        
        try:
            telemetry = {
                'timestamp': time.time(),
                'robot_id': self.client_id,
                'temperature': round(self.temperature, 2),
                'power': round(self.power, 2),
                'gas_detected': self.gas_detected,
                'cpu_usage': 0.0,  # You can add actual CPU monitoring
                'memory_usage': 0.0  # You can add actual memory monitoring
            }
            
            # Publish to AWS IoT
            self.mqtt_client.publish(
                self.mqtt_topic_telemetry,
                json.dumps(telemetry),
                1  # QoS 1
            )
            
            rospy.logdebug("Published telemetry to cloud")
            
        except Exception as e:
            rospy.logwarn("Failed to publish telemetry: %s", str(e))
    
    def publish_alert(self, alert_type, value):
        """Publish alert to AWS IoT Core"""
        if not self.cloud_connected or not self.mqtt_client:
            return
        
        try:
            alert = {
                'timestamp': time.time(),
                'robot_id': self.client_id,
                'alert_type': alert_type,
                'value': value,
                'message': 'Robot alert'
            }
            
            self.mqtt_client.publish(
                'elderly_bot/alerts',
                json.dumps(alert),
                1
            )
            
        except Exception as e:
            rospy.logerr("Failed to publish alert: %s", str(e))
    
    def shutdown(self):
        """Clean shutdown"""
        rospy.loginfo("Shutting down Cloud Bridge")
        if self.cloud_connected and self.mqtt_client:
            try:
                self.mqtt_client.disconnect()
                rospy.loginfo("Disconnected from AWS IoT Core")
            except:
                rospy.logwarn("Error during disconnect")

if __name__ == '__main__':
    try:
        node = CloudBridgeNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
    except Exception as e:
        rospy.logerr("Fatal error in cloud_bridge_node: %s", str(e))