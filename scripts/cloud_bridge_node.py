#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""
Cloud Bridge Node for AWS IoT Core - SIMPLIFIED WORKING VERSION
"""
from __future__ import print_function
import rospy
import json
import threading
import os
from std_msgs.msg import Float32, Bool, UInt16
from sensor_msgs.msg import Temperature, LaserScan, Imu
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist

# Try to import AWS SDK
try:
    from AWSIoTPythonSDK.MQTTLib import AWSIoTMQTTClient
    AWS_AVAILABLE = True
except ImportError:
    AWS_AVAILABLE = False
    rospy.logwarn("AWS IoT SDK not installed. Run: pip install AWSIoTPythonSDK")

class CloudBridgeNode:
    def __init__(self):
        rospy.init_node('cloud_bridge_node', anonymous=False)
        rospy.loginfo("=== Cloud Bridge Node Starting ===")
        
        # Get parameters
        self.enable_cloud = rospy.get_param('~enable_cloud', False)
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
        
        # Sensor data storage
        self.temperature = 0.0
        self.power = 0.0
        self.gas_detected = False
        
        # Validate
        if not self._validate_config():
            rospy.logwarn("Cloud bridge disabled: Invalid configuration")
            rospy.signal_shutdown("Invalid config")
            return
        
        # Initialize AWS connection
        if not self._init_aws():
            rospy.logerr("Failed to initialize AWS IoT connection")
            return
        
        # ROS Subscribers
        rospy.Subscriber('/jetson_temperature', Temperature, self.temp_cb)
        rospy.Subscriber('/jetson_power', Float32, self.power_cb)
        rospy.Subscriber('/gas_detected', Bool, self.gas_cb)
        rospy.Subscriber('/scan', LaserScan, self.scan_cb)
        rospy.Subscriber('/odom', Odometry, self.odom_cb)
        rospy.Subscriber('/imu/data', Imu, self.imu_cb)
        
        # ROS Publishers
        self.buzzer_pub = rospy.Publisher('/buzzer_command', UInt16, queue_size=1)
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        
        rospy.loginfo("Cloud Bridge initialized")
        rospy.loginfo("AWS Endpoint: %s", self.aws_endpoint)
        
        # Start telemetry publishing
        self.publish_rate = rospy.get_param('~publish_rate', 2.0)
        rospy.Timer(rospy.Duration(1.0/self.publish_rate), self.publish_telemetry)
        
        rospy.on_shutdown(self.shutdown)
    
    def _validate_config(self):
        """Validate configuration"""
        if not self.enable_cloud:
            rospy.loginfo("Cloud bridge disabled by parameter")
            return False
        
        if not AWS_AVAILABLE:
            rospy.logerr("AWS IoT SDK not available")
            return False
        
        # Check certificate files
        certs = [self.root_ca_path, self.cert_path, self.key_path]
        for cert in certs:
            if not os.path.isfile(cert):
                rospy.logerr("Certificate not found: %s", cert)
                return False
        
        return True
    
    def _init_aws(self):
        """Initialize AWS IoT connection"""
        try:
            self.mqtt_client = AWSIoTMQTTClient(self.client_id)
            self.mqtt_client.configureEndpoint(self.aws_endpoint, self.port)
            self.mqtt_client.configureCredentials(
                self.root_ca_path,
                self.key_path,
                self.cert_path
            )
            
            # MQTT configuration
            self.mqtt_client.configureAutoReconnectBackoffTime(1, 32, 20)
            self.mqtt_client.configureOfflinePublishQueueing(-1)
            self.mqtt_client.configureDrainingFrequency(2)
            self.mqtt_client.configureConnectDisconnectTimeout(30)
            self.mqtt_client.configureMQTTOperationTimeout(10)
            
            # Connect
            if self.mqtt_client.connect():
                self.cloud_connected = True
                # Subscribe to commands
                self.mqtt_client.subscribe(self.mqtt_topic_commands, 1, self.mqtt_callback)
                rospy.loginfo("Connected to AWS IoT Core")
                return True
            else:
                rospy.logerr("Failed to connect to AWS IoT")
                return False
                
        except Exception as e:
            rospy.logerr("AWS init error: %s", str(e))
            return False
    
    # ROS Callbacks
    def temp_cb(self, msg):
        self.temperature = msg.temperature
    
    def power_cb(self, msg):
        self.power = msg.data
    
    def gas_cb(self, msg):
        self.gas_detected = msg.data
        if msg.data:
            self.publish_alert("Gas detected!")
    
    def scan_cb(self, msg):
        self.scan_ranges = msg.ranges if hasattr(msg, 'ranges') else []
    
    def odom_cb(self, msg):
        self.odom_x = msg.pose.pose.position.x
        self.odom_y = msg.pose.pose.position.y
    
    def imu_cb(self, msg):
        self.imu_data = msg.angular_velocity.z if hasattr(msg, 'angular_velocity') else 0.0
    
    # MQTT Callback
    def mqtt_callback(self, client, userdata, message):
        """Handle MQTT commands from AWS"""
        try:
            payload = json.loads(message.payload)
            command = payload.get('command', '')
            value = payload.get('value')
            
            rospy.loginfo("Cloud command: %s = %s", command, value)
            
            if command == 'buzzer':
                freq = int(value) if value else 0
                self.buzzer_pub.publish(UInt16(freq))
                rospy.loginfo("Buzzer: %d Hz", freq)
            
            elif command == 'move':
                twist = Twist()
                if isinstance(value, dict):
                    twist.linear.x = value.get('linear_x', 0.0)
                    twist.angular.z = value.get('angular_z', 0.0)
                self.cmd_vel_pub.publish(twist)
            
            elif command == 'stop':
                twist = Twist()
                twist.linear.x = 0.0
                twist.angular.z = 0.0
                self.cmd_vel_pub.publish(twist)
            
        except Exception as e:
            rospy.logerr("MQTT callback error: %s", str(e))
    
    # Telemetry Publishing
    def publish_telemetry(self, event=None):
        """Publish telemetry to AWS"""
        if not self.cloud_connected:
            return
        
        try:
            telemetry = {
                'timestamp': rospy.Time.now().to_sec(),
                'robot_id': self.client_id,
                'temperature': self.temperature,
                'power': self.power,
                'gas_detected': self.gas_detected,
                'odom_x': getattr(self, 'odom_x', 0.0),
                'odom_y': getattr(self, 'odom_y', 0.0),
                'scan_points': len(getattr(self, 'scan_ranges', []))
            }
            
            self.mqtt_client.publish(
                self.mqtt_topic_telemetry,
                json.dumps(telemetry),
                1
            )
            
            rospy.logdebug("Published telemetry")
            
        except Exception as e:
            rospy.logwarn("Telemetry publish failed: %s", str(e))
    
    def publish_alert(self, message):
        """Publish alert to AWS"""
        if not self.cloud_connected:
            return
        
        try:
            alert = {
                'timestamp': rospy.Time.now().to_sec(),
                'robot_id': self.client_id,
                'alert': message
            }
            self.mqtt_client.publish('elderly_bot/alerts', json.dumps(alert), 1)
        except Exception as e:
            rospy.logerr("Alert publish failed: %s", str(e))
    
    def shutdown(self):
        """Clean shutdown"""
        rospy.loginfo("Shutting down Cloud Bridge")
        if self.cloud_connected and self.mqtt_client:
            try:
                self.mqtt_client.disconnect()
            except:
                pass

if __name__ == '__main__':
    try:
        node = CloudBridgeNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass