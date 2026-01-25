#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""
AWS IoT Core Bridge Node - ENHANCED VERSION
"""

from __future__ import print_function
import rospy
import json
import threading
import time
import os
import sys
from std_msgs.msg import Float32, Bool, UInt16, String
from sensor_msgs.msg import Temperature, LaserScan, Odometry, Imu
from geometry_msgs.msg import Twist
from nav_msgs.msg import OccupancyGrid

# === AWS IoT SDK IMPORT ===
AWS_AVAILABLE = False
try:
    from AWSIoTPythonSDK.MQTTLib import AWSIoTMQTTClient
    AWS_AVAILABLE = True
except ImportError:
    AWSIoTMQTTClient = None
    rospy.logwarn("AWS IoT SDK not available. Install: pip install AWSIoTPythonSDK")

class CloudBridgeNode(object):
    
    def __init__(self):
        rospy.init_node('cloud_bridge_node', anonymous=False)
        rospy.loginfo("=== Cloud Bridge Node Starting ===")
        
        # === PARAMETERS ===
        self.enable_cloud = rospy.get_param('~enable_cloud', True)
        
        if not self.enable_cloud:
            rospy.loginfo("Cloud bridge DISABLED by parameter.")
            rospy.signal_shutdown("Cloud bridge disabled")
            return
        
        # AWS IoT Core connection parameters
        self.aws_endpoint = rospy.get_param('~aws_endpoint', '')
        self.client_id = rospy.get_param('~client_id', 'robot_nano')
        self.port = rospy.get_param('~port', 8883)
        
        # Certificate paths
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
        self.latest_scan = None
        self.latest_odom = None
        self.latest_imu = None
        self.data_lock = threading.Lock()
        
        # === VALIDATION ===
        if not self._validate_config():
            rospy.logwarn("Cloud bridge DISABLED: Configuration incomplete")
            rospy.signal_shutdown("Invalid configuration")
            return
        
        # === INITIALIZE AWS CONNECTION ===
        if not self._init_aws_client():
            rospy.logerr("Failed to initialize AWS client. Will retry in background.")
            # Don't shutdown - try to reconnect in background
        
        # === ROS SUBSCRIBERS ===
        rospy.Subscriber('/jetson_temperature', Temperature, self.temperature_callback, queue_size=10)
        rospy.Subscriber('/jetson_power', Float32, self.power_callback, queue_size=10)
        rospy.Subscriber('/gas_detected', Bool, self.gas_detected_callback, queue_size=10)
        rospy.Subscriber('/scan', LaserScan, self.scan_callback, queue_size=5)
        rospy.Subscriber('/odom', Odometry, self.odom_callback, queue_size=5)
        rospy.Subscriber('/imu/data', Imu, self.imu_callback, queue_size=5)
        
        # === ROS PUBLISHERS ===
        self.pub_buzzer = rospy.Publisher('/buzzer_command', UInt16, queue_size=1, latch=True)
        self.pub_cmd_vel = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        
        rospy.loginfo("=== Cloud Bridge Node Initialized ===")
        rospy.loginfo("AWS Endpoint: {}".format(self.aws_endpoint))
        rospy.loginfo("Client ID: {}".format(self.client_id))
        
        # === CONNECTION MONITOR THREAD ===
        self.monitor_thread = threading.Thread(target=self.connection_monitor)
        self.monitor_thread.daemon = True
        self.monitor_thread.start()
        
        rospy.on_shutdown(self.shutdown_hook)
    
    def _validate_config(self):
        """Validate all required configuration parameters."""
        if not AWS_AVAILABLE:
            rospy.logerr("AWS IoT SDK not available.")
            return False
        
        required = [
            ('aws_endpoint', self.aws_endpoint),
            ('root_ca_path', self.root_ca_path),
            ('cert_path', self.cert_path),
            ('key_path', self.key_path)
        ]
        
        for name, value in required:
            if not value:
                rospy.logerr("Parameter '{}' not configured.".format(name))
                return False
        
        # Check if certificate files exist
        certs = [
            (self.root_ca_path, "Root CA"),
            (self.cert_path, "Device certificate"),
            (self.key_path, "Private key")
        ]
        
        for path, name in certs:
            if not os.path.isfile(path):
                rospy.logerr("{} not found: {}".format(name, path))
                return False
        
        rospy.loginfo("Configuration validated")
        return True
    
    def _init_aws_client(self):
        """Initialize AWS IoT MQTT client with certificates."""
        try:
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
            self.mqtt_client.configureOfflinePublishQueueing(-1)
            self.mqtt_client.configureDrainingFrequency(2)
            self.mqtt_client.configureConnectDisconnectTimeout(30)
            self.mqtt_client.configureMQTTOperationTimeout(10)
            
            # Connect to AWS IoT Core
            rospy.loginfo("Connecting to AWS IoT Core...")
            connect_result = self.mqtt_client.connect(self.keepalive_interval)
            
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
            rospy.logerr("AWS client init error: {}".format(e))
            return False
    
    def connection_monitor(self):
        """Background thread to monitor and maintain connection."""
        while not rospy.is_shutdown():
            if not self.cloud_connected and self.enable_cloud:
                rospy.logwarn("Attempting to reconnect to AWS IoT...")
                time.sleep(5)  # Wait before retry
                self._init_aws_client()
            time.sleep(10)
    
    # === ROS CALLBACKS ===
    
    def temperature_callback(self, msg):
        with self.data_lock:
            self.latest_temperature = msg.temperature
    
    def power_callback(self, msg):
        with self.data_lock:
            self.latest_power = msg.data
    
    def gas_detected_callback(self, msg):
        with self.data_lock:
            self.latest_gas_detected = msg.data
        
        if msg.data:
            self.publish_alert("gas_detected", True)
    
    def scan_callback(self, msg):
        """Extract minimal scan data for telemetry."""
        with self.data_lock:
            if len(msg.ranges) > 0:
                # Store just min, max, and avg for telemetry
                valid_ranges = [r for r in msg.ranges if r > msg.range_min and r < msg.range_max]
                if valid_ranges:
                    self.latest_scan = {
                        'min': min(valid_ranges),
                        'max': max(valid_ranges),
                        'avg': sum(valid_ranges) / len(valid_ranges),
                        'count': len(valid_ranges)
                    }
    
    def odom_callback(self, msg):
        with self.data_lock:
            self.latest_odom = {
                'x': msg.pose.pose.position.x,
                'y': msg.pose.pose.position.y,
                'z': msg.pose.pose.position.z,
                'orientation': {
                    'x': msg.pose.pose.orientation.x,
                    'y': msg.pose.pose.orientation.y,
                    'z': msg.pose.pose.orientation.z,
                    'w': msg.pose.pose.orientation.w
                }
            }
    
    def imu_callback(self, msg):
        with self.data_lock:
            self.latest_imu = {
                'linear_acceleration': {
                    'x': msg.linear_acceleration.x,
                    'y': msg.linear_acceleration.y,
                    'z': msg.linear_acceleration.z
                },
                'angular_velocity': {
                    'x': msg.angular_velocity.x,
                    'y': msg.angular_velocity.y,
                    'z': msg.angular_velocity.z
                }
            }
    
    # === MQTT CALLBACK ===
    
    def mqtt_command_callback(self, client, userdata, message):
        """Handle incoming MQTT commands from AWS IoT Core."""
        try:
            payload = json.loads(message.payload.decode('utf-8'))
            command = payload.get('command', '')
            value = payload.get('value')
            
            rospy.loginfo("Cloud command: {} = {}".format(command, value))
            
            if command == 'buzzer':
                # Buzzer control
                buzzer_freq = int(value) if value is not None else 0
                self.pub_buzzer.publish(UInt16(buzzer_freq))
                rospy.loginfo("Buzzer: {} Hz".format(buzzer_freq))
                
            elif command == 'move':
                # Velocity command
                if isinstance(value, dict):
                    twist = Twist()
                    twist.linear.x = value.get('linear_x', 0.0)
                    twist.angular.z = value.get('angular_z', 0.0)
                    self.pub_cmd_vel.publish(twist)
                    rospy.loginfo("Move: linear={}, angular={}".format(
                        twist.linear.x, twist.angular.z))
                
            elif command == 'stop':
                # Emergency stop
                twist = Twist()
                twist.linear.x = 0.0
                twist.angular.z = 0.0
                self.pub_cmd_vel.publish(twist)
                rospy.loginfo("Emergency stop")
                
            elif command == 'status':
                # Request status - publish immediate telemetry
                self.publish_telemetry()
                
            else:
                rospy.logwarn("Unknown command: {}".format(command))
                
        except Exception as e:
            rospy.logerr("Error parsing MQTT command: {}".format(e))
    
    # === CLOUD PUBLISHING ===
    
    def publish_telemetry(self):
        """Publish telemetry data to AWS IoT Core."""
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
                        'gas_detected': self.latest_gas_detected,
                        'scan': self.latest_scan,
                        'odom': self.latest_odom,
                        'imu': self.latest_imu
                    }
                }
            
            json_payload = json.dumps(payload)
            self.mqtt_client.publish(self.mqtt_topic_telemetry, json_payload, 1)
            
            rospy.logdebug("Published telemetry")
            
        except Exception as e:
            rospy.logwarn("Failed to publish telemetry: {}".format(e))
    
    def publish_alert(self, alert_type, value):
        """Publish alert to AWS IoT Core."""
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
            
            rospy.loginfo("Alert: {} = {}".format(alert_type, value))
            
        except Exception as e:
            rospy.logerr("Failed to publish alert: {}".format(e))
    
    def run(self):
        """Main loop: periodically publish telemetry to cloud."""
        if not self.enable_cloud:
            rospy.logwarn("Cloud not enabled, exiting node")
            return
        
        rospy.loginfo("Cloud Bridge running at {} Hz".format(self.publish_rate))
        
        # Create a timer for periodic telemetry
        rospy.Timer(rospy.Duration(1.0/self.publish_rate), self.timer_callback)
        rospy.spin()
    
    def timer_callback(self, event):
        """Timer callback for periodic telemetry publishing."""
        if self.cloud_connected:
            try:
                self.publish_telemetry()
            except Exception as e:
                rospy.logerr("Error publishing telemetry: {}".format(e))
    
    def shutdown_hook(self):
        """Clean shutdown."""
        rospy.loginfo("=== Shutting down Cloud Bridge ===")
        
        if self.cloud_connected and self.mqtt_client:
            try:
                self.mqtt_client.disconnect()
                rospy.loginfo("Disconnected from AWS IoT Core")
            except Exception as e:
                rospy.logwarn("Error during disconnect: {}".format(e))


if __name__ == '__main__':
    try:
        node = CloudBridgeNode()
        node.run()
    except rospy.ROSInterruptException:
        pass
    except Exception as e:
        rospy.logerr("Fatal error: {}".format(e))