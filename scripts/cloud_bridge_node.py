#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""
Cloud Bridge Node for AWS IoT Core - FIXED FOR THING NAME "robot"
"""
import rospy
import json
import time
import os
import sys
import socket
import paho.mqtt.client as mqtt
import ssl
from std_msgs.msg import Float32, Bool, UInt16
from sensor_msgs.msg import Temperature
from geometry_msgs.msg import Twist

class CloudBridgeNode:
    def __init__(self):
        rospy.init_node('cloud_bridge_node')
        
        rospy.loginfo("=== Cloud Bridge Node Starting ===")
        
        # Get parameters
        self.enable_cloud = rospy.get_param('~enable_cloud', False)
        
        if not self.enable_cloud:
            rospy.loginfo("Cloud bridge disabled by parameter")
            rospy.signal_shutdown("Cloud bridge disabled")
            return
        
        # AWS IoT Configuration - MUST MATCH YOUR THING NAME "robot"
        self.aws_endpoint = rospy.get_param('~aws_endpoint', 'a1k8itxfx77i0w-ats.iot.us-east-1.amazonaws.com')
        self.client_id = rospy.get_param('~client_id', 'robot')  # IMPORTANT: MUST BE "robot"
        self.port = 8883
        
        # Certificate paths
        self.root_ca = rospy.get_param('~root_ca_path', '')
        self.cert = rospy.get_param('~cert_path', '')
        self.key = rospy.get_param('~key_path', '')
        
        # Verify files exist
        for path, name in [(self.root_ca, "Root CA"), (self.cert, "Certificate"), (self.key, "Private key")]:
            if not os.path.exists(path):
                rospy.logerr("%s not found: %s", name, path)
                rospy.signal_shutdown("Certificate file not found")
                return
        
        # Topics
        self.telemetry_topic = rospy.get_param('~mqtt_topic_telemetry', 'elderly_bot/telemetry')
        self.commands_topic = rospy.get_param('~mqtt_topic_commands', 'elderly_bot/commands')
        self.alerts_topic = rospy.get_param('~mqtt_topic_alerts', 'elderly_bot/alerts')
        
        # Data storage
        self.temperature = 0.0
        self.power = 0.0
        self.gas_detected = False
        self.connected = False
        
        rospy.loginfo("Configuration:")
        rospy.loginfo("  Endpoint: %s", self.aws_endpoint)
        rospy.loginfo("  Client ID: %s (MUST match AWS IoT Thing name)", self.client_id)
        rospy.loginfo("  Certificates: OK")
        
        # Test connection first
        if not self.test_connection():
            rospy.logerr("Initial connection test failed")
            return
        
        # Setup MQTT
        self.mqtt_client = None
        self.setup_mqtt()
        
        # ROS subscribers
        rospy.Subscriber('/jetson_temperature', Temperature, self.temp_callback)
        rospy.Subscriber('/jetson_power', Float32, self.power_callback)
        rospy.Subscriber('/gas_detected', Bool, self.gas_callback)
        
        # ROS publishers
        self.buzzer_pub = rospy.Publisher('/buzzer_command', UInt16, queue_size=1)
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        
        rospy.loginfo("Cloud Bridge initialized")
        
        # Start telemetry publishing
        publish_rate = rospy.get_param('~publish_rate', 2.0)
        rospy.Timer(rospy.Duration(1.0/publish_rate), self.send_telemetry)
        
        rospy.on_shutdown(self.shutdown)
    
    def test_connection(self):
        """Test basic TCP connection to AWS IoT"""
        try:
            rospy.loginfo("Testing TCP connection to %s:%s...", self.aws_endpoint, self.port)
            sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            sock.settimeout(5)
            sock.connect((self.aws_endpoint, self.port))
            sock.close()
            rospy.loginfo("TCP connection test: OK")
            return True
        except Exception as e:
            rospy.logerr("TCP connection test failed: %s", str(e))
            return False
    
    def setup_mqtt(self):
        """Setup MQTT connection to AWS IoT"""
        try:
            # Create client with the Thing name as client ID
            self.mqtt_client = mqtt.Client(client_id=self.client_id, protocol=mqtt.MQTTv311)
            
            # Configure TLS
            self.mqtt_client.tls_set(
                ca_certs=self.root_ca,
                certfile=self.cert,
                keyfile=self.key,
                tls_version=ssl.PROTOCOL_TLSv1_2
            )
            
            # Set callbacks
            self.mqtt_client.on_connect = self.on_connect
            self.mqtt_client.on_message = self.on_message
            self.mqtt_client.on_disconnect = self.on_disconnect
            
            # Connect
            rospy.loginfo("Connecting to AWS IoT MQTT broker as '%s'...", self.client_id)
            self.mqtt_client.connect(self.aws_endpoint, self.port, 60)
            self.mqtt_client.loop_start()
            
        except Exception as e:
            rospy.logerr("MQTT setup failed: %s", str(e))
    
    def on_connect(self, client, userdata, flags, rc):
        if rc == 0:
            self.connected = True
            rospy.loginfo("Successfully connected to AWS IoT Core as '%s'!", self.client_id)
            
            # Subscribe to commands
            client.subscribe(self.commands_topic, qos=1)
            rospy.loginfo("Subscribed to: %s", self.commands_topic)
            
            # Send connection success message
            self.send_telemetry(immediate=True)
            
        else:
            self.connected = False
            error_messages = {
                1: "Incorrect protocol version",
                2: "Invalid client identifier",
                3: "Server unavailable", 
                4: "Bad username or password",
                5: "Not authorized"
            }
            error_msg = error_messages.get(rc, "Unknown error: {}".format(rc))
            rospy.logerr("Connection failed: %s", error_msg)
            
            if rc == 5:  # Not authorized
                rospy.logerr("ERROR: Client ID '%s' does not match AWS IoT Thing name!", self.client_id)
                rospy.logerr("Your Thing is named 'robot', so client_id MUST be 'robot'")
    
    def on_disconnect(self, client, userdata, rc):
        self.connected = False
        if rc != 0:
            rospy.logwarn("Disconnected from AWS IoT. Code: %s", rc)
    
    def on_message(self, client, userdata, msg):
        """Handle commands from AWS IoT"""
        try:
            payload = json.loads(msg.payload)
            command = payload.get('command', '')
            value = payload.get('value', '')
            
            rospy.loginfo("Cloud command: %s = %s", command, value)
            
            if command == 'buzzer':
                freq = int(value) if value else 0
                self.buzzer_pub.publish(UInt16(freq))
                rospy.loginfo("Buzzer: %d Hz", freq)
                
            elif command == 'move':
                twist = Twist()
                if isinstance(value, dict):
                    twist.linear.x = float(value.get('linear_x', 0.0))
                    twist.angular.z = float(value.get('angular_z', 0.0))
                self.cmd_vel_pub.publish(twist)
                
            elif command == 'stop':
                twist = Twist()
                twist.linear.x = 0.0
                twist.angular.z = 0.0
                self.cmd_vel_pub.publish(twist)
                rospy.logwarn("Emergency stop from cloud")
                
            else:
                rospy.logwarn("Unknown command: %s", command)
                
        except Exception as e:
            rospy.logerr("Command error: %s", str(e))
    
    # ROS callbacks
    def temp_callback(self, msg):
        self.temperature = msg.temperature
    
    def power_callback(self, msg):
        self.power = msg.data
    
    def gas_callback(self, msg):
        old_value = self.gas_detected
        self.gas_detected = msg.data
        
        if msg.data and not old_value:
            self.send_alert("Gas detected!")
            rospy.logwarn("Gas detected! Alert sent.")
    
    def send_telemetry(self, event=None, immediate=False):
        """Send telemetry to AWS IoT"""
        if not self.connected or not self.mqtt_client:
            return
        
        try:
            telemetry = {
                'timestamp': time.time(),
                'robot_id': self.client_id,
                'temperature': round(self.temperature, 2),
                'power': round(self.power, 2),
                'gas_detected': self.gas_detected
            }
            
            self.mqtt_client.publish(
                self.telemetry_topic,
                json.dumps(telemetry),
                qos=1
            )
            
            if immediate:
                rospy.loginfo("Telemetry sent to cloud")
            
        except Exception as e:
            rospy.logwarn("Telemetry error: %s", str(e))
    
    def send_alert(self, message):
        """Send alert to AWS IoT"""
        if not self.connected or not self.mqtt_client:
            return
        
        try:
            alert = {
                'timestamp': time.time(),
                'robot_id': self.client_id,
                'alert': message
            }
            
            self.mqtt_client.publish(
                self.alerts_topic,
                json.dumps(alert),
                qos=1
            )
            
        except Exception as e:
            rospy.logerr("Alert error: %s", str(e))
    
    def shutdown(self):
        """Clean shutdown"""
        rospy.loginfo("Shutting down Cloud Bridge")
        if self.mqtt_client:
            try:
                self.mqtt_client.loop_stop()
                self.mqtt_client.disconnect()
            except:
                pass

if __name__ == '__main__':
    try:
        node = CloudBridgeNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
    except Exception as e:
        rospy.logerr("Fatal error: %s", str(e))