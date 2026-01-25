#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""
Cloud Bridge Node - OPTIMIZED FINAL VERSION
AWS IoT Core connection with successful certificate setup
"""

import rospy
import json
import time
import os
import paho.mqtt.client as mqtt
import ssl
from std_msgs.msg import Float32, Bool, UInt16
from sensor_msgs.msg import Temperature
from geometry_msgs.msg import Twist

class CloudBridgeNode:
    def __init__(self):
        rospy.init_node('cloud_bridge_node', anonymous=False)
        rospy.loginfo("🚀 AWS IoT Cloud Bridge Starting...")
        
        # Get AWS configuration
        self.aws_endpoint = rospy.get_param('~aws_endpoint', 'a1k8itxfx77i0w-ats.iot.us-east-1.amazonaws.com')
        self.client_id = rospy.get_param('~client_id', 'robot')
        
        # Certificate paths - VERIFIED WORKING PATHS
        self.root_ca = rospy.get_param('~root_ca_path', '/home/omar/catkin_ws/src/elderly_bot/aws_certs/AmazonRootCA1.pem')
        self.cert = rospy.get_param('~cert_path', '/home/omar/catkin_ws/src/elderly_bot/aws_certs/certificate.pem.crt')
        self.key = rospy.get_param('~key_path', '/home/omar/catkin_ws/src/elderly_bot/aws_certs/private.pem.key')
        
        # Verify certificates exist
        self.verify_certificates()
        
        # Topics
        self.telemetry_topic = rospy.get_param('~mqtt_topic_telemetry', 'robot/telemetry')
        self.commands_topic = rospy.get_param('~mqtt_topic_commands', 'robot/commands')
        
        # Data storage
        self.temperature = 45.0
        self.power = 12.3
        self.gas_detected = False
        self.connected = False
        
        # Setup MQTT
        self.setup_mqtt()
        
        # ROS subscribers
        rospy.Subscriber('/jetson_temperature', Temperature, self.temp_callback)
        rospy.Subscriber('/jetson_power', Float32, self.power_callback)
        rospy.Subscriber('/gas_detected', Bool, self.gas_callback)
        
        # ROS publishers
        self.buzzer_pub = rospy.Publisher('/buzzer_command', UInt16, queue_size=1)
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        
        # Telemetry timer
        publish_rate = rospy.get_param('~publish_rate', 2.0)
        rospy.Timer(rospy.Duration(publish_rate), self.send_telemetry)
        
        rospy.on_shutdown(self.shutdown)
        rospy.loginfo("✅ Cloud Bridge Ready")
    
    def verify_certificates(self):
        """Verify all certificate files exist"""
        files = [
            (self.root_ca, "Root CA"),
            (self.cert, "Device Certificate"),
            (self.key, "Private Key")
        ]
        
        for path, name in files:
            if os.path.exists(path):
                rospy.loginfo("✓ %s: %s", name, os.path.basename(path))
            else:
                rospy.logerr("❌ %s not found: %s", name, path)
                rospy.signal_shutdown("Missing certificate file")
    
    def setup_mqtt(self):
        """Setup MQTT connection to AWS IoT"""
        try:
            self.mqtt_client = mqtt.Client(client_id=self.client_id)
            
            # TLS configuration - VERIFIED WORKING SETTINGS
            self.mqtt_client.tls_set(
                ca_certs=self.root_ca,
                certfile=self.cert,
                keyfile=self.key,
                tls_version=ssl.PROTOCOL_TLSv1_2
            )
            self.mqtt_client.tls_insecure_set(True)
            
            # Callbacks
            self.mqtt_client.on_connect = self.on_connect
            self.mqtt_client.on_message = self.on_message
            
            # Connect
            rospy.loginfo("Connecting to AWS IoT...")
            self.mqtt_client.connect_async(self.aws_endpoint, 8883, 60)
            self.mqtt_client.loop_start()
            
        except Exception as e:
            rospy.logerr("MQTT setup error: %s", str(e))
    
    def on_connect(self, client, userdata, flags, rc):
        """Connection callback"""
        if rc == 0:
            self.connected = True
            rospy.loginfo("✅ CONNECTED to AWS IoT Core")
            client.subscribe(self.commands_topic, qos=1)
        else:
            rospy.logerr("Connection failed: %s", rc)
    
    def on_message(self, client, userdata, msg):
        """Handle incoming commands"""
        try:
            data = json.loads(msg.payload)
            cmd = data.get('command', '')
            
            if cmd == 'buzzer':
                freq = int(data.get('value', 0))
                self.buzzer_pub.publish(UInt16(freq))
                rospy.loginfo("Buzzer: %d Hz", freq)
                
            elif cmd == 'move':
                t = Twist()
                if isinstance(data.get('value'), dict):
                    t.linear.x = float(data['value'].get('linear_x', 0))
                    t.angular.z = float(data['value'].get('angular_z', 0))
                self.cmd_vel_pub.publish(t)
                rospy.loginfo("Move: linear=%.2f, angular=%.2f", t.linear.x, t.angular.z)
                
            elif cmd == 'stop':
                self.cmd_vel_pub.publish(Twist())
                rospy.loginfo("Stop command")
                
        except Exception as e:
            rospy.logerr("Command error: %s", str(e))
    
    def temp_callback(self, msg):
        self.temperature = msg.temperature
    
    def power_callback(self, msg):
        self.power = msg.data
    
    def gas_callback(self, msg):
        self.gas_detected = msg.data
        if msg.data:
            rospy.logwarn("⚠️ Gas detected!")
            self.send_alert()
    
    def send_telemetry(self, event=None):
        """Send telemetry to AWS IoT"""
        if not self.connected:
            return
            
        try:
            data = {
                'timestamp': time.time(),
                'temperature': round(self.temperature, 2),
                'power': round(self.power, 2),
                'gas_detected': self.gas_detected,
                'robot_id': self.client_id
            }
            
            self.mqtt_client.publish(self.telemetry_topic, json.dumps(data), qos=1)
            rospy.loginfo("📡 Telemetry sent: temp=%.1f°C, power=%.1fW", 
                         self.temperature, self.power)
            
        except Exception as e:
            rospy.logwarn("Telemetry error: %s", str(e))
    
    def send_alert(self):
        """Send alert to AWS IoT"""
        if not self.connected:
            return
            
        try:
            alert = {
                'timestamp': time.time(),
                'alert': 'Gas detected!',
                'severity': 'HIGH',
                'robot_id': self.client_id
            }
            self.mqtt_client.publish('robot/alerts', json.dumps(alert), qos=1)
        except:
            pass
    
    def shutdown(self):
        """Clean shutdown"""
        rospy.loginfo("Shutting down Cloud Bridge...")
        if self.mqtt_client:
            self.mqtt_client.loop_stop()
            self.mqtt_client.disconnect()

if __name__ == '__main__':
    try:
        CloudBridgeNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("Cloud Bridge stopped")
    except Exception as e:
        rospy.logerr("Fatal error: %s", str(e))