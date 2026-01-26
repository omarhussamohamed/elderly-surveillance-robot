#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""
Cloud Bridge Node - FINAL FIXED VERSION
Uses Bool for buzzer (matching sensors_actuators_node)
"""

import rospy
import json
import time
import os
import subprocess
import threading
import paho.mqtt.client as mqtt
import ssl
from std_msgs.msg import Float32, Bool
from sensor_msgs.msg import Temperature
from geometry_msgs.msg import Twist

class CloudBridgeNode:
    def __init__(self):
        rospy.init_node('cloud_bridge_node')
        rospy.loginfo("Starting AWS IoT Bridge...")
        
        # AWS Configuration
        self.aws_endpoint = 'a1k8itxfx77i0w-ats.iot.us-east-1.amazonaws.com'
        self.client_id = 'robot'
        
        # Certificate paths
        cert_dir = '/home/omar/catkin_ws/src/elderly_bot/aws_certs'
        self.root_ca = cert_dir + '/AmazonRootCA1.pem'
        self.cert = cert_dir + '/certificate.pem.crt'
        self.key = cert_dir + '/private.pem.key'
        
        # Verify certificates
        self.check_certs()
        
        # Robot state
        self.temperature = 0.0
        self.power_voltage = 0.0
        self.gas_detected = False
        
        # Connect to AWS
        self.connect_aws()
        
        # ROS setup
        rospy.Subscriber('/jetson_temperature', Temperature, self.temp_cb)
        rospy.Subscriber('/jetson_power', Float32, self.power_cb)
        rospy.Subscriber('/gas_detected', Bool, self.gas_cb)
        
        # ROS publishers - USING BOOL for buzzer (matching your sensors_actuators_node)
        self.buzzer_pub = rospy.Publisher('/buzzer_command', Bool, queue_size=1)
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        
        # Send telemetry every 3 seconds
        rospy.Timer(rospy.Duration(3), self.send_telemetry)
        
        rospy.on_shutdown(self.cleanup)
        rospy.loginfo("✅ AWS Bridge Ready")
    
    def check_certs(self):
        """Check certificate files"""
        for path, name in [(self.root_ca, 'Root CA'), (self.cert, 'Certificate'), (self.key, 'Private Key')]:
            if not os.path.exists(path):
                rospy.logerr("Missing: %s - %s", name, path)
                rospy.signal_shutdown("Certificate missing")
    
    def connect_aws(self):
        """Connect to AWS IoT"""
        try:
            self.mqtt = mqtt.Client(client_id=self.client_id)
            
            # TLS setup
            self.mqtt.tls_set(
                ca_certs=self.root_ca,
                certfile=self.cert,
                keyfile=self.key,
                tls_version=ssl.PROTOCOL_TLSv1_2
            )
            self.mqtt.tls_insecure_set(True)
            
            # Callbacks
            self.mqtt.on_connect = self.on_connect
            self.mqtt.on_message = self.on_message
            
            # Connect
            rospy.loginfo("Connecting to AWS IoT...")
            self.mqtt.connect_async(self.aws_endpoint, 8883, 60)
            self.mqtt.loop_start()
            
        except Exception as e:
            rospy.logerr("AWS connection error: %s", str(e))
    
    def on_connect(self, client, userdata, flags, rc):
        """AWS connection established"""
        if rc == 0:
            rospy.loginfo("✅ AWS IoT Connected")
            client.subscribe('robot/commands', qos=1)
        else:
            rospy.logerr("Connection failed: %s", rc)
    
    def on_message(self, client, userdata, msg):
        """Handle commands from cloud"""
        try:
            rospy.loginfo("Received command: %s", msg.payload)
            data = json.loads(msg.payload)
            cmd = data.get('command', '').lower()
            value = data.get('value', '')
            
            if cmd == 'buzzer':
                # Convert to boolean (True/False)
                if isinstance(value, bool):
                    buzzer_state = value
                elif isinstance(value, (int, float)):
                    buzzer_state = bool(value)
                elif isinstance(value, str):
                    lower_val = value.lower().strip()
                    buzzer_state = lower_val in ['true', 'on', '1', 'yes', 'high']
                else:
                    buzzer_state = False
                
                # Publish Bool message (True=ON, False=OFF)
                msg_bool = Bool()
                msg_bool.data = buzzer_state
                self.buzzer_pub.publish(msg_bool)
                rospy.loginfo("🔔 Buzzer: %s", "ON" if buzzer_state else "OFF")
            
            elif cmd == 'sleep':
                # Sleep for specified minutes (default 30)
                minutes = float(data.get('minutes', 30))
                rospy.loginfo("😴 Sleeping for %.1f minutes", minutes)
                
                # Stop everything
                msg_bool = Bool()
                msg_bool.data = False
                self.buzzer_pub.publish(msg_bool)  # Buzzer off
                self.cmd_vel_pub.publish(Twist())  # Stop movement
                
                # Schedule wakeup
                self.schedule_wakeup(minutes * 60)
            
            elif cmd == 'restart':
                # Restart ROS system
                rospy.logwarn("🔄 Restart command received")
                self.restart_ros()
            
            elif cmd == 'test':
                # Test command
                rospy.loginfo("🧪 Test: %s", value)
            
        except Exception as e:
            rospy.logerr("Command error: %s", str(e))
    
    def schedule_wakeup(self, sleep_seconds):
        """Schedule automatic wakeup after sleep period"""
        def wakeup():
            rospy.sleep(sleep_seconds)
            rospy.loginfo("☀️ Wakeup from sleep")
        
        thread = threading.Thread(target=wakeup)
        thread.daemon = True
        thread.start()
    
    def restart_ros(self):
        """Kill ROS processes and restart bringup.launch"""
        def do_restart():
            # Give time for last message to send
            time.sleep(1)
            
            rospy.loginfo("Killing ROS nodes...")
            subprocess.call(['rosnode', 'kill', '-a'], 
                          stdout=subprocess.PIPE, stderr=subprocess.PIPE)
            time.sleep(3)
            
            rospy.loginfo("Restarting bringup.launch...")
            subprocess.Popen(['roslaunch', 'elderly_bot', 'bringup.launch'], 
                           stdout=subprocess.PIPE, stderr=subprocess.PIPE)
        
        thread = threading.Thread(target=do_restart)
        thread.daemon = True
        thread.start()
    
    def temp_cb(self, msg):
        self.temperature = msg.temperature
    
    def power_cb(self, msg):
        self.power_voltage = msg.data
    
    def gas_cb(self, msg):
        self.gas_detected = msg.data
        if msg.data:
            rospy.logwarn("⚠️ Gas detected!")
            self.send_alert()
    
    def send_alert(self):
        """Send gas alert to AWS"""
        try:
            alert = {
                'time': time.strftime('%H:%M:%S'),
                'alert': 'GAS_DETECTED',
                'temperature': round(self.temperature, 1),
                'power': round(self.power_voltage, 2)
            }
            self.mqtt.publish('robot/alerts', json.dumps(alert), qos=1)
        except:
            pass
    
    def send_telemetry(self, event=None):
        """Send sensor data to AWS IoT"""
        try:
            data = {
                'time': time.strftime('%H:%M:%S'),
                'temperature': round(self.temperature, 1),
                'power': round(self.power_voltage, 2),
                'gas': self.gas_detected
            }
            
            self.mqtt.publish('robot/telemetry', json.dumps(data), qos=1)
            rospy.loginfo("📡 Sent: %.1f°C, %.1fV, gas=%s", 
                         self.temperature, self.power_voltage, self.gas_detected)
            
        except Exception as e:
            rospy.logwarn("Telemetry error: %s", str(e))
    
    def cleanup(self):
        """Clean shutdown"""
        rospy.loginfo("Shutting down AWS Bridge")
        if hasattr(self, 'mqtt'):
            self.mqtt.loop_stop()
            self.mqtt.disconnect()

if __name__ == '__main__':
    try:
        CloudBridgeNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("Bridge stopped")
    except Exception as e:
        rospy.logerr("Error: %s", str(e))