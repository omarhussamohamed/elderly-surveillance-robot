#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""
Cloud Bridge Node - FINAL VERSION
With buzzer fix, sleep mode, restart command
"""

import rospy
import json
import time
import os
import subprocess
import threading
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
        
        # Certificate paths
        self.root_ca = rospy.get_param('~root_ca_path', '/home/omar/catkin_ws/src/elderly_bot/aws_certs/AmazonRootCA1.pem')
        self.cert = rospy.get_param('~cert_path', '/home/omar/catkin_ws/src/elderly_bot/aws_certs/certificate.pem.crt')
        self.key = rospy.get_param('~key_path', '/home/omar/catkin_ws/src/elderly_bot/aws_certs/private.pem.key')
        
        # Verify certificates exist
        self.verify_certificates()
        
        # Topics
        self.telemetry_topic = rospy.get_param('~mqtt_topic_telemetry', 'robot/telemetry')
        self.commands_topic = rospy.get_param('~mqtt_topic_commands', 'robot/commands')
        
        # Robot state
        self.temperature = 45.0
        self.power_voltage = 12.3  # Changed to more descriptive name
        self.gas_detected = False
        self.connected = False
        self.robot_mode = "normal"  # normal, sleep, idle
        
        # Setup MQTT
        self.setup_mqtt()
        
        # ROS subscribers
        rospy.Subscriber('/jetson_temperature', Temperature, self.temp_callback)
        rospy.Subscriber('/jetson_power', Float32, self.power_callback)
        rospy.Subscriber('/gas_detected', Bool, self.gas_callback)
        
        # ROS publishers
        self.buzzer_pub = rospy.Publisher('/buzzer_command', Bool, queue_size=1)
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
            
            # TLS configuration
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
            rospy.loginfo("✅ Subscribed to: %s", self.commands_topic)
        else:
            rospy.logerr("Connection failed: %s", rc)
    
    def on_message(self, client, userdata, msg):
        """Handle incoming commands"""
        try:
            rospy.loginfo("📨 Received cloud command: %s", msg.payload)
            data = json.loads(msg.payload)
            cmd = data.get('command', '').lower()
            value = data.get('value', '')
            
            # Handle different commands
            if cmd == 'buzzer':
                self.handle_buzzer(value)
                
            elif cmd == 'move':
                self.handle_move(value)
                
            elif cmd == 'stop':
                self.handle_stop()
                
            elif cmd == 'test':
                self.handle_test(value)
                
            elif cmd == 'sleep':
                self.handle_sleep()
                
            elif cmd == 'wake':
                self.handle_wake()
                
            elif cmd == 'restart':
                self.handle_restart()
                
            elif cmd == 'status':
                self.handle_status()
                
            else:
                rospy.logwarn("❓ Unknown command: %s", cmd)
                
        except Exception as e:
            rospy.logerr("❌ Command error: %s", str(e))
    
    def handle_buzzer(self, value):
        """Handle buzzer command - FIXED VERSION"""
        # DEBUG: Log what we received
        rospy.loginfo("Buzzer command value: %s (type: %s)", value, type(value))
        
        # Convert to boolean with more robust parsing
        if isinstance(value, bool):
            buzzer_state = value
        elif isinstance(value, (int, float)):
            buzzer_state = bool(int(value))
        elif isinstance(value, str):
            lower_val = value.lower().strip()
            buzzer_state = (lower_val in ['true', 'on', '1', 'yes', 'high', 'enable'])
        else:
            buzzer_state = False
        
        rospy.loginfo("Buzzer parsed to: %s", "ON" if buzzer_state else "OFF")
        
        # Publish with explicit Bool type
        msg = Bool()
        msg.data = buzzer_state
        self.buzzer_pub.publish(msg)
        rospy.loginfo("🔔 Buzzer: %s", "ON" if buzzer_state else "OFF")
    
    def handle_move(self, value):
        """Handle move command"""
        if self.robot_mode != "normal":
            rospy.logwarn("⚠️ Robot in %s mode - movement blocked", self.robot_mode)
            return
            
        t = Twist()
        try:
            if isinstance(value, dict):
                t.linear.x = float(value.get('linear_x', 0))
                t.angular.z = float(value.get('angular_z', 0))
            elif isinstance(value, str):
                # Parse string like "0.5,0.1"
                parts = value.split(',')
                if len(parts) >= 2:
                    t.linear.x = float(parts[0])
                    t.angular.z = float(parts[1])
            
            # Limit speed for safety
            t.linear.x = max(-0.5, min(0.5, t.linear.x))
            t.angular.z = max(-1.0, min(1.0, t.angular.z))
            
            self.cmd_vel_pub.publish(t)
            rospy.loginfo("🤖 Move: linear=%.2f m/s, angular=%.2f rad/s", 
                         t.linear.x, t.angular.z)
        except Exception as e:
            rospy.logerr("Move error: %s", str(e))
    
    def handle_stop(self):
        """Emergency stop"""
        t = Twist()
        self.cmd_vel_pub.publish(t)
        rospy.logwarn("🛑 Emergency STOP from cloud")
    
    def handle_test(self, value):
        """Test command"""
        rospy.loginfo("🧪 Test command: %s", value)
        # Send test response back
        self.send_mqtt_message('robot/test_response', {
            'message': 'Test acknowledged',
            'test_value': str(value),
            'timestamp': self.get_formatted_time()
        })
    
    def handle_sleep(self):
        """Put robot in sleep/idle mode"""
        self.robot_mode = "sleep"
        
        # Stop all movement
        t = Twist()
        self.cmd_vel_pub.publish(t)
        
        # Turn off buzzer
        msg = Bool()
        msg.data = False
        self.buzzer_pub.publish(msg)
        
        rospy.loginfo("😴 Robot entering SLEEP mode")
        self.send_mqtt_message('robot/status', {
            'mode': 'sleep',
            'message': 'Robot in sleep mode',
            'timestamp': self.get_formatted_time()
        })
    
    def handle_wake(self):
        """Wake robot from sleep mode"""
        self.robot_mode = "normal"
        rospy.loginfo("☀️ Robot WAKING UP to normal mode")
        self.send_mqtt_message('robot/status', {
            'mode': 'normal',
            'message': 'Robot in normal mode',
            'timestamp': self.get_formatted_time()
        })
    
    def handle_restart(self):
        """Restart ROS and bringup launch"""
        rospy.logwarn("🔄 RESTART command received - restarting ROS nodes")
        
        # Send response before restarting
        self.send_mqtt_message('robot/status', {
            'mode': 'restarting',
            'message': 'Restarting ROS system',
            'timestamp': self.get_formatted_time()
        })
        
        # Run restart in background thread
        def restart_ros():
            time.sleep(1)  # Give MQTT time to send message
            try:
                # Kill all ROS nodes
                subprocess.call(['rosnode', 'kill', '-a'])
                time.sleep(2)
                
                # Restart bringup launch
                rospy.loginfo("Restarting bringup.launch...")
                subprocess.Popen(['roslaunch', 'elderly_bot', 'bringup.launch'], 
                                stdout=subprocess.PIPE, 
                                stderr=subprocess.PIPE)
            except Exception as e:
                rospy.logerr("Restart error: %s", str(e))
        
        thread = threading.Thread(target=restart_ros)
        thread.daemon = True
        thread.start()
    
    def handle_status(self):
        """Send immediate status report"""
        rospy.loginfo("📊 Status report requested")
        self.send_telemetry(immediate=True)
    
    def temp_callback(self, msg):
        self.temperature = msg.temperature
    
    def power_callback(self, msg):
        self.power_voltage = msg.data
    
    def gas_callback(self, msg):
        self.gas_detected = msg.data
        if msg.data:
            rospy.logwarn("⚠️ Gas detected!")
            self.send_alert()
    
    def get_formatted_time(self):
        """Get readable timestamp"""
        return time.strftime('%Y-%m-%d %H:%M:%S')
    
    def send_telemetry(self, event=None, immediate=False):
        """Send telemetry to AWS IoT"""
        if not self.connected:
            if immediate:
                rospy.logdebug("Not connected, skipping telemetry")
            return
            
        try:
            # Prepare telemetry data - CLEAN FORMAT
            telemetry = {
                'timestamp': self.get_formatted_time(),  # Human readable
                'temperature_c': round(self.temperature, 1),  # Clear units
                'power_voltage': round(self.power_voltage, 2),  # Clear metric
                'power_status': 'Normal' if self.power_voltage > 11.0 else 'Low',  # Status indicator
                'gas_detected': self.gas_detected,
                'robot_mode': self.robot_mode,
                'uptime_seconds': int(time.time() - rospy.get_time())  # How long running
            }
            
            self.mqtt_client.publish(self.telemetry_topic, json.dumps(telemetry), qos=1)
            
            if immediate:
                rospy.loginfo("📡 Telemetry sent (immediate)")
            else:
                rospy.logdebug("📡 Telemetry sent: %.1f°C, %.1fV, gas=%s", 
                             self.temperature, self.power_voltage, self.gas_detected)
            
        except Exception as e:
            rospy.logwarn("Telemetry error: %s", str(e))
    
    def send_alert(self):
        """Send alert to AWS IoT"""
        if not self.connected:
            return
            
        try:
            alert = {
                'timestamp': self.get_formatted_time(),
                'alert_type': 'GAS_DETECTED',
                'severity': 'HIGH',
                'temperature_c': round(self.temperature, 1),
                'power_voltage': round(self.power_voltage, 2),
                'message': 'Gas detected! Immediate action required.'
            }
            self.mqtt_client.publish('robot/alerts', json.dumps(alert), qos=1)
            rospy.logwarn("🚨 Gas alert sent to cloud!")
        except Exception as e:
            rospy.logerr("Alert error: %s", str(e))
    
    def send_mqtt_message(self, topic, data):
        """Helper to send MQTT messages"""
        if not self.connected:
            return
            
        try:
            self.mqtt_client.publish(topic, json.dumps(data), qos=1)
        except Exception as e:
            rospy.logwarn("MQTT send error: %s", str(e))
    
    def shutdown(self):
        """Clean shutdown"""
        rospy.loginfo("Shutting down Cloud Bridge...")
        
        # Send disconnect message
        if self.connected:
            self.send_mqtt_message('robot/status', {
                'mode': 'offline',
                'message': 'Robot shutting down',
                'timestamp': self.get_formatted_time()
            })
        
        # Clean up MQTT
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