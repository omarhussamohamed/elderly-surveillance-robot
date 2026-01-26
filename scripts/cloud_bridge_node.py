#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""
Cloud Bridge Node - SIMPLIFIED WORKING VERSION
"""

import rospy
import json
import time
import os
import traceback
from std_msgs.msg import Float32, Bool
from sensor_msgs.msg import Temperature
from geometry_msgs.msg import Twist

class CloudBridgeNode:
    def __init__(self):
        try:
            rospy.init_node('cloud_bridge_node')
            rospy.loginfo("Starting AWS IoT Bridge...")
            
            # Load configuration with defaults
            config_file = '/home/omar/catkin_ws/src/elderly_bot/config/cloud_config.yaml'
            
            # Read config file manually
            config = {}
            if os.path.exists(config_file):
                with open(config_file, 'r') as f:
                    import yaml
                    config = yaml.safe_load(f)
            
            # AWS Configuration
            self.aws_endpoint = config.get('aws_endpoint', 'a1k8itxfx77i0w-ats.iot.us-east-1.amazonaws.com')
            self.client_id = config.get('client_id', 'robot')
            
            # Certificate paths
            cert_dir = '/home/omar/catkin_ws/src/elderly_bot/aws_certs'
            self.root_ca = cert_dir + '/AmazonRootCA1.pem'
            self.cert = cert_dir + '/certificate.pem.crt'
            self.key = cert_dir + '/private.pem.key'
            
            # Verify certificates exist
            for path in [self.root_ca, self.cert, self.key]:
                if not os.path.exists(path):
                    rospy.logerr("Certificate missing: %s", path)
                    return
            
            # Try to import paho-mqtt
            try:
                import paho.mqtt.client as mqtt
                import ssl
                self.mqtt_module = mqtt
                self.ssl_module = ssl
                self.mqtt_available = True
            except ImportError:
                rospy.logerr("paho-mqtt not installed. Install with: sudo pip install paho-mqtt")
                self.mqtt_available = False
                return
            
            # Robot state
            self.temperature = 0.0
            self.power_voltage = 0.0
            self.gas_detected = False
            
            # Setup ROS
            rospy.Subscriber('/jetson_temperature', Temperature, self.temp_cb)
            rospy.Subscriber('/jetson_power', Float32, self.power_cb)
            rospy.Subscriber('/gas_detected', Bool, self.gas_cb)
            
            # ROS publishers
            self.buzzer_pub = rospy.Publisher('/buzzer_command', Bool, queue_size=1)
            self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
            
            # Try to connect to AWS
            self.connect_aws()
            
            # Send telemetry every 3 seconds
            rospy.Timer(rospy.Duration(3), self.send_telemetry)
            
            rospy.on_shutdown(self.cleanup)
            rospy.loginfo("AWS Bridge Ready")
            
        except Exception as e:
            rospy.logerr("Failed to initialize cloud bridge: %s", str(e))
            rospy.logerr(traceback.format_exc())
            raise
    
    def connect_aws(self):
        """Connect to AWS IoT."""
        if not self.mqtt_available:
            return
        
        try:
            self.mqtt = self.mqtt_module.Client(client_id=self.client_id)
            
            # TLS setup
            self.mqtt.tls_set(
                ca_certs=self.root_ca,
                certfile=self.cert,
                keyfile=self.key,
                tls_version=self.ssl_module.PROTOCOL_TLSv1_2
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
        """AWS connection established."""
        if rc == 0:
            rospy.loginfo("AWS IoT Connected")
            client.subscribe('robot/commands', qos=1)
        else:
            rospy.logerr("Connection failed: %s", rc)
    
    def on_message(self, client, userdata, msg):
        """Handle commands from cloud."""
        try:
            rospy.loginfo("Received command: %s", msg.payload)
            data = json.loads(msg.payload)
            cmd = data.get('command', '').lower()
            value = data.get('value', '')
            
            if cmd == 'buzzer':
                # Convert to boolean
                if isinstance(value, bool):
                    buzzer_state = value
                elif isinstance(value, (int, float)):
                    buzzer_state = bool(value)
                elif isinstance(value, str):
                    lower_val = value.lower().strip()
                    buzzer_state = lower_val in ['true', 'on', '1', 'yes', 'high']
                else:
                    buzzer_state = False
                
                # Publish Bool message
                msg_bool = Bool()
                msg_bool.data = buzzer_state
                self.buzzer_pub.publish(msg_bool)
                rospy.loginfo("Buzzer: %s", "ON" if buzzer_state else "OFF")
            
            elif cmd == 'sleep':
                minutes = float(data.get('minutes', 30))
                rospy.loginfo("Sleeping for %.1f minutes", minutes)
                
                # Stop everything
                msg_bool = Bool()
                msg_bool.data = False
                self.buzzer_pub.publish(msg_bool)
                self.cmd_vel_pub.publish(Twist())
            
            elif cmd == 'restart':
                rospy.logwarn("Restart command received")
            
            elif cmd == 'test':
                rospy.loginfo("Test: %s", value)
            
        except Exception as e:
            rospy.logerr("Command error: %s", str(e))
    
    def temp_cb(self, msg):
        self.temperature = msg.temperature
    
    def power_cb(self, msg):
        self.power_voltage = msg.data
    
    def gas_cb(self, msg):
        self.gas_detected = msg.data
        if msg.data:
            rospy.logwarn("Gas detected!")
            self.send_alert()
    
    def send_alert(self):
        """Send gas alert to AWS."""
        try:
            if hasattr(self, 'mqtt') and self.mqtt:
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
        """Send sensor data to AWS IoT."""
        try:
            if hasattr(self, 'mqtt') and self.mqtt:
                data = {
                    'time': time.strftime('%H:%M:%S'),
                    'temperature': round(self.temperature, 1),
                    'power': round(self.power_voltage, 2),
                    'gas': self.gas_detected
                }
                
                self.mqtt.publish('robot/telemetry', json.dumps(data), qos=1)
                rospy.loginfo("Sent: %.1fC, %.1fV, gas=%s", 
                             self.temperature, self.power_voltage, self.gas_detected)
                
        except Exception as e:
            rospy.logwarn("Telemetry error: %s", str(e))
    
    def cleanup(self):
        """Clean shutdown."""
        rospy.loginfo("Shutting down AWS Bridge")
        if hasattr(self, 'mqtt'):
            try:
                self.mqtt.loop_stop()
                self.mqtt.disconnect()
            except:
                pass

if __name__ == '__main__':
    try:
        CloudBridgeNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("Bridge stopped")
    except Exception as e:
        rospy.logerr("Error: %s", str(e))
        rospy.logerr(traceback.format_exc())