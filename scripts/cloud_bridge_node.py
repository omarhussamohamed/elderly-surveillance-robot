#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""
Cloud Bridge Node - FIXED VERSION
Fixed MQTT connection and command parsing
"""

import rospy
import json
import time
import traceback
import os
from std_msgs.msg import Float32, Bool
from sensor_msgs.msg import Temperature
from geometry_msgs.msg import Twist

class CloudBridgeNode:
    def __init__(self):
        try:
            rospy.init_node('cloud_bridge_node')
            rospy.loginfo("Starting AWS IoT Bridge...")
            
            # Load parameters from ROS parameter server
            self.aws_endpoint = rospy.get_param('~aws_endpoint', 'a1k8itxfx77i0w-ats.iot.us-east-1.amazonaws.com')
            self.client_id = rospy.get_param('~client_id', 'robot')
            
            # Certificate paths - check if they exist
            self.root_ca = rospy.get_param('~root_ca_path', 
                                         '/home/omar/catkin_ws/src/elderly_bot/aws_certs/AmazonRootCA1.pem')
            self.cert = rospy.get_param('~cert_path', 
                                       '/home/omar/catkin_ws/src/elderly_bot/aws_certs/certificate.pem.crt')
            self.key = rospy.get_param('~key_path', 
                                      '/home/omar/catkin_ws/src/elderly_bot/aws_certs/private.pem.key')
            
            # Check if certs exist
            for path in [self.root_ca, self.cert, self.key]:
                if not os.path.exists(path):
                    rospy.logerr("Certificate file not found: %s", path)
            
            # MQTT Topics
            self.mqtt_topic_telemetry = rospy.get_param('~mqtt_topic_telemetry', 'robot/telemetry')
            self.mqtt_topic_alerts = rospy.get_param('~mqtt_topic_alerts', 'robot/alerts')
            self.mqtt_topic_commands = rospy.get_param('~mqtt_topic_commands', 'robot/commands')
            
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
            self.last_telemetry_log = 0
            
            # Setup ROS subscribers
            rospy.Subscriber('/jetson_temperature', Temperature, self.temp_cb)
            rospy.Subscriber('/jetson_power', Float32, self.power_cb)
            rospy.Subscriber('/gas_detected', Bool, self.gas_cb)
            
            # ROS publishers
            self.buzzer_pub = rospy.Publisher('/buzzer_command', Bool, queue_size=1, latch=True)
            self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
            
            # Try to connect to AWS
            if self.mqtt_available:
                self.connect_aws()
            else:
                rospy.logwarn("Running in simulation mode - MQTT not available")
            
            # Send telemetry every 3 seconds
            rospy.Timer(rospy.Duration(3), self.send_telemetry)
            
            rospy.on_shutdown(self.cleanup)
            rospy.loginfo("AWS Bridge Ready - Topics: telemetry=%s, commands=%s", 
                         self.mqtt_topic_telemetry, self.mqtt_topic_commands)
            
        except Exception as e:
            rospy.logerr("Failed to initialize cloud bridge: %s", str(e))
            rospy.logerr(traceback.format_exc())
            raise
    
    def connect_aws(self):
        """Connect to AWS IoT."""
        try:
            self.mqtt = self.mqtt_module.Client(client_id=self.client_id, protocol=self.mqtt_module.MQTTv311)
            
            # TLS setup
            self.mqtt.tls_set(
                ca_certs=self.root_ca,
                certfile=self.cert,
                keyfile=self.key,
                tls_version=self.ssl_module.PROTOCOL_TLSv1_2,
                ciphers=None
            )
            
            # Callbacks
            self.mqtt.on_connect = self.on_connect
            self.mqtt.on_message = self.on_message
            self.mqtt.on_disconnect = self.on_disconnect
            
            # Connect
            rospy.loginfo("Connecting to AWS IoT endpoint: %s", self.aws_endpoint)
            self.mqtt.connect(self.aws_endpoint, 8883, 60)
            self.mqtt.loop_start()
            
        except Exception as e:
            rospy.logerr("AWS connection error: %s", str(e))
            rospy.logerr(traceback.format_exc())
    
    def on_connect(self, client, userdata, flags, rc):
        """AWS connection established."""
        if rc == 0:
            rospy.loginfo("AWS IoT Connected successfully")
            client.subscribe(self.mqtt_topic_commands, qos=1)
            rospy.loginfo("Subscribed to: %s", self.mqtt_topic_commands)
        else:
            rospy.logerr("Connection failed with code: %s", rc)
    
    def on_disconnect(self, client, userdata, rc):
        """Handle disconnection."""
        if rc != 0:
            rospy.logwarn("Unexpected disconnection from AWS IoT")
            # Try to reconnect
            try:
                client.reconnect()
            except:
                pass
    
    def on_message(self, client, userdata, msg):
        """Handle commands from cloud."""
        try:
            rospy.loginfo("Received MQTT command: %s", msg.payload)
            data = json.loads(msg.payload)
            cmd = data.get('command', '').lower()
            value = data.get('value', '')
            
            rospy.loginfo("Parsed command: %s, value: %s", cmd, value)
            
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
                rospy.loginfo("Buzzer command: %s", "ON" if buzzer_state else "OFF")
            
            elif cmd == 'sleep':
                minutes = float(data.get('minutes', 30))
                rospy.loginfo("Sleep command: %.1f minutes", minutes)
                # Stop everything
                self.buzzer_pub.publish(Bool(False))
                self.cmd_vel_pub.publish(Twist())
            
            elif cmd == 'restart':
                rospy.logwarn("Restart command received")
            
            elif cmd == 'test':
                rospy.loginfo("Test command: %s", value)
            else:
                rospy.logwarn("Unknown command: %s", cmd)
            
        except Exception as e:
            rospy.logerr("Command parsing error: %s", str(e))
            rospy.logerr("Raw payload: %s", msg.payload)
    
    def temp_cb(self, msg):
        self.temperature = msg.temperature
    
    def power_cb(self, msg):
        self.power_voltage = msg.data
    
    def gas_cb(self, msg):
        self.gas_detected = msg.data
        # Only log gas detection, don't trigger buzzer automatically
    
    def send_telemetry(self, event=None):
        """Send sensor data to AWS IoT."""
        try:
            if hasattr(self, 'mqtt') and self.mqtt:
                data = {
                    'timestamp': time.time(),
                    'temperature': round(self.temperature, 1),
                    'power': round(self.power_voltage, 2),
                    'gas_detected': self.gas_detected
                }
                
                payload = json.dumps(data)
                self.mqtt.publish(self.mqtt_topic_telemetry, payload, qos=0)
                
                # Log every 30 seconds to avoid spam
                current_time = time.time()
                if current_time - self.last_telemetry_log >= 30:
                    rospy.loginfo("Telemetry sent: %.1f°C, %.1fW, gas=%s", 
                                self.temperature, self.power_voltage, self.gas_detected)
                    self.last_telemetry_log = current_time
                
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