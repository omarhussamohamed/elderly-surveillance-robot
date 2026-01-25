#!/usr/bin/env python2
"""
simple_cloud_bridge.py - Working AWS IoT bridge without SDK issues
"""
import rospy
import json
import time
import os
import sys
from std_msgs.msg import Float32, Bool, UInt16
from sensor_msgs.msg import Temperature
from geometry_msgs.msg import Twist

# Use MQTT with TLS
import paho.mqtt.client as mqtt
import ssl

class SimpleCloudBridge:
    def __init__(self):
        rospy.init_node('simple_cloud_bridge')
        
        # Configuration
        self.enable_cloud = rospy.get_param('~enable_cloud', True)
        if not self.enable_cloud:
            rospy.loginfo("Cloud disabled")
            return
        
        self.endpoint = "a1k8itxfx77i0w-ats.iot.us-east-1.amazonaws.com"
        self.port = 8883
        self.client_id = "robot_nano"
        
        # Certificate paths
        self.ca_cert = "/home/omar/catkin_ws/src/elderly_bot/aws_certs/AmazonRootCA1.pem"
        self.device_cert = "/home/omar/catkin_ws/src/elderly_bot/aws_certs/certificate.pem.crt"
        self.private_key = "/home/omar/catkin_ws/src/elderly_bot/aws_certs/private.pem.key"
        
        # Verify files exist
        for f in [self.ca_cert, self.device_cert, self.private_key]:
            if not os.path.exists(f):
                rospy.logerr("Certificate file not found: %s", f)
                return
        
        # Data storage
        self.temperature = 0.0
        self.power = 0.0
        self.gas_detected = False
        
        # Setup MQTT
        self.setup_mqtt()
        
        # ROS subscribers
        rospy.Subscriber('/jetson_temperature', Temperature, self.temp_cb)
        rospy.Subscriber('/jetson_power', Float32, self.power_cb)
        rospy.Subscriber('/gas_detected', Bool, self.gas_cb)
        
        # ROS publishers
        self.buzzer_pub = rospy.Publisher('/buzzer_command', UInt16, queue_size=1)
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        
        rospy.loginfo("Simple Cloud Bridge ready")
        
        # Start telemetry timer
        self.telemetry_timer = rospy.Timer(rospy.Duration(2.0), self.send_telemetry)
        
    def setup_mqtt(self):
        """Setup MQTT connection to AWS IoT"""
        try:
            self.mqtt_client = mqtt.Client(client_id=self.client_id)
            
            # Configure TLS
            self.mqtt_client.tls_set(
                ca_certs=self.ca_cert,
                certfile=self.device_cert,
                keyfile=self.private_key,
                tls_version=ssl.PROTOCOL_TLSv1_2
            )
            
            # Callbacks
            self.mqtt_client.on_connect = self.on_mqtt_connect
            self.mqtt_client.on_message = self.on_mqtt_message
            self.mqtt_client.on_disconnect = self.on_mqtt_disconnect
            
            # Connect
            rospy.loginfo("Connecting to AWS IoT...")
            self.mqtt_client.connect(self.endpoint, self.port, 60)
            self.mqtt_client.loop_start()
            
        except Exception as e:
            rospy.logerr("MQTT setup failed: %s", str(e))
    
    def on_mqtt_connect(self, client, userdata, flags, rc):
        if rc == 0:
            rospy.loginfo("Connected to AWS IoT")
            # Subscribe to commands
            client.subscribe("elderly_bot/commands", qos=1)
        else:
            rospy.logerr("Connection failed: %s", rc)
    
    def on_mqtt_message(self, client, userdata, msg):
        """Handle incoming commands"""
        try:
            data = json.loads(msg.payload)
            cmd = data.get('command', '')
            value = data.get('value', '')
            
            rospy.loginfo("Command: %s = %s", cmd, value)
            
            if cmd == 'buzzer':
                self.buzzer_pub.publish(UInt16(int(value)))
            elif cmd == 'stop':
                twist = Twist()
                self.cmd_vel_pub.publish(twist)
                
        except Exception as e:
            rospy.logerr("Command error: %s", str(e))
    
    def on_mqtt_disconnect(self, client, userdata, rc):
        rospy.logwarn("Disconnected from AWS IoT: %s", rc)
    
    # ROS callbacks
    def temp_cb(self, msg):
        self.temperature = msg.temperature
    
    def power_cb(self, msg):
        self.power = msg.data
    
    def gas_cb(self, msg):
        self.gas_detected = msg.data
        if msg.data:
            self.send_alert("Gas detected!")
    
    def send_telemetry(self, event=None):
        """Send telemetry to AWS IoT"""
        if not hasattr(self, 'mqtt_client'):
            return
        
        try:
            telemetry = {
                'time': time.time(),
                'temp': self.temperature,
                'power': self.power,
                'gas': self.gas_detected
            }
            
            self.mqtt_client.publish(
                "elderly_bot/telemetry",
                json.dumps(telemetry),
                qos=1
            )
            
        except Exception as e:
            rospy.logwarn("Telemetry send failed: %s", str(e))
    
    def send_alert(self, message):
        """Send alert to AWS IoT"""
        if not hasattr(self, 'mqtt_client'):
            return
        
        try:
            alert = {
                'time': time.time(),
                'alert': message
            }
            self.mqtt_client.publish("elderly_bot/alerts", json.dumps(alert), qos=1)
        except Exception as e:
            rospy.logerr("Alert send failed: %s", str(e))
    
    def run(self):
        rospy.spin()

if __name__ == '__main__':
    try:
        bridge = SimpleCloudBridge()
        bridge.run()
    except rospy.ROSInterruptException:
        pass