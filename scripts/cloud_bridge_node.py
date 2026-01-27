#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""
Cloud Bridge Node - COMPLETE WORKING VERSION
FIXED: Now actually bridges cloud ↔ ROS both ways
"""

import rospy
import json
import time
import traceback
import os
import sys
from std_msgs.msg import Float32, Bool
from sensor_msgs.msg import Temperature
from geometry_msgs.msg import Twist
import paho.mqtt.client as mqtt

class CloudBridgeNode:
    def __init__(self):
        try:
            rospy.init_node('cloud_bridge_node')
            rospy.loginfo("=" * 50)
            rospy.loginfo("Starting AWS IoT Bridge...")
            rospy.loginfo("=" * 50)

            # Load parameters
            self.aws_endpoint = rospy.get_param('~aws_endpoint', 'a1k8itxfx77i0w-ats.iot.us-east-1.amazonaws.com')
            self.client_id = rospy.get_param('~client_id', 'robot')

            # Certificate paths
            self.root_ca = rospy.get_param('~root_ca_path', '/home/omar/catkin_ws/src/elderly_bot/aws_certs/AmazonRootCA1.pem')
            self.cert = rospy.get_param('~cert_path', '/home/omar/catkin_ws/src/elderly_bot/aws_certs/certificate.pem.crt')
            self.key = rospy.get_param('~key_path', '/home/omar/catkin_ws/src/elderly_bot/aws_certs/private.pem.key')

            # Verify certificates exist
            for path in [self.root_ca, self.cert, self.key]:
                if not os.path.exists(path):
                    rospy.logerr("Certificate file missing: %s", path)
                    sys.exit(1)

            # MQTT topics from config
            self.mqtt_telemetry = rospy.get_param('~mqtt_topic_telemetry', 'robot/telemetry')
            self.mqtt_alerts = rospy.get_param('~mqtt_topic_alerts', 'robot/alerts')
            self.mqtt_commands = rospy.get_param('~mqtt_topic_commands', 'robot/commands')

            # MQTT setup
            self.mqtt = mqtt.Client(client_id=self.client_id, clean_session=True)
            self.mqtt.tls_set(ca_certs=self.root_ca, certfile=self.cert, keyfile=self.key)
            self.mqtt.on_connect = self.on_connect
            self.mqtt.on_disconnect = self.on_disconnect
            self.mqtt.on_message = self.on_message

            self.reconnect_attempts = 0
            self.max_backoff = 300  # 5 minutes max

            # ROS Publishers (for commands FROM cloud TO robot)
            self.buzzer_pub = rospy.Publisher('/buzzer_command', Bool, queue_size=10)
            self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
            self.status_pub = rospy.Publisher('/cloud/status', Bool, queue_size=10, latch=True)

            # ROS Subscribers (for telemetry FROM robot TO cloud)
            rospy.Subscriber('/gas_detected', Bool, self.gas_callback)
            rospy.Subscriber('/jetson_temperature', Temperature, self.temp_callback)
            
            # Connect to AWS
            self.connect_with_backoff()

        except Exception as e:
            rospy.logerr("Initialization failed: %s", str(e))
            rospy.logerr(traceback.format_exc())
            sys.exit(1)

    def connect_with_backoff(self):
        while not rospy.is_shutdown():
            try:
                rospy.loginfo("Connecting to AWS IoT...")
                self.mqtt.connect(self.aws_endpoint, port=8883, keepalive=60)
                self.mqtt.loop_start()
                return
            except Exception as e:
                self.reconnect_attempts += 1
                backoff = min(2 ** self.reconnect_attempts, self.max_backoff)
                rospy.logwarn("Connection failed: %s → retry in %d seconds (attempt %d)", 
                              str(e), backoff, self.reconnect_attempts)
                rospy.sleep(backoff)

    def on_connect(self, client, userdata, flags, rc):
        if rc == 0:
            rospy.loginfo("Connected to AWS IoT (rc=%d)", rc)
            self.reconnect_attempts = 0
            # Subscribe to commands FROM cloud
            client.subscribe(self.mqtt_commands)
            rospy.loginfo("Subscribed to MQTT topic: %s", self.mqtt_commands)
            self.status_pub.publish(Bool(True))
        else:
            rospy.logerr("Connection refused (rc=%d)", rc)

    def on_disconnect(self, client, userdata, rc):
        rospy.logwarn("Disconnected from AWS IoT (rc=%d)", rc)
        self.status_pub.publish(Bool(False))
        if not rospy.is_shutdown():
            self.connect_with_backoff()

    def on_message(self, client, userdata, msg):
        """Handle commands FROM cloud TO robot"""
        try:
            payload = json.loads(msg.payload)
            rospy.loginfo("Received cloud command: %s", payload)
            
            # Handle different command types
            if 'command' in payload:
                cmd = payload['command']
                
                if cmd == 'buzzer':
                    # Example: {"command": "buzzer", "state": true}
                    state = payload.get('state', False)
                    self.buzzer_pub.publish(Bool(data=state))
                    rospy.loginfo("Buzzer command: %s", "ON" if state else "OFF")
                    
                elif cmd == 'move':
                    # Example: {"command": "move", "linear_x": 0.1, "angular_z": 0.0}
                    twist = Twist()
                    twist.linear.x = payload.get('linear_x', 0.0)
                    twist.angular.z = payload.get('angular_z', 0.0)
                    self.cmd_vel_pub.publish(twist)
                    rospy.loginfo("Move command: vx=%.2f, vz=%.2f", twist.linear.x, twist.angular.z)
                    
                elif cmd == 'patrol':
                    # Example: {"command": "patrol", "action": "start"}
                    action = payload.get('action', '')
                    # Could trigger patrol_client.py here via service call
                    rospy.loginfo("Patrol command: %s", action)
                    
                else:
                    rospy.logwarn("Unknown command: %s", cmd)
            else:
                rospy.logwarn("Invalid command format, missing 'command' field")
                
        except Exception as e:
            rospy.logerr("Command parsing error: %s", str(e))
            rospy.logerr("Raw message: %s", msg.payload)

    def gas_callback(self, msg):
        """Send gas detection TO cloud"""
        try:
            telemetry = {
                'timestamp': time.time(),
                'type': 'gas_detection',
                'detected': msg.data,
                'robot_id': self.client_id
            }
            self.mqtt.publish(self.mqtt_alerts, json.dumps(telemetry), qos=1)
            rospy.logdebug("Published gas detection: %s", msg.data)
        except Exception as e:
            rospy.logerr("Failed to publish gas detection: %s", str(e))

    def temp_callback(self, msg):
        """Send temperature TO cloud"""
        try:
            telemetry = {
                'timestamp': time.time(),
                'type': 'temperature',
                'temp_c': msg.temperature,
                'robot_id': self.client_id
            }
            self.mqtt.publish(self.mqtt_telemetry, json.dumps(telemetry), qos=1)
            rospy.logdebug("Published temperature: %.1f°C", msg.temperature)
        except Exception as e:
            rospy.logerr("Failed to publish temperature: %s", str(e))

    def run(self):
        rospy.spin()

    def shutdown(self):
        rospy.loginfo("Shutting down AWS Bridge...")
        if hasattr(self, 'mqtt'):
            self.mqtt.loop_stop()
            self.mqtt.disconnect()
        rospy.loginfo("AWS Bridge shutdown complete")

if __name__ == '__main__':
    try:
        node = CloudBridgeNode()
        rospy.on_shutdown(node.shutdown)
        node.run()
    except rospy.ROSInterruptException:
        pass
    except Exception as e:
        rospy.logerr("Fatal error: %s", str(e))
        rospy.logerr(traceback.format_exc())
        sys.exit(1)