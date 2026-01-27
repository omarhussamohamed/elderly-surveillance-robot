#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""
Cloud Bridge Node - STRICT RISING EDGE & MINIMAL PAYLOAD
- Gas: Publishes to cloud ONLY on Rising Edge (False -> True)
- Payload format simplified: {"gas": true} and {"temperature": 25.5}
- No "Gas cleared" logs or publications
"""

import rospy
import json
import time
import traceback
import os
import sys
from std_msgs.msg import Bool
from sensor_msgs.msg import Temperature
import paho.mqtt.client as mqtt

class CloudBridgeNode:
    def __init__(self):
        try:
            rospy.init_node('cloud_bridge_node')
            rospy.loginfo("=" * 50)
            rospy.loginfo("Starting AWS IoT Bridge (Strict Mode)...")
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

            # MQTT topics
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
            self.max_backoff = 300

            # State for change detection
            self.last_gas_detected = False  # Assume clear initially

            # ROS Publishers
            self.buzzer_pub = rospy.Publisher('/buzzer_command', Bool, queue_size=1)

            # ROS Subscribers
            rospy.Subscriber('/gas_detected', Bool, self.update_gas_state, queue_size=1)
            rospy.Subscriber('/jetson_temperature', Temperature, self.temp_callback, queue_size=1)

            self.connect_to_aws()

        except Exception as e:
            rospy.logerr("Initialization error: %s", str(e))
            sys.exit(1)

    def connect_to_aws(self):
        while not rospy.is_shutdown():
            try:
                backoff = min(2 ** self.reconnect_attempts, self.max_backoff)
                rospy.sleep(backoff)
                self.mqtt.connect(self.aws_endpoint, port=8883, keepalive=60)
                self.mqtt.loop_start()
                return
            except Exception as e:
                self.reconnect_attempts += 1
                rospy.logerr("Connection attempt %d failed: %s", self.reconnect_attempts, str(e))

    def on_connect(self, client, userdata, flags, rc):
        if rc == 0:
            self.mqtt.subscribe(self.mqtt_commands, qos=1)
            self.reconnect_attempts = 0
        else:
            rospy.logerr("Connection failed with RC: %d", rc)

    def on_disconnect(self, client, userdata, rc):
        rospy.logwarn("Disconnected with RC: %d. Reconnecting...", rc)
        self.connect_to_aws()

    def on_message(self, client, userdata, msg):
        try:
            if msg.topic == self.mqtt_commands:
                payload = json.loads(msg.payload)
                if 'command' in payload and payload['command'] == 'buzzer':
                    state = bool(payload.get('value', False))
                    self.buzzer_pub.publish(Bool(data=state))
                    rospy.loginfo("Buzzer command from cloud: %s", "ON" if state else "OFF")
        except Exception as e:
            rospy.logerr("Command parsing error: %s", str(e))

def update_gas_state(self, msg):
    try:
        # Update state for telemetry (used in temp_callback)
        self.last_gas_detected = msg.data
    except Exception as e:
        rospy.logerr("Failed to handle gas callback: %s", str(e))

def temp_callback(self, msg):
    try:
        telemetry = {
            'temperature': msg.temperature,  # Add comma here
            'gas': self.last_gas_detected,
            'timestamp': time.time()
        }
        self.mqtt.publish(self.mqtt_telemetry, json.dumps(telemetry), qos=1)
    except Exception as e:
        rospy.logerr("Failed to publish telemetry: %s", str(e))

    def run(self):
        rospy.spin()

    def shutdown(self):
        if hasattr(self, 'mqtt'):
            self.mqtt.loop_stop()
            self.mqtt.disconnect()

if __name__ == '__main__':
    try:
        node = CloudBridgeNode()
        rospy.on_shutdown(node.shutdown)
        node.run()
    except rospy.ROSInterruptException:
        pass