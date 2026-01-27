#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""
Cloud Bridge Node - RESTORED ORIGINAL BEHAVIOR
- Gas alert published ONLY on change (rising edge to True → alert, falling to False → cleared)
- No "Published gas alert" info logs (quiet like before)
- Temperature published every callback (10Hz) quietly (no per-publish logs)
- Original payload format with robot_id and type
- Buzzer parsing uses 'value' for cloud commands
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
            self.max_backoff = 300

            # State for change detection
            self.last_gas_detected = None

            # ROS Publishers (cloud → robot)
            self.buzzer_pub = rospy.Publisher('/buzzer_command', Bool, queue_size=1)

            # ROS Subscribers (robot → cloud)
            rospy.Subscriber('/gas_detected', Bool, self.gas_callback, queue_size=1)
            rospy.Subscriber('/jetson_temperature', Temperature, self.temp_callback, queue_size=1)

            # Connect to AWS
            self.connect_to_aws()

        except Exception as e:
            rospy.logerr("Initialization error: %s", str(e))
            rospy.logerr(traceback.format_exc())
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
                rospy.loginfo("Received cloud command: %s", payload)
                if 'command' in payload:
                    cmd = payload['command']
                    if cmd == 'buzzer':
                        state = bool(payload.get('value', False))
                        self.buzzer_pub.publish(Bool(data=state))
                        rospy.loginfo("Buzzer command from cloud: %s", "ON" if state else "OFF")
                    else:
                        rospy.logwarn("Unknown command: %s", cmd)
        except Exception as e:
            rospy.logerr("Command parsing error: %s", str(e))

    def gas_callback(self, msg):
        """Send gas detection TO cloud ONLY on change"""
        try:
            if msg.data != self.last_gas_detected:
                self.last_gas_detected = msg.data
                telemetry = {
                    'timestamp': time.time(),
                    'type': 'gas_detection',
                    'detected': msg.data,
                    'robot_id': self.client_id
                }
                self.mqtt.publish(self.mqtt_alerts, json.dumps(telemetry), qos=1)
                if msg.data:
                    rospy.logwarn("GAS DETECTED - Alert sent to cloud")
                else:
                    rospy.loginfo("Gas cleared - Notification sent to cloud")
        except Exception as e:
            rospy.logerr("Failed to handle gas callback: %s", str(e))

    def temp_callback(self, msg):
        """Send temperature TO cloud every time (quiet, like original)"""
        try:
            telemetry = {
                'timestamp': time.time(),
                'type': 'temperature',
                'temp_c': msg.temperature,
                'robot_id': self.client_id
            }
            self.mqtt.publish(self.mqtt_telemetry, json.dumps(telemetry), qos=1)
            # No log - quiet like original behavior
        except Exception as e:
            rospy.logerr("Failed to publish temperature: %s", str(e))

    def run(self):
        rospy.spin()

    def shutdown(self):
        rospy.loginfo("Shutting down AWS Bridge...")
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
    except Exception as e:
        rospy.logerr("Fatal error: %s", str(e))
        rospy.logerr(traceback.format_exc())
        sys.exit(1)