#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""
Cloud Bridge Node (AWS IoT)

- Mirrors local system stats to the cloud
- SAME data, SAME rate as local ROS logs (1 Hz)
- Receives cloud commands (buzzer)
- Cloud failure NEVER stops the robot
"""

import rospy
import json
import os
from std_msgs.msg import Bool
from sensor_msgs.msg import Temperature
import paho.mqtt.client as mqtt


class CloudBridgeNode(object):
    def __init__(self):
        rospy.init_node('cloud_bridge_node', anonymous=False)
        rospy.loginfo("Starting Cloud Bridge Node")

        # ── Parameters ─────────────────────────────────────────────
        self.aws_endpoint = rospy.get_param('~aws_endpoint', '')
        self.client_id = rospy.get_param('~client_id', 'elderly_bot')

        self.root_ca = rospy.get_param('~root_ca_path', '')
        self.cert = rospy.get_param('~cert_path', '')
        self.key = rospy.get_param('~key_path', '')

        self.topic_telemetry = rospy.get_param(
            '~mqtt_topic_telemetry', 'robot/telemetry'
        )
        self.topic_commands = rospy.get_param(
            '~mqtt_topic_commands', 'robot/commands'
        )

        # ── State ─────────────────────────────────────────────────
        self.last_gas_state = False
        self.connected = False

        # ── ROS I/O ───────────────────────────────────────────────
        self.buzzer_pub = rospy.Publisher(
            '/buzzer_command', Bool, queue_size=1
        )

        rospy.Subscriber('/gas_detected', Bool, self._gas_cb, queue_size=1)
        rospy.Subscriber('/jetson_temperature', Temperature, self._temp_cb, queue_size=1)

        # ── MQTT Setup ────────────────────────────────────────────
        self.mqtt = mqtt.Client(client_id=self.client_id, clean_session=True)
        self.mqtt.on_connect = self._on_connect
        self.mqtt.on_disconnect = self._on_disconnect
        self.mqtt.on_message = self._on_message

        if not self._validate_certificates():
            rospy.logerr("Cloud certificates missing — cloud bridge DISABLED")
            rospy.logwarn("Robot will continue running without cloud")
            return

        try:
            self.mqtt.tls_set(
                ca_certs=self.root_ca,
                certfile=self.cert,
                keyfile=self.key
            )
        except Exception as e:
            rospy.logerr("TLS setup failed: %s", str(e))
            return

        self._connect()
        rospy.on_shutdown(self.shutdown)

    # ─────────────────────────────────────────────────────────────
    def _validate_certificates(self):
        required = [self.aws_endpoint, self.root_ca, self.cert, self.key]
        for item in required:
            if not item:
                return False
        for path in [self.root_ca, self.cert, self.key]:
            if not os.path.exists(path):
                rospy.logerr("Missing certificate file: %s", path)
                return False
        return True

    # ─────────────────────────────────────────────────────────────
    def _connect(self):
        try:
            rospy.loginfo("Connecting to AWS IoT...")
            self.mqtt.connect(self.aws_endpoint, port=8883, keepalive=60)
            self.mqtt.loop_start()
        except Exception as e:
            rospy.logwarn("AWS connection failed: %s", str(e))

    # ─────────────────────────────────────────────────────────────
    def _on_connect(self, client, userdata, flags, rc):
        if rc == 0:
            self.connected = True
            rospy.loginfo("Connected to AWS IoT")
            client.subscribe(self.topic_commands, qos=1)
        else:
            rospy.logwarn("AWS connection failed (rc=%d)", rc)

    def _on_disconnect(self, client, userdata, rc):
        self.connected = False
        rospy.logwarn("Disconnected from AWS (rc=%d)", rc)

    # ─────────────────────────────────────────────────────────────
    def _on_message(self, client, userdata, msg):
        try:
            payload = json.loads(msg.payload)
            if payload.get('command') == 'buzzer':
                state = bool(payload.get('value', False))
                self.buzzer_pub.publish(Bool(state))
                rospy.loginfo("Cloud buzzer command: %s", state)
        except Exception as e:
            rospy.logwarn("Invalid cloud command: %s", str(e))

    # ─────────────────────────────────────────────────────────────
    def _gas_cb(self, msg):
        # Gas state is stored and forwarded with temperature
        self.last_gas_state = msg.data

    # ─────────────────────────────────────────────────────────────
    def _temp_cb(self, msg):
        """
        Temperature arrives at 1 Hz from sensors node.
        This callback is the ONLY place telemetry is published.
        """
        if not self.connected:
            return

        telemetry = {
            'temperature': msg.temperature,
            'gas': self.last_gas_state
        }

        try:
            self.mqtt.publish(
                self.topic_telemetry,
                json.dumps(telemetry),
                qos=1
            )
        except Exception as e:
            rospy.logwarn("Telemetry publish failed: %s", str(e))

    # ─────────────────────────────────────────────────────────────
    def shutdown(self):
        rospy.loginfo("Shutting down Cloud Bridge")
        try:
            self.mqtt.loop_stop()
            self.mqtt.disconnect()
        except Exception:
            pass


if __name__ == '__main__':
    try:
        CloudBridgeNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
