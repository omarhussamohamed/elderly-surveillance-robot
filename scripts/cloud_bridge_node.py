#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""
Cloud Bridge Node - COMPLETELY FIXED
Fixed: Better logging, Connection handling, Command parsing
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
            rospy.loginfo("Checking certificates...")
            for name, path in [("Root CA", self.root_ca), ("Certificate", self.cert), ("Private Key", self.key)]:
                if os.path.exists(path):
                    rospy.loginfo("✓ %s: %s", name, path)
                else:
                    rospy.logerr("✗ %s not found: %s", name, path)
                    self.mqtt_available = False
                    return
            
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
                rospy.loginfo("✓ paho-mqtt imported successfully")
            except ImportError as e:
                rospy.logerr("✗ paho-mqtt not installed: %s", str(e))
                rospy.logerr("Install with: sudo pip install paho-mqtt")
                self.mqtt_available = False
                return
            
            # Robot state
            self.temperature = 35.0
            self.power_voltage = 5.0
            self.gas_detected = False
            self.connection_status = "disconnected"
            self.last_telemetry_time = 0
            self.telemetry_count = 0
            
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
            
            # Send telemetry every 5 seconds
            rospy.Timer(rospy.Duration(5), self.send_telemetry)
            
            # Monitor connection every 30 seconds (increased from 10 for less noise)
            rospy.Timer(rospy.Duration(30), self.check_connection)
            
            rospy.on_shutdown(self.cleanup)
            
            rospy.loginfo("=" * 50)
            rospy.loginfo("AWS Bridge Initialized")
            rospy.loginfo("  Endpoint: %s", self.aws_endpoint)
            rospy.loginfo("  Client ID: %s", self.client_id)
            rospy.loginfo("  Telemetry: %s", self.mqtt_topic_telemetry)
            rospy.loginfo("  Commands: %s", self.mqtt_topic_commands)
            rospy.loginfo("=" * 50)
            
        except Exception as e:
            rospy.logerr("Failed to initialize cloud bridge: %s", str(e))
            rospy.logerr(traceback.format_exc())
            raise
    
    def connect_aws(self):
        """Connect to AWS IoT."""
        try:
            rospy.loginfo("Connecting to AWS IoT...")
            rospy.loginfo("  Endpoint: %s", self.aws_endpoint)
            rospy.loginfo("  Port: 8883")
            
            self.mqtt = self.mqtt_module.Client(
                client_id=self.client_id,
                protocol=self.mqtt_module.MQTTv311,
                clean_session=True
            )
            
            # Enable logging for debugging
            # self.mqtt.enable_logger()
            
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
            self.mqtt.on_publish = self.on_publish
            self.mqtt.on_subscribe = self.on_subscribe
            
            # Set Last Will and Testament (optional)
            self.mqtt.will_set("robot/status", json.dumps({"status": "offline"}), qos=1, retain=True)
            
            # Connect with timeout
            self.mqtt.connect(self.aws_endpoint, 8883, 60)
            self.mqtt.loop_start()
            
            rospy.loginfo("Connection attempt started...")
            
        except Exception as e:
            rospy.logerr("AWS connection error: %s", str(e))
            rospy.logerr(traceback.format_exc())
            self.connection_status = "error"
    
    def on_connect(self, client, userdata, flags, rc):
        """AWS connection established."""
        if rc == 0:
            self.connection_status = "connected"
            rospy.loginfo("✓ AWS IoT Connected Successfully!")
            rospy.loginfo("  Subscribing to: %s", self.mqtt_topic_commands)
            
            # Subscribe to commands
            result = client.subscribe(self.mqtt_topic_commands, qos=1)
            if result[0] == self.mqtt_module.MQTT_ERR_SUCCESS:
                rospy.loginfo("✓ Subscribed to commands topic")
            else:
                rospy.logerr("✗ Failed to subscribe: %s", result)
            
            # Publish online status
            client.publish("robot/status", json.dumps({
                "status": "online",
                "client_id": self.client_id,
                "timestamp": time.time()
            }), qos=1, retain=True)
            
        else:
            self.connection_status = "failed"
            error_messages = {
                1: "Connection refused - incorrect protocol version",
                2: "Connection refused - invalid client identifier",
                3: "Connection refused - server unavailable",
                4: "Connection refused - bad username or password",
                5: "Connection refused - not authorized"
            }
            error_msg = error_messages.get(rc, "Unknown error")
            rospy.logerr("✗ Connection failed: %s (code: %s)", error_msg, rc)
    
    def on_subscribe(self, client, userdata, mid, granted_qos):
        rospy.logdebug("Subscribed: mid=%s, qos=%s", mid, granted_qos)
    
    def on_publish(self, client, userdata, mid):
        rospy.logdebug("Message published: mid=%s", mid)
    
    def on_disconnect(self, client, userdata, rc):
        """Handle disconnection."""
        self.connection_status = "disconnected"
        if rc != 0:
            rospy.logwarn("Unexpected disconnection from AWS IoT (rc=%s)", rc)
            # Try to reconnect after 5 seconds
            rospy.loginfo("Attempting to reconnect in 5 seconds...")
            rospy.Timer(rospy.Duration(5), self.reconnect, oneshot=True)
    
    def reconnect(self, event=None):
        """Attempt to reconnect."""
        if hasattr(self, 'mqtt') and self.mqtt:
            try:
                rospy.loginfo("Reconnecting to AWS IoT...")
                self.mqtt.reconnect()
            except Exception as e:
                rospy.logerr("Reconnect failed: %s", str(e))
    
    def on_message(self, client, userdata, msg):
        """Handle commands from cloud."""
        try:
            rospy.loginfo("=" * 50)
            rospy.loginfo("📨 RECEIVED CLOUD COMMAND")
            rospy.loginfo("  Topic: %s", msg.topic)
            rospy.loginfo("  Payload: %s", msg.payload)
            rospy.loginfo("=" * 50)
            
            data = json.loads(msg.payload)
            cmd = data.get('command', '').lower().strip()
            value = data.get('value', '')
            
            rospy.loginfo("Parsed: command='%s', value='%s'", cmd, value)
            
            if cmd == 'buzzer':
                # Convert to boolean
                if isinstance(value, bool):
                    buzzer_state = value
                elif isinstance(value, (int, float)):
                    buzzer_state = bool(value)
                elif isinstance(value, str):
                    lower_val = value.lower().strip()
                    buzzer_state = lower_val in ['true', 'on', '1', 'yes', 'high', 'enable']
                else:
                    buzzer_state = False
                    rospy.logwarn("Unknown buzzer value type: %s", type(value))
                
                # Publish to ROS
                msg_bool = Bool()
                msg_bool.data = buzzer_state
                self.buzzer_pub.publish(msg_bool)
                
                rospy.loginfo("🔔 BUZZER COMMAND: %s", "ON" if buzzer_state else "OFF")
                
                # Send acknowledgment
                ack = {
                    "command": "buzzer",
                    "value": buzzer_state,
                    "ack": True,
                    "timestamp": time.time()
                }
                client.publish("robot/ack", json.dumps(ack), qos=1)
                
            elif cmd == 'sleep':
                minutes = float(data.get('minutes', 30))
                rospy.loginfo("💤 SLEEP COMMAND: %.1f minutes", minutes)
                # Stop everything
                self.buzzer_pub.publish(Bool(False))
                self.cmd_vel_pub.publish(Twist())
                
            elif cmd == 'restart':
                rospy.logwarn("🔄 RESTART COMMAND RECEIVED")
                
            elif cmd == 'test':
                rospy.loginfo("🧪 TEST COMMAND: %s", value)
                
            elif cmd == 'status':
                rospy.loginfo("📊 STATUS REQUEST")
                self.send_telemetry(immediate=True)
                
            else:
                rospy.logwarn("⚠️ UNKNOWN COMMAND: %s", cmd)
                rospy.loginfo("Valid commands: buzzer, sleep, restart, test, status")
            
        except ValueError as e:  # Use ValueError for Python 2 JSON decode errors
            rospy.logerr("❌ JSON decode error: %s", str(e))
            rospy.logerr("Raw payload: %s", msg.payload)
        except Exception as e:
            rospy.logerr("❌ Command processing error: %s", str(e))
            rospy.logerr(traceback.format_exc())
    
    def temp_cb(self, msg):
        self.temperature = msg.temperature
    
    def power_cb(self, msg):
        self.power_voltage = msg.data
    
    def gas_cb(self, msg):
        self.gas_detected = msg.data
        # Log only, no auto-buzzer
    
    def send_telemetry(self, event=None, immediate=False):
        """Send sensor data to AWS IoT."""
        try:
            if hasattr(self, 'mqtt') and self.mqtt and self.connection_status == "connected":
                current_time = time.time()
                
                # Throttle logging
                if not immediate and (current_time - self.last_telemetry_time < 30):
                    return
                
                data = {
                    'timestamp': current_time,
                    'temperature': round(self.temperature, 1),
                    'power': round(self.power_voltage, 2),
                    'gas_detected': self.gas_detected,
                    'client_id': self.client_id,
                    'connection': self.connection_status
                }
                
                payload = json.dumps(data)
                result = self.mqtt.publish(self.mqtt_topic_telemetry, payload, qos=0)
                
                self.telemetry_count += 1
                self.last_telemetry_time = current_time
                
                if self.telemetry_count % 10 == 0 or immediate:  # Log every 10th telemetry
                    rospy.loginfo("📡 TELEMETRY SENT #%d: %.1f°C, %.1fW, Gas: %s", 
                                self.telemetry_count, self.temperature, 
                                self.power_voltage, self.gas_detected)
                
        except Exception as e:
            rospy.logwarn("Telemetry error: %s", str(e))
    
    def check_connection(self, event=None):
        """Monitor connection status."""
        status_msg = {
            "connected": "✓ Connected to AWS IoT",
            "disconnected": "⚠️ Disconnected from AWS IoT",
            "failed": "❌ Connection failed",
            "error": "❌ Connection error"
        }
        
        if self.connection_status in status_msg:
            if self.connection_status == "connected":
                rospy.logdebug(status_msg[self.connection_status])  # Changed to debug for less noise
            else:
                rospy.logwarn(status_msg[self.connection_status])
        
        # Try to reconnect if disconnected
        if self.connection_status in ["disconnected", "failed", "error"]:
            self.reconnect()
    
    def cleanup(self):
        """Clean shutdown."""
        rospy.loginfo("=" * 50)
        rospy.loginfo("Shutting down AWS Bridge...")
        rospy.loginfo("Total telemetry sent: %d", self.telemetry_count)
        
        if hasattr(self, 'mqtt'):
            try:
                # Publish offline status
                if self.connection_status == "connected":
                    self.mqtt.publish("robot/status", 
                                     json.dumps({"status": "offline", "timestamp": time.time()}), 
                                     qos=1, retain=True)
                
                self.mqtt.loop_stop()
                self.mqtt.disconnect()
                rospy.loginfo("MQTT connection closed")
            except Exception as e:
                rospy.logwarn("Error during shutdown: %s", str(e))
        
        rospy.loginfo("AWS Bridge shutdown complete")
        rospy.loginfo("=" * 50)

if __name__ == '__main__':
    try:
        node = CloudBridgeNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("Bridge stopped by ROS interrupt")
    except KeyboardInterrupt:
        rospy.loginfo("Bridge stopped by user")
    except Exception as e:
        rospy.logerr("Fatal error: %s", str(e))
        rospy.logerr(traceback.format_exc())
        sys.exit(1)