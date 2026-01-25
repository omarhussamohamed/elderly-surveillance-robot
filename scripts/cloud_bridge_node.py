#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""
Cloud Bridge Node - FINAL WORKING VERSION
AWS IoT Core connection with proper error handling
"""
import rospy
import json
import time
import os
import sys
import paho.mqtt.client as mqtt
import ssl
import threading
from std_msgs.msg import Float32, Bool, UInt16, String
from sensor_msgs.msg import Temperature
from geometry_msgs.msg import Twist

class CloudBridgeNode:
    def __init__(self):
        rospy.init_node('cloud_bridge_node', anonymous=False)
        rospy.loginfo("=" * 50)
        rospy.loginfo("AWS IoT CLOUD BRIDGE STARTING")
        rospy.loginfo("=" * 50)
        
        # Get parameters
        self.enable_cloud = rospy.get_param('~enable_cloud', False)
        
        if not self.enable_cloud:
            rospy.loginfo("Cloud bridge disabled by parameter")
            rospy.signal_shutdown("Cloud bridge disabled")
            return
        
        # AWS IoT Configuration
        self.aws_endpoint = rospy.get_param('~aws_endpoint', '')
        self.client_id = rospy.get_param('~client_id', 'robot')
        self.port = 8883
        
        # Certificate paths
        self.root_ca = rospy.get_param('~root_ca_path', '')
        self.cert = rospy.get_param('~cert_path', '')
        self.key = rospy.get_param('~key_path', '')
        
        # Verify certificates
        rospy.loginfo("Checking certificates...")
        for path, name in [(self.root_ca, "Root CA"), (self.cert, "Certificate"), (self.key, "Private key")]:
            if not path:
                rospy.logerr("❌ %s path not configured", name)
                rospy.signal_shutdown("Missing configuration")
                return
            if not os.path.exists(path):
                rospy.logerr("❌ %s not found: %s", name, path)
                rospy.signal_shutdown("File not found")
                return
            rospy.loginfo("✓ %s: %s", name, os.path.basename(path))
        
        # Topics
        self.telemetry_topic = rospy.get_param('~mqtt_topic_telemetry', 'robot/telemetry')
        self.commands_topic = rospy.get_param('~mqtt_topic_commands', 'robot/commands')
        self.alerts_topic = rospy.get_param('~mqtt_topic_alerts', 'robot/alerts')
        
        # Data storage
        self.temperature = 45.0  # Default value
        self.power = 12.3       # Default value
        self.gas_detected = False
        self.connected = False
        self.connection_attempts = 0
        self.max_attempts = 5
        
        # MQTT client
        self.mqtt_client = None
        
        rospy.loginfo("AWS IoT Configuration:")
        rospy.loginfo("  Endpoint: %s", self.aws_endpoint)
        rospy.loginfo("  Client ID: %s", self.client_id)
        rospy.loginfo("  Topics: %s, %s, %s", 
                     self.telemetry_topic, self.commands_topic, self.alerts_topic)
        
        # Setup MQTT connection
        self.setup_mqtt()
        
        # ROS subscribers (subscribe to sensor data)
        rospy.Subscriber('/jetson_temperature', Temperature, self.temp_callback)
        rospy.Subscriber('/jetson_power', Float32, self.power_callback)
        rospy.Subscriber('/gas_detected', Bool, self.gas_callback)
        
        # ROS publishers (for commands from cloud)
        self.buzzer_pub = rospy.Publisher('/buzzer_command', UInt16, queue_size=1)
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        
        rospy.loginfo("ROS topics initialized")
        
        # Start telemetry publishing (every 2 seconds)
        publish_rate = rospy.get_param('~publish_rate', 2.0)
        self.telemetry_timer = rospy.Timer(rospy.Duration(1.0/publish_rate), self.send_telemetry)
        
        # Connection monitor (try to reconnect if needed)
        self.connection_timer = rospy.Timer(rospy.Duration(10.0), self.monitor_connection)
        
        rospy.on_shutdown(self.shutdown)
        rospy.loginfo("✅ Cloud Bridge Node Initialization Complete")
    
    def setup_mqtt(self):
        """Setup MQTT connection to AWS IoT"""
        try:
            rospy.loginfo("Setting up MQTT connection...")
            
            # Create client with unique ID (append timestamp to avoid conflicts)
            unique_id = "{}_{}".format(self.client_id, int(time.time()))
            self.mqtt_client = mqtt.Client(client_id=unique_id, protocol=mqtt.MQTTv311)
            
            # Configure TLS for AWS IoT
            self.mqtt_client.tls_set(
                ca_certs=self.root_ca,
                certfile=self.cert,
                keyfile=self.key,
                tls_version=ssl.PROTOCOL_TLSv1_2,
                cert_reqs=ssl.CERT_REQUIRED
            )
            
            # Disable hostname verification (AWS IoT certificates don't have hostnames)
            self.mqtt_client.tls_insecure_set(True)
            
            # Set callbacks
            self.mqtt_client.on_connect = self.on_connect
            self.mqtt_client.on_disconnect = self.on_disconnect
            self.mqtt_client.on_message = self.on_message
            self.mqtt_client.on_log = self.on_log
            
            # Set connection parameters
            self.mqtt_client.connect_timeout = 10
            self.mqtt_client.keepalive = 30
            
            # Connect
            rospy.loginfo("Connecting to AWS IoT endpoint: %s:%s", self.aws_endpoint, self.port)
            self.connection_attempts += 1
            
            # Start connection in a separate thread to avoid blocking
            self.mqtt_client.connect_async(self.aws_endpoint, self.port, 60)
            self.mqtt_client.loop_start()
            
        except Exception as e:
            rospy.logerr("❌ MQTT setup error: %s", str(e))
            self.connected = False
    
    def on_connect(self, client, userdata, flags, rc):
        """MQTT connection callback"""
        self.connection_attempts = 0  # Reset attempts on successful connection
        
        if rc == 0:
            self.connected = True
            rospy.loginfo("✅ SUCCESS: Connected to AWS IoT Core!")
            rospy.loginfo("   Client ID: %s", client._client_id)
            
            # Subscribe to command topic
            try:
                result = client.subscribe(self.commands_topic, qos=1)
                if result[0] == mqtt.MQTT_ERR_SUCCESS:
                    rospy.loginfo("✅ Subscribed to topic: %s", self.commands_topic)
                else:
                    rospy.logwarn("⚠️ Failed to subscribe to %s: %s", self.commands_topic, result[0])
            except Exception as e:
                rospy.logerr("❌ Subscription error: %s", str(e))
            
            # Send initial connection message
            self.send_connection_event("connected")
            
        else:
            self.connected = False
            error_messages = {
                1: "Incorrect protocol version",
                2: "Invalid client identifier",
                3: "Server unavailable",
                4: "Bad username or password",
                5: "Not authorized - CHECK AWS IoT POLICY!",
            }
            error_msg = error_messages.get(rc, "Unknown error code: {}".format(rc))
            rospy.logerr("❌ Connection failed: %s", error_msg)
            
            if rc == 5:  # Not authorized - POLICY ISSUE
                rospy.logerr("=" * 60)
                rospy.logerr("POLICY ERROR: Your AWS IoT Policy doesn't allow connection!")
                rospy.logerr("Go to AWS IoT Console → Secure → Policies")
                rospy.logerr("Attach a policy with these permissions:")
                rospy.logerr('  {"Effect":"Allow","Action":"iot:Connect","Resource":"*"}')
                rospy.logerr('  {"Effect":"Allow","Action":"iot:Publish","Resource":"*"}')
                rospy.logerr('  {"Effect":"Allow","Action":"iot:Subscribe","Resource":"*"}')
                rospy.logerr("=" * 60)
    
    def on_disconnect(self, client, userdata, rc):
        """MQTT disconnection callback"""
        self.connected = False
        if rc != 0:
            rospy.logwarn("⚠️ Disconnected from AWS IoT. Reason: %s", rc)
            # Don't try to reconnect here - let monitor_connection handle it
    
    def on_message(self, client, userdata, msg):
        """Handle incoming MQTT messages (commands from cloud)"""
        try:
            rospy.loginfo("📨 Received message on %s", msg.topic)
            
            payload = json.loads(msg.payload)
            command = payload.get('command', '').lower()
            value = payload.get('value', '')
            
            rospy.loginfo("   Command: %s, Value: %s", command, value)
            
            if command == 'buzzer':
                # Control buzzer
                try:
                    freq = int(value) if value else 0
                    if freq > 0:
                        self.buzzer_pub.publish(UInt16(freq))
                        rospy.loginfo("🔔 Buzzer ON: %d Hz", freq)
                    else:
                        self.buzzer_pub.publish(UInt16(0))
                        rospy.loginfo("🔔 Buzzer OFF")
                except ValueError:
                    rospy.logwarn("Invalid buzzer frequency: %s", value)
            
            elif command == 'move':
                # Move robot
                twist = Twist()
                try:
                    if isinstance(value, dict):
                        twist.linear.x = float(value.get('linear_x', 0.0))
                        twist.angular.z = float(value.get('angular_z', 0.0))
                    else:
                        # Try to parse as string
                        parts = str(value).split(',')
                        if len(parts) >= 2:
                            twist.linear.x = float(parts[0])
                            twist.angular.z = float(parts[1])
                    
                    self.cmd_vel_pub.publish(twist)
                    rospy.loginfo("🤖 Move: linear=%.2f, angular=%.2f", 
                                 twist.linear.x, twist.angular.z)
                except (ValueError, TypeError) as e:
                    rospy.logwarn("Invalid move command: %s", str(e))
            
            elif command == 'stop':
                # Emergency stop
                twist = Twist()
                twist.linear.x = 0.0
                twist.angular.z = 0.0
                self.cmd_vel_pub.publish(twist)
                rospy.logwarn("🛑 Emergency STOP from cloud")
            
            elif command == 'status':
                # Request status - send immediate telemetry
                rospy.loginfo("📊 Status requested from cloud")
                self.send_telemetry(immediate=True)
            
            elif command == 'test':
                # Test command
                rospy.loginfo("🧪 Test command received: %s", value)
                self.send_alert("Test alert from robot")
            
            else:
                rospy.logwarn("❓ Unknown command: %s", command)
                
        except Exception as e:
            rospy.logerr("❌ Error processing message: %s", str(e))
    
    def on_log(self, client, userdata, level, buf):
        """MQTT log callback (for debugging)"""
        if level == mqtt.MQTT_LOG_DEBUG:
            rospy.logdebug("MQTT: %s", buf)
        elif level == mqtt.MQTT_LOG_INFO:
            rospy.loginfo("MQTT: %s", buf)
        elif level == mqtt.MQTT_LOG_WARNING:
            rospy.logwarn("MQTT: %s", buf)
        elif level == mqtt.MQTT_LOG_ERR:
            rospy.logerr("MQTT: %s", buf)
    
    # ROS Callbacks
    def temp_callback(self, msg):
        """Store temperature data"""
        self.temperature = msg.temperature
    
    def power_callback(self, msg):
        """Store power data"""
        self.power = msg.data
    
    def gas_callback(self, msg):
        """Handle gas detection and send alert"""
        old_value = self.gas_detected
        self.gas_detected = msg.data
        
        # Send alert if gas is newly detected
        if msg.data and not old_value:
            rospy.logwarn("⚠️ GAS DETECTED!")
            self.send_alert("GAS_DETECTED")
    
    def send_telemetry(self, event=None, immediate=False):
        """Send telemetry data to AWS IoT"""
        if not self.connected or not self.mqtt_client:
            if immediate:
                rospy.logdebug("Not connected, skipping telemetry")
            return
        
        try:
            # Prepare telemetry data
            telemetry = {
                'timestamp': time.time(),
                'robot_id': self.client_id,
                'temperature': round(self.temperature, 2),
                'power': round(self.power, 2),
                'gas_detected': self.gas_detected,
                'cpu_temp': self.temperature,
                'connection_status': 'connected' if self.connected else 'disconnected'
            }
            
            # Publish to AWS IoT
            result = self.mqtt_client.publish(
                self.telemetry_topic,
                json.dumps(telemetry),
                qos=1,
                retain=False
            )
            
            if result.rc == mqtt.MQTT_ERR_SUCCESS:
                if immediate:
                    rospy.loginfo("📡 Telemetry sent to cloud")
            else:
                rospy.logwarn("⚠️ Telemetry publish failed: %s", result.rc)
                
        except Exception as e:
            rospy.logwarn("⚠️ Telemetry error: %s", str(e))
    
    def send_alert(self, alert_type):
        """Send alert to AWS IoT"""
        if not self.connected or not self.mqtt_client:
            return
        
        try:
            alert = {
                'timestamp': time.time(),
                'robot_id': self.client_id,
                'alert_type': alert_type,
                'severity': 'high' if alert_type == 'GAS_DETECTED' else 'medium',
                'message': 'Robot alert: {}'.format(alert_type)
            }
            
            self.mqtt_client.publish(
                self.alerts_topic,
                json.dumps(alert),
                qos=1,
                retain=False
            )
            
            rospy.loginfo("🚨 Alert sent: %s", alert_type)
            
        except Exception as e:
            rospy.logerr("❌ Alert error: %s", str(e))
    
    def send_connection_event(self, event_type):
        """Send connection event to AWS IoT"""
        if not self.connected or not self.mqtt_client:
            return
        
        try:
            event = {
                'timestamp': time.time(),
                'robot_id': self.client_id,
                'event': event_type,
                'message': 'Robot {} to AWS IoT'.format(event_type)
            }
            
            self.mqtt_client.publish(
                'robot/events',
                json.dumps(event),
                qos=1,
                retain=False
            )
            
        except Exception as e:
            rospy.logdebug("Event error: %s", str(e))
    
    def monitor_connection(self, event=None):
        """Monitor and maintain connection"""
        if not self.connected:
            if self.connection_attempts < self.max_attempts:
                rospy.loginfo("🔧 Attempting to reconnect (%d/%d)...", 
                             self.connection_attempts + 1, self.max_attempts)
                self.connection_attempts += 1
                self.setup_mqtt()
            else:
                rospy.logerr("❌ Max connection attempts reached. Giving up.")
    
    def shutdown(self):
        """Clean shutdown"""
        rospy.loginfo("=" * 50)
        rospy.loginfo("SHUTTING DOWN CLOUD BRIDGE")
        rospy.loginfo("=" * 50)
        
        if self.mqtt_client:
            try:
                # Send disconnect event
                if self.connected:
                    self.send_connection_event("disconnected")
                
                # Clean up MQTT client
                self.mqtt_client.loop_stop()
                self.mqtt_client.disconnect()
                rospy.loginfo("✅ Disconnected from AWS IoT")
            except Exception as e:
                rospy.logwarn("⚠️ Error during disconnect: %s", str(e))
        
        rospy.loginfo("✅ Cloud Bridge shutdown complete")

if __name__ == '__main__':
    try:
        node = CloudBridgeNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("ROS interrupt received")
    except Exception as e:
        rospy.logerr("❌ Fatal error in cloud_bridge_node: %s", str(e))
        import traceback
        rospy.logerr(traceback.format_exc())