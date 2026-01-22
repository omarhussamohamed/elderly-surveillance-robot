#!/usr/bin/env python3
"""
AWS IoT Core Bridge Node for Elderly Bot
=========================================
Bridges local ROS topics with AWS IoT Core MQTT for cloud communication.

SAFETY FEATURES:
- Runs without crashing even if AWS unavailable, certificates missing, or internet down
- All configuration via parameters (no hardcoded paths)
- Optional enable/disable via parameter
- Extensive error handling with clear log messages
- Auto-reconnection on connection loss

CLOUD INTEGRATION:
- Publishes robot sensor data to AWS IoT Core (gas, temperature, power)
- Receives commands from cloud (buzzer control, emergency stop)
- JSON message format for easy cloud team integration

DEPENDENCIES (install when cloud ready):
  pip3 install AWSIoTPythonSDK
  
CERTIFICATES REQUIRED (from AWS IoT Core console):
  1. Root CA certificate (Amazon Root CA 1)
  2. Device certificate (.pem.crt)
  3. Private key (.pem.key)
  
TO ENABLE: Set parameters in config/cloud_config.yaml and launch with enable_cloud:=true
"""

import rospy
import json
import threading
import time
from std_msgs.msg import Float32, Bool
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Temperature

# === AWS IoT SDK IMPORT (with graceful failure) ===
AWS_AVAILABLE = False
try:
    from AWSIoTPythonSDK.MQTTLib import AWSIoTMQTTClient
    AWS_AVAILABLE = True
except ImportError:
    AWSIoTMQTTClient = None
    # Will log warning in __init__ when enable_cloud=True


class CloudBridgeNode:
    """
    Main bridge node connecting ROS topics to AWS IoT Core via MQTT.
    Handles bidirectional communication with graceful degradation.
    """
    
    def __init__(self):
        rospy.init_node('cloud_bridge_node', anonymous=False)
        rospy.loginfo("=== Cloud Bridge Node Starting ===")
        
        # === PARAMETERS ===
        self.enable_cloud = rospy.get_param('~enable_cloud', False)
        
        if not self.enable_cloud:
            rospy.loginfo("Cloud bridge DISABLED by parameter. Set enable_cloud:=true to activate.")
            rospy.signal_shutdown("Cloud bridge disabled")
            return
        
        # AWS IoT Core connection parameters
        # *** PASTE YOUR AWS ENDPOINT HERE (from AWS IoT Core console) ***
        # Example: "a1b2c3d4e5f6g7-ats.iot.us-east-1.amazonaws.com"
        self.aws_endpoint = rospy.get_param('~aws_endpoint', '')
        
        self.client_id = rospy.get_param('~client_id', 'robot_01')
        self.port = rospy.get_param('~port', 8883)  # Default MQTT over TLS port
        
        # Certificate paths (must be absolute paths)
        self.root_ca_path = rospy.get_param('~root_ca_path', '')
        self.cert_path = rospy.get_param('~cert_path', '')
        self.key_path = rospy.get_param('~key_path', '')
        
        # MQTT topics (configurable for different cloud setups)
        self.mqtt_topic_sensors = rospy.get_param('~mqtt_topic_sensors', 'robot/sensors/data')
        self.mqtt_topic_commands = rospy.get_param('~mqtt_topic_commands', 'robot/commands')
        
        # Connection settings
        self.publish_rate = rospy.get_param('~publish_rate', 1.0)  # Hz
        self.keepalive_interval = rospy.get_param('~keepalive_interval', 30)  # seconds
        
        # === STATE ===
        self.cloud_connected = False
        self.mqtt_client = None
        self.connection_lock = threading.Lock()
        
        # Latest sensor data (cached for periodic publishing)
        self.latest_gas_level = 0.0
        self.latest_temperature = 0.0
        self.latest_power = 0.0
        self.data_lock = threading.Lock()
        
        # === VALIDATION ===
        if not self._validate_config():
            rospy.logwarn("Cloud bridge DISABLED: Configuration incomplete")
            rospy.signal_shutdown("Invalid configuration")
            return
        
        # === INITIALIZE AWS CONNECTION ===
        if not self._init_aws_client():
            rospy.logwarn("Cloud bridge DISABLED: AWS client initialization failed")
            rospy.signal_shutdown("AWS initialization failed")
            return
        
        # === ROS SUBSCRIBERS (Local → Cloud) ===
        rospy.Subscriber('/gas_level', Float32, self.gas_level_callback, queue_size=10)
        rospy.Subscriber('/jetson_temperature', Temperature, self.temperature_callback, queue_size=10)
        rospy.Subscriber('/jetson_power', Float32, self.power_callback, queue_size=10)
        
        # === ROS PUBLISHERS (Cloud → Local) ===
        self.pub_buzzer = rospy.Publisher('/buzzer_command', Bool, queue_size=1, latch=True)
        self.pub_cmd_vel = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        
        rospy.loginfo("=== Cloud Bridge Node Initialized ===")
        rospy.loginfo(f"AWS Endpoint: {self.aws_endpoint}")
        rospy.loginfo(f"Client ID: {self.client_id}")
        rospy.loginfo(f"Cloud Connection: {'ACTIVE' if self.cloud_connected else 'DISABLED'}")
        
        # === SHUTDOWN HOOK ===
        rospy.on_shutdown(self.shutdown_hook)
    
    def _validate_config(self):
        """Validate all required configuration parameters."""
        if not AWS_AVAILABLE:
            rospy.logerr("AWS IoT SDK not available. Install: pip3 install AWSIoTPythonSDK")
            return False
        
        if not self.aws_endpoint:
            rospy.logerr("AWS endpoint not configured. Set aws_endpoint parameter.")
            rospy.logerr("Get endpoint from: AWS IoT Core Console → Settings → Device data endpoint")
            return False
        
        if not self.root_ca_path or not self.cert_path or not self.key_path:
            rospy.logerr("Certificate paths not configured. Set root_ca_path, cert_path, key_path parameters.")
            rospy.logerr("Download certificates from: AWS IoT Core Console → Security → Certificates")
            return False
        
        # Check if certificate files exist
        import os
        if not os.path.isfile(self.root_ca_path):
            rospy.logerr(f"Root CA not found: {self.root_ca_path}")
            return False
        if not os.path.isfile(self.cert_path):
            rospy.logerr(f"Device certificate not found: {self.cert_path}")
            return False
        if not os.path.isfile(self.key_path):
            rospy.logerr(f"Private key not found: {self.key_path}")
            return False
        
        rospy.loginfo("✓ Configuration validated")
        return True
    
    def _init_aws_client(self):
        """Initialize AWS IoT MQTT client with certificates."""
        try:
            # Create MQTT client
            self.mqtt_client = AWSIoTMQTTClient(self.client_id)
            
            # Configure endpoint and port
            self.mqtt_client.configureEndpoint(self.aws_endpoint, self.port)
            
            # Configure certificates
            self.mqtt_client.configureCredentials(
                self.root_ca_path,
                self.key_path,
                self.cert_path
            )
            
            # Configure MQTT client settings
            self.mqtt_client.configureAutoReconnectBackoffTime(1, 32, 20)
            self.mqtt_client.configureOfflinePublishQueueing(-1)  # Infinite queue
            self.mqtt_client.configureDrainingFrequency(2)  # 2 Hz
            self.mqtt_client.configureConnectDisconnectTimeout(10)  # 10 seconds
            self.mqtt_client.configureMQTTOperationTimeout(5)  # 5 seconds
            
            # Connect to AWS IoT Core
            rospy.loginfo("Connecting to AWS IoT Core...")
            self.mqtt_client.connect(self.keepalive_interval)
            rospy.loginfo("✓ Connected to AWS IoT Core")
            
            # Subscribe to command topic
            self.mqtt_client.subscribe(self.mqtt_topic_commands, 1, self.mqtt_command_callback)
            rospy.loginfo(f"✓ Subscribed to MQTT topic: {self.mqtt_topic_commands}")
            
            self.cloud_connected = True
            return True
            
        except Exception as e:
            rospy.logerr(f"Failed to initialize AWS client: {e}")
            rospy.logerr("Check: Internet connection, endpoint URL, certificate paths")
            return False
    
    # === ROS CALLBACKS (Local → Cloud) ===
    
    def gas_level_callback(self, msg):
        """Cache gas level data for periodic cloud publishing."""
        with self.data_lock:
            self.latest_gas_level = msg.data
    
    def temperature_callback(self, msg):
        """Cache temperature data for periodic cloud publishing."""
        with self.data_lock:
            self.latest_temperature = msg.temperature
    
    def power_callback(self, msg):
        """Cache power data for periodic cloud publishing."""
        with self.data_lock:
            self.latest_power = msg.data
    
    # === MQTT CALLBACK (Cloud → Local) ===
    
    def mqtt_command_callback(self, client, userdata, message):
        """
        Handle incoming MQTT messages from AWS IoT Core.
        Expected JSON format:
        {
            "command": "buzzer" | "stop" | "move",
            "value": true/false for buzzer, or {linear: x, angular: z} for move
        }
        """
        try:
            payload = json.loads(message.payload.decode('utf-8'))
            command = payload.get('command', '')
            value = payload.get('value')
            
            rospy.loginfo(f"Received cloud command: {command} = {value}")
            
            if command == 'buzzer':
                # Buzzer control command
                buzzer_state = bool(value)
                self.pub_buzzer.publish(Bool(buzzer_state))
                rospy.loginfo(f"Buzzer command sent: {buzzer_state}")
                
            elif command == 'stop':
                # Emergency stop command
                stop_twist = Twist()
                stop_twist.linear.x = 0.0
                stop_twist.angular.z = 0.0
                self.pub_cmd_vel.publish(stop_twist)
                rospy.logwarn("EMERGENCY STOP commanded from cloud")
                
            elif command == 'move':
                # Movement command (use with caution!)
                if isinstance(value, dict):
                    move_twist = Twist()
                    move_twist.linear.x = float(value.get('linear', 0.0))
                    move_twist.angular.z = float(value.get('angular', 0.0))
                    self.pub_cmd_vel.publish(move_twist)
                    rospy.loginfo(f"Movement command: linear={move_twist.linear.x}, angular={move_twist.angular.z}")
                else:
                    rospy.logwarn(f"Invalid move command value: {value}")
                    
            else:
                rospy.logwarn(f"Unknown command: {command}")
                
        except json.JSONDecodeError as e:
            rospy.logerr(f"Failed to parse MQTT message: {e}")
        except Exception as e:
            rospy.logerr(f"Error handling MQTT command: {e}")
    
    # === CLOUD PUBLISHING ===
    
    def publish_sensor_data(self):
        """
        Publish latest sensor data to AWS IoT Core.
        Format: JSON with timestamp and all sensor readings.
        """
        if not self.cloud_connected:
            return
        
        try:
            with self.data_lock:
                # Create JSON payload
                payload = {
                    'timestamp': rospy.Time.now().to_sec(),
                    'robot_id': self.client_id,
                    'sensors': {
                        'gas_level': self.latest_gas_level,
                        'temperature': self.latest_temperature,
                        'power': self.latest_power
                    }
                }
            
            # Publish to AWS IoT Core
            json_payload = json.dumps(payload)
            self.mqtt_client.publish(self.mqtt_topic_sensors, json_payload, 1)
            
            rospy.logdebug(f"Published to cloud: {json_payload}")
            
        except Exception as e:
            rospy.logwarn_throttle(10, f"Failed to publish to cloud: {e}")
    
    def run(self):
        """Main loop: periodically publish sensor data to cloud."""
        if not self.cloud_connected:
            rospy.logwarn("Cloud not connected, node exiting")
            return
        
        rate = rospy.Rate(self.publish_rate)
        
        rospy.loginfo(f"Main loop starting at {self.publish_rate} Hz")
        
        while not rospy.is_shutdown():
            try:
                self.publish_sensor_data()
                rate.sleep()
                
            except rospy.ROSInterruptException:
                break
            except Exception as e:
                rospy.logerr(f"Error in main loop: {e}")
                rospy.sleep(1.0)  # Prevent rapid error spam
    
    def shutdown_hook(self):
        """Clean shutdown: disconnect from AWS IoT Core."""
        rospy.loginfo("=== Shutting down Cloud Bridge Node ===")
        
        if self.cloud_connected and self.mqtt_client:
            try:
                self.mqtt_client.disconnect()
                rospy.loginfo("Disconnected from AWS IoT Core")
            except Exception as e:
                rospy.logwarn(f"Error during disconnect: {e}")
        
        rospy.loginfo("Shutdown complete")


if __name__ == '__main__':
    try:
        node = CloudBridgeNode()
        if node.enable_cloud:
            node.run()
    except rospy.ROSInterruptException:
        pass
    except Exception as e:
        rospy.logerr(f"Fatal error: {e}")
