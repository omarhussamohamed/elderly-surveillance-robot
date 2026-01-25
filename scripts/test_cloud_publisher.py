#!/usr/bin/env python2
import rospy
import json
import paho.mqtt.client as mqtt
from std_msgs.msg import String
from geometry_msgs.msg import Twist

class CloudTestPublisher:
    def __init__(self):
        rospy.init_node('cloud_test_publisher')
        
        # Create test publisher
        self.cmd_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        
        # Connect to AWS IoT MQTT
        self.mqtt_client = mqtt.Client(client_id="cloud_test")
        self.mqtt_client.on_connect = self.on_mqtt_connect
        self.mqtt_client.on_message = self.on_mqtt_message
        
        # Connect (remove for actual AWS IoT - this is for testing)
        # self.mqtt_client.connect("localhost", 1883, 60)
        # self.mqtt_client.loop_start()
        
        rospy.loginfo("Cloud Test Publisher ready")
        
        # Publish test command
        rospy.Timer(rospy.Duration(5), self.publish_test_command)
        
    def on_mqtt_connect(self, client, userdata, flags, rc):
        rospy.loginfo("MQTT Connected with result code " + str(rc))
        client.subscribe("elderly_bot/commands")
        
    def on_mqtt_message(self, client, userdata, msg):
        rospy.loginfo("MQTT Message: {} = {}".format(msg.topic, msg.payload))
        
    def publish_test_command(self, event):
        # Create a simple movement command
        twist = Twist()
        twist.linear.x = 0.1
        twist.angular.z = 0.0
        self.cmd_pub.publish(twist)
        rospy.loginfo("Published test command: linear.x=0.1")
        
        # Also simulate telemetry
        telemetry = {
            'robot_id': 'robot_nano',
            'temperature': 45.5,
            'battery': 12.3,
            'status': 'active'
        }
        # self.mqtt_client.publish("elderly_bot/telemetry", json.dumps(telemetry))
        
    def run(self):
        rospy.spin()

if __name__ == '__main__':
    node = CloudTestPublisher()
    node.run()