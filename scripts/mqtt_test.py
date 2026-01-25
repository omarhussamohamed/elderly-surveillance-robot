#!/usr/bin/env python2
# test_aws_robot.py
import paho.mqtt.client as mqtt
import ssl
import time

# Configuration
ENDPOINT = "a1k8itxfx77i0w-ats.iot.us-east-1.amazonaws.com"
CLIENT_ID = "robot"  # MUST MATCH YOUR THING NAME
CA_FILE = "/home/omar/catkin_ws/src/elderly_bot/aws_certs/AmazonRootCA1.pem"
CERT_FILE = "/home/omar/catkin_ws/src/elderly_bot/aws_certs/certificate.pem.crt"
KEY_FILE = "/home/omar/catkin_ws/src/elderly_bot/aws_certs/private.pem.key"

def on_connect(client, userdata, flags, rc):
    print("Connection result: {}".format(rc))
    if rc == 0:
        print("SUCCESS! Connected as client: {}".format(CLIENT_ID))
        client.publish("robot/test", "Hello from robot", qos=1)
    else:
        print("Failed to connect. Error code: {}".format(rc))

def on_publish(client, userdata, mid):
    print("Message published")

def on_disconnect(client, userdata, rc):
    print("Disconnected")

# Create client
client = mqtt.Client(client_id=CLIENT_ID)

# Setup TLS
client.tls_set(
    ca_certs=CA_FILE,
    certfile=CERT_FILE,
    keyfile=KEY_FILE,
    tls_version=ssl.PROTOCOL_TLSv1_2
)

# Set callbacks
client.on_connect = on_connect
client.on_publish = on_publish
client.on_disconnect = on_disconnect

print("Connecting to AWS IoT as '{}'...".format(CLIENT_ID))
try:
    client.connect(ENDPOINT, 8883, 60)
    client.loop_start()
    time.sleep(3)
    client.loop_stop()
    client.disconnect()
except Exception as e:
    print("Error: {}".format(e))