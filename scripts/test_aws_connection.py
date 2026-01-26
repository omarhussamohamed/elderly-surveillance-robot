#!/usr/bin/env python2
# -*- coding: utf-8 -*-
# Test script to verify AWS IoT connection

import paho.mqtt.client as mqtt
import ssl
import json
import time

def on_connect(client, userdata, flags, rc):
    if rc == 0:
        print("SUCCESS: Connected to AWS IoT!")
        print("Subscribing to commands...")
        client.subscribe("robot/commands", qos=1)
        
        # Send a test telemetry
        telemetry = {"test": "connection", "timestamp": time.time()}
        client.publish("robot/telemetry", json.dumps(telemetry), qos=0)
        print("Test telemetry sent")
        
        # Send a buzzer command
        cmd = {"command": "buzzer", "value": True}
        client.publish("robot/commands", json.dumps(cmd), qos=1)
        print("Test command sent")
        
    else:
        print("FAILED: Connection error code: %s" % rc)

def on_message(client, userdata, msg):
    print("Message received on %s: %s" % (msg.topic, msg.payload.decode()))

def on_log(client, userdata, level, buf):
    print("Log: %s" % buf)

# Setup client
client = mqtt.Client(client_id="connection_test", protocol=mqtt.MQTTv311)
client.on_connect = on_connect
client.on_message = on_message
# client.on_log = on_log  # Uncomment for detailed logs

# TLS
try:
    client.tls_set(
        ca_certs="/home/omar/catkin_ws/src/elderly_bot/aws_certs/AmazonRootCA1.pem",
        certfile="/home/omar/catkin_ws/src/elderly_bot/aws_certs/certificate.pem.crt",
        keyfile="/home/omar/catkin_ws/src/elderly_bot/aws_certs/private.pem.key",
        tls_version=ssl.PROTOCOL_TLSv1_2
    )
    print("Certificates loaded")
except Exception as e:
    print("Certificate error: %s" % e)
    exit(1)

# Connect
try:
    print("Connecting to a1k8itxfx77i0w-ats.iot.us-east-1.amazonaws.com:8883...")
    client.connect("a1k8itxfx77i0w-ats.iot.us-east-1.amazonaws.com", 8883, 60)
    client.loop_start()
    time.sleep(5)
    client.loop_stop()
    client.disconnect()
except Exception as e:
    print("Connection error: %s" % e)