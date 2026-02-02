import json
import ssl
import threading
import os
from dotenv import load_dotenv
import paho.mqtt.client as mqtt

load_dotenv()

AWS_ENDPOINT = os.getenv("AWS_IOT_ENDPOINT", "a1k8itxfx77i0w-ats.iot.us-east-1.amazonaws.com")
CLIENT_ID = os.getenv("AWS_IOT_CLIENT_ID", "backend-fastapi")
AWS_ROOT_CA = os.getenv("AWS_ROOT_CA", "certs/AmazonRootCA1.pem")
AWS_CERT_FILE = os.getenv("AWS_CERT_FILE", "certs/backend.cert.pem")
AWS_PRIVATE_KEY = os.getenv("AWS_PRIVATE_KEY", "certs/backend.private.key")

TOPICS = [
    ("elderly_bot/telemetry", 0),
]

# 🔑 Shared global state
LAST_MESSAGE = {
    "gas": False,
    "battery": None,
    "temperature": None
}

def on_connect(client, userdata, flags, rc):
    print("✅ MQTT Connected, code:", rc)
    for topic, qos in TOPICS:
        client.subscribe(topic)
        print(f"📡 Subscribed to {topic}")

def on_message(client, userdata, msg):
    global LAST_MESSAGE
    try:
        payload = json.loads(msg.payload.decode())
        print("📥 MQTT message:", payload)
        LAST_MESSAGE.update(payload)
    except Exception as e:
        print("❌ MQTT parse error:", e)

def start_mqtt():
    try:
        client = mqtt.Client(client_id=CLIENT_ID)
        client.on_connect = on_connect
        client.on_message = on_message

        client.tls_set(
            ca_certs=AWS_ROOT_CA,
            certfile=AWS_CERT_FILE,
            keyfile=AWS_PRIVATE_KEY,
            tls_version=ssl.PROTOCOL_TLSv1_2,
        )

        print("🔐 Connecting to AWS IoT...")
        client.connect(AWS_ENDPOINT, 8883)
        client.loop_forever()
    except Exception as e:
        print(f"❌ MQTT Connection failed: {e}")
        # App continues even if MQTT fails

def start_mqtt_thread():
    threading.Thread(target=start_mqtt, daemon=True).start()
