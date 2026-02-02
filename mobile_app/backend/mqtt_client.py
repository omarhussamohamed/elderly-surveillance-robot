import json
import ssl
import threading
import paho.mqtt.client as mqtt

AWS_ENDPOINT = "a1k8itxfx77i0w-ats.iot.us-east-1.amazonaws.com"
CLIENT_ID = "backend-fastapi"

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
    client = mqtt.Client(client_id=CLIENT_ID)
    client.on_connect = on_connect
    client.on_message = on_message

    client.tls_set(
        ca_certs="/home/ubuntu/grad_project_backend/certs/AmazonRootCA1.pem",
        certfile="/home/ubuntu/grad_project_backend/certs/backend.cert.pem",
        keyfile="/home/ubuntu/grad_project_backend/certs/backend.private.key",
        tls_version=ssl.PROTOCOL_TLSv1_2,
    )

    print("🔐 Connecting to AWS IoT...")
    client.connect(AWS_ENDPOINT, 8883)
    client.loop_forever()

def start_mqtt_thread():
    threading.Thread(target=start_mqtt, daemon=True).start()
