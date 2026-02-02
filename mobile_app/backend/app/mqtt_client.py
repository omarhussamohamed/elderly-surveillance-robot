import json
import ssl
import threading
import os
import logging
from dotenv import load_dotenv
import paho.mqtt.client as mqtt

load_dotenv()

logger = logging.getLogger(__name__)

# AWS IoT Core Configuration - All values required
AWS_ENDPOINT = os.getenv("AWS_IOT_ENDPOINT")
if not AWS_ENDPOINT:
    raise ValueError("AWS_IOT_ENDPOINT must be set in .env file")

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
MESSAGE_LOCK = threading.Lock()

def on_connect(client, userdata, flags, rc):
    logger.info(f"MQTT Connected, code: {rc}")
    for topic, qos in TOPICS:
        client.subscribe(topic)
        logger.info(f"Subscribed to {topic}")

def on_message(client, userdata, msg):
    global LAST_MESSAGE
    try:
        payload = json.loads(msg.payload.decode())
        logger.info(f"MQTT message: {payload}")
        with MESSAGE_LOCK:
            LAST_MESSAGE.update(payload)
    except Exception as e:
        logger.error(f"MQTT parse error: {e}")

def on_disconnect(client, userdata, rc):
    if rc != 0:
        logger.warning(f"Unexpected MQTT disconnect. Code: {rc}")
        logger.info("Attempting to reconnect...")

def start_mqtt():
    try:
        client = mqtt.Client(client_id=CLIENT_ID)
        client.on_connect = on_connect
        client.on_message = on_message
        client.on_disconnect = on_disconnect

        # Enable automatic reconnection
        client.reconnect_delay_set(min_delay=1, max_delay=120)

        client.tls_set(
            ca_certs=AWS_ROOT_CA,
            certfile=AWS_CERT_FILE,
            keyfile=AWS_PRIVATE_KEY,
            tls_version=ssl.PROTOCOL_TLSv1_2,
        )

        logger.info("Connecting to AWS IoT...")
        client.connect(AWS_ENDPOINT, 8883)
        client.loop_forever()
    except Exception as e:
        logger.error(f"MQTT Connection failed: {e}")
        # App continues even if MQTT fails

def start_mqtt_thread():
    threading.Thread(target=start_mqtt, daemon=True).start()
