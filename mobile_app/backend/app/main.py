from fastapi import FastAPI
from pydantic import BaseModel
import mqtt_client

app = FastAPI()

# ======== MODELS ========
class User(BaseModel):
    phone: str
    password: str

# ======== FAKE STORAGE (DEMO ONLY) ========
USERS = {}

@app.on_event("startup")
def startup():
    print("🚀 Starting MQTT thread")
    mqtt_client.start_mqtt_thread()

# ======== AUTH ========
@app.post("/register")
def register(user: User):
    USERS[user.phone] = user.password
    return {"status": "registered"}

@app.post("/login")
def login(user: User):
    if USERS.get(user.phone) == user.password:
        return {"status": "login_success"}
    return {"status": "login_failed"}

# ======== TELEMETRY ========
@app.get("/telemetry")
def telemetry():
    return mqtt_client.LAST_MESSAGE
