from fastapi import FastAPI
from fastapi.middleware.cors import CORSMiddleware
from pydantic import BaseModel
from . import mqtt_client
from .routers import auth, system_router

app = FastAPI()

# Add CORS middleware
app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],  # Configure appropriately for production
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

# Include routers
app.include_router(auth.router)
app.include_router(system_router.router)

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
