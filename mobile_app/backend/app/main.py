from fastapi import FastAPI
from fastapi.middleware.cors import CORSMiddleware
import logging
from . import mqtt_client
from .routers import auth, system_router

logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(name)s - %(levelname)s - %(message)s'
)
logger = logging.getLogger(__name__)

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

@app.on_event("startup")
def startup():
    logger.info("Starting MQTT thread")
    mqtt_client.start_mqtt_thread()

@app.get("/telemetry")
def telemetry():
    with mqtt_client.MESSAGE_LOCK:
        return mqtt_client.LAST_MESSAGE.copy()
