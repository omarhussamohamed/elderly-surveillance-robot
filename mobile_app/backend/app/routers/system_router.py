from fastapi import APIRouter, Depends
from pydantic import BaseModel, Field
from typing import Dict, Any
from sqlalchemy.orm import Session
from app.database import get_db
from app.models import SystemHealth, SafetyMonitor, User
from app.auth_utils import get_current_user


# Request schemas
class HealthUpdate(BaseModel):
    """Request model for health updates."""
    power: int = Field(..., ge=0, le=100, description="Battery percentage")
    temperature: float = Field(..., ge=-40, le=150, description="Temperature in Celsius")


class SafetyUpdate(BaseModel):
    """Request model for safety status updates."""
    gas: str = Field(default="Normal", max_length=50)
    fire: str = Field(default="None", max_length=50)
    fall: str = Field(default="None", max_length=50)
    stranger: str = Field(default="None", max_length=50)


router = APIRouter(
    tags=["System"]
)


@router.get("/health")
def get_system_health(
    db: Session = Depends(get_db),
    current_user: User = Depends(get_current_user)
) -> Dict[str, Any]:
    """Get latest system health metrics."""
    latest = (
        db.query(SystemHealth)
        .order_by(SystemHealth.created_at.desc())
        .first()
    )

    if not latest:
        return {"power": 0, "temperature": 0.0}

    return {
        "power": latest.power,
        "temperature": latest.temperature
    }


@router.post("/health")
def update_system_health(
    health: HealthUpdate,
    db: Session = Depends(get_db),
    current_user: User = Depends(get_current_user)
) -> Dict[str, Any]:
    """Update system health metrics."""
    data = SystemHealth(
        power=health.power,
        temperature=health.temperature
    )
    db.add(data)
    db.commit()
    db.refresh(data)

    return {
        "status": "updated",
        "power": data.power,
        "temperature": data.temperature
    }


@router.get("/safety")
def get_safety_status(
    db: Session = Depends(get_db),
    current_user: User = Depends(get_current_user)
) -> Dict[str, Any]:
    """Get latest safety monitor status."""
    latest = (
        db.query(SafetyMonitor)
        .order_by(SafetyMonitor.created_at.desc())
        .first()
    )

    if not latest:
        return {
            "gas": "Unknown",
            "fire": "Unknown",
            "fall": "Unknown",
            "stranger": "Unknown"
        }

    return {
        "gas": latest.gas,
        "fire": latest.fire,
        "fall": latest.fall,
        "stranger": latest.stranger
    }

# POST update safety status
@router.post("/safety")
def update_safety_status(
    safety: SafetyUpdate,
    db: Session = Depends(get_db),
    current_user: User = Depends(get_current_user)
) -> Dict[str, Any]:
    """Update safety monitor status."""
    data = SafetyMonitor(
        gas=safety.gas,
        fire=safety.fire,
        fall=safety.fall,
        stranger=safety.stranger
    )
    db.add(data)
    db.commit()
    db.refresh(data)

    return {
        "status": "updated",
        "gas": data.gas,
        "fire": data.fire,
        "fall": data.fall,
        "stranger": data.stranger
    }

