from fastapi import APIRouter, Depends
from sqlalchemy.orm import Session
from app.database import get_db
from app.models import SystemHealth, SafetyMonitor
from app.auth_utils import get_current_user
from app.models import User


router = APIRouter(
    tags=["System"]
)


@router.get("/health")
def get_system_health(
    db: Session = Depends(get_db),
    current_user: User = Depends(get_current_user)
):
    latest = (
        db.query(SystemHealth)
        .order_by(SystemHealth.created_at.desc())
        .first()
    )

    if not latest:
        return {"power": 0, "temperature": 0}

    return {
        "power": latest.power,
        "temperature": latest.temperature
    }


@router.post("/health")
def update_system_health(
    power: int,
    temperature: float,
    db: Session = Depends(get_db),
    current_user: User = Depends(get_current_user)
):
    data = SystemHealth(
        power=power,
        temperature=temperature
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
):
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
    gas: str,
    fire: str,
    fall: str,
    stranger: str,
    db: Session = Depends(get_db),
    current_user: User = Depends(get_current_user)
):
    data = SafetyMonitor(
        gas=gas,
        fire=fire,
        fall=fall,
        stranger=stranger
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

