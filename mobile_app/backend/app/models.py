from sqlalchemy import Column, Integer, String, Float, DateTime
from app.database import Base
from datetime import datetime, timezone


def utc_now() -> datetime:
    """Return current UTC time (timezone-aware)."""
    return datetime.now(timezone.utc)


class User(Base):
    __tablename__ = "users"

    id = Column(Integer, primary_key=True, index=True)
    phone = Column(String(20), unique=True, index=True, nullable=False)
    hashed_password = Column(String(255), nullable=False)


class SystemHealth(Base):
    __tablename__ = "system_health"
    id = Column(Integer, primary_key=True, index=True)
    power = Column(Integer, nullable=False)
    temperature = Column(Float, nullable=False)
    created_at = Column(DateTime, default=utc_now)


class SafetyMonitor(Base):
    __tablename__ = "safety_monitor"

    id = Column(Integer, primary_key=True, index=True)
    gas = Column(String, default="Normal")
    fire = Column(String, default="None")
    fall = Column(String, default="None")
    stranger = Column(String, default="None")
    created_at = Column(DateTime, default=utc_now)