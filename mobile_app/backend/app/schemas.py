"""Pydantic schemas for request/response validation."""
from pydantic import BaseModel, Field


class UserCreate(BaseModel):
    """Request schema for user registration."""
    phone: str = Field(..., min_length=8, max_length=15, description="User phone number")
    password: str = Field(..., min_length=6, description="User password")


class UserLogin(BaseModel):
    """Request schema for user login."""
    phone: str = Field(..., min_length=8, max_length=15, description="User phone number")
    password: str = Field(..., min_length=6, description="User password")


class Token(BaseModel):
    """Response schema for authentication token."""
    access_token: str
    token_type: str = "bearer"
