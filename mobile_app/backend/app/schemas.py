from pydantic import BaseModel, Field

# Request عند تسجيل مستخدم جديد
class UserCreate(BaseModel):
    phone: str = Field(..., min_length=8, max_length=15, description="User phone number")
    password: str = Field(..., min_length=6, description="User password")

# Request عند تسجيل الدخول
class UserLogin(BaseModel):
    phone: str = Field(..., min_length=8, max_length=15, description="User phone number")
    password: str = Field(..., min_length=6, description="User password")

# Response عند تسجيل الدخول
class Token(BaseModel):
    access_token: str
    token_type: str = "bearer"
