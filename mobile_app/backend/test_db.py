# from app.database import SessionLocal
# from app.models import User  # تأكدي إن ده هو موديل المستخدمين

# # افتح session للـ DB
# db = SessionLocal()

# # جلب كل المستخدمين
# users = db.query(User).all()

# for user in users:
#     print(f"ID: {user.id}, Phone: {user.phone}, Hashed: {user.hashed_password}")


# db.close()

# from app.database import SessionLocal
# from app import models

# db = SessionLocal()

# # هنا حطي رقم الهاتف اللي عايزة تتأكدي منه
# phone = "01012345678"

# user = db.query(models.User).filter(models.User.phone == phone).first()

# if user:
#     print(f"User exists! ID: {user.id}, Phone: {user.phone}")
# else:
#     print("User not found")

from jose import jwt, JWTError
import os
from dotenv import load_dotenv

load_dotenv()

SECRET_KEY = os.getenv("SECRET_KEY")
ALGORITHM = os.getenv("ALGORITHM")

# حطي هنا الـ token اللي أخدتيه من login
token = "eyJhbGciOiJIUzI1NiIsInR5cCI6IkpXVCJ9.eyJ1c2VyX2lkIjoxLCJleHAiOjE3NjkyNzY5MDZ9.SYDcs-UGVg6m_ByxhFeiy9mhJD3ZBexG4WXQoiuZRrU"

try:
    payload = jwt.decode(token, SECRET_KEY, algorithms=[ALGORITHM])
    user_id = payload.get("user_id")
    if user_id:
        print(f"✅ Token صحيح! User ID: {user_id}")
    else:
        print("❌ Token غير صالح: مفيش user_id")
except JWTError as e:
    print(f"❌ Token غير صالح: {str(e)}")

