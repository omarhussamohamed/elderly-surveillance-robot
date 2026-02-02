# from sqlalchemy import create_engine
# from sqlalchemy.orm import sessionmaker, declarative_base
# from dotenv import load_dotenv
# import os

# load_dotenv()

# DATABASE_URL = os.getenv("DATABASE_URL")

# engine = create_engine(DATABASE_URL)
# SessionLocal = sessionmaker(autocommit=False, autoflush=False, bind=engine)


# Base = declarative_base()

# def get_db():
#     db = SessionLocal()
#     try:
#         yield db
#     finally:
#         db.close()

# conn = engine.connect()
# print("Connected!")
# conn.close()


import os
from sqlalchemy import create_engine
from sqlalchemy.orm import sessionmaker, declarative_base
from dotenv import load_dotenv

# Load the .env file
load_dotenv()

DATABASE_URL = os.getenv("DATABASE_URL")

# Debugging: This will print the URL to your console so you can see if it's loading
print(f"DEBUG: Loaded DATABASE_URL is: {DATABASE_URL}")

if DATABASE_URL is None:
    raise ValueError("DATABASE_URL is None. Ensure your .env file has 'DATABASE_URL=postgresql://...'")

# Fix for SQLAlchemy 2.0+ to ensure the driver is explicitly stated
if DATABASE_URL.startswith("postgres://"):
    DATABASE_URL = DATABASE_URL.replace("postgres://", "postgresql://", 1)

engine = create_engine(DATABASE_URL)
SessionLocal = sessionmaker(autocommit=False, autoflush=False, bind=engine)

Base = declarative_base()

def get_db():
    db = SessionLocal()
    try:
        yield db
    finally:
        db.close()

# Test the connection immediately
try:
    with engine.connect() as conn:
        print("Successfully connected to the RDS database!")
except Exception as e:
    print(f"Connection failed: {e}")