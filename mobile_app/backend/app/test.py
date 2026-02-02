# from fastapi import FastAPI, HTTPException
# from fastapi.middleware.cors import CORSMiddleware # Added this
# from pydantic import BaseModel
# from typing import List, Optional
# import psycopg2
# from psycopg2.extras import RealDictCursor

# app = FastAPI()

# # --- STEP 1: ADD CORS (This fixes the Connection Error) ---
# app.add_middleware(
#     CORSMiddleware,
#     allow_origins=["*"],  # Allows your app to connect
#     allow_credentials=True,
#     allow_methods=["*"],
#     allow_headers=["*"],
# )

# # DATABASE CONNECTION
# def get_db():
#     return psycopg2.connect(
#         # Updated to the host that worked in your setup_db.py script
#         host="elderly-care-db.cr66282o8nxf.eu-north-1.rds.amazonaws.com", 
#         database="postgres",
#         user="postgres",
#         password="robotpassword123",
#         port="5432",
#         cursor_factory=RealDictCursor
#     )

# # SCHEMAS
# class MedUpdate(BaseModel):
#     is_taken: bool

# # ENDPOINTS
# @app.get("/login/{code}")
# def login(code: str):
#     try:
#         conn = get_db()
#         cur = conn.cursor()
#         cur.execute("SELECT * FROM care_circles WHERE family_code = %s", (code,))
#         res = cur.fetchone()
#         cur.close()
#         conn.close()
#         if not res: 
#             raise HTTPException(status_code=404, detail="Invalid Code")
#         return res
#     except Exception as e:
#         print(f"Error: {e}")
#         raise HTTPException(status_code=500, detail="Database connection failed")

# # ... rest of your endpoints remain the same ...

import fastapi
print(fastapi.__version__)
