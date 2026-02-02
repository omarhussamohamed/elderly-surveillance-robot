# Backend Server - Elderly Surveillance Robot

FastAPI backend server that provides authentication, telemetry processing, and system monitoring for the Elderly Surveillance Robot mobile application.

---

## Overview

This backend server acts as the central hub connecting the mobile application to the robot system via AWS IoT Core. It handles user authentication, stores system data in PostgreSQL, and provides RESTful APIs for the mobile app.

**Key Responsibilities:**
- User authentication and authorization (JWT)
- Real-time telemetry data processing from robot via MQTT
- System health monitoring (battery, temperature)
- Safety status tracking (gas, fire, fall, stranger detection)
- Secure API endpoints for mobile app

---

## Technology Stack

- **Framework:** FastAPI (Python)
- **Web Server:** Uvicorn
- **Database:** PostgreSQL (AWS RDS)
- **ORM:** SQLAlchemy
- **Authentication:** JWT (python-jose) with bcrypt password hashing
- **MQTT Client:** paho-mqtt (AWS IoT Core)
- **Environment:** python-dotenv

---

## Folder Structure

```
backend/
├── app/
│   ├── main.py              # FastAPI application entry point
│   ├── database.py          # Database connection and session management
│   ├── models.py            # SQLAlchemy database models
│   ├── schemas.py           # Pydantic request/response schemas
│   ├── auth_utils.py        # JWT token and password utilities
│   ├── mqtt_client.py       # AWS IoT Core MQTT client
│   └── routers/
│       ├── __init__.py
│       ├── auth.py          # Authentication endpoints
│       └── system_router.py # System monitoring endpoints
├── certs/                   # AWS IoT certificates (not committed)
│   ├── AmazonRootCA1.pem
│   ├── backend.cert.pem
│   └── backend.private.key
├── requirements.txt         # Python dependencies
├── .env                     # Environment variables (not committed)
├── .env.example            # Environment template
└── README.md               # This file
```

---

## Prerequisites

Before setting up the backend, ensure you have:

- **Python 3.9 or higher**
- **PostgreSQL database** (local or AWS RDS)
- **AWS IoT Core** setup with certificates
- **Git** for version control

---

## Installation

### 1. Clone the Repository

```bash
cd mobile_app/backend
```

### 2. Create Virtual Environment

```bash
# Create virtual environment
python -m venv venv

# Activate on Windows
venv\Scripts\activate

# Activate on macOS/Linux
source venv/bin/activate
```

### 3. Install Dependencies

```bash
pip install -r requirements.txt
```

### 4. Configure Environment Variables

Copy the example environment file and configure it:

```bash
cp .env.example .env
```

Edit `.env` with your actual configuration (see Configuration section below).

### 5. Place AWS IoT Certificates

Download your AWS IoT certificates and place them in the `certs/` directory:

```
certs/
├── AmazonRootCA1.pem
├── backend.cert.pem
└── backend.private.key
```

**Security Note:** Never commit certificates or `.env` files to version control.

---

## Configuration

### Environment Variables

Edit the `.env` file with the following variables:

| Variable | Description | Example |
|----------|-------------|---------|
| `DATABASE_URL` | PostgreSQL connection string | `postgresql://user:password@localhost:5432/elderly_robot` |
| `SECRET_KEY` | JWT signing secret (generate strong random key) | `your-super-secret-key-change-me` |
| `ALGORITHM` | JWT algorithm | `HS256` |
| `AWS_IOT_ENDPOINT` | AWS IoT Core endpoint | `xxxxx-ats.iot.us-east-1.amazonaws.com` |
| `AWS_IOT_CLIENT_ID` | MQTT client identifier | `backend-fastapi` |
| `AWS_ROOT_CA` | Root CA certificate path | `certs/AmazonRootCA1.pem` |
| `AWS_CERT_FILE` | Client certificate path | `certs/backend.cert.pem` |
| `AWS_PRIVATE_KEY` | Private key path | `certs/backend.private.key` |

**Generating SECRET_KEY:**

```bash
python -c "import secrets; print(secrets.token_urlsafe(32))"
```

---

## Database Setup

### Database Models

The application uses three main models:

#### 1. User
Stores user authentication data.

| Column | Type | Description |
|--------|------|-------------|
| id | Integer | Primary key |
| phone | String(20) | Unique phone number |
| hashed_password | String(255) | Bcrypt hashed password |

#### 2. SystemHealth
Tracks robot system health metrics.

| Column | Type | Description |
|--------|------|-------------|
| id | Integer | Primary key |
| power | Integer | Battery percentage |
| temperature | Float | System temperature in Celsius |
| created_at | DateTime | Timestamp |

#### 3. SafetyMonitor
Records safety detection events.

| Column | Type | Description |
|--------|------|-------------|
| id | Integer | Primary key |
| gas | String | Gas detection status |
| fire | String | Fire detection status |
| fall | String | Fall detection status |
| stranger | String | Stranger detection status |
| created_at | DateTime | Timestamp |

### Create Database Tables

After configuring `DATABASE_URL`, create the tables:

```python
# In Python shell or create a migration script
from app.database import engine
from app.models import Base

Base.metadata.create_all(bind=engine)
```

Or use Alembic for migrations (recommended for production):

```bash
pip install alembic
alembic init alembic
# Configure alembic.ini and create migrations
```

---

## Running the Server

### Development Mode

Run with auto-reload enabled:

```bash
uvicorn app.main:app --reload --host 0.0.0.0 --port 8000
```

### Production Mode

Run without reload:

```bash
uvicorn app.main:app --host 0.0.0.0 --port 8000 --workers 4
```

### Using Gunicorn (Production)

```bash
pip install gunicorn
gunicorn app.main:app --workers 4 --worker-class uvicorn.workers.UvicornWorker --bind 0.0.0.0:8000
```

### Verify Server is Running

Open browser and navigate to:
- **API Docs:** http://localhost:8000/docs (Swagger UI)
- **Alternative Docs:** http://localhost:8000/redoc

---

## API Endpoints

### Authentication

#### Register New User

```http
POST /auth/register
Content-Type: application/json

{
  "phone": "01012345678",
  "password": "securepassword"
}
```

**Response:**
```json
{
  "message": "User registered successfully",
  "access_token": "eyJhbGciOiJIUzI1NiIsInR5cCI6IkpXVCJ9...",
  "token_type": "bearer"
}
```

#### Login

```http
POST /auth/login
Content-Type: application/json

{
  "phone": "01012345678",
  "password": "securepassword"
}
```

**Response:**
```json
{
  "access_token": "eyJhbGciOiJIUzI1NiIsInR5cCI6IkpXVCJ9...",
  "token_type": "bearer"
}
```

### System Health Monitoring

#### Get System Health

```http
GET /health
Authorization: Bearer <token>
```

**Response:**
```json
{
  "power": 85,
  "temperature": 42.5
}
```

#### Update System Health

```http
POST /health?power=85&temperature=42.5
Authorization: Bearer <token>
```

### Safety Monitoring

#### Get Safety Status

```http
GET /safety
Authorization: Bearer <token>
```

**Response:**
```json
{
  "gas": "Normal",
  "fire": "None",
  "fall": "None",
  "stranger": "None"
}
```

#### Update Safety Status

```http
POST /safety?gas=Normal&fire=None&fall=None&stranger=None
Authorization: Bearer <token>
```

### Telemetry

#### Get Real-Time MQTT Data

```http
GET /telemetry
```

**Response:**
```json
{
  "gas": false,
  "battery": 85,
  "temperature": 42.5
}
```

**Note:** This endpoint returns the latest data received from the robot via MQTT.

---

## MQTT Configuration

### Topics

The backend subscribes to the following MQTT topic:

| Topic | QoS | Description |
|-------|-----|-------------|
| `elderly_bot/telemetry` | 0 | Robot telemetry data (battery, temperature, sensors) |

### Message Format

Expected JSON payload from robot:

```json
{
  "gas": false,
  "battery": 85,
  "temperature": 42.5
}
```

### Connection Details

- **Protocol:** MQTT over TLS (port 8883)
- **TLS Version:** TLSv1.2
- **Authentication:** X.509 certificates
- **Automatic Reconnection:** Enabled (1-120 seconds exponential backoff)

---

## Architecture

### Request Flow

```
Mobile App → FastAPI Endpoints → Database (PostgreSQL)
                ↓
        MQTT Client → AWS IoT Core → Robot
```

### Authentication Flow

1. User sends credentials to `/auth/login`
2. Backend validates credentials against database
3. Backend generates JWT token
4. Token returned to mobile app
5. Mobile app includes token in Authorization header for protected endpoints

### MQTT Flow

1. Backend connects to AWS IoT Core on startup
2. Subscribes to `elderly_bot/telemetry` topic
3. Robot publishes telemetry data
4. Backend receives and stores in `LAST_MESSAGE` global state
5. Mobile app requests `/telemetry` to get latest data

---

## Testing

### Manual Testing with curl

```bash
# Register
curl -X POST http://localhost:8000/auth/register \
  -H "Content-Type: application/json" \
  -d '{"phone":"01012345678","password":"test123"}'

# Login
curl -X POST http://localhost:8000/auth/login \
  -H "Content-Type: application/json" \
  -d '{"phone":"01012345678","password":"test123"}'

# Get health (replace TOKEN with actual token)
curl -X GET http://localhost:8000/health \
  -H "Authorization: Bearer TOKEN"

# Get telemetry
curl -X GET http://localhost:8000/telemetry
```

### Using FastAPI Swagger UI

Navigate to `http://localhost:8000/docs` and test endpoints interactively.

---

## Troubleshooting

### Database Connection Failed

**Error:** `Connection failed: could not connect to server`

**Solution:**
- Verify PostgreSQL is running
- Check `DATABASE_URL` in `.env`
- Ensure database exists
- Check network connectivity

### MQTT Connection Failed

**Error:** `MQTT Connection failed: [SSL: CERTIFICATE_VERIFY_FAILED]`

**Solution:**
- Verify certificates are in `certs/` directory
- Check certificate paths in `.env`
- Ensure certificates are valid and not expired
- Verify AWS IoT endpoint is correct

### SECRET_KEY Not Set

**Error:** `ValueError: SECRET_KEY must be set in .env file`

**Solution:**
- Ensure `.env` file exists
- Add `SECRET_KEY=<your-secret-key>` to `.env`
- Generate a strong key using the command in Configuration section

### Import Module Error

**Error:** `ModuleNotFoundError: No module named 'app'`

**Solution:**
- Ensure you're in the `backend/` directory
- Run with `python -m uvicorn app.main:app`
- Verify virtual environment is activated

### Port Already in Use

**Error:** `OSError: [Errno 98] Address already in use`

**Solution:**
- Find and kill process using port 8000
- Or use different port: `--port 8001`

---

## Security Best Practices

1. **Never commit:**
   - `.env` file
   - Certificates in `certs/` directory
   - Database credentials

2. **Use strong SECRET_KEY:**
   - Minimum 32 characters
   - Generate using `secrets` module

3. **Update dependencies regularly:**
   ```bash
   pip list --outdated
   pip install --upgrade <package>
   ```

4. **Enable HTTPS in production:**
   - Use reverse proxy (nginx)
   - Configure SSL certificates

5. **Restrict CORS origins:**
   - Update `allow_origins` in `main.py` for production

---

## Deployment

### AWS EC2 Deployment

1. Launch Ubuntu EC2 instance
2. Install Python 3.9+
3. Clone repository
4. Install dependencies
5. Configure `.env` and certificates
6. Set up systemd service
7. Configure nginx as reverse proxy
8. Enable SSL/TLS

### Systemd Service Example

Create `/etc/systemd/system/elderly-backend.service`:

```ini
[Unit]
Description=Elderly Robot Backend
After=network.target

[Service]
User=ubuntu
WorkingDirectory=/home/ubuntu/elderly-surveillance-robot/mobile_app/backend
Environment="PATH=/home/ubuntu/elderly-surveillance-robot/mobile_app/backend/venv/bin"
ExecStart=/home/ubuntu/elderly-surveillance-robot/mobile_app/backend/venv/bin/uvicorn app.main:app --host 0.0.0.0 --port 8000

[Install]
WantedBy=multi-user.target
```

Enable and start:

```bash
sudo systemctl enable elderly-backend
sudo systemctl start elderly-backend
sudo systemctl status elderly-backend
```

---

## Logging

Logs are output to console with the following format:

```
2026-02-03 10:30:45 - app.mqtt_client - INFO - MQTT Connected, code: 0
2026-02-03 10:30:45 - app.mqtt_client - INFO - Subscribed to elderly_bot/telemetry
2026-02-03 10:31:12 - app.main - INFO - Starting MQTT thread
```

For production, redirect logs to file:

```bash
uvicorn app.main:app --log-config logging_config.yaml
```

---

## Authors

**Maryse Hani** - Backend Development & Cloud Integration

**Mariam Waleed** - Mobile App & Backend Development

---

## License

This project is part of a graduation project for elderly care monitoring system.
