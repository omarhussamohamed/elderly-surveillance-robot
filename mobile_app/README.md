# Mobile App - Elderly Surveillance Robot

This folder contains the complete mobile application system for the Elderly Surveillance Robot project, including both the Flutter frontend and FastAPI backend server.

---

## Overview

The mobile application system enables caregivers and family members to remotely monitor elderly individuals through a user-friendly interface. The system consists of two main components:

- **Frontend:** Flutter mobile app (Android/iOS)
- **Backend:** FastAPI server hosted on AWS EC2

Together, they provide real-time monitoring, emergency alerts, user authentication, and remote robot control capabilities.

---

## System Components

### Frontend (Flutter Mobile App)

**Location:** `mobile_app/frontend/`

**Purpose:** User interface for monitoring and controlling the robot

**Key Features:**
- User authentication (login/registration)
- Real-time system health monitoring
- Safety status dashboard
- Live camera feed
- Remote robot control
- Emergency call functionality
- Profile and caregiver management
- Push notifications

**Technology:**
- Flutter 3.x
- Dart SDK 3.7.0
- BLoC state management
- Dio HTTP client

**Documentation:** See [frontend/README.md](frontend/README.md)

---

### Backend (FastAPI Server)

**Location:** `mobile_app/backend/`

**Purpose:** Server-side logic, database management, and robot communication

**Key Responsibilities:**
- User authentication and authorization (JWT)
- PostgreSQL database management
- MQTT client for robot telemetry
- RESTful API endpoints
- Real-time data aggregation
- Security and validation

**Technology:**
- Python 3.9+
- FastAPI framework
- PostgreSQL (AWS RDS)
- AWS IoT Core (MQTT)
- SQLAlchemy ORM

**Documentation:** See [backend/README.md](backend/README.md)

---

## Architecture

```
┌──────────────────┐
│  Mobile App UI   │
│    (Flutter)     │
└────────┬─────────┘
         │ HTTPS/REST
         │ JWT Auth
         ▼
┌──────────────────┐
│  Backend Server  │
│    (FastAPI)     │
└────┬────────┬────┘
     │        │
     │        └─────────► PostgreSQL
     │                    (User data, logs)
     │
     │ MQTT/TLS
     ▼
┌──────────────────┐
│  AWS IoT Core    │
└────────┬─────────┘
         │
         ▼
    Robot System
```

---

## Quick Start

### Prerequisites

- Python 3.9+ (for backend)
- Flutter 3.7.0+ (for frontend)
- PostgreSQL database
- AWS IoT Core setup

### 1. Setup Backend

```bash
cd backend

# Create virtual environment
python -m venv venv
venv\Scripts\activate  # Windows
# source venv/bin/activate  # Linux/Mac

# Install dependencies
pip install -r requirements.txt

# Configure environment
cp .env.example .env
# Edit .env with your database and AWS credentials

# Place AWS IoT certificates in certs/

# Run server
uvicorn app.main:app --reload --host 0.0.0.0 --port 8000
```

Backend will be available at: `http://localhost:8000`

API documentation: `http://localhost:8000/docs`

### 2. Setup Frontend

```bash
cd frontend

# Install dependencies
flutter pub get

# Update backend URL
# Edit lib/core/network/dio_client.dart
# Change baseUrl to your backend address

# Run app
flutter run
```

---

## Project Structure

```
mobile_app/
├── backend/                      # Backend server
│   ├── app/
│   │   ├── main.py              # FastAPI app entry
│   │   ├── database.py          # DB configuration
│   │   ├── models.py            # Database models
│   │   ├── schemas.py           # Pydantic schemas
│   │   ├── auth_utils.py        # JWT utilities
│   │   ├── mqtt_client.py       # MQTT client
│   │   └── routers/
│   │       ├── auth.py          # Auth endpoints
│   │       └── system_router.py # System endpoints
│   ├── certs/                   # AWS IoT certificates
│   ├── requirements.txt
│   ├── .env.example
│   └── README.md
│
├── frontend/                     # Flutter mobile app
│   ├── lib/
│   │   ├── main.dart            # App entry point
│   │   ├── core/                # Core utilities
│   │   │   ├── const.dart
│   │   │   ├── network/
│   │   │   └── on_generate_route.dart
│   │   └── features/            # App features
│   │       ├── authentication/
│   │       ├── home/
│   │       ├── emergency/
│   │       ├── notifications/
│   │       └── profile/
│   ├── assets/                  # Images, fonts, icons
│   ├── pubspec.yaml
│   └── README.md
│
└── README.md                    # This file
```

---

## API Endpoints

### Authentication

| Method | Endpoint | Description | Auth Required |
|--------|----------|-------------|---------------|
| POST | `/auth/register` | Register new user | No |
| POST | `/auth/login` | User login | No |

### System Monitoring

| Method | Endpoint | Description | Auth Required |
|--------|----------|-------------|---------------|
| GET | `/health` | Get system health | Yes |
| POST | `/health` | Update system health | Yes |
| GET | `/safety` | Get safety status | Yes |
| POST | `/safety` | Update safety status | Yes |

### Telemetry

| Method | Endpoint | Description | Auth Required |
|--------|----------|-------------|---------------|
| GET | `/telemetry` | Get real-time MQTT data | No |

---

## Communication Flow

### 1. User Authentication

```
Mobile App → POST /auth/login → Backend
              ↓
         Validate credentials
              ↓
         Generate JWT token
              ↓
Mobile App ← Return token ← Backend
```

### 2. Data Retrieval

```
Mobile App → GET /health (+ JWT token) → Backend
                                           ↓
                                    Query Database
                                           ↓
Mobile App ← Return health data ← Backend
```

### 3. Robot Telemetry

```
Robot → Publish MQTT → AWS IoT Core
                           ↓
                    Backend subscribes
                           ↓
                    Store in memory
                           ↓
Mobile App → GET /telemetry → Backend → Return latest data
```

---

## Environment Variables

### Backend Configuration

Create `backend/.env` file with:

```bash
# Database
DATABASE_URL=postgresql://username:password@host:5432/database_name

# JWT Authentication
SECRET_KEY=your-super-secret-key-generate-new-one
ALGORITHM=HS256

# AWS IoT Core
AWS_IOT_ENDPOINT=xxxxx-ats.iot.us-east-1.amazonaws.com
AWS_IOT_CLIENT_ID=backend-fastapi
AWS_ROOT_CA=certs/AmazonRootCA1.pem
AWS_CERT_FILE=certs/backend.cert.pem
AWS_PRIVATE_KEY=certs/backend.private.key
```

### Frontend Configuration

Update backend URL in `frontend/lib/core/network/dio_client.dart`:

```dart
baseUrl: 'http://YOUR_BACKEND_IP:8000'
```

**Common configurations:**
- Local development: `http://localhost:8000`
- Android emulator: `http://10.0.2.2:8000`
- Production: `https://your-api-domain.com`

---

## Database Schema

### Users Table

| Column | Type | Description |
|--------|------|-------------|
| id | Integer | Primary key |
| phone | String(20) | Unique phone number |
| hashed_password | String(255) | Bcrypt hashed password |

### SystemHealth Table

| Column | Type | Description |
|--------|------|-------------|
| id | Integer | Primary key |
| power | Integer | Battery percentage |
| temperature | Float | System temperature |
| created_at | DateTime | Timestamp |

### SafetyMonitor Table

| Column | Type | Description |
|--------|------|-------------|
| id | Integer | Primary key |
| gas | String | Gas detection status |
| fire | String | Fire detection status |
| fall | String | Fall detection status |
| stranger | String | Stranger detection status |
| created_at | DateTime | Timestamp |

---

## MQTT Topics

| Topic | Direction | Description | QoS |
|-------|-----------|-------------|-----|
| `elderly_bot/telemetry` | Robot → Backend | Sensor data, battery, temperature | 0 |

---

## Development Workflow

### Running Locally

1. **Start Backend:**
   ```bash
   cd backend
   uvicorn app.main:app --reload
   ```

2. **Start Frontend:**
   ```bash
   cd frontend
   flutter run
   ```

3. **Test API:**
   - Visit `http://localhost:8000/docs`
   - Test endpoints with Swagger UI

### Testing

**Backend:**
```bash
cd backend
pytest  # (tests to be implemented)
```

**Frontend:**
```bash
cd frontend
flutter test
```

---

## Deployment

### Backend Deployment (AWS EC2)

1. Launch Ubuntu EC2 instance
2. Install Python 3.9+
3. Clone repository
4. Install dependencies
5. Configure `.env` and certificates
6. Setup systemd service
7. Configure nginx reverse proxy
8. Enable SSL/TLS

Detailed instructions: [backend/README.md](backend/README.md)

### Frontend Deployment

**Android:**
```bash
flutter build apk --release
```

**iOS:**
```bash
flutter build ios --release
```

Detailed instructions: [frontend/README.md](frontend/README.md)

---

## Security Best Practices

1. **Never commit:**
   - `.env` files
   - AWS certificates
   - Database credentials
   - JWT secret keys

2. **Use HTTPS** in production for API communication

3. **Enable CORS** restrictions in backend for production

4. **Implement rate limiting** for API endpoints

5. **Use secure storage** for JWT tokens in production app

6. **Regularly update** dependencies for security patches

---

## Troubleshooting

### Backend Issues

**Connection to database failed:**
- Check PostgreSQL is running
- Verify `DATABASE_URL` in `.env`
- Ensure database exists

**MQTT connection failed:**
- Verify AWS IoT certificates are correct
- Check certificate paths in `.env`
- Ensure AWS IoT endpoint is correct

### Frontend Issues

**Cannot connect to backend:**
- Ensure backend is running
- Check `baseUrl` in dio_client.dart
- For Android emulator, use `http://10.0.2.2:8000`

**Assets not loading:**
- Run `flutter clean`
- Run `flutter pub get`
- Verify asset paths in pubspec.yaml

---

## Support

For detailed documentation:
- **Backend:** [backend/README.md](backend/README.md)
- **Frontend:** [frontend/README.md](frontend/README.md)
- **Cloud Setup:** [../cloud/README.md](../cloud/README.md)

---

## Authors

**Maryse Hani** - Mobile App & Backend Development

**Mariam Waleed** - Mobile App Development

---

## License

This project is part of a graduation project for elderly care monitoring system.