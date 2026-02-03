# Mobile App - Elderly Surveillance Robot

> **Part of:** [Elderly Surveillance Robot](../README.md)

Mobile application system with Flutter frontend and FastAPI backend.

---

## Overview

The mobile app enables caregivers to remotely monitor elderly individuals through:

- **Frontend:** Flutter mobile app (Android/iOS)
- **Backend:** FastAPI server on AWS EC2

```
┌──────────────────┐
│  Mobile App UI   │
│    (Flutter)     │
└────────┬─────────┘
         │ HTTPS/REST + JWT
         ▼
┌──────────────────┐
│  Backend Server  │
│    (FastAPI)     │
└────┬────────┬────┘
     │        │
     │ MQTT   └──► PostgreSQL (RDS)
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

### Backend

```bash
cd backend
python -m venv venv
venv\Scripts\activate  # Windows
pip install -r requirements.txt

# Configure environment
cp .env.example .env
# Edit .env with database + AWS credentials

uvicorn app.main:app --reload --host 0.0.0.0 --port 8000
```

**API docs:** `http://localhost:8000/docs`

### Frontend

```bash
cd frontend
flutter pub get

# Update lib/core/network/dio_client.dart with backend URL

flutter run
```

---

## API Endpoints

### Authentication
| Method | Endpoint | Description |
|--------|----------|-------------|
| POST | `/auth/register` | Register user |
| POST | `/auth/login` | Login (returns JWT) |

### Monitoring
| Method | Endpoint | Description |
|--------|----------|-------------|
| GET | `/health` | System health (battery, temp) |
| GET | `/safety` | Safety status (gas, fire, fall, stranger) |
| GET | `/telemetry` | Real-time MQTT data |

---

## Configuration

### Backend `.env`
```bash
DATABASE_URL=postgresql://user:pass@host:5432/dbname
SECRET_KEY=your-jwt-secret
AWS_IOT_ENDPOINT=xxxxx-ats.iot.eu-north-1.amazonaws.com
```

### Frontend `dio_client.dart`
```dart
baseUrl: 'http://YOUR_BACKEND_IP:8000'
// Android emulator: http://10.0.2.2:8000
```

---

## Project Structure

```
mobile_app/
├── backend/
│   ├── app/
│   │   ├── main.py           # FastAPI entry
│   │   ├── database.py       # PostgreSQL connection
│   │   ├── models.py         # SQLAlchemy models
│   │   ├── mqtt_client.py    # AWS IoT client
│   │   └── routers/
│   ├── certs/                # AWS IoT certificates
│   └── requirements.txt
│
├── frontend/
│   ├── lib/
│   │   ├── main.dart
│   │   ├── core/             # Network, routing
│   │   └── features/         # BLoC modules
│   │       ├── authentication/
│   │       ├── home/
│   │       ├── emergency/
│   │       └── profile/
│   └── pubspec.yaml
│
└── README.md
```

---

## Detailed Documentation

- [Backend Setup](backend/README.md) — API, database, MQTT
- [Frontend Setup](frontend/README.md) — Flutter, BLoC, screens
- [Root README](../README.md) — System overview

---

## Authors

- **Maryse Hani** — Mobile App & Backend
- **Mariam Waleed** — Mobile App