# Elderly Surveillance Robot

An intelligent surveillance system designed to monitor and ensure the safety of elderly individuals through autonomous robot navigation, real-time monitoring, and AI-powered detection capabilities.

> **Project Type:** Graduation Project / Research Prototype  
> **Status:** Academic implementation, not intended for production medical use

---

## System Overview

The Elderly Surveillance Robot is a comprehensive IoT system combining robotics, artificial intelligence, cloud computing, and mobile technology for autonomous monitoring and safety alerts in elderly care.

### Key Capabilities
- 🤖 Autonomous navigation and patrol
- 🔥 Fire and smoke detection
- ⚠️ Fall detection using computer vision
- 👤 Stranger detection via face recognition
- 🎥 Real-time video streaming
- 📱 Remote control via mobile app
- 🚨 Emergency alert system
- 💨 Gas leak detection
- 📊 System health monitoring

---

## Architecture

```
┌─────────────────────────────────────────────────────────────────────────────┐
│                           ELDERLY SURVEILLANCE ROBOT                         │
│                              System Architecture                             │
└─────────────────────────────────────────────────────────────────────────────┘

┌─────────────────┐      HTTPS/REST      ┌──────────────────┐
│   Mobile App    │◄────────────────────►│  Backend Server  │
│   (Flutter)     │      JWT Auth        │    (FastAPI)     │
└─────────────────┘                      └──────────────────┘
                                                  │
                                           PostgreSQL (RDS)
                                                  │
                                         MQTT (AWS IoT Core)
                                                  │
┌─────────────────┐      ROS Topics      ┌──────────────────┐
│  AI Pipeline    │◄────────────────────►│   Robot (ROS)    │
│  (YOLOv8/MP)    │                      │  (Jetson Nano)   │
└─────────────────┘                      └──────────────────┘
                                                  │
                                           Hardware Layer
                                                  │
                              ┌───────────────────┴───────────────────┐
                              │   Sensors         │   Actuators       │
                              │  • Camera         │  • Motors         │
                              │  • IMU (MPU9250)  │  • Buzzer         │
                              │  • Gas Sensor     │  • LEDs           │
                              │  • LiDAR          │                   │
                              └───────────────────────────────────────┘
```

### Data Flow
```
Camera → AI Pipeline → Alerts → Cloud DB → Mobile App → Notifications
                ↓
        [Priority Order]
        1. Fire/Smoke → STOP + Alert
        2. Fall       → STOP + Alert
        3. Stranger   → Alert (continue)
```

---

## Repository Structure

```
elderly-surveillance-robot/
├── ai/                     # AI Detection Pipeline
│   ├── src/               # Production code (pipeline, layers)
│   ├── experiments/       # Research notebooks (not required to run)
│   ├── assets/            # Models and known faces (gitignored)
│   └── README.md
│
├── mobile_app/             # Mobile Application System
│   ├── frontend/          # Flutter mobile app (Android/iOS)
│   ├── backend/           # FastAPI server (hosted on EC2)
│   └── README.md
│
├── ros/                    # Robot Operating System
│   ├── launch/            # ROS launch files
│   ├── scripts/           # ROS nodes (Python)
│   ├── config/            # Configuration files
│   ├── maps/              # SLAM maps
│   └── README.md
│
├── cloud/                  # Cloud Infrastructure Documentation
│   └── README.md          # AWS services and deployment
│
└── README.md              # This file (single source of truth)
```

---

## Technology Stack

| Component | Technology | Version/Notes |
|-----------|------------|---------------|
| **Mobile App** | Flutter | 3.x (Dart SDK 3.7+) |
| **Backend** | FastAPI | Python 3.9+ |
| **Database** | PostgreSQL | AWS RDS |
| **Robot OS** | ROS Melodic | Ubuntu 18.04 (Jetson Nano) |
| **AI - Fire Detection** | YOLOv8 | Custom trained model |
| **AI - Fall Detection** | MediaPipe Pose | Real-time pose estimation |
| **AI - Face Recognition** | face_recognition | HOG-based with PostgreSQL cache |
| **Cloud** | AWS | EC2, RDS, IoT Core, S3 |
| **Communication** | MQTT | AWS IoT Core (TLS) |

---

## Quick Start

### Prerequisites
- Python 3.9+ (backend), Python 3.10 (AI)
- Flutter 3.7.0+
- PostgreSQL database (local or AWS RDS)
- AWS IoT Core certificates

### 1. Clone Repository
```bash
git clone https://github.com/your-org/elderly-surveillance-robot.git
cd elderly-surveillance-robot
```

### 2. Backend Server
```bash
cd mobile_app/backend
python -m venv venv
source venv/bin/activate  # Windows: venv\Scripts\activate
pip install -r requirements.txt

# Configure .env (see Environment Variables section)
uvicorn app.main:app --reload --host 0.0.0.0 --port 8000
```
📖 **Details:** [mobile_app/backend/README.md](mobile_app/backend/README.md)

### 3. Mobile App
```bash
cd mobile_app/frontend
flutter pub get
# Update backend URL in lib/core/network/dio_client.dart
flutter run
```
📖 **Details:** [mobile_app/frontend/README.md](mobile_app/frontend/README.md)

### 4. AI Pipeline
```bash
cd ai
pip install -r requirements.txt
export DB_PASSWORD="your_password"  # For face recognition
python src/main.py
```
📖 **Details:** [ai/README.md](ai/README.md)

### 5. Robot (ROS)
```bash
cd ros
./install_dependencies.sh
catkin_make
source devel/setup.bash
roslaunch elderly_bot bringup.launch
```
📖 **Details:** [ros/README.md](ros/README.md)

---

## Environment Variables

### Backend Server (.env)
```bash
DATABASE_URL=postgresql://user:pass@host:5432/dbname
SECRET_KEY=your-secret-key-min-32-chars
ALGORITHM=HS256
AWS_IOT_ENDPOINT=xxx.iot.region.amazonaws.com
AWS_IOT_CLIENT_ID=backend-fastapi
AWS_ROOT_CA=certs/AmazonRootCA1.pem
AWS_CERT_FILE=certs/backend.cert.pem
AWS_PRIVATE_KEY=certs/backend.private.key
```

### AI Pipeline
```bash
DB_PASSWORD=your_postgres_password  # For face recognition cloud storage
```

---

## Database Schema

### Users Table
```sql
CREATE TABLE users (
    id SERIAL PRIMARY KEY,
    phone VARCHAR(20) UNIQUE NOT NULL,
    hashed_password VARCHAR(255) NOT NULL,
    created_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP
);
```

### System Health
```sql
CREATE TABLE system_health (
    id SERIAL PRIMARY KEY,
    power INTEGER NOT NULL,
    temperature FLOAT NOT NULL,
    created_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP
);
```

### Safety Monitor
```sql
CREATE TABLE safety_monitor (
    id SERIAL PRIMARY KEY,
    gas VARCHAR(20) NOT NULL,
    fire VARCHAR(20) NOT NULL,
    fall VARCHAR(20) NOT NULL,
    stranger VARCHAR(20) NOT NULL,
    created_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP
);
```

### Known Faces (AI Module)
```sql
CREATE TABLE known_faces (
    id SERIAL PRIMARY KEY,
    name TEXT NOT NULL,
    encodings BYTEA NOT NULL,
    created_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP
);
```

---

## Communication Protocols

| Path | Protocol | Format | Auth |
|------|----------|--------|------|
| Mobile ↔ Backend | HTTPS/REST | JSON | JWT Bearer |
| Backend ↔ Robot | MQTT (IoT Core) | JSON | X.509 Certificates |
| Robot Internal | ROS Topics | ROS Messages | N/A |

### MQTT Topics
| Topic | Direction | Purpose |
|-------|-----------|---------|
| `robot/telemetry` | Robot → Cloud | Sensor data (ROS default) |
| `robot/commands` | Cloud → Robot | Control commands (ROS default) |
| `elderly_bot/telemetry` | Robot → Cloud | Backend subscription |

> **Note:** Topic names are configurable via ROS parameters and backend `.env`

---

## Security

### Credentials Management
⚠️ **Never commit:**
- `.env` files
- AWS certificates (`.pem`, `.key`)
- Database credentials
- JWT secret keys

### Authentication
- JWT tokens with expiration (backend)
- Bcrypt password hashing
- X.509 certificates (IoT Core)
- TLS for all network communication

---

## Troubleshooting

| Issue | Solution |
|-------|----------|
| Backend won't start | Check Python 3.9+, verify `.env`, check DB connection |
| Mobile app can't connect | Verify backend URL, check network, test with curl |
| AI low FPS | Use YOLOv8n model, reduce frame resolution |
| Face recognition offline | Check `DB_PASSWORD` env variable |
| ROS nodes not running | Run `rosnode list`, check `roscore` |

---

## Module Documentation

| Module | Description | README |
|--------|-------------|--------|
| **AI** | Vision pipeline (fire, fall, face) | [ai/README.md](ai/README.md) |
| **Backend** | FastAPI server, auth, MQTT | [mobile_app/backend/README.md](mobile_app/backend/README.md) |
| **Frontend** | Flutter mobile app | [mobile_app/frontend/README.md](mobile_app/frontend/README.md) |
| **ROS** | Robot control, navigation | [ros/README.md](ros/README.md) |
| **Cloud** | AWS deployment guide | [cloud/README.md](cloud/README.md) |

---

## Team

- **Maryse Hani** — Mobile App & Backend Development, Cloud Architecture
- **Mariam Waleed** — Mobile App Development
- [Additional team members] — ROS, AI, Hardware

---

## License

Graduation project for elderly care monitoring. Academic use only.