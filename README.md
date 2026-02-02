# Elderly Surveillance Robot

An intelligent surveillance system designed to monitor and ensure the safety of elderly individuals through autonomous robot navigation, real-time monitoring, and AI-powered detection capabilities.

---

## Project Overview

The Elderly Surveillance Robot is a comprehensive IoT system that combines robotics, artificial intelligence, cloud computing, and mobile technology to provide autonomous monitoring and safety alerts for elderly care. The system enables caregivers and family members to remotely monitor their loved ones, receive instant alerts for emergencies, and control the robot through a mobile application.

**Key Capabilities:**
- Autonomous navigation and patrol
- Fall detection using computer vision
- Fire and smoke detection
- Gas leak detection
- Stranger recognition through facial recognition
- Real-time video streaming
- Remote control via mobile app
- Emergency alert system
- System health monitoring

---

## System Architecture

```
┌─────────────────┐      HTTPS/REST      ┌──────────────────┐
│  Mobile App     │ ◄──────────────────► │  Backend Server  │
│  (Flutter)      │                       │  (FastAPI)       │
└─────────────────┘                       └──────────────────┘
                                                   │
                                            MQTT (AWS IoT)
                                                   │
┌─────────────────┐      ROS Topics       ┌──────────────────┐
│  AI Detection   │ ◄──────────────────► │  Robot (ROS)     │
│  (Python/YOLO)  │                       │  (Jetson Nano)   │
└─────────────────┘                       └──────────────────┘
                                                   │
                                            Hardware Layer
                                                   │
                                    ┌──────────────┴──────────────┐
                                    │   Sensors    │   Actuators  │
                                    │  - Camera    │  - Motors    │
                                    │  - MPU9250   │  - Buzzer    │
                                    │  - Gas       │  - LEDs      │
                                    └─────────────────────────────┘
```

---

## Repository Structure

```
elderly-surveillance-robot/
├── mobile_app/              # Mobile application and backend
│   ├── frontend/           # Flutter mobile app
│   └── backend/            # FastAPI backend server
│
├── ros/                    # Robot Operating System components
│   ├── launch/            # ROS launch files
│   ├── scripts/           # ROS nodes (Python)
│   ├── config/            # Configuration files
│   ├── maps/              # SLAM maps
│   └── urdf/              # Robot model description
│
├── ai/                     # AI detection models
│   ├── src/               # Detection pipeline source
│   │   └── layers/
│   │       ├── face_recognition/
│   │       ├── fall_detection/
│   │       └── fire_detection/
│   ├── experiments/       # Model training notebooks
│   └── assets/            # Model weights and known faces
│
├── cloud/                  # Cloud infrastructure documentation
│   └── README.md          # AWS services and deployment
│
├── docs/                   # Additional documentation
└── README.md              # This file
```

---

## Technology Stack

### Mobile Application
- **Frontend:** Flutter 3.x (Dart)
- **State Management:** BLoC pattern
- **HTTP Client:** Dio

### Backend Server
- **Framework:** FastAPI (Python)
- **Database:** PostgreSQL (AWS RDS)
- **Authentication:** JWT with bcrypt
- **MQTT Client:** paho-mqtt

### Robot System
- **OS:** Ubuntu 20.04 (Jetson Nano)
- **Framework:** ROS Noetic
- **Navigation:** move_base, AMCL, gmapping
- **Hardware:** Jetson Nano, Arduino/ESP32

### AI Detection
- **Frameworks:** YOLOv8, MediaPipe, OpenCV
- **Models:** 
  - Face Recognition (face_recognition library)
  - Fall Detection (MediaPipe Pose)
  - Fire Detection (YOLOv8 custom trained)

### Cloud Infrastructure
- **Hosting:** AWS EC2
- **Database:** AWS RDS (PostgreSQL)
- **IoT:** AWS IoT Core (MQTT)
- **Storage:** AWS S3

---

## Prerequisites

### Development Environment
- **Python:** 3.9 or higher
- **Flutter:** 3.7.0 or higher
- **ROS:** Noetic (Ubuntu 20.04)
- **Git:** Latest version

### Hardware Requirements
- **Robot Platform:** Jetson Nano or equivalent
- **Sensors:** Camera, IMU (MPU9250), Gas sensor
- **Actuators:** DC motors, Buzzer
- **Network:** WiFi connectivity

### Cloud Requirements
- AWS account with access to:
  - EC2 (backend hosting)
  - RDS (database)
  - IoT Core (MQTT)
  - S3 (storage)

---

## Quick Start

### 1. Clone Repository

```bash
git clone https://github.com/your-org/elderly-surveillance-robot.git
cd elderly-surveillance-robot
```

### 2. Setup Backend

```bash
cd mobile_app/backend

# Create virtual environment
python -m venv venv
source venv/bin/activate  # On Windows: venv\Scripts\activate

# Install dependencies
pip install -r requirements.txt

# Configure environment
cp .env.example .env
# Edit .env with your configuration

# Place AWS IoT certificates in certs/

# Run server
uvicorn app.main:app --reload
```

See [Backend README](mobile_app/backend/README.md) for detailed instructions.

### 3. Setup Frontend

```bash
cd mobile_app/frontend

# Install dependencies
flutter pub get

# Update backend URL in lib/core/network/dio_client.dart

# Run app
flutter run
```

See [Frontend README](mobile_app/frontend/README.md) for detailed instructions.

### 4. Setup Robot (ROS)

```bash
cd ros

# Install ROS dependencies
./install_dependencies.sh

# Build workspace
catkin_make

# Source workspace
source devel/setup.bash

# Launch robot
roslaunch elderly_bot bringup.launch
```

See [ROS README](ros/README.md) for detailed instructions.

### 5. Setup AI Detection

```bash
cd ai

# Install dependencies
pip install -r requirements.txt

# Download model weights (instructions in ai/README.md)

# Run detection pipeline
python src/main.py
```

See [AI README](ai/README.md) for detailed instructions.

---

## System Components

### Mobile Application

**Purpose:** User interface for caregivers to monitor and control the robot

**Features:**
- User authentication
- Real-time telemetry display (battery, temperature)
- Safety status monitoring (gas, fire, fall, stranger)
- Live camera feed
- Remote robot control
- Emergency call functionality
- User profile management

**Documentation:** [mobile_app/frontend/README.md](mobile_app/frontend/README.md)

### Backend Server

**Purpose:** Central hub for data processing and communication

**Responsibilities:**
- User authentication and authorization
- Database management (user data, logs)
- MQTT client for robot communication
- RESTful API for mobile app
- Telemetry data aggregation

**Documentation:** [mobile_app/backend/README.md](mobile_app/backend/README.md)

### Robot System (ROS)

**Purpose:** Autonomous navigation and sensor data collection

**Components:**
- Navigation stack (move_base, AMCL)
- SLAM (gmapping)
- Sensor drivers (camera, IMU, gas)
- Cloud bridge for MQTT communication
- System health monitoring

**Documentation:** [ros/README.md](ros/README.md)

### AI Detection

**Purpose:** Real-time detection of safety hazards

**Capabilities:**
- Face recognition (known vs stranger)
- Fall detection
- Fire and smoke detection
- Gas leak detection

**Documentation:** [ai/README.md](ai/README.md)

---

## Communication Protocols

### Mobile App ↔ Backend
- **Protocol:** HTTPS/REST
- **Authentication:** JWT Bearer tokens
- **Format:** JSON

### Backend ↔ Robot
- **Protocol:** MQTT over TLS (AWS IoT Core)
- **Topic:** `elderly_bot/telemetry`
- **QoS:** 0
- **Format:** JSON

### ROS Internal
- **Protocol:** ROS Topics/Services/Actions
- **Format:** ROS messages

---

## Environment Variables

### Backend (.env)

```bash
DATABASE_URL=postgresql://user:pass@host:5432/db
SECRET_KEY=your-secret-key
ALGORITHM=HS256
AWS_IOT_ENDPOINT=xxx.iot.region.amazonaws.com
AWS_IOT_CLIENT_ID=backend-fastapi
AWS_ROOT_CA=certs/AmazonRootCA1.pem
AWS_CERT_FILE=certs/backend.cert.pem
AWS_PRIVATE_KEY=certs/backend.private.key
```

### Frontend

Update backend URL in `lib/core/network/dio_client.dart`

---

## Security Considerations

### Credentials Management
- **Never commit:**
  - `.env` files
  - AWS certificates
  - Database credentials
  - JWT secret keys

### Authentication
- JWT tokens with expiration
- Bcrypt password hashing
- HTTPS for API communication
- TLS for MQTT connections

### Network Security
- Firewall rules for EC2
- Security groups configuration
- VPC isolation for database
- Certificate-based IoT authentication

---

## Development Workflow

### Local Development

1. Start backend server
2. Start mobile app (update backend URL)
3. (Optional) Start robot simulator or connect to actual robot
4. Test features end-to-end

### Testing

- **Backend:** pytest (coming soon)
- **Frontend:** flutter test
- **ROS:** rostest
- **Integration:** Manual testing with all components

### Deployment

1. Deploy backend to AWS EC2
2. Configure AWS IoT Core
3. Setup AWS RDS database
4. Build mobile app (APK/iOS)
5. Deploy robot on Jetson Nano

---

## Troubleshooting

### Backend Won't Start

- Check Python version (3.9+)
- Verify virtual environment is activated
- Ensure all dependencies installed
- Check `.env` configuration
- Verify database is accessible

### Mobile App Can't Connect

- Verify backend server is running
- Check backend URL configuration
- Ensure phone and backend on same network (development)
- Test backend endpoint with curl/Postman

### Robot Not Responding

- Check ROS master is running
- Verify all nodes are active: `rosnode list`
- Check topic communication: `rostopic echo /topic_name`
- Verify MQTT connection to AWS IoT

---

## Contributing

This is a graduation project. For questions or collaboration:

**Team Members:**
- Maryse Hani - Mobile App & Backend Development
- Mariam Waleed - Mobile App Development
- [Other team members] - ROS, AI, Hardware

---

## License

This project is developed as part of a graduation project for an elderly care monitoring system.

---

## Acknowledgments

- Supervisors and instructors
- Open-source communities (ROS, Flutter, FastAPI)
- AWS for cloud infrastructure
- OpenCV and YOLO communities

---

## Documentation Index

- [Mobile App Frontend](mobile_app/frontend/README.md)
- [Mobile App Backend](mobile_app/backend/README.md)
- [ROS System](ros/README.md)
- [AI Detection](ai/README.md)
- [Cloud Infrastructure](cloud/README.md)
- [Hardware Setup](ros/docs/HARDWARE.md)
- [System Overview](ros/docs/SYSTEM_OVERVIEW.md)

---

For detailed setup instructions, refer to the README in each component directory.