# Cloud Infrastructure for Elderly Surveillance Robot

This document describes the **cloud architecture, services, and deployment details** used in the Elderly Surveillance Robot project.

---

## Overview

The cloud layer serves as the backbone of the system, providing secure communication, data storage, real-time messaging, and backend processing. It enables seamless interaction between the robot, mobile application, and backend services.

The cloud infrastructure is hosted on **Amazon Web Services (AWS)** and follows a scalable and secure design.

---

## Cloud Objectives

The cloud system is responsible for:

- Hosting the backend server (Python/FastAPI)
- Managing user data and authentication (JWT-based)
- Handling real-time communication with the robot via MQTT
- Processing and storing sensor data in PostgreSQL
- Storing logs and media files
- Ensuring secure access and data protection

---

## Cloud Architecture

```
┌─────────────────┐                ┌──────────────────────┐
│   Mobile App    │◄──────────────►│      AWS Cloud       │
│   (Flutter)     │   HTTPS/REST   │                      │
└─────────────────┘      JWT       │  ┌────────────────┐  │
                                    │  │  EC2 Instance  │  │
                                    │  │  Ubuntu 22.04  │  │
                                    │  │                │  │
                                    │  │  FastAPI App   │  │
                                    │  │  + MQTT Client │  │
                                    │  └───────┬────────┘  │
                                    │          │           │
                                    │          │           │
                        ┌───────────┼──────────┼───────────┼───────────┐
                        │           │          │           │           │
                        ▼           ▼          ▼           ▼           ▼
              ┌──────────────┐  ┌──────┐  ┌────────┐  ┌────────┐  ┌────────┐
              │ AWS IoT Core │  │  RDS │  │   S3   │  │  IAM   │  │ CloudW │
              │ (MQTT Broker)│  │ (DB) │  │(Files) │  │(Roles) │  │ (Logs) │
              └──────┬───────┘  └──────┘  └────────┘  └────────┘  └────────┘
                     │
                     │ MQTT/TLS
                     ▼
            ┌─────────────────┐
            │   Robot (ROS)   │
            │  + ESP32 WiFi   │
            └─────────────────┘
```

---

## AWS Services Used

### 1. **Amazon EC2 (Elastic Compute Cloud)**

**Purpose:** Hosts the FastAPI backend server

**Instance Type:** t2.micro or t2.small (Ubuntu 22.04 LTS)

**Responsibilities:**
- Running FastAPI application with Uvicorn
- Handling API requests from mobile app
- Processing user authentication (JWT tokens)
- Managing robot commands and telemetry
- Acting as MQTT client subscribed to robot topics

**Installed Software:**
- Python 3.9+
- FastAPI + Uvicorn
- PostgreSQL client libraries
- paho-mqtt (MQTT client)
- Nginx (reverse proxy)
- Certbot (SSL certificates)

**Security Groups:**
- Port 22 (SSH) - restricted to your IP
- Port 80 (HTTP) - redirect to HTTPS
- Port 443 (HTTPS) - API traffic
- Port 8883 (MQTT/TLS) - AWS IoT Core communication

---

### 2. **AWS RDS (Relational Database Service)**

**Database Engine:** PostgreSQL 15

**Purpose:** Stores all application data

**Schema:**
- **Users:** Authentication and profile data
- **SystemHealth:** Battery, temperature, status logs
- **SafetyMonitor:** Fire, gas, fall, stranger detection records

**Security:**
- VPC isolated
- Security group allows only EC2 instance access
- Encrypted at rest
- Automated backups enabled

**Connection from EC2:**
```bash
DATABASE_URL=postgresql://username:password@rds-endpoint:5432/database_name
```

---

### 3. **AWS IoT Core (MQTT Broker)**

**Purpose:** Real-time bidirectional communication between robot and backend

**Communication Flow:**

**Robot → Cloud:**
```
Topic: elderly_bot/telemetry
Payload: {
  "battery": 85,
  "temperature": 42.5,
  "status": "operational",
  "sensors": {...}
}
```

**Cloud → Robot:**
```
Topic: elderly_bot/commands
Payload: {
  "action": "move_forward",
  "speed": 0.3
}
```

**Security:**
- X.509 certificate authentication
- TLS 1.2+ encryption
- Policy-based access control
- Each device has unique certificates

**Why MQTT?**
- Lightweight protocol (minimal bandwidth)
- Low latency (< 100ms)
- QoS levels for reliability
- Publish/Subscribe pattern
- Built-in security with AWS IoT

**Backend Integration:**
- FastAPI app uses paho-mqtt library
- Subscribes to `elderly_bot/telemetry`
- Publishes to `elderly_bot/commands`
- Stores latest telemetry in memory for `/telemetry` API endpoint

---

### 4. **Amazon S3 (Simple Storage Service)**

**Purpose:** Long-term storage for media and logs

**Stored Content:**
- System logs (application logs, error logs)
- Camera snapshots (fall detection events)
- Backup files
- Configuration files

**Bucket Structure:**
```
elderly-surveillance-bucket/
├── logs/
│   ├── 2024-01-15.log
│   └── 2024-01-16.log
├── snapshots/
│   └── fall-events/
└── backups/
```

**Access:**
- IAM role assigned to EC2 for automatic authentication
- Boto3 library for Python integration

---

### 5. **AWS IAM (Identity and Access Management)**

**Purpose:** Secure access control

**Roles Created:**
- **EC2-Backend-Role:** Allows EC2 to access RDS, S3, IoT Core
- **IoT-Device-Role:** Allows robot to publish/subscribe to specific topics

**Policies:**
- Read/write access to specific S3 buckets
- RDS connection permissions
- IoT Core publish/subscribe permissions
- CloudWatch log creation

---

### 6. **AWS CloudWatch (Optional)**

**Purpose:** Monitoring and logging

**Metrics Tracked:**
- EC2 CPU and memory usage
- RDS database connections
- API request rates
- Error rates

**Logs Stored:**
- FastAPI application logs
- MQTT connection logs
- System health logs

---

## Security Measures

The cloud system implements multiple layers of security:

### 1. **HTTPS/TLS Communication**
- All mobile app ↔ backend communication uses HTTPS
- SSL certificate from Let's Encrypt (via Certbot)
- Nginx configured as reverse proxy with SSL

### 2. **MQTT over TLS (Port 8883)**
- All robot ↔ AWS IoT Core communication encrypted
- X.509 certificate authentication
- Each device has unique private key
- Certificates never transmitted over network

### 3. **JWT Authentication**
- Mobile app login returns JWT token
- Token required for all protected endpoints
- Secret key stored in environment variables
- Tokens expire after 7 days

### 4. **Database Security**
- PostgreSQL password authentication
- VPC isolation (not publicly accessible)
- Security group restricts to EC2 IP only
- Connection strings in `.env` (never committed)

### 5. **AWS IAM Roles**
- Principle of least privilege
- No hardcoded AWS credentials
- EC2 instance role for service access
- Separate roles for each component

### 6. **Environment Variables**
- All secrets stored in `.env` file
- Never committed to version control
- `.env.example` template provided
- Loaded via `python-dotenv`

---

## Backend Technology Stack

| Component | Technology | Purpose |
|-----------|-----------|---------|
| Framework | FastAPI | RESTful API server |
| Server | Uvicorn | ASGI server |
| Database ORM | SQLAlchemy | Database models and queries |
| Database | PostgreSQL | Relational data storage |
| Authentication | JWT (python-jose) | Token-based auth |
| Password Hashing | bcrypt | Secure password storage |
| MQTT Client | paho-mqtt | AWS IoT communication |
| Reverse Proxy | Nginx | SSL termination, load balancing |
| Process Manager | systemd | Keep app running |

---

## Deployment Steps

### Prerequisites

1. AWS Account with billing enabled
2. Domain name (optional, for HTTPS)
3. SSH key pair for EC2 access

### Step 1: Setup RDS PostgreSQL

```bash
# 1. Go to AWS RDS Console
# 2. Create Database
# 3. Choose PostgreSQL 15
# 4. Select Free tier (or Production for scaling)
# 5. Set master username and password
# 6. Note down the endpoint URL
# 7. Configure security group to allow EC2 access
```

### Step 2: Setup AWS IoT Core

```bash
# 1. Go to AWS IoT Core Console
# 2. Create a Thing (name: elderly-bot)
# 3. Create and download certificates:
#    - Certificate file (.pem.crt)
#    - Private key (.pem.key)
#    - Root CA certificate
# 4. Attach policy to certificate (allow publish/subscribe)
# 5. Note down IoT endpoint URL
# 6. Store certificates securely
```

### Step 3: Launch EC2 Instance

```bash
# 1. Go to EC2 Console
# 2. Launch Ubuntu 22.04 LTS instance
# 3. Select t2.micro (free tier eligible)
# 4. Configure security group:
#    - Port 22 (SSH)
#    - Port 80 (HTTP)
#    - Port 443 (HTTPS)
# 5. Attach IAM role with S3, RDS, IoT permissions
# 6. Launch and save key pair
```

### Step 4: Connect to EC2 and Install Dependencies

```bash
# SSH into EC2
ssh -i your-key.pem ubuntu@<ec2-public-ip>

# Update system
sudo apt update && sudo apt upgrade -y

# Install Python 3.9+
sudo apt install python3-pip python3-venv -y

# Install Nginx
sudo apt install nginx -y

# Install Git
sudo apt install git -y
```

### Step 5: Deploy Backend Application

```bash
# Clone repository
git clone https://github.com/your-repo/elderly-surveillance-robot.git
cd elderly-surveillance-robot/mobile_app/backend

# Create virtual environment
python3 -m venv venv
source venv/bin/activate

# Install dependencies
pip install -r requirements.txt

# Create .env file
nano .env
```

**Add to .env:**
```bash
DATABASE_URL=postgresql://username:password@rds-endpoint:5432/dbname
SECRET_KEY=your-super-secret-key-minimum-32-characters
ALGORITHM=HS256

AWS_IOT_ENDPOINT=xxxxx-ats.iot.us-east-1.amazonaws.com
AWS_IOT_CLIENT_ID=backend-fastapi
AWS_ROOT_CA=/home/ubuntu/certs/AmazonRootCA1.pem
AWS_CERT_FILE=/home/ubuntu/certs/backend.cert.pem
AWS_PRIVATE_KEY=/home/ubuntu/certs/backend.private.key
```

```bash
# Upload AWS IoT certificates
mkdir -p /home/ubuntu/elderly-surveillance-robot/mobile_app/backend/certs
# Upload your certificate files to this directory using scp:
# scp -i your-key.pem certs/* ubuntu@<ec2-ip>:/home/ubuntu/.../backend/certs/

# Test run
uvicorn app.main:app --host 0.0.0.0 --port 8000
# Visit http://<ec2-public-ip>:8000/docs
```

### Step 6: Configure Nginx Reverse Proxy

```bash
sudo nano /etc/nginx/sites-available/fastapi
```

**Add configuration:**
```nginx
server {
    listen 80;
    server_name your-domain.com;  # or EC2 public IP

    location / {
        proxy_pass http://127.0.0.1:8000;
        proxy_set_header Host $host;
        proxy_set_header X-Real-IP $remote_addr;
        proxy_set_header X-Forwarded-For $proxy_add_x_forwarded_for;
        proxy_set_header X-Forwarded-Proto $scheme;
    }
}
```

```bash
# Enable site
sudo ln -s /etc/nginx/sites-available/fastapi /etc/nginx/sites-enabled/
sudo nginx -t
sudo systemctl restart nginx
```

### Step 7: Setup SSL (Optional but Recommended)

```bash
# Install Certbot
sudo apt install certbot python3-certbot-nginx -y

# Get SSL certificate
sudo certbot --nginx -d your-domain.com

# Auto-renewal is configured automatically
```

### Step 8: Create Systemd Service

```bash
sudo nano /etc/systemd/system/fastapi.service
```

**Add configuration:**
```ini
[Unit]
Description=FastAPI Backend Service
After=network.target

[Service]
User=ubuntu
Group=ubuntu
WorkingDirectory=/home/ubuntu/elderly-surveillance-robot/mobile_app/backend
Environment="PATH=/home/ubuntu/elderly-surveillance-robot/mobile_app/backend/venv/bin"
ExecStart=/home/ubuntu/elderly-surveillance-robot/mobile_app/backend/venv/bin/uvicorn app.main:app --host 0.0.0.0 --port 8000
Restart=always

[Install]
WantedBy=multi-user.target
```

```bash
# Enable and start service
sudo systemctl daemon-reload
sudo systemctl enable fastapi
sudo systemctl start fastapi

# Check status
sudo systemctl status fastapi
```

### Step 9: Configure Database

```bash
# The database tables will be created automatically on first run
# SQLAlchemy's create_all() is called in database.py

# To manually connect and verify:
psql -h <rds-endpoint> -U <username> -d <database>
# Enter password when prompted
# List tables: \dt
```

---

## API Endpoints

The backend exposes the following RESTful API endpoints:

| Method | Endpoint | Description | Auth Required |
|--------|----------|-------------|---------------|
| POST | `/auth/register` | Register new user (phone + password) | No |
| POST | `/auth/login` | Login user (returns JWT token) | No |
| GET | `/health` | Get system health data | Yes (JWT) |
| POST | `/health` | Update system health (battery, temp) | Yes (JWT) |
| GET | `/safety` | Get safety monitor status | Yes (JWT) |
| POST | `/safety` | Update safety status (fire, gas, fall, stranger) | Yes (JWT) |
| GET | `/telemetry` | Get real-time MQTT robot data | No |

**Example Request (Login):**
```bash
curl -X POST "https://your-domain.com/auth/login" \
  -H "Content-Type: application/json" \
  -d '{"phone": "1234567890", "password": "securepass"}'
```

**Example Response:**
```json
{
  "access_token": "eyJhbGciOiJIUzI1NiIsInR5cCI6IkpXVCJ9...",
  "token_type": "bearer"
}
```

---

## MQTT Topics and Payloads

### Robot → Cloud

**Topic:** `elderly_bot/telemetry`

**Payload Example:**
```json
{
  "timestamp": "2024-01-15T10:30:00Z",
  "battery": 85,
  "temperature": 42.5,
  "status": "operational",
  "sensors": {
    "fire": false,
    "gas": false,
    "fall": false,
    "stranger": false
  },
  "location": {
    "x": 2.5,
    "y": 3.2,
    "orientation": 90
  }
}
```

**Backend subscribes to this topic and stores the latest data in memory**

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

### SystemHealth Table

```sql
CREATE TABLE system_health (
    id SERIAL PRIMARY KEY,
    power INTEGER NOT NULL,
    temperature FLOAT NOT NULL,
    created_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP
);
```

### SafetyMonitor Table

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

---

## Monitoring and Maintenance

### Check Application Status

```bash
# Check service status
sudo systemctl status fastapi

# View logs
sudo journalctl -u fastapi -f

# View Nginx logs
sudo tail -f /var/log/nginx/access.log
sudo tail -f /var/log/nginx/error.log
```

### Database Maintenance

```bash
# Connect to database
psql -h <rds-endpoint> -U <username> -d <database>

# View table sizes
SELECT pg_size_pretty(pg_total_relation_size('users'));

# Backup database
pg_dump -h <rds-endpoint> -U <username> -d <database> > backup.sql
```

### Update Application

```bash
# SSH into EC2
cd ~/elderly-surveillance-robot

# Pull latest changes
git pull origin main

# Restart service
sudo systemctl restart fastapi
```

---

## Troubleshooting

### Backend won't start

**Error:** `SECRET_KEY environment variable is not set`
- **Solution:** Ensure `.env` file exists with `SECRET_KEY=...`

**Error:** `Could not connect to database`
- **Solution:** Check `DATABASE_URL` format and RDS security group allows EC2

### MQTT connection failed

**Error:** `Connection refused to AWS IoT`
- **Solution:** Verify certificates are correct and paths in `.env` are absolute
- Check AWS IoT policy allows connect/publish/subscribe
- Verify endpoint URL is correct (include `-ats`)

### API returns 502 Bad Gateway

- **Solution:** Backend service is down
  ```bash
  sudo systemctl status fastapi
  sudo systemctl restart fastapi
  ```

### SSL certificate expired

```bash
# Renew certificate
sudo certbot renew
sudo systemctl reload nginx
```

---

## Cost Optimization

**AWS Free Tier (12 months):**
- EC2 t2.micro: 750 hours/month free
- RDS db.t2.micro: 750 hours/month free
- S3: 5GB storage free
- Data transfer: 15GB/month free

**Production Costs (estimated):**
- EC2 t2.small (24/7): ~$15/month
- RDS db.t2.micro: ~$15/month
- AWS IoT Core: ~$1/month (1M messages)
- S3: ~$1/month (10GB storage)
- **Total: ~$32/month**

---

## Integration with Mobile App

**Mobile App Configuration:**

Update `lib/core/network/dio_client.dart`:

```dart
baseUrl: 'https://your-domain.com'  // or http://<ec2-ip>
```

**Authentication Flow:**

1. User enters phone + password
2. App sends POST to `/auth/login`
3. Backend validates credentials
4. Returns JWT token
5. App stores token (SharedPreferences)
6. All subsequent requests include token in Authorization header

---

## Notes

- Cloud infrastructure is scalable (can upgrade instance types)
- System is designed for real-time performance (< 100ms latency)
- Internet connection required for both mobile app and robot
- AWS credentials must be properly configured (IAM roles preferred over access keys)
- Regular backups are recommended (RDS automated backups enabled)
- Monitor CloudWatch for performance metrics
- Keep Python dependencies updated for security patches

---

## Support

For backend-specific details, see [../mobile_app/backend/README.md](../mobile_app/backend/README.md)

For mobile app integration, see [../mobile_app/README.md](../mobile_app/README.md)

---

## Authors

**Maryse Hani** – Cloud Architecture & Backend Implementation

Project Team – System Design and Supervision