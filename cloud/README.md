# Cloud Infrastructure

**AWS-based backend and communication layer for the Elderly Surveillance Robot**

This document describes the cloud architecture, AWS services, and deployment procedures that enable secure communication between the robot, backend server, and mobile application.

---

## Overview

The cloud infrastructure provides:
- Backend server hosting (FastAPI on EC2)
- User authentication and data storage (PostgreSQL on RDS)
- Real-time robot communication (MQTT via AWS IoT Core)
- Media and log storage (S3)
- System monitoring (CloudWatch)

---

## Architecture

```
┌─────────────┐   HTTPS/REST   ┌──────────────┐   MQTT/TLS   ┌──────────┐
│ Mobile App  │◄──────────────►│  EC2 Server  │◄────────────►│  Robot   │
│  (Flutter)  │    JWT Auth    │   (FastAPI)  │  IoT Core    │  (ROS)   │
└─────────────┘                └──────┬───────┘              └──────────┘
                                      │
                        ┌─────────────┼─────────────┐
                        │             │             │
                        ▼             ▼             ▼
                   ┌────────┐   ┌─────────┐   ┌────────┐
                   │  RDS   │   │   S3    │   │  IAM   │
                   │ (Data) │   │ (Files) │   │(Roles) │
                   └────────┘   └─────────┘   └────────┘
```

---

## AWS Services

### EC2 (Backend Server)
- **Instance:** t2.micro/t2.small, Ubuntu 22.04
- **Stack:** FastAPI, Uvicorn, Nginx, paho-mqtt
- **Ports:** 22 (SSH), 80/443 (HTTP/HTTPS), 8883 (MQTT)
- **Role:** API endpoints, authentication, MQTT client

### RDS (Database)
- **Engine:** PostgreSQL 15
- **Tables:** Users, SystemHealth, SafetyMonitor
- **Security:** VPC isolated, EC2-only access, encrypted
- **Connection:** `postgresql://user:pass@endpoint:5432/db`

### IoT Core (MQTT Broker)
- **Topics:**
  - `elderly_bot/telemetry` (robot → cloud)
  - `elderly_bot/commands` (cloud → robot)
- **Security:** X.509 certificates, TLS 1.2+, policy-based access
- **Why MQTT:** Low latency (<100ms), lightweight, QoS support

### S3 (Storage)
- **Content:** System logs, camera snapshots, backups
- **Access:** IAM role-based (no hardcoded credentials)

### IAM (Access Control)
- **Roles:** EC2-Backend-Role, IoT-Device-Role
- **Policies:** Least-privilege access to RDS, S3, IoT Core

---

## Security

| Layer | Method |
|-------|--------|
| Mobile ↔ Backend | HTTPS with Let's Encrypt SSL |
| Robot ↔ IoT Core | MQTT over TLS (port 8883) with X.509 certificates |
| Authentication | JWT tokens (7-day expiration) |
| Database | VPC isolated, EC2-only security group |
| Credentials | Environment variables (`.env`), never committed |

---

## Technology Stack

| Component | Technology |
|-----------|-----------|
| Framework | FastAPI |
| Server | Uvicorn (ASGI) |
| Database | PostgreSQL + SQLAlchemy |
| Auth | JWT (python-jose) + bcrypt |
| MQTT | paho-mqtt |
| Proxy | Nginx |
| Process Manager | systemd |

---

## Quick Deployment

### 1. Setup RDS PostgreSQL
1. Create PostgreSQL 15 database (free tier)
2. Save endpoint URL and credentials
3. Configure security group for EC2 access

### 2. Setup IoT Core
1. Create Thing: `elderly-bot`
2. Download certificates (.pem.crt, .pem.key, Root CA)
3. Attach policy (publish/subscribe)
4. Note IoT endpoint

### 3. Launch EC2
```bash
# Launch Ubuntu 22.04 t2.micro
# Configure security groups (22, 80, 443)
# Attach IAM role (S3, RDS, IoT permissions)

# SSH and install
ssh -i key.pem ubuntu@<ec2-ip>
sudo apt update && sudo apt upgrade -y
sudo apt install python3-pip python3-venv nginx git -y
```

### 4. Deploy Backend
```bash
git clone <repo-url>
cd elderly-surveillance-robot/mobile_app/backend
python3 -m venv venv
source venv/bin/activate
pip install -r requirements.txt

# Create .env
nano .env
```

**.env configuration:**
```bash
DATABASE_URL=postgresql://user:pass@rds-endpoint:5432/db
SECRET_KEY=your-secret-key-min-32-chars
AWS_IOT_ENDPOINT=xxxxx-ats.iot.region.amazonaws.com
AWS_ROOT_CA=/path/to/AmazonRootCA1.pem
AWS_CERT_FILE=/path/to/cert.pem.crt
AWS_PRIVATE_KEY=/path/to/private.pem.key
```

```bash
# Test run
uvicorn app.main:app --host 0.0.0.0 --port 8000
```

### 5. Configure Nginx + SSL
```bash
sudo nano /etc/nginx/sites-available/fastapi
```

```nginx
server {
    listen 80;
    server_name your-domain.com;
    location / {
        proxy_pass http://127.0.0.1:8000;
        proxy_set_header Host $host;
    }
}
```

```bash
sudo ln -s /etc/nginx/sites-available/fastapi /etc/nginx/sites-enabled/
sudo systemctl restart nginx

# Setup SSL
sudo apt install certbot python3-certbot-nginx -y
sudo certbot --nginx -d your-domain.com
```

### 6. Create Systemd Service
```bash
sudo nano /etc/systemd/system/fastapi.service
```

```ini
[Unit]
Description=FastAPI Backend
After=network.target

[Service]
User=ubuntu
WorkingDirectory=/home/ubuntu/elderly-surveillance-robot/mobile_app/backend
Environment="PATH=/home/ubuntu/elderly-surveillance-robot/mobile_app/backend/venv/bin"
ExecStart=/home/ubuntu/elderly-surveillance-robot/mobile_app/backend/venv/bin/uvicorn app.main:app --host 0.0.0.0 --port 8000
Restart=always

[Install]
WantedBy=multi-user.target
```

```bash
sudo systemctl enable fastapi
sudo systemctl start fastapi
```

---

## API Endpoints

| Method | Endpoint | Description | Auth |
|--------|----------|-------------|------|
| POST | `/auth/register` | Register new user | No |
| POST | `/auth/login` | Login (returns JWT) | No |
| GET | `/health` | System health data | JWT |
| POST | `/health` | Update health status | JWT |
| GET | `/safety` | Safety monitor status | JWT |
| POST | `/safety` | Update safety alerts | JWT |
| GET | `/telemetry` | Real-time MQTT data | No |

**Example:**
```bash
curl -X POST "https://your-domain.com/auth/login" \
  -H "Content-Type: application/json" \
  -d '{"phone":"1234567890","password":"pass123"}'
```

---

## Database Schema

```sql
-- Users
CREATE TABLE users (
    id SERIAL PRIMARY KEY,
    phone VARCHAR(20) UNIQUE NOT NULL,
    hashed_password VARCHAR(255) NOT NULL,
    created_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP
);

-- System Health
CREATE TABLE system_health (
    id SERIAL PRIMARY KEY,
    power INTEGER NOT NULL,
    temperature FLOAT NOT NULL,
    created_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP
);

-- Safety Monitor
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

## Monitoring & Maintenance

```bash
# Check service
sudo systemctl status fastapi
sudo journalctl -u fastapi -f

# View logs
sudo tail -f /var/log/nginx/access.log

# Database backup
pg_dump -h <endpoint> -U <user> -d <db> > backup.sql

# Update application
cd ~/elderly-surveillance-robot
git pull origin main
sudo systemctl restart fastapi
```

---

## Troubleshooting

| Issue | Solution |
|-------|----------|
| `SECRET_KEY not set` | Check `.env` file exists |
| Database connection failed | Verify RDS security group and `DATABASE_URL` |
| MQTT connection refused | Verify certificates and IoT endpoint |
| 502 Bad Gateway | Restart FastAPI: `sudo systemctl restart fastapi` |
| SSL expired | Run: `sudo certbot renew && sudo systemctl reload nginx` |

---

## Cost Estimate

**AWS Free Tier (12 months):**
- EC2 t2.micro: 750 hrs/month
- RDS db.t2.micro: 750 hrs/month
- S3: 5GB, IoT: 250K messages/month

**Production (~$32/month):**
- EC2 t2.small: ~$15
- RDS db.t2.micro: ~$15
- IoT Core: ~$1
- S3: ~$1

---

## Related Documentation

- [Backend README](../mobile_app/backend/README.md) - Detailed backend setup
- [Mobile App README](../mobile_app/README.md) - Frontend integration
- [Main README](../README.md) - Complete system overview

---

**Author:** Maryse Hani – Cloud Architecture & Backend Implementation