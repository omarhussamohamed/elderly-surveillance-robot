# Cloud Infrastructure for Elderly Surveillance Robot

This document describes the **cloud architecture, services, and deployment details** used in the Elderly Surveillance Robot project.

---

## Overview

The cloud layer serves as the backbone of the system, providing secure communication, data storage, real-time messaging, and backend processing. It enables seamless interaction between the robot, mobile application, and backend services.

The cloud infrastructure is hosted on **Amazon Web Services (AWS)** and follows a scalable and secure design.

---

## Cloud Objectives

The cloud system is responsible for:

- Hosting the backend server
- Managing user data and authentication
- Handling real-time communication with the robot
- Processing and storing sensor data
- Sending notifications to the mobile application
- Ensuring secure access and data protection

---

## Cloud Architecture

The cloud architecture consists of the following main components:

Mobile App <--> AWS Cloud <--> Robot


More detailed structure:

+---------------------------+
| AWS Cloud |
+---------------------------+
| EC2 -> Backend Server |
| RDS -> Database |
| S3 -> Storage |
| AWS IoT Core -> MQTT |
| SNS -> Notifications |
+---------------------------+


---

## AWS Services Used

### 1. **Amazon EC2 (Elastic Compute Cloud)**

- Hosts the backend server
- Runs the application logic and APIs
- Communicates with the mobile app and robot
- Ubuntu-based instance

**Responsibilities:**
- Handling API requests
- Processing user authentication
- Managing robot commands

---

### 2. **AWS RDS (Relational Database Service)**

- Stores user data, profiles, and system logs
- Secure and scalable database
- Used by the backend to retrieve and store information

---

### 3. **AWS IoT Core (MQTT Communication)**

- Used for real-time communication between:
  - Robot ↔ Cloud
  - Cloud ↔ Mobile App

**Why MQTT?**
- Lightweight and efficient
- Low latency
- Suitable for IoT devices
- Secure via TLS encryption

---

### 4. **Amazon S3 (Storage)**

Used for:
- Storing images, logs, or media files
- Saving system backups
- Keeping cloud assets

---

### 5. **AWS SNS (Simple Notification Service)**

Used to:
- Send alerts and notifications to the mobile application
- Trigger emergency messages
- Notify caregivers in critical situations

---

## Security Measures

The cloud system implements:

- Secure HTTPS communication
- Encrypted MQTT over TLS
- AWS IAM roles and permissions
- Restricted access to EC2 instance
- Secure database credentials

---

## Deployment Steps (Summary)

### Backend Deployment on EC2

1. Launch Ubuntu EC2 instance
2. Install required dependencies
3. Clone backend repository:
```bash
git clone <backend-repo-url>
Install packages:

npm install
Start server:

npm start
Integration with Mobile App
Mobile app sends requests to AWS EC2 backend

Backend processes requests and communicates with the robot via AWS IoT Core

Data is stored in RDS and S3 when needed

Notes
Cloud infrastructure is scalable

System is designed for real-time performance

Internet connection is required

AWS credentials must be properly configured

Authors
Maryse Hani – Cloud & Backend Implementation

Project Team – System Design and Supervision