# Mobile App for Elderly Surveillance Robot

This folder contains the **Mobile Application** for the Elderly Surveillance Robot project, which includes:

- **grad_project_backend**: Backend server deployed on AWS EC2, handles data storage, notifications, and robot commands.
- **grad_project_frontend**: Flutter mobile app for controlling and monitoring the robot in real-time.

---

## Project Overview

The Elderly Surveillance Robot system is designed to assist in the monitoring and care of elderly people. The mobile app provides remote control, live monitoring, emergency alerts, and user management features to ensure safety and easy interaction.

The system enables:

- Monitoring the elderly remotely  
- Controlling the robot via the mobile app  
- Sending notifications and emergency alerts  
- Managing user profiles and authorized caregivers  

---

## Features

- User authentication (login/signup)
- Real-time robot monitoring
- Remote control functionality
- Notifications
- Manage people (add/remove caregivers or family members)
- Emergency call
- Profile management

---

## Project Structure

Mobile App/
├── grad_project_backend/ # Backend server code
│ ├── src/ # Source code files
│ ├── routes/ # API routes
│ ├── controllers/ # Controllers for backend logic
│ ├── models/ # Database models
│ ├── config/ # Config files (DB, AWS, etc.)
│ └── README.md # Backend-specific instructions
│
├── grad_project_frontend/ # Flutter mobile application
│ ├── lib/ # Dart code for app features
│ ├── assets/ # Images, icons, and other assets
│ ├── android/ # Android platform-specific code
│ ├── ios/ # iOS platform-specific code
│ └── pubspec.yaml # Flutter dependencies
│
└── README.md # This file


---

## How to Run

### Backend

```bash
cd Mobile App/grad_project_backend
npm install
npm start
Confirm the backend is running at the configured URL (e.g., http://localhost:3000).

Frontend
cd Mobile App/grad_project_frontend
flutter pub get
flutter run
Make sure the backend server is running before launching the frontend.



Notes
Compatible with Flutter 3.x or later

Ensure backend is running for the frontend app to function

Internet connection required for real-time monitoring and notifications

Tested on Android and iOS platforms

Authors
Maryse Hani – Mariam Waleed _ Mobile App & Backend development

Project Team – Supervision and design