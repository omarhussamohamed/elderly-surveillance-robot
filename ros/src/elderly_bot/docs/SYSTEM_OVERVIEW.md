# System Overview (Authoritative)

This document describes the **verified architecture** of the Elderly Bot system.
It reflects the current implementation, not future plans.

---

## System Purpose

Elderly Bot is designed to:
- Build indoor maps
- Monitor environmental safety
- Provide a base for future autonomous navigation

---

## System Phases

### 1. Mapping Phase (Current)

Active components:
- RPLidar → `/scan`
- ESP32 → wheel encoder data
- MPU9250 IMU → `/imu/data_raw`
- EKF (robot_localization) → `odom → base_link`
- GMapping → `map → odom`, `/map`
- Sensors & actuators node
- Foxglove visualization

Navigation stack is **not required** in this phase.

---

### 2. Navigation Phase (Future)

- AMCL localization
- move_base
- Local planners
- Patrol client

These are intentionally disabled during mapping.

---

## TF Tree (Mapping Phase)

map (gmapping)
└── odom (EKF)
└── base_link
├── laser
├── imu_link
└── camera_link


Key points:
- No TF published by ESP32
- No odom/map frames in URDF
- No duplicate TF publishers

---

## Responsibilities

| Component | Responsibility |
|---------|----------------|
URDF | Static geometry only |
ESP32 | Encoder data only |
EKF | Robot motion estimation |
GMapping | SLAM & map generation |
Foxglove | Visualization |

---

## Safety & Stability Rules

- Mapping and localization never run together
- Sensitive hardware logic is not refactored in software
- Cloud services never block robot operation

---

## Status

This architecture is:
- Mapping-validated
- Hardware-consistent
- Reviewer-safe