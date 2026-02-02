# Elderly Monitoring AI System

A **graduation project / research prototype** for monitoring elderly safety
using a hierarchical computer vision pipeline.

This repository contains both:
- a **runnable AI monitoring system**
- **research notebooks** used during development and evaluation

---

## 🎓 Project Status

**Academic / Graduation Project**  
Not intended as a production or medical system.

---

## 🧠 System Overview

The system processes video frames through **three hierarchical AI layers**:

```
Video Frame
↓
Layer 1: Fire / Smoke / Person Detection (YOLOv8)
↓ (only if person & no fire/smoke)
Layer 2: Fall Detection (MediaPipe Pose)
↓ (only if person is upright)
Layer 3: Stranger Detection (Face Recognition)
```

### Priority Rules
- 🔥 Fire or Smoke → **Immediate alert, stop processing**
- ⚠️ Fall detected → **Immediate alert, stop processing**
- 👤 Stranger detected → **Alert only (monitoring continues)**

---

## 📁 Repository Structure

```
elderly-monitoring-ai/
│
├── src/ # Runnable AI system
│   ├── main.py
│   ├── pipeline.py
│   └── layers/
│
├── experiments/ # Research & validation notebooks
│   ├── fire_detection/
│   ├── fall_detection/
│   └── face_recognition/
│
├── assets/ # Models & local data (gitignored)
│   ├── models/
│   └── known_faces/
│
├── docs/ # Setup & technical documentation
│
├── requirements.txt # Runtime dependencies (stable)
├── requirements-experiments.txt # Notebook dependencies
└── README.md
```

---

## 🚀 Running the System (Runtime)

### Requirements
- **Python 3.10 ONLY** (recommended & tested)
- CPU-only setup works by default

### Installation (IMPORTANT ORDER)

```bash
pip install torch torchvision
pip install ultralytics
pip install -r requirements.txt
```

### Run

```bash
python src/main.py
```

#### Controls:
- `q` → Quit
- `s` → Save current frame

---

## 🧪 Experiments & Notebooks

The `/experiments` folder contains Jupyter notebooks used for:
- model training
- parameter tuning
- validation
- analysis

⚠️ These notebooks are NOT required to run the system.

To use notebooks:

```bash
pip install -r requirements-experiments.txt
```

---

## 📌 Important Notes

- Model weights and face data are not included in the repository.
- Alerts are visual/log-based only.
- No cloud services or automation are assumed.
- Face encodings are stored locally and are irreversible.

---

## 📄 Documentation

Additional documentation is available in `/docs`:
- `SETUP_GUIDE.md`
- `DEPENDENCIES.md`
- `LIMITATIONS.md`
