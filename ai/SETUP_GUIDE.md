# Setup Guide

This guide explains how to set up the environment and run the
**Elderly Monitoring AI System**.

For system architecture and project overview, see `README.md`.

---

## ✅ Supported Environment

| Component | Requirement |
|-----------|-------------|
| Python    | **3.10 ONLY** (recommended & tested) |
| OS        | Windows / Linux / macOS |
| CPU       | Any modern CPU |
| GPU       | Optional (not required) |

⚠️ Python 3.11 is **not recommended** due to instability with MediaPipe and dlib.

---

## 🔧 Step 1: Create a Virtual Environment (Recommended)

```bash
python -m venv venv
```

Activate it:

**Windows**

```bash
venv\Scripts\activate
```

**Linux / macOS**

```bash
source venv/bin/activate
```

---

## 🔧 Step 2: Install Runtime Dependencies (IMPORTANT ORDER)

⚠️ The installation order matters

```bash
pip install torch torchvision
pip install ultralytics
pip install -r requirements.txt
```

**Notes:**
- This installs a CPU-only setup by default
- Do NOT install torch after ultralytics
- GPU support is optional and not required for evaluation

---

## 🔧 Step 3: Add Required Model Files

### Layer 1 – Fire / Smoke / Person Detection (YOLOv8)

Place your trained YOLOv8 model here:

```
assets/models/fire/
└── yolov8_fire_smoke_person.pt
```

### Layer 2 – Fall Detection (MediaPipe Pose)

Download the MediaPipe pose model:

```bash
wget https://storage.googleapis.com/mediapipe-models/pose_landmarker/pose_landmarker_full/float16/latest/pose_landmarker_full.task
```

Move it to:

```
assets/models/fall/
└── pose_landmarker_full.task
```

### Layer 3 – Face Recognition (Known Faces)

Create the following structure:

```
assets/known_faces/
└── Person_Name/
    ├── image1.jpg
    └── image2.jpg
```

**Notes:**
- Use clear, front-facing photos
- 2–5 images per person recommended
- Face encodings are generated automatically on first run

---

## ▶️ Step 4: Run the System

From the project root directory:

```bash
python src/main.py
```

**Controls:**
- `q` → Quit
- `s` → Save current frame

---

## 🧪 Optional: Using the Experiment Notebooks

Notebooks are located in `/experiments` and are not required to run the system.

To use notebooks:

```bash
pip install -r requirements-experiments.txt
```

Then launch Jupyter:

```bash
jupyter lab
```

---

## 🧪 Common Setup Issues

### MediaPipe Import Error
- Ensure Python 3.10 is used
- Reinstall mediapipe only:

```bash
pip install --force-reinstall mediapipe
```

### dlib Installation Issues (Windows)
- Use a prebuilt wheel if available
- Ensure Visual C++ Build Tools are installed

---

## ✅ Setup Checklist

- Python 3.10 installed
- Virtual environment activated
- Dependencies installed in correct order
- Model files placed correctly
- Run from project root directory

---