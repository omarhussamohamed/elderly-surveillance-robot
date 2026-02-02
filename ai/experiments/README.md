# Experiments

This directory contains **research and experimental notebooks** developed during
the design and evaluation of the **Elderly Monitoring AI System**.

⚠️ **Important**

- The notebooks in this folder are **NOT required to run the system**.
- They are provided for **research transparency, validation, and academic review**.
- The runnable AI pipeline is implemented in the `/src` directory.

---

## Purpose of the Experiments

The notebooks in this directory were used to:

- Explore different AI approaches and design alternatives
- Train and evaluate detection models
- Validate assumptions before final system integration
- Compare architectural choices (local vs cloud, complex vs lightweight)

Insights from these experiments informed the **final system design**, but the
experiment code itself is **not deployed**.

---

## Directory Structure

```
experiments/
├── fire_detection/
│   └── fire_smoke_person_yolov8_experiments.ipynb
│
├── fall_detection/
│   └── fall_detection_mediapipe_experiments.ipynb
│
└── face_recognition/
    └── face_recognition_stranger_detection_experiments.ipynb
```

---

## Experiment Summaries

### 🔥 Fire / Smoke / Person Detection

- YOLOv8 training and evaluation
- Dataset merging and class remapping
- Performance analysis on train/validation/test splits
- Resulting weights used **only for inference** in the final system

---

### ⚠️ Fall Detection

- MediaPipe Pose Landmarker experiments
- Analysis of body orientation and motion velocity
- Temporal confirmation strategies
- Findings guided the simplified runtime logic

---

### 👤 Stranger Detection

- Experimental cloud-based face recognition (PostgreSQL / AWS RDS)
- Evaluation of latency, privacy, and dependency trade-offs
- Results motivated the **local, offline approach** used in the final system

---

## Dependencies

Some experiments require additional libraries not used in the runtime system
(e.g., `psycopg2`, training-only YOLO dependencies).

See:

- `requirements-experiments.txt`

---

## Notes

- Experiments may rely on external datasets or cloud services
- Credentials and sensitive data are intentionally omitted
- Results may vary depending on environment and hardware

For running the actual system, refer to:

- Root `README.md`
- `SETUP_GUIDE.md`