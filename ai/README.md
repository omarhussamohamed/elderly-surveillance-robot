# AI Detection Pipeline

> **Part of:** [Elderly Surveillance Robot](../README.md)

Real-time computer vision pipeline for safety monitoring using hierarchical AI layers.

---

## Overview

The AI module processes video frames through **three hierarchical layers** with strict priority:

```
Frame → Layer 1 (YOLOv8) → Fire/Smoke? → STOP + ALERT
                        → Person? → Layer 2 (MediaPipe)
                                    → Fall? → STOP + ALERT
                                    → Upright? → Layer 3 (Face Recognition)
                                                 → Stranger? → ALERT
                                                 → Known → Continue
```

### Priority Rules
1. 🔥 **Fire/Smoke** → Immediate alert, stop pipeline
2. ⚠️ **Fall** → Immediate alert, stop pipeline
3. 👤 **Stranger** → Alert only (monitoring continues)

---

## Architecture

```
┌─────────────────────────────────────────────────────────────┐
│                      main.py                                 │
│  • Configuration • Environment validation • Signal handling │
└────────────────────┬────────────────────────────────────────┘
                     ▼
┌─────────────────────────────────────────────────────────────┐
│                   pipeline.py                                │
│  • Frame processing • Priority enforcement • FPS monitoring │
└───┬─────────────────┬─────────────────┬──────────────────────┘
    ▼                 ▼                 ▼
┌─────────┐   ┌─────────────┐   ┌──────────────────────┐
│ Layer 1 │   │  Layer 2    │   │     Layer 3          │
│ YOLOv8  │   │ MediaPipe   │   │  Face Recognition    │
│ Fire/   │   │ Pose        │   │  + PostgreSQL RDS    │
│ Smoke   │   │ Fall        │   │  + Local Cache       │
└─────────┘   └─────────────┘   └──────────────────────┘
```

---

## Quick Start

### Requirements
- **Python 3.10** (tested and recommended)
- CPU works by default; GPU optional

### Installation
```bash
cd ai
pip install torch torchvision
pip install ultralytics
pip install -r requirements.txt
```

### Run
```bash
# Optional: Enable cloud face recognition
export DB_PASSWORD="your_postgres_password"  # Linux/macOS
$env:DB_PASSWORD="your_postgres_password"    # Windows PowerShell

python src/main.py
```

### Controls
- `q` — Quit
- `s` — Save frame

---

## Face Recognition (Cloud)

Layer 3 uses PostgreSQL RDS for known face storage with local caching.

### Database Schema
```sql
CREATE TABLE known_faces (
    id SERIAL PRIMARY KEY,
    name TEXT NOT NULL,
    encodings BYTEA NOT NULL,
    created_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP
);
```

### Cache Behavior
1. **Cache exists** → Load instantly
2. **No cache** → Load from PostgreSQL → Save to cache
3. **DB unavailable** → Offline mode (all faces = strangers)

### Clear Cache
```bash
del encodings.joblib  # Windows
rm encodings.joblib   # Linux/macOS
```

---

## Configuration

Configuration is in `main.py`:

```python
config = {
    "layer1": {
        "model_path": "assets/models/fire/yolov8_fire_smoke_person.pt",
        "confidence": 0.5,
        "device": "cpu"  # or "cuda"
    },
    "layer2": {
        "min_detection_confidence": 0.5,
        "min_tracking_confidence": 0.5
    },
    "layer3": {
        "tolerance": 0.45,
        "detection_model": "hog",  # or "cnn" (requires GPU)
        "db": { ... }  # PostgreSQL config
    }
}
```

---

## Performance

| Layer | Model | FPS (CPU) | Memory |
|-------|-------|-----------|--------|
| 1 | YOLOv8n | ~15-20 | ~200MB |
| 2 | MediaPipe | ~25-30 | ~50MB |
| 3 | face_recognition | ~10-15 | ~100MB |
| **Combined** | Full pipeline | **~10-15** | **~350MB** |

**Optimization:** Layers 2-3 skipped when fire/smoke detected.

---

## Folder Structure

```
ai/
├── src/                    # Production code
│   ├── main.py            # Entry point
│   ├── pipeline.py        # Frame processing
│   └── layers/
│       ├── fire_detection/
│       ├── fall_detection/
│       └── face_recognition/
├── experiments/           # Research notebooks (not required)
├── assets/               # Models, known faces (gitignored)
├── requirements.txt      # Runtime dependencies
└── README.md
```

---

## Troubleshooting

| Issue | Solution |
|-------|----------|
| `DB_PASSWORD not set` | Set environment variable or run offline |
| Low FPS | Use YOLOv8n, reduce resolution, use `hog` model |
| No faces detected | Check lighting, face size > 80x80px |
| Model not found | Download model to `assets/models/fire/` |

---

## Known Limitations

1. **Fall Detection:** Placeholder logic (always returns `False`)
2. **Face Recognition:** No re-identification tracking
3. **Single Camera:** No multi-camera support
4. **Alerts:** Display only (no push notifications)

---

## Related Documentation

- [Main README](../README.md) — System overview
- [Cloud Setup](../cloud/README.md) — PostgreSQL RDS configuration
- [Backend](../mobile_app/backend/README.md) — API server

---

## Experiments

The `/experiments` folder contains research notebooks (not required to run):
- Fire detection training
- Fall detection evaluation
- Face recognition experiments

```bash
pip install -r requirements-experiments.txt
```
