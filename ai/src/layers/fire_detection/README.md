# Fire / Smoke / Person Detection Layer

This layer is responsible for detecting **persons**, **fire**, and **smoke**
using a YOLOv8 object detection model.

It is the **highest-priority layer** in the system.

---

## Purpose

- Detect the presence of:
  - People
  - Fire
  - Smoke
- Trigger immediate alerts for fire or smoke
- Allow downstream processing only when it is safe

---

## Input

- RGB video frame (`numpy.ndarray`, OpenCV format)

---

## Output

The detector returns a dictionary with the following fields:

- `person_detected` (bool)
- `fire_detected` (bool)
- `smoke_detected` (bool)
- `detections` (list of bounding boxes with class and confidence)

---

## Model

- **Framework:** YOLOv8 (Ultralytics)
- **Model file expected at:**

```
assets/models/fire/yolov8_fire_smoke_person.pt
```

The model file is not included in the repository.

---

## Pipeline Behavior

- Fire or smoke detection causes **immediate pipeline termination**
- If a person is detected and no fire/smoke is present, the pipeline continues

---

## Notes

- Designed to run on **CPU by default**
- GPU usage is optional and not required
- No training or fine-tuning is performed at runtime