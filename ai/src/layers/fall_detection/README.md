# Fall Detection Layer

This layer detects potential falls using **MediaPipe Pose** landmarks.
It is executed only when a person is detected and no fire or smoke is present.

---

## Purpose

- Monitor human posture across consecutive frames
- Detect fall events based on pose continuity
- Trigger an immediate safety alert when a fall is confirmed

---

## Input

- RGB video frame (`numpy.ndarray`, OpenCV format)

---

## Output

- Returns a boolean value:
  - `True` → fall detected
  - `False` → no fall detected

The detector internally maintains a short confirmation window
to reduce false positives.

---

## Model / Framework

- **Framework:** MediaPipe Pose
- **Runtime model:** Loaded internally by MediaPipe
- **External model file:** Not required by this layer

---

## Pipeline Behavior

- Executed only if:
  - a person is detected
  - no fire or smoke is present
- A confirmed fall causes **immediate pipeline termination**

---

## Notes

- Designed for **CPU-only execution**
- Uses consecutive-frame confirmation for stability
- No training or learning occurs at runtime