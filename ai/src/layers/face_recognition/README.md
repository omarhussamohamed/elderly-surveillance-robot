# Face Recognition / Stranger Detection Layer

This layer performs **stranger detection** using facial encodings
computed with the `face_recognition` (dlib-based) library.

It is the **final layer** in the hierarchical vision pipeline.

---

## Purpose

- Detect human faces in video frames
- Compare detected faces against a local database of known faces
- Raise an alert when an unknown (stranger) face is detected

---

## Input

- RGB video frame (`numpy.ndarray`, OpenCV format)

---

## Output

The detector returns a dictionary with the following fields:

- `stranger_detected` (bool)
- `faces` (list, reserved for future visualization)

---

## Known Faces Database

Known faces are loaded from the following directory:

```
assets/known_faces/
```

Expected structure:

```
assets/known_faces/
└── Person_Name/
    ├── image1.jpg
    └── image2.jpg
```

---

## Important Behavior

⚠️ **If no known face encodings are available**,  
**all detected faces are treated as strangers.**

This behavior is intentional and allows the system
to operate safely even without a predefined database.

---

## Pipeline Behavior

- Executed only when:
  - no fire or smoke is detected
  - no fall is detected
- Stranger detection **does not stop** the pipeline
- Alerts are visual/log-based only

---

## Notes

- Designed for **CPU-only execution**
- Uses HOG-based face detection by default
- No training or learning occurs at runtime
- Face data remains local and is not transmitted