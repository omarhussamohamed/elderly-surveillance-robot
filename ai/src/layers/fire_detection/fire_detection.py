from ultralytics import YOLO
import cv2
import numpy as np
import logging
from pathlib import Path
from typing import Dict, Optional, List


class PersonFireDetector:
    """YOLOv8 detector for person, fire, and smoke."""

    CLASS_NAMES = {0: "person", 1: "fire", 2: "smoke"}

    COLORS = {
        "person": (0, 255, 0),
        "fire": (0, 0, 255),
        "smoke": (128, 128, 128),
    }

    def __init__(self, config: Dict):
        self.logger = logging.getLogger("ai.layer1")
        model_path = Path(config["model_path"])

        if not model_path.exists():
            raise FileNotFoundError(
                f"YOLOv8 model not found: {model_path}\n"
                "Check docs/SETUP_GUIDE.md for model placement instructions."
            )

        self.model = YOLO(str(model_path))
        self.model.to(config.get("device", "cpu"))

        self.confidence = config.get("confidence", 0.5)
        self.iou_threshold = config.get("iou_threshold", 0.45)

    def detect(self, frame: np.ndarray) -> Dict:
        results = self.model.predict(
            frame,
            conf=self.confidence,
            iou=self.iou_threshold,
            verbose=False,
        )[0]

        person = fire = smoke = False
        detections = []

        for box in results.boxes:
            cls = int(box.cls[0])
            name = self.CLASS_NAMES.get(cls, "unknown")
            conf = float(box.conf[0])
            x1, y1, x2, y2 = map(int, box.xyxy[0])

            detections.append(
                {"class": name, "confidence": conf, "bbox": [x1, y1, x2, y2]}
            )

            if name == "person":
                person = True
            elif name == "fire":
                fire = True
            elif name == "smoke":
                smoke = True

        return {
            "person_detected": person,
            "fire_detected": fire,
            "smoke_detected": smoke,
            "detections": detections,
        }

    def draw_detections(self, frame, results):
        out = frame.copy()
        for d in results["detections"]:
            x1, y1, x2, y2 = d["bbox"]
            color = self.COLORS.get(d["class"], (255, 255, 255))
            cv2.rectangle(out, (x1, y1), (x2, y2), color, 2)
        return out
