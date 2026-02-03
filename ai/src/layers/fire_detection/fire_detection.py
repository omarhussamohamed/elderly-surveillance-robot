from ultralytics import YOLO
import cv2
import logging
import numpy as np
from pathlib import Path
from typing import Dict, Any, List


class PersonFireDetector:
    """Layer 1: Fire, Smoke, and Person Detection using YOLOv8."""

    CLASS_NAMES: Dict[int, str] = {0: "person", 1: "fire", 2: "smoke"}

    def __init__(self, config: Dict[str, Any]) -> None:
        self.logger = logging.getLogger("ai.fire_detection")
        
        model_path = Path(config["model_path"])
        if not model_path.exists():
            self.logger.error(f"Model file not found: {model_path}")
            raise FileNotFoundError(f"Model not found: {model_path}")
        
        self.logger.info(f"Loading YOLO model from {model_path}")
        self.model = YOLO(str(model_path))
        
        device = config.get("device", "cpu")
        self.model.to(device)
        self.logger.info(f"Model loaded on device: {device}")

        self.confidence: float = config.get("confidence", 0.5)
        self.iou_threshold: float = config.get("iou_threshold", 0.45)
        
        self.logger.info(f"Detector initialized (conf={self.confidence}, iou={self.iou_threshold})")

    def detect(self, frame: np.ndarray) -> Dict[str, Any]:
        """Detect person, fire, and smoke in frame.
        
        Args:
            frame: BGR image from OpenCV
            
        Returns:
            Dictionary with detection results
        """
        try:
            results = self.model.predict(
                frame,
                conf=self.confidence,
                iou=self.iou_threshold,
                verbose=False,
            )[0]

            person = fire = smoke = False
            detections: List[Dict[str, Any]] = []

            for box in results.boxes:
                cls = int(box.cls[0])
                conf = float(box.conf[0])
                name = self.CLASS_NAMES.get(cls, "unknown")
                xyxy = box.xyxy[0].tolist()  # [x1, y1, x2, y2]

                detections.append({
                    "class": name,
                    "confidence": conf,
                    "bbox": xyxy
                })

                if name == "person":
                    person = True
                elif name == "fire":
                    fire = True
                    self.logger.warning("FIRE detected in frame!")
                elif name == "smoke":
                    smoke = True
                    self.logger.warning("SMOKE detected in frame!")

            return {
                "person_detected": person,
                "fire_detected": fire,
                "smoke_detected": smoke,
                "detections": detections,
            }
            
        except Exception as e:
            self.logger.error(f"Error in fire detection: {e}")
            return {
                "person_detected": False,
                "fire_detected": False,
                "smoke_detected": False,
                "detections": [],
            }

    def draw_detections(self, frame: np.ndarray, results: Dict[str, Any]) -> np.ndarray:
        """Draw detection boxes on frame (optional visualization).
        
        Args:
            frame: BGR image from OpenCV
            results: Detection results dictionary
            
        Returns:
            Frame with drawn detections (currently unchanged)
        """
        return frame
