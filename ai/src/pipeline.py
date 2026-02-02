import cv2
import time
import logging
from typing import Dict


class VisionPipeline:
    """
    Hierarchical 3-layer AI vision pipeline.

    Layer 1 (PRIORITY): Fire / Smoke / Person Detection
    Layer 2 (SAFETY):   Fall Detection
    Layer 3 (SECURITY): Stranger Detection

    IMPORTANT:
    - Each layer runs ONLY if the previous layer allows it
    - Fire or fall immediately stop further processing
    """

    def __init__(self, config: Dict):
        self.logger = logging.getLogger("ai.pipeline")
        self.config = config

        # Lazy imports to avoid crashing if dependencies are missing
        try:
            from layers.fire_detection.fire_detector import PersonFireDetector
            from layers.fall_detection.fall_detector import FallDetector
            from layers.face_recognition.face_recognizer import FaceRecognizer
        except ImportError as e:
            raise ImportError(
                "Dependency import failed.\n"
                "Ensure that the 'layers' modules are present and correctly structured.\n"
                "→ Check README.md for setup instructions.\n"
                f"Original error: {e}"
            )


        self.logger.info("Initializing Layer 1 (Fire / Smoke / Person)")
        self.layer1 = PersonFireDetector(config["layer1"])

        self.logger.info("Initializing Layer 2 (Fall Detection)")
        self.layer2 = FallDetector(config["layer2"])

        self.logger.info("Initializing Layer 3 (Face Recognition)")
        self.layer3 = FaceRecognizer(config["layer3"])

        self.logger.info("All layers initialized successfully")

    def process_frame(self, frame):
        """
        Process a single frame through the hierarchical pipeline.

        Returns:
            processed_frame (np.ndarray)
            alerts (list[str]) – human-readable alert messages
        """
        alerts = []
        processed_frame = frame.copy()

        # ---------- LAYER 1 ----------
        layer1_results = self.layer1.detect(frame)
        processed_frame = self.layer1.draw_detections(processed_frame, layer1_results)

        if layer1_results.get("fire_detected"):
            alerts.append("🔥 FIRE DETECTED!")
            return processed_frame, alerts

        if layer1_results.get("smoke_detected"):
            alerts.append("💨 SMOKE DETECTED!")
            return processed_frame, alerts

        # ---------- LAYER 2 ----------
        if layer1_results.get("person_detected"):
            fall_detected = self.layer2.detect(frame)
            processed_frame = self.layer2.draw_pose(processed_frame)

            if fall_detected:
                alerts.append("⚠️ FALL DETECTED!")
                return processed_frame, alerts

            # ---------- LAYER 3 ----------
            face_results = self.layer3.recognize(frame)
            processed_frame = self.layer3.draw_faces(processed_frame, face_results)

            if face_results.get("stranger_detected"):
                alerts.append("👤 STRANGER DETECTED!")

        return processed_frame, alerts

    def run(self, source=0):
        """Run pipeline on webcam or video file."""
        cap = cv2.VideoCapture(source)
        if not cap.isOpened():
            raise ValueError(f"Cannot open video source: {source}")

        frame_count = 0
        start_time = time.time()

        try:
            while True:
                ret, frame = cap.read()
                if not ret:
                    break

                frame_count += 1
                processed_frame, alerts = self.process_frame(frame)

                for i, alert in enumerate(alerts):
                    cv2.putText(
                        processed_frame,
                        alert,
                        (10, 30 + i * 35),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.8,
                        (0, 0, 255),
                        2,
                    )

                fps = frame_count / max(time.time() - start_time, 1e-6)
                cv2.putText(
                    processed_frame,
                    f"FPS: {fps:.1f}",
                    (processed_frame.shape[1] - 140, 30),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.7,
                    (0, 255, 0),
                    2,
                )

                cv2.imshow("AI Vision Pipeline", processed_frame)

                if cv2.waitKey(1) & 0xFF == ord("q"):
                    break
        finally:
            cap.release()
            cv2.destroyAllWindows()
