import cv2
import time
import logging
import numpy as np
from typing import Dict, List, Tuple, Optional, Any


class VisionPipeline:
    """Hierarchical vision pipeline for elderly monitoring.
    
    Priority: FIRE > SMOKE > FALL > STRANGER
    """

    def __init__(self, config: Dict[str, Any]) -> None:
        self.logger = logging.getLogger("ai.pipeline")
        self.config = config
        
        # Performance monitoring
        self._frame_count: int = 0
        self._start_time: float = time.time()
        self._fps: float = 0.0
        
        # Initialize layers with error handling
        try:
            from layers.fire_detection.fire_detection import PersonFireDetector
            from layers.fall_detection.fall_detection import FallDetector
            from layers.face_recognition.face_recognition import FaceRecognizer

            self.layer1 = PersonFireDetector(config["layer1"])
            self.layer2 = FallDetector(config["layer2"])
            self.layer3 = FaceRecognizer(config["layer3"])
            
            self.logger.info("All pipeline layers initialized successfully")
        except ImportError as e:
            self.logger.error(f"Failed to import layer module: {e}")
            raise
        except KeyError as e:
            self.logger.error(f"Missing configuration for layer: {e}")
            raise
        except Exception as e:
            self.logger.error(f"Failed to initialize pipeline: {e}")
            raise

    # ─────────────────────────────────────────────
    def process_frame(self, frame: np.ndarray) -> Tuple[np.ndarray, List[str]]:
        """
        Process a single frame through the hierarchical pipeline.
        
        Args:
            frame: BGR image from OpenCV
        
        Returns:
            Tuple of (processed_frame, list_of_alerts)
        """
        alerts: List[str] = []
        processed_frame = frame.copy()

        try:
            # ===== LAYER 1: FIRE/SMOKE/PERSON DETECTION =====
            layer1_results = self.layer1.detect(frame)
            processed_frame = self.layer1.draw_detections(
                processed_frame, layer1_results
            )

            # PRIORITY 1: Fire
            if layer1_results.get("fire_detected"):
                alerts.append("🔥 FIRE DETECTED!")
                self.logger.warning("CRITICAL: Fire detected - stopping pipeline")
                return processed_frame, alerts

            # PRIORITY 2: Smoke
            if layer1_results.get("smoke_detected"):
                alerts.append("💨 SMOKE DETECTED!")
                self.logger.warning("CRITICAL: Smoke detected - stopping pipeline")
                return processed_frame, alerts

            # ===== LAYER 2: FALL DETECTION (only if person detected) =====
            if layer1_results.get("person_detected"):
                try:
                    fall = self.layer2.detect(frame)
                    processed_frame = self.layer2.draw_pose(processed_frame)

                    # PRIORITY 3: Fall
                    if fall:
                        alerts.append("⚠️ FALL DETECTED!")
                        self.logger.warning("CRITICAL: Fall detected - stopping pipeline")
                        return processed_frame, alerts

                except Exception as e:
                    self.logger.error(f"Layer 2 (fall detection) error: {e}")
                    # Continue to face recognition despite fall detection failure

                # ===== LAYER 3: FACE RECOGNITION (only if no fall) =====
                try:
                    face_results = self.layer3.recognize(frame)
                    processed_frame = self.layer3.draw_faces(
                        processed_frame, face_results
                    )

                    # PRIORITY 4: Stranger
                    if face_results.get("stranger_detected"):
                        alerts.append("👤 STRANGER DETECTED!")
                        self.logger.info("Alert: Stranger detected")
                        
                except Exception as e:
                    self.logger.error(f"Layer 3 (face recognition) error: {e}")
                    # Non-critical: system continues without face recognition

        except Exception as e:
            self.logger.error(f"Critical error in process_frame: {e}")
            # Return original frame with error indicator
            alerts.append("❌ SYSTEM ERROR")
            return frame, alerts

        return processed_frame, alerts

    # ─────────────────────────────────────────────
    def run(self, source: int = 0) -> None:
        """
        Run the pipeline on a video source.
        
        Args:
            source: Camera index (default 0) or video file path
        """
        cap: Optional[cv2.VideoCapture] = None
        
        try:
            cap = cv2.VideoCapture(source)
            
            if not cap.isOpened():
                self.logger.error(f"Failed to open video source: {source}")
                raise RuntimeError(f"Cannot open video source: {source}")
            
            self.logger.info(f"Video source opened: {source}")
            self._start_time = time.time()
            self._frame_count = 0

            while True:
                ret, frame = cap.read()
                
                if not ret:
                    self.logger.warning("Failed to read frame - end of stream or error")
                    break

                # Process frame through pipeline
                processed_frame, alerts = self.process_frame(frame)
                
                # Calculate FPS
                self._frame_count += 1
                if self._frame_count % 30 == 0:  # Update every 30 frames
                    elapsed = time.time() - self._start_time
                    self._fps = self._frame_count / elapsed if elapsed > 0 else 0

                # Draw alerts on frame
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
                
                # Draw FPS
                cv2.putText(
                    processed_frame,
                    f"FPS: {self._fps:.1f}",
                    (10, processed_frame.shape[0] - 10),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.5,
                    (0, 255, 0),
                    1,
                )

                cv2.imshow("AI Vision Pipeline", processed_frame)

                key = cv2.waitKey(1) & 0xFF
                if key == ord("q"):
                    self.logger.info("User requested quit")
                    break
                elif key == ord("s"):
                    timestamp = time.strftime("%Y%m%d_%H%M%S")
                    filename = f"capture_{timestamp}.jpg"
                    cv2.imwrite(filename, processed_frame)
                    self.logger.info(f"Saved frame to {filename}")

        except KeyboardInterrupt:
            self.logger.info("Interrupted by user (Ctrl+C)")
        except Exception as e:
            self.logger.error(f"Critical error in pipeline.run(): {e}")
            raise
        finally:
            # Ensure proper cleanup
            if cap is not None:
                cap.release()
                self.logger.info("Video capture released")
            cv2.destroyAllWindows()
            self.logger.info("Pipeline shutdown complete")
