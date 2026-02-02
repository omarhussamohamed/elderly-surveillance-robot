import sys
import logging
from pathlib import Path

# Defensive import for OpenCV
try:
    import cv2
except ImportError as e:
    raise ImportError(
        "Failed to import OpenCV (cv2). Ensure that OpenCV is installed.\n"
        "→ Run 'pip install opencv-python' to install it."
    ) from e

# Defensive import for VisionPipeline
try:
    from pipeline import VisionPipeline
except ImportError as e:
    raise ImportError(
        "Failed to import VisionPipeline. Ensure that the 'pipeline' module is present and correctly installed.\n"
        "→ Check the README.md for setup instructions."
    ) from e


def setup_logging():
    try:
        logging.basicConfig(
            level=logging.INFO,
            format="%(asctime)s | %(levelname)s | %(name)s | %(message)s"
        )
    except Exception as e:
        raise RuntimeError(
            "Failed to set up logging. Ensure the logging configuration is correct."
        ) from e


def validate_paths(config: dict):
    """Fail early if required model files are missing."""
    logger = logging.getLogger("ai.validation")
    required_paths = [
        config["layer1"]["model_path"],
        config["layer2"]["model_path"],
        config["layer3"]["encodings_path"],
    ]

    for p in required_paths:
        path = Path(p)
        if not path.exists():
            logger.error(f"Missing required file/folder: {path}")
            raise FileNotFoundError(
                f"Required file/folder not found: {path}\n"
                f"→ Check README.md for setup instructions."
            )


def main():
    setup_logging()
    logger = logging.getLogger("ai.main")

    # Validate Python version
    if not (sys.version_info.major == 3 and sys.version_info.minor == 10):
        raise RuntimeError("This project requires Python 3.10. Please install the correct version and try again.")

    config = {
        "layer1": {
            "model_path": "assets/models/fire/yolov8_fire_smoke_person.pt",
            "confidence": 0.5,
            "iou_threshold": 0.45,
            "device": "cpu"
        },
        "layer2": {
            "model_path": "assets/models/fall/pose_landmarker_full.task",
            "confidence_threshold": 0.7,
            "angle_threshold": 45,
            "min_detection_confidence": 0.5,
            "min_tracking_confidence": 0.5
        },
        "layer3": {
            "encodings_path": "assets/known_faces",
            "tolerance": 0.6,
            "detection_model": "hog",
            "num_jitters": 1
        }
    }

    validate_paths(config)

    pipeline = VisionPipeline(config)
    logger.info("Pipeline started. Press 'q' to quit.")
    pipeline.run(source=0)


if __name__ == "__main__":
    main()
