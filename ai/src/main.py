import sys
import logging
import os
import signal
from pathlib import Path
from typing import Dict, Any, NoReturn
import cv2

from pipeline import VisionPipeline


def setup_logging() -> None:
    """Configure logging for the entire application."""
    logging.basicConfig(
        level=logging.INFO,
        format="%(asctime)s | %(levelname)s | %(name)s | %(message)s",
        handlers=[
            logging.StreamHandler(sys.stdout),
        ]
    )


def validate_paths(config: Dict[str, Any]) -> None:
    """Validate that required file paths exist.
    
    Args:
        config: Configuration dictionary
        
    Raises:
        FileNotFoundError: If required files are missing
    """
    logger = logging.getLogger("ai.validation")

    required_paths = [
        config["layer1"]["model_path"]
    ]

    for p in required_paths:
        path = Path(p)
        if not path.exists():
            logger.error(f"Missing required file: {path}")
            logger.error(f"Please ensure YOLOv8 model is downloaded to: {path}")
            raise FileNotFoundError(f"Required file not found: {path}")
    
    logger.info("All required paths validated successfully")


def validate_environment() -> None:
    """Validate environment setup and warn about missing configuration."""
    logger = logging.getLogger("ai.validation")
    
    # Check DB password (optional but recommended)
    if not os.getenv("DB_PASSWORD"):
        logger.warning("DB_PASSWORD environment variable not set")
        logger.warning("Face recognition will run in OFFLINE mode")
        logger.info("To enable cloud face recognition, set DB_PASSWORD:")
        logger.info("  Windows: $env:DB_PASSWORD=\"your_password\"")
        logger.info("  Linux/macOS: export DB_PASSWORD=\"your_password\"")
    else:
        logger.info("DB_PASSWORD configured - cloud face recognition enabled")


def signal_handler(signum: int, frame: Any) -> NoReturn:
    """Handle interrupt signals gracefully."""
    logger = logging.getLogger("ai.main")
    logger.info(f"\nReceived signal {signum}, shutting down gracefully...")
    sys.exit(0)


def main() -> None:
    """Main entry point for the elderly monitoring system."""
    setup_logging()
    logger = logging.getLogger("ai.main")
    
    # Register signal handlers
    signal.signal(signal.SIGINT, signal_handler)
    if hasattr(signal, 'SIGTERM'):
        signal.signal(signal.SIGTERM, signal_handler)
    
    logger.info("="*60)
    logger.info("Elderly Monitoring AI System - Starting...")
    logger.info("="*60)

    try:
        # Validate environment
        validate_environment()
        
        # Configuration
        config: Dict[str, Any] = {
            "layer1": {
                "model_path": "assets/models/fire/yolov8_fire_smoke_person.pt",
                "confidence": 0.5,
                "iou_threshold": 0.45,
                "device": "cpu"
            },

            "layer2": {
                "min_detection_confidence": 0.5,
                "min_tracking_confidence": 0.5
            },

            "layer3": {
                "tolerance": 0.45,
                "detection_model": "hog",

                # ───── CLOUD CONFIG ─────
                "db": {
                    "host": "elderly-care-db.cr66282o8nxf.eu-north-1.rds.amazonaws.com",
                    "database": "elderly-care-db",
                    "user": "postgres",
                    "password": os.getenv("DB_PASSWORD"),
                    "port": "5432"
                }
            }
        }

        # Validate required files exist
        validate_paths(config)

        # Initialize and run pipeline
        logger.info("Initializing vision pipeline...")
        pipeline = VisionPipeline(config)
        
        logger.info("Starting video processing...")
        logger.info("Press 'q' to quit, 's' to save frame")
        pipeline.run(source=0)
        
    except FileNotFoundError as e:
        logger.error(f"Setup error: {e}")
        logger.error("Please check the installation and model files")
        sys.exit(1)
    except ImportError as e:
        logger.error(f"Import error: {e}")
        logger.error("Please ensure all dependencies are installed: pip install -r requirements.txt")
        sys.exit(1)
    except Exception as e:
        logger.error(f"Unexpected error: {e}", exc_info=True)
        sys.exit(1)
    finally:
        logger.info("Shutdown complete")


if __name__ == "__main__":
    main()
