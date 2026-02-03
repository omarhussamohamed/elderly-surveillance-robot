import cv2
import face_recognition
import psycopg2
from psycopg2 import OperationalError, DatabaseError
import joblib
import pickle
import logging
import numpy as np
from pathlib import Path
from typing import Dict, List, Optional, Any, Tuple

CACHE_FILE = "encodings.joblib"


class FaceRecognizer:
    """
    Face Recognition Layer (Layer 3)
    
    Loads known face encodings from PostgreSQL with local caching.
    Falls back to treating all faces as strangers if DB unavailable.
    """

    def __init__(self, config: Dict[str, Any]) -> None:
        self.logger = logging.getLogger("ai.face_recognition")
        self.db_config: Dict[str, str] = config["db"]
        self.tolerance: float = config.get("tolerance", 0.45)
        self.model: str = config.get("detection_model", "hog")

        self.known_encodings: List[np.ndarray] = []
        self.known_names: List[str] = []
        self.db_available: bool = False
        self._connection_timeout: int = 5  # seconds

        # Validate detection model
        if self.model not in ["hog", "cnn"]:
            self.logger.warning(f"Invalid detection model '{self.model}', defaulting to 'hog'")
            self.model = "hog"

        self._load_encodings()

    # ─────────────────────────────────────────────
    def _load_encodings(self) -> None:
        """
        Load face encodings with priority:
        1. From local cache (if exists)
        2. From PostgreSQL RDS (if available)
        3. Fallback: empty list (all faces are strangers)
        """
        
        # Try loading from cache first
        if Path(CACHE_FILE).exists():
            try:
                data = joblib.load(CACHE_FILE)
                
                # Validate cache structure
                if not isinstance(data, dict) or "encodings" not in data or "names" not in data:
                    self.logger.warning("Invalid cache format, deleting and trying DB...")
                    Path(CACHE_FILE).unlink()
                else:
                    self.known_encodings = data["encodings"]
                    self.known_names = data["names"]
                    self.logger.info(f"✓ Loaded {len(self.known_names)} faces from cache")
                    self.db_available = True  # Cache implies previous DB success
                    return
            except (EOFError, pickle.UnpicklingError) as e:
                self.logger.warning(f"Corrupted cache file: {e}. Deleting and trying DB...")
                Path(CACHE_FILE).unlink()
            except Exception as e:
                self.logger.warning(f"Cache load failed: {e}. Trying DB...")

        # Try loading from PostgreSQL
        conn = None
        try:
            self.logger.info("Connecting to PostgreSQL RDS...")
            
            # Add connection timeout
            db_config_with_timeout = self.db_config.copy()
            db_config_with_timeout["connect_timeout"] = self._connection_timeout
            
            conn = psycopg2.connect(**db_config_with_timeout)
            
            with conn.cursor() as cur:
                # SQL is safe - no user input, using parameterized query structure
                cur.execute("SELECT name, encodings FROM known_faces ORDER BY name;")
                rows = cur.fetchall()

                for name, blob in rows:
                    if not blob:
                        self.logger.warning(f"Empty encoding for {name}, skipping")
                        continue
                    try:
                        encoding = pickle.loads(blob)
                        self.known_names.append(name)
                        self.known_encodings.append(encoding)
                    except (pickle.UnpicklingError, ValueError) as e:
                        self.logger.warning(f"Failed to deserialize encoding for {name}: {e}")

            # Save to cache for next time
            if self.known_encodings:
                joblib.dump({
                    "encodings": self.known_encodings,
                    "names": self.known_names
                }, CACHE_FILE)
                self.logger.info(f"✓ Loaded {len(self.known_names)} faces from PostgreSQL")
                self.db_available = True
            else:
                self.logger.warning("DB connected but no known faces found")
                self.db_available = True

        except OperationalError as e:
            self.logger.error(f"PostgreSQL connection failed (network/timeout): {e}")
            self.logger.warning("⚠️ Running in OFFLINE mode - all faces treated as strangers")
            self.db_available = False
        except DatabaseError as e:
            self.logger.error(f"PostgreSQL database error: {e}")
            self.logger.warning("⚠️ Running in OFFLINE mode - all faces treated as strangers")
            self.db_available = False
        except Exception as e:
            self.logger.error(f"Unexpected error loading encodings: {e}")
            self.logger.warning("⚠️ Running in OFFLINE mode - all faces treated as strangers")
            self.db_available = False
        finally:
            if conn:
                conn.close()

    # ─────────────────────────────────────────────
    def refresh_encodings(self) -> None:
        """
        Force refresh from PostgreSQL, bypassing cache.
        Useful after adding new faces to the database.
        """
        self.logger.info("Refreshing encodings from PostgreSQL...")
        
        # Clear cache
        try:
            if Path(CACHE_FILE).exists():
                Path(CACHE_FILE).unlink()
                self.logger.info("✓ Cache cleared")
        except OSError as e:
            self.logger.warning(f"Failed to delete cache file: {e}")

        # Clear current data
        self.known_encodings = []
        self.known_names = []

        # Reload from DB
        self._load_encodings()

    # ─────────────────────────────────────────────
    def recognize(self, frame: np.ndarray) -> Dict[str, bool]:
        """
        Detect faces and identify strangers.
        
        Args:
            frame: BGR image frame from OpenCV
        
        Returns:
            {"stranger_detected": bool}
        """
        try:
            # Validate input
            if frame is None or frame.size == 0:
                self.logger.warning("Empty frame received in recognize()")
                return {"stranger_detected": False}
            
            rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)

            # Detect faces
            locations = face_recognition.face_locations(rgb, model=self.model)
            
            # No faces detected - not a stranger alert situation
            if not locations:
                return {"stranger_detected": False}
            
            encodings = face_recognition.face_encodings(rgb, locations)

            # If no known faces loaded, treat all detected faces as strangers
            if not self.known_encodings:
                return {"stranger_detected": True}

            # Compare each detected face
            for enc in encodings:
                matches = face_recognition.compare_faces(
                    self.known_encodings, enc, self.tolerance
                )

                if not any(matches):
                    return {"stranger_detected": True}

            return {"stranger_detected": False}
            
        except cv2.error as e:
            self.logger.error(f"OpenCV error in face recognition: {e}")
            return {"stranger_detected": False}
        except Exception as e:
            self.logger.error(f"Unexpected error in recognize(): {e}")
            return {"stranger_detected": False}

    # ─────────────────────────────────────────────
    def draw_faces(self, frame: np.ndarray, results: Dict[str, bool]) -> np.ndarray:
        """
        Draw face detection boxes (optional visualization).
        Currently returns frame unchanged.
        
        Args:
            frame: BGR image frame from OpenCV
            results: Recognition results dictionary
        
        Returns:
            Frame with drawn annotations (currently unchanged)
        """
        return frame
