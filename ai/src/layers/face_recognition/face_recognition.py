import cv2
import face_recognition
import logging
from pathlib import Path
from typing import Dict


class FaceRecognizer:
    """
    Stranger detection using dlib face encodings.

    If no known face encodings are available, all detected faces
    are treated as strangers.
    """

    def __init__(self, config: Dict):
        self.logger = logging.getLogger("ai.layer3")

        self.encodings_path = Path(config["encodings_path"])
        if not self.encodings_path.exists():
            self.logger.warning(
                "Known faces folder not found. All detected faces will be treated as strangers."
            )

        self.tolerance = config.get("tolerance", 0.6)
        self.model = config.get("detection_model", "hog")

        self.known_encodings = []
        self.known_names = []

    def recognize(self, frame):
        rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        locations = face_recognition.face_locations(rgb, model=self.model)
        encodings = face_recognition.face_encodings(rgb, locations)

        stranger = False
        faces = []

        for enc in encodings:
            matches = face_recognition.compare_faces(
                self.known_encodings, enc, self.tolerance
            )
            if not any(matches):
                stranger = True

        return {"stranger_detected": stranger, "faces": faces}

    def draw_faces(self, frame, results):
        return frame
