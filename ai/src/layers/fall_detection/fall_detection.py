import cv2
import numpy as np
import mediapipe as mp
import logging
from typing import Dict


class FallDetector:
    """Fall detection using MediaPipe Pose landmarks."""

    def __init__(self, config: Dict):
        self.logger = logging.getLogger("ai.layer2")

        self.angle_threshold = config.get("angle_threshold", 45)
        self.confidence_threshold = config.get("confidence_threshold", 0.7)

        self.mp_pose = mp.solutions.pose
        self.pose = self.mp_pose.Pose(
            min_detection_confidence=config.get("min_detection_confidence", 0.5),
            min_tracking_confidence=config.get("min_tracking_confidence", 0.5),
        )

        self.fall_counter = 0
        self.confirm_frames = 3
        self.latest_landmarks = None

    def detect(self, frame):
        rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        result = self.pose.process(rgb)

        if not result.pose_landmarks:
            self.fall_counter = 0
            return False

        self.latest_landmarks = result.pose_landmarks
        self.fall_counter += 1

        return self.fall_counter >= self.confirm_frames

    def draw_pose(self, frame):
        if not self.latest_landmarks:
            return frame

        out = frame.copy()
        mp.solutions.drawing_utils.draw_landmarks(
            out, self.latest_landmarks, self.mp_pose.POSE_CONNECTIONS
        )
        return out
