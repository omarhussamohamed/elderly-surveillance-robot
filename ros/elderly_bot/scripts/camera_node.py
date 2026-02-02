#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""
ROS Camera Publisher Node for ElderlyBot
- Publishes raw and compressed images
- Gracefully handles camera disconnects
"""

import rospy
import cv2
import time
from sensor_msgs.msg import Image, CompressedImage
from cv_bridge import CvBridge


class CameraPublisher(object):
    def __init__(self):
        rospy.init_node('camera_node', anonymous=False)

        # ── Parameters ─────────────────────────────────────────────
        self.device = rospy.get_param('~device', '/dev/video0')
        self.width = rospy.get_param('~width', 1280)
        self.height = rospy.get_param('~height', 720)
        self.fps = rospy.get_param('~fps', 30)
        self.frame_id = rospy.get_param('~frame_id', 'camera_link')
        self.jpeg_quality = rospy.get_param('~jpeg_quality', 70)
        self.retry_interval = rospy.get_param('~retry_interval', 10.0)

        # ── Publishers ─────────────────────────────────────────────
        self.image_pub = rospy.Publisher(
            '/camera/image_raw', Image, queue_size=2
        )
        self.compressed_pub = rospy.Publisher(
            '/camera/image_raw/compressed', CompressedImage, queue_size=2
        )

        self.bridge = CvBridge()
        self.cap = None

        self.frame_count = 0
        self.start_time = time.time()
        self.last_retry_time = 0.0

        rospy.loginfo("Camera node configured: %s %dx%d @ %dfps",
                      self.device, self.width, self.height, self.fps)

        self._try_open_camera()

    # ──────────────────────────────────────────────────────────────
    def _try_open_camera(self):
        """Attempt to open the camera device."""
        if self.cap:
            self.cap.release()
            self.cap = None

        self.cap = cv2.VideoCapture(self.device, cv2.CAP_V4L2)
        if not self.cap.isOpened():
            self.cap = None
            return False

        # Camera properties
        fourcc = cv2.VideoWriter_fourcc('M', 'J', 'P', 'G')
        self.cap.set(cv2.CAP_PROP_FOURCC, fourcc)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.width)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)
        self.cap.set(cv2.CAP_PROP_FPS, self.fps)
        self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 2)

        actual_w = int(self.cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        actual_h = int(self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT))

        rospy.loginfo("Camera opened: %s (%dx%d)", self.device, actual_w, actual_h)
        return True

    # ──────────────────────────────────────────────────────────────
    def _publish_frame(self, frame):
        """Publish raw and compressed frames."""
        stamp = rospy.Time.now()

        img_msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
        img_msg.header.stamp = stamp
        img_msg.header.frame_id = self.frame_id
        self.image_pub.publish(img_msg)

        compressed = CompressedImage()
        compressed.header = img_msg.header
        compressed.format = "jpeg"
        compressed.data = cv2.imencode(
            '.jpg', frame,
            [cv2.IMWRITE_JPEG_QUALITY, self.jpeg_quality]
        )[1].tobytes()
        self.compressed_pub.publish(compressed)

    # ──────────────────────────────────────────────────────────────
    def run(self):
        rate = rospy.Rate(self.fps)

        while not rospy.is_shutdown():
            if not self.cap:
                now = time.time()
                if now - self.last_retry_time >= self.retry_interval:
                    rospy.logwarn("Camera not available, retrying...")
                    if self._try_open_camera():
                        rospy.loginfo("Camera reconnected")
                    self.last_retry_time = now

                rospy.sleep(1.0)
                continue

            ret, frame = self.cap.read()
            if not ret:
                rospy.logwarn("Camera frame grab failed")
                self.cap.release()
                self.cap = None
                continue

            self.frame_count += 1
            if self.frame_count % (self.fps * 30) == 0:
                elapsed = time.time() - self.start_time
                if elapsed > 0:
                    rospy.loginfo("Camera FPS: %.1f",
                                  self.frame_count / elapsed)

            try:
                self._publish_frame(frame)
            except Exception as e:
                rospy.logwarn("Publish error: %s", str(e))

            rate.sleep()

    # ──────────────────────────────────────────────────────────────
    def shutdown(self):
        rospy.loginfo("Shutting down camera node")
        if self.cap:
            self.cap.release()
        cv2.destroyAllWindows()


if __name__ == '__main__':
    try:
        node = CameraPublisher()
        rospy.on_shutdown(node.shutdown)
        node.run()
    except rospy.ROSInterruptException:
        pass
