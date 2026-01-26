#!/usr/bin/env python2
"""
ROS Camera Publisher Node for ElderlyBot - GRACEFUL VERSION
"""

import rospy
import cv2
from sensor_msgs.msg import Image, CompressedImage
from cv_bridge import CvBridge
import sys
import time

class CameraPublisher:
    def __init__(self):
        rospy.init_node('camera_publisher', anonymous=False)
        
        # Parameters
        self.device = rospy.get_param('~device', '/dev/video0')
        self.width = rospy.get_param('~width', 1280)
        self.height = rospy.get_param('~height', 720)
        self.fps = rospy.get_param('~fps', 30)
        self.frame_id = rospy.get_param('~frame_id', 'camera_link')
        
        # Publishers
        self.image_pub = rospy.Publisher('/camera/image_raw', Image, queue_size=2)
        self.compressed_pub = rospy.Publisher('/camera/image_raw/compressed', CompressedImage, queue_size=2)
        
        self.bridge = CvBridge()
        self.cap = None
        
        # Try to open camera
        if not self._try_open_camera():
            rospy.logwarn("⚠️ Camera not found at %s. Will retry periodically.", self.device)
            rospy.loginfo("To enable camera, connect USB camera and check: ls /dev/video*")
            # Don't exit - just continue and retry
        
        self.rate = rospy.Rate(self.fps)
        self.frame_count = 0
        self.start_time = time.time()
        
    def _try_open_camera(self):
        """Try to open camera, return True if successful."""
        try:
            # Release previous capture if exists
            if self.cap is not None:
                self.cap.release()
                self.cap = None
            
            # Try to open
            self.cap = cv2.VideoCapture(self.device, cv2.CAP_V4L2)
            if not self.cap.isOpened():
                return False
            
            # Set camera properties
            fourcc = cv2.VideoWriter_fourcc('M','J','P','G')
            self.cap.set(cv2.CAP_PROP_FOURCC, fourcc)
            self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.width)
            self.cap.set(cvray.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)
            self.cap.set(cv2.CAP_PROP_FPS, self.fps)
            self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 2)
            
            # Verify
            actual_width = int(self.cap.get(cv2.CAP_PROP_FRAME_WIDTH))
            actual_height = int(self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
            
            rospy.loginfo("✅ Camera opened: %s", self.device)
            rospy.loginfo("Resolution: %dx%d @ %dfps", actual_width, actual_height, self.fps)
            return True
            
        except Exception as e:
            rospy.logdebug("Camera open failed: %s", str(e))
            return False
    
    def run(self):
        """Main loop with camera retry logic."""
        last_retry_time = 0
        retry_interval = 10.0  # Retry every 10 seconds
        
        while not rospy.is_shutdown():
            current_time = time.time()
            
            # If camera is not open, try to open it periodically
            if self.cap is None or not self.cap.isOpened():
                if current_time - last_retry_time > retry_interval:
                    rospy.loginfo("🔄 Attempting to open camera...")
                    if self._try_open_camera():
                        rospy.loginfo("✅ Camera reconnected!")
                    else:
                        rospy.loginfo("📷 Camera not available. Next retry in %.0f seconds.", retry_interval)
                    last_retry_time = current_time
                
                # Sleep briefly and continue
                rospy.sleep(1.0)
                continue
            
            # Camera is open, try to read frame
            try:
                ret, frame = self.cap.read()
                
                if not ret:
                    rospy.logwarn("Frame grab failed. Camera might be disconnected.")
                    self.cap.release()
                    self.cap = None
                    continue
                
                self.frame_count += 1
                
                # Log FPS every 30 seconds
                if self.frame_count % (self.fps * 30) == 0:
                    elapsed = time.time() - self.start_time
                    actual_fps = self.frame_count / elapsed
                    rospy.loginfo("📷 Camera FPS: %.1f", actual_fps)
                
                # Create ROS Image message
                img_msg = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")
                img_msg.header.stamp = rospy.Time.now()
                img_msg.header.frame_id = self.frame_id
                self.image_pub.publish(img_msg)
                
                # Publish compressed version
                compressed_msg = CompressedImage()
                compressed_msg.header = img_msg.header
                compressed_msg.format = "jpeg"
                compressed_msg.data = cv2.imencode('.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, 70])[1].tobytes()
                self.compressed_pub.publish(compressed_msg)
                
            except Exception as e:
                rospy.logwarn("Camera error: %s", str(e))
                if self.cap is not None:
                    self.cap.release()
                    self.cap = None
            
            self.rate.sleep()
    
    def shutdown(self):
        """Clean shutdown"""
        rospy.loginfo("Shutting down camera node")
        if self.cap is not None:
            self.cap.release()
        cv2.destroyAllWindows()

if __name__ == '__main__':
    try:
        node = CameraPublisher()
        rospy.on_shutdown(node.shutdown)
        node.run()
    except rospy.ROSInterruptException:
        pass
    except Exception as e:
        rospy.logerr("Camera node error: %s", str(e))