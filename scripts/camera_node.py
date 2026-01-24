#!/usr/bin/env python2
"""
ROS Camera Publisher Node for ElderlyBot
Publishes MJPEG camera feed at 1280x720 @ 30fps
Compatible with ROS Melodic (Python 2.7)
OPTIMIZED: Uses MJPEG format for better Jetson Nano performance
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
        
        # Parameters - SET TO HIGH RESOLUTION
        self.device = rospy.get_param('~device', '/dev/video0')
        self.width = rospy.get_param('~width', 1280)
        self.height = rospy.get_param('~height', 720)
        self.fps = rospy.get_param('~fps', 30)
        self.frame_id = rospy.get_param('~frame_id', 'camera_link')
        
        # Publishers
        self.image_pub = rospy.Publisher('/camera/image_raw', Image, queue_size=2)  # Small buffer for latency
        self.compressed_pub = rospy.Publisher('/camera/image_raw/compressed', CompressedImage, queue_size=2)
        
        self.bridge = CvBridge()
        
        # Open camera
        self.cap = cv2.VideoCapture(self.device, cv2.CAP_V4L2)  # V4L2 backend for better performance
        if not self.cap.isOpened():
            rospy.logerr("Failed to open camera: {}".format(self.device))
            sys.exit(1)
        
        # Set camera properties - MJPEG format for best performance on Jetson
        # Try MJPEG first, fall back to YUYV
        fourcc = cv2.VideoWriter_fourcc('M','J','P','G')
        self.cap.set(cv2.CAP_PROP_FOURCC, fourcc)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.width)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)
        self.cap.set(cv2.CAP_PROP_FPS, self.fps)
        
        # Additional performance tweaks
        self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 2)  # Reduce buffer to minimize latency
        try:
            self.cap.set(cv2.CAP_PROP_AUTOFOCUS, 0)   # Disable autofocus for consistency
        except Exception as e:
            rospy.logwarn("Failed to disable autofocus: {}".format(e))
        
        # Verify settings
        actual_width = int(self.cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        actual_height = int(self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
        actual_fps = int(self.cap.get(cv2.CAP_PROP_FPS))
        actual_fourcc = int(self.cap.get(cv2.CAP_PROP_FOURCC))
        
        # Convert fourcc code to string
        fourcc_str = ''.join([chr((actual_fourcc >> 8*i) & 0xFF) for i in range(4)])
        
        rospy.loginfo("Camera opened: {}".format(self.device))
        rospy.loginfo("Resolution: {}x{} @ {}fps".format(actual_width, actual_height, actual_fps))
        rospy.loginfo("Video format: {}".format(fourcc_str))
        
        # If actual resolution doesn't match, warn user
        if actual_width != self.width or actual_height != self.height:
            rospy.logwarn("Camera using {}x{} instead of requested {}x{}".format(
                actual_width, actual_height, self.width, self.height))
            self.width = actual_width
            self.height = actual_height
            
        self.rate = rospy.Rate(self.fps)
        self.frame_count = 0
        self.start_time = time.time()
        
    def run(self):
        """Main loop with performance monitoring"""
        while not rospy.is_shutdown():
            ret, frame = self.cap.read()
            
            if not ret:
                rospy.logwarn("Failed to grab frame")
                # Try to reinitialize camera
                self.cap.release()
                self.cap = cv2.VideoCapture(self.device)
                continue
            
            self.frame_count += 1
            
            # Log FPS every 5 seconds
            if self.frame_count % (self.fps * 5) == 0:
                elapsed = time.time() - self.start_time
                actual_fps = self.frame_count / elapsed
                rospy.loginfo("Camera running at {:.1f} FPS".format(actual_fps))
            
            # Create ROS Image message
            try:
                img_msg = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")
                img_msg.header.stamp = rospy.Time.now()
                img_msg.header.frame_id = self.frame_id
                self.image_pub.publish(img_msg)
                
                # Publish compressed version (JPEG) - lower quality for bandwidth
                compressed_msg = CompressedImage()
                compressed_msg.header = img_msg.header
                compressed_msg.format = "jpeg"
                compressed_msg.data = cv2.imencode('.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, 70])[1].tobytes()
                self.compressed_pub.publish(compressed_msg)
                
            except Exception as e:
                rospy.logerr("Error publishing image: {}".format(e))
            
            self.rate.sleep()
    
    def shutdown(self):
        """Clean shutdown"""
        rospy.loginfo("Shutting down camera node")
        rospy.loginfo("Average FPS: {:.1f}".format(self.frame_count / (time.time() - self.start_time)))
        if self.cap:
            self.cap.release()

if __name__ == '__main__':
    try:
        node = CameraPublisher()
        rospy.on_shutdown(node.shutdown)
        node.run()
    except rospy.ROSInterruptException:
        pass
    except Exception as e:
        rospy.logerr("Fatal error in camera node: {}".format(e))