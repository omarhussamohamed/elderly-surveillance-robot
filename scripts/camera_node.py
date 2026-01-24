#!/usr/bin/env python2
"""
ROS Camera Publisher Node for ElderlyBot
Publishes MJPEG camera feed at 1280x720 @ 30fps
Compatible with ROS Melodic (Python 2.7)
""

import rospy
import cv2
from sensor_msgs.msg import Image, CompressedImage
from cv_bridge import CvBridge
import sys

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
        self.image_pub = rospy.Publisher('/camera/image_raw', Image, queue_size=1)
        self.compressed_pub = rospy.Publisher('/camera/image_raw/compressed', CompressedImage, queue_size=1)
        
        self.bridge = CvBridge()
        
        # Open camera
        self.cap = cv2.VideoCapture(self.device)
        if not self.cap.isOpened():
            rospy.logerr(f"Failed to open camera: {self.device}")
            sys.exit(1)
        
        # Set camera properties (MJPEG format for best performance)
        self.cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc('M','J','P','G'))
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.width)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)
        self.cap.set(cv2.CAP_PROP_FPS, self.fps)
        
        # Verify settings
        actual_width = int(self.cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        actual_height = int(self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
        actual_fps = int(self.cap.get(cv2.CAP_PROP_FPS))
        
        rospy.loginfo(f"Camera opened: {self.device}")
        rospy.loginfo(f"Resolution: {actual_width}x{actual_height} @ {actual_fps}fps")
        
        self.rate = rospy.Rate(self.fps)
        
    def run(self):
        """Main loop"""
        while not rospy.is_shutdown():
            ret, frame = self.cap.read()
            
            if not ret:
                rospy.logwarn("Failed to grab frame")
                continue
            
            # Create ROS Image message
            try:
                img_msg = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")
                img_msg.header.stamp = rospy.Time.now()
                img_msg.header.frame_id = self.frame_id
                self.image_pub.publish(img_msg)
                
                # Publish compressed version (JPEG)
                compressed_msg = CompressedImage()
                compressed_msg.header = img_msg.header
                compressed_msg.format = "jpeg"
                compressed_msg.data = cv2.imencode('.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, 80])[1].tobytes()
                self.compressed_pub.publish(compressed_msg)
                
            except Exception as e:
                rospy.logerr(f"Error publishing image: {e}")
            
            self.rate.sleep()
    
    def shutdown(self):
        """Clean shutdown"""
        rospy.loginfo("Shutting down camera node")
        if self.cap:
            self.cap.release()

if __name__ == '__main__':
    try:
        node = CameraPublisher()
        rospy.on_shutdown(node.shutdown)
        node.run()
    except rospy.ROSInterruptException:
        pass
