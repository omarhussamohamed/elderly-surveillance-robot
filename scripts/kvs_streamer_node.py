#!/usr/bin/env python2
"""
AWS Kinesis Video Streams Integration Node
Streams camera feed to AWS KVS for cloud AI inference
Hardcoded for RobotStream in eu-west-1
"""

import rospy
import subprocess
import os
import signal
import sys
from std_msgs.msg import Bool
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np

class KVSStreamer:
    def __init__(self):
        rospy.init_node('kvs_streamer', anonymous=False)
        
        # Hardcoded parameters for RobotStream
        self.stream_name = 'RobotStream'
        self.aws_region = 'eu-west-1'
        self.width = rospy.get_param('~width', 1280)
        self.height = rospy.get_param('~height', 720)
        self.fps = rospy.get_param('~fps', 30)
        self.bitrate = rospy.get_param('~bitrate', 2048)  # kbps
        self.enable_streaming = rospy.get_param('~enable_streaming', True)
        
        # Publisher for stream status
        self.status_pub = rospy.Publisher('/kvs/streaming', Bool, queue_size=1)
        
        # Subscriber to enable/disable streaming
        rospy.Subscriber('/kvs/enable', Bool, self.enable_callback)
        
        # Subscribe to camera topic instead of accessing hardware
        self.bridge = CvBridge()
        self.latest_frame = None
        self.frame_processing = False  # Flag to drop frames if busy
        rospy.Subscriber('/camera/image_raw', Image, self.image_callback, 
                        queue_size=1, buff_size=65536*20)
        
        self.process = None
        self.streaming = False
        
        rospy.loginfo("KVS Streamer initialized for stream: {}".format(self.stream_name))
        rospy.loginfo("Region: {}, Resolution: {}x{}".format(self.aws_region, self.width, self.height))
        
        # Check AWS credentials
        if not self.check_aws_credentials():
            rospy.logwarn("AWS credentials not found in environment")
            rospy.logwarn("Ensure launch file sets AWS_ACCESS_KEY_ID and AWS_SECRET_ACCESS_KEY")
        else:
            rospy.loginfo("AWS credentials found in environment")
        
        # Start streaming if enabled
        if self.enable_streaming:
            self.start_streaming()
    
    def image_callback(self, msg):
        """Store latest frame from camera_node (drop if busy)"""
        # Drop frame if still processing previous one to prevent backlog
        if self.frame_processing:
            return
        
        try:
            self.latest_frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception as e:
            rospy.logerr("Failed to convert image: {}".format(e))
    
    def check_aws_credentials(self):
        """Check if AWS credentials are configured in environment"""
        aws_key = os.environ.get('AWS_ACCESS_KEY_ID', '')
        aws_secret = os.environ.get('AWS_SECRET_ACCESS_KEY', '')
        return bool(aws_key and aws_secret)
    
    def start_streaming(self):
        """Start GStreamer pipeline to AWS KVS using appsrc"""
        if self.streaming:
            rospy.logwarn("Streaming already active")
            return
        
        # Wait for first frame from camera_node
        rospy.loginfo("Waiting for camera frames on /camera/image_raw...")
        timeout = rospy.Time.now() + rospy.Duration(10)
        while self.latest_frame is None and rospy.Time.now() < timeout:
            rospy.sleep(0.1)
        
        if self.latest_frame is None:
            rospy.logerr("No camera frames received! Make sure camera_node is running")
            return
        
        rospy.loginfo("Camera frames available, starting KVS pipeline")
        
        # Verify AWS credentials are set for kvssink
        aws_key = os.environ.get('AWS_ACCESS_KEY_ID', '')
        aws_secret = os.environ.get('AWS_SECRET_ACCESS_KEY', '')
        aws_region = os.environ.get('AWS_DEFAULT_REGION', self.aws_region)
        
        if not aws_key or not aws_secret:
            rospy.logerr("AWS credentials missing! kvssink will fail")
            return
        
        # GStreamer pipeline using appsrc with explicit format negotiation
        gst_pipeline = (
            "gst-launch-1.0 -v "
            "appsrc name=source is-live=true format=time do-timestamp=true "
            "caps=video/x-raw,format=BGR,width={},height={},framerate={}/1 ! "
            "videoconvert ! "
            "videoscale ! "
            "video/x-raw,format=I420,width={},height={} ! "
            "x264enc bframes=0 key-int-max=30 bitrate={} tune=zerolatency speed-preset=ultrafast ! "
            "video/x-h264,stream-format=avc,alignment=au,profile=baseline ! "
            "kvssink stream-name={} "
            "storage-size=128 "
            "aws-region={}"
        ).format(self.width, self.height, self.fps, 
                 self.width, self.height, self.bitrate, 
                 self.stream_name, aws_region)
        
        rospy.loginfo("Starting GStreamer pipeline with appsrc...")
        rospy.loginfo("Command: {}".format(gst_pipeline))
        rospy.loginfo("AWS credentials verified: Key={}..., Region={}".format(aws_key[:8], aws_region))
        
        # Prepare environment with AWS credentials for kvssink
        env = os.environ.copy()
        env['AWS_ACCESS_KEY_ID'] = aws_key
        env['AWS_SECRET_ACCESS_KEY'] = aws_secret
        env['AWS_DEFAULT_REGION'] = aws_region
        
        try:
            self.process = subprocess.Popen(
                gst_pipeline,
                shell=True,
                stdin=subprocess.PIPE,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                env=env,
                preexec_fn=os.setsid
            )
            self.streaming = True
            rospy.loginfo("KVS streaming started successfully")
            self.status_pub.publish(Bool(data=True))
            
            # Start frame feeding thread
            import threading
            self.feed_thread = threading.Thread(target=self.feed_frames)
            self.feed_thread.daemon = True
            self.feed_thread.start()
            
        except Exception as e:
            rospy.logerr("Failed to start streaming: {}".format(e))
            self.streaming = False
            self.status_pub.publish(Bool(data=False))
    
    def feed_frames(self):
        """Feed frames to GStreamer appsrc"""
        rate = rospy.Rate(self.fps)
        
        while self.streaming and not rospy.is_shutdown():
            if self.latest_frame is not None and self.process and self.process.stdin:
                self.frame_processing = True
                try:
                    # Write raw BGR frame to appsrc stdin
                    frame_bytes = self.latest_frame.tobytes()
                    self.process.stdin.write(frame_bytes)
                    self.process.stdin.flush()
                except Exception as e:
                    rospy.logerr("Failed to feed frame: {}".format(e))
                    self.frame_processing = False
                    break
                finally:
                    self.frame_processing = False
            
            rate.sleep()
    
    def stop_streaming(self):
        """Stop GStreamer pipeline"""
        if not self.streaming or not self.process:
            return
        
        rospy.loginfo("Stopping KVS streaming...")
        try:
            os.killpg(os.getpgid(self.process.pid), signal.SIGTERM)
            self.process.wait(timeout=5)
        except:
            try:
                os.killpg(os.getpgid(self.process.pid), signal.SIGKILL)
            except:
                pass
        
        self.process = None
        self.streaming = False
        self.status_pub.publish(Bool(data=False))
        rospy.loginfo("KVS streaming stopped")
    
    def enable_callback(self, msg):
        """Handle enable/disable streaming requests"""
        if msg.data and not self.streaming:
            self.start_streaming()
        elif not msg.data and self.streaming:
            self.stop_streaming()
    
    def monitor_process(self):
        """Monitor streaming process and restart if crashed"""
        rate = rospy.Rate(1)  # 1Hz
        
        while not rospy.is_shutdown():
            if self.streaming and self.process:
                # Check if process is still running
                poll = self.process.poll()
                if poll is not None:
                    rospy.logwarn("Streaming process died with code {}".format(poll))
                    self.streaming = False
                    self.status_pub.publish(Bool(data=False))
                    
                    if self.enable_streaming:
                        rospy.loginfo("Restarting stream in 5 seconds...")
                        rospy.sleep(5)
                        self.start_streaming()
            
            rate.sleep()
    
    def shutdown(self):
        """Clean shutdown"""
        rospy.loginfo("Shutting down KVS streamer")
        self.stop_streaming()

if __name__ == '__main__':
    try:
        node = KVSStreamer()
        rospy.on_shutdown(node.shutdown)
        node.monitor_process()
    except rospy.ROSInterruptException:
        pass
