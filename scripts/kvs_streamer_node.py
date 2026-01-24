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

class KVSStreamer:
    def __init__(self):
        rospy.init_node('kvs_streamer', anonymous=False)
        
        # Hardcoded parameters for RobotStream
        self.stream_name = 'RobotStream'
        self.aws_region = 'eu-west-1'
        self.device = rospy.get_param('~device', '/dev/video0')
        self.width = rospy.get_param('~width', 1280)
        self.height = rospy.get_param('~height', 720)
        self.fps = rospy.get_param('~fps', 30)
        self.bitrate = rospy.get_param('~bitrate', 2048)  # kbps
        self.enable_streaming = rospy.get_param('~enable_streaming', True)
        
        # Publisher for stream status
        self.status_pub = rospy.Publisher('/kvs/streaming', Bool, queue_size=1)
        
        # Subscriber to enable/disable streaming
        rospy.Subscriber('/kvs/enable', Bool, self.enable_callback)
        
        self.process = None
        self.streaming = False
        
        rospy.loginfo("KVS Streamer initialized for stream: {}".format(self.stream_name))
        rospy.loginfo("Region: {}, Resolution: {}x{}".format(self.aws_region, self.width, self.height))
        
        # Check AWS credentials
        if not self.check_aws_credentials():
            rospy.logwarn("AWS credentials not configured! Run: aws configure")
            rospy.logwarn("Streaming will fail until credentials are set")
        
        # Start streaming if enabled
        if self.enable_streaming:
            self.start_streaming()
    
    def check_aws_credentials(self):
        """Check if AWS credentials are configured"""
        try:
            result = subprocess.run(['aws', 'configure', 'get', 'aws_access_key_id'], 
                                  capture_output=True, text=True, timeout=5)
            return result.returncode == 0 and result.stdout.strip()
        except:
            return False
    
    def start_streaming(self):
        """Start GStreamer pipeline to AWS KVS"""
        if self.streaming:
            rospy.logwarn("Streaming already active")
            return
        
        # GStreamer pipeline for USB camera -> AWS KVS
        gst_pipeline = (
            "gst-launch-1.0 -v "
            "v4l2src device={} ! "
            "image/jpeg,width={},height={},framerate={}/1 ! "
            "jpegdec ! "
            "videoconvert ! "
            "video/x-raw,format=I420 ! "
            "x264enc bframes=0 key-int-max={} bitrate={} tune=zerolatency ! "
            "video/x-h264,stream-format=avc,alignment=au,profile=baseline ! "
            "kvssink stream-name={} "
            "storage-size=128 "
            "aws-region={}"
        ).format(self.device, self.width, self.height, self.fps, 
                 self.fps*2, self.bitrate, self.stream_name, self.aws_region)
        
        rospy.loginfo("Starting GStreamer pipeline...")
        rospy.loginfo("Command: {}".format(gst_pipeline))
        
        try:
            self.process = subprocess.Popen(
                gst_pipeline,
                shell=True,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                preexec_fn=os.setsid
            )
            self.streaming = True
            rospy.loginfo("KVS streaming started successfully")
            self.status_pub.publish(Bool(data=True))
            
        except Exception as e:
            rospy.logerr("Failed to start streaming: {}".format(e))
            self.streaming = False
            self.status_pub.publish(Bool(data=False))
    
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
