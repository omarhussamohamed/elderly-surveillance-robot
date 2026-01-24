#!/usr/bin/env python2
"""
AWS Kinesis Video Streams Integration Node
Streams camera feed to AWS KVS for cloud AI inference
Hardcoded for RobotStream in us-east-1
Note: Uses Python 2.7 compatible syntax for ROS Melodic
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
import time
import gi
from gi.repository import Gst

class KVSStreamer:
    def __init__(self):
        rospy.init_node('kvs_streamer', anonymous=False)
        
        # Hardcoded parameters for RobotStream
        self.stream_name = 'RobotStream'
        self.aws_region = 'us-east-1'
        self.width = rospy.get_param('~width', 1280)
        self.height = rospy.get_param('~height', 720)
        self.fps = rospy.get_param('~fps', 30)
        self.bitrate = rospy.get_param('~bitrate', 2048)  # kbps
        self.enable_streaming = rospy.get_param('~enable_streaming', True)
        
        # Publisher for stream status
        self.status_pub = rospy.Publisher(
            "/kvs/streaming",
            std_msgs.msg.String,
            queue_size=10
        )
        
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
        self.frame_count = 0  # Initialize frame counter
        self.last_status_time = rospy.Time.now()

        # Ensure KVS logging configuration
        log_config_path = os.path.join(os.path.dirname(__file__), "kvs_log_configuration")
        os.environ["KVS_LOG_CONFIG"] = log_config_path
        if not os.path.exists(log_config_path):
            with open(log_config_path, "w") as log_file:
                log_file.write("log4cplus.rootLogger=INFO, stdout\n")

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
            rospy.loginfo("Received frame from camera, encoding: bgr8, size: {}x{}".format(
                self.latest_frame.shape[1], self.latest_frame.shape[0]))
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
        # Frame size: width * height * 3 (BGR channels)
        expected_frame_size = self.width * self.height * 3
        rospy.loginfo("Expected frame size: {} bytes".format(expected_frame_size))
        
        gst_pipeline = (
            "appsrc name=source is-live=true format=time do-timestamp=false emit-signals=true "
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
            
            # Start stderr monitoring thread
            import threading
            self.stderr_thread = threading.Thread(target=self.monitor_stderr)
            self.stderr_thread.daemon = True
            self.stderr_thread.start()
            
            # Start frame feeding thread
            self.feed_thread = threading.Thread(target=self.feed_frames)
            self.feed_thread.daemon = True
            self.feed_thread.start()
            
            # Attach GStreamer bus watch
            bus = self.process.get_bus()
            bus.add_signal_watch()
            bus.connect("message::error", self.on_gst_error)
            bus.connect("message::warning", self.on_gst_warning)
            bus.connect("message::eos", self.on_gst_eos)
            
        except Exception as e:
            rospy.logerr("Failed to start streaming: {}".format(e))
            self.streaming = False
            self.status_pub.publish(Bool(data=False))
    
    def monitor_stderr(self):
        """Monitor GStreamer stderr output for debugging"""
        if not self.process or not self.process.stderr:
            return
        
        rospy.loginfo("Started GStreamer stderr monitoring")
        
        while self.streaming and not rospy.is_shutdown():
            try:
                line = self.process.stderr.readline()
                if line:
                    line_str = line.strip()
                    # Log important GStreamer messages
                    if 'ERROR' in line_str or 'error' in line_str:
                        rospy.logerr("GStreamer ERROR: {}".format(line_str))
                    elif 'WARNING' in line_str or 'warning' in line_str:
                        rospy.logwarn("GStreamer WARNING: {}".format(line_str))
                    elif 'caps' in line_str.lower() or 'negotiation' in line_str.lower():
                        rospy.loginfo("GStreamer CAPS: {}".format(line_str))
                else:
                    # Process closed stderr
                    break
            except:
                break
        
        rospy.loginfo("GStreamer stderr monitoring stopped")
    
    def feed_frames(self):
        """Feed frames to GStreamer appsrc"""
        rate = rospy.Rate(self.fps)
        expected_size = self.width * self.height * 3
        self.pts = 0  # Initialize running PTS counter
        self.frame_duration = int(1e9 / self.fps)  # Duration per frame in nanoseconds
        start_time = time.time()

        # Configure appsrc properties
        appsrc = self.process.args[0]
        appsrc.set_property("is-live", True)
        appsrc.set_property("format", Gst.Format.TIME)
        appsrc.set_property("do-timestamp", False)

        # Attach GStreamer bus watch
        def on_message(bus, message):
            t = message.type
            if t == Gst.MessageType.ERROR:
                err, debug = message.parse_error()
                rospy.logerr("GStreamer ERROR: %s, %s" % (err, debug))
                self.status_pub.publish("ERROR: %s" % err)
            elif t == Gst.MessageType.WARNING:
                warn, debug = message.parse_warning()
                rospy.logwarn("GStreamer WARNING: %s, %s" % (warn, debug))
                self.status_pub.publish("WARNING: %s" % warn)
            elif t == Gst.MessageType.EOS:
                rospy.loginfo("GStreamer EOS reached")
                self.status_pub.publish("EOS")

        bus = pipeline.get_bus()
        bus.add_signal_watch()
        bus.connect("message", on_message)

        # Update frame feeding loop
        while self.streaming and not rospy.is_shutdown():
            if self.latest_frame is not None and self.process and self.process.stdin:
                self.frame_processing = True
                try:
                    # Verify frame dimensions
                    if self.latest_frame.shape[0] != self.height or self.latest_frame.shape[1] != self.width:
                        rospy.logwarn("Frame size mismatch! Expected {}x{}, got {}x{}".format(
                            self.width, self.height, 
                            self.latest_frame.shape[1], self.latest_frame.shape[0]))
                        self.frame_processing = False
                        rate.sleep()
                        continue

                    # Convert to raw BGR byte array
                    frame_bytes = self.latest_frame.tobytes()

                    # Verify byte size
                    if len(frame_bytes) != expected_size:
                        rospy.logerr("Frame byte size mismatch! Expected {}, got {}".format(
                            expected_size, len(frame_bytes)))
                        self.frame_processing = False
                        rate.sleep()
                        continue

                    # Create Gst.Buffer
                    buffer = Gst.Buffer.new_allocate(None, len(frame_bytes), None)
                    buffer.fill(0, frame_bytes)

                    # Set timestamps using running PTS counter
                    buffer.pts = self.pts
                    buffer.dts = self.pts
                    buffer.duration = self.frame_duration
                    self.pts += self.frame_duration

                    # Write to appsrc stdin
                    self.process.stdin.write(frame_bytes)
                    self.process.stdin.flush()

                    # Publish status
                    self.frame_count += 1
                    if self.frame_count % 10 == 0:
                        rospy.loginfo("Pushed {} frames to appsrc".format(self.frame_count))
                        self.status_pub.publish("FRAME_PUSHED")

                except IOError as e:
                    # Handle Broken Pipe (Errno 32)
                    if e.errno == 32:
                        rospy.logerr("[Errno 32] Broken pipe - GStreamer pipeline closed stdin")
                        rospy.logerr("This usually means the pipeline crashed or rejected the frame format")
                        rospy.logerr("Check GStreamer stderr output above for specific error")
                    else:
                        rospy.logerr("IOError while feeding frame: %s" % e)
                    self.frame_processing = False
                    self.streaming = False
                    self.status_pub.publish("ERROR: Broken pipe")
                    break

                except Exception as e:
                    rospy.logerr("Failed to feed frame: {}".format(e))
                    self.frame_processing = False
                    self.status_pub.publish("ERROR: {}".format(str(e)))
                    break
                finally:
                    self.frame_processing = False

            rate.sleep()
    
    def on_gst_error(self, bus, message):
        err, debug = message.parse_error()
        rospy.logerr("GStreamer ERROR: {}".format(err))
        self.status_pub.publish("ERROR: {}".format(err))

    def on_gst_warning(self, bus, message):
        warn, debug = message.parse_warning()
        rospy.logwarn("GStreamer WARNING: {}".format(warn))

    def on_gst_eos(self, bus, message):
        rospy.loginfo("GStreamer EOS (End of Stream) reached")
        self.status_pub.publish("EOS")
    
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
