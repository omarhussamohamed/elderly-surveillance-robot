#!/usr/bin/env python2
"""
AWS Kinesis Video Streams Integration Node
Streams camera feed to AWS KVS for cloud AI inference
Hardcoded for RobotStream in us-east-1
Note: Uses Python 2.7 compatible syntax for ROS Melodic
"""

import rospy
from std_msgs.msg import String
import gi
import cv2
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
import os
import time
import logging

gi.require_version('Gst', '1.0')
from gi.repository import Gst, GObject

class KVSStreamerNode:
    def __init__(self):
        rospy.init_node('kvs_streamer_node', anonymous=True)

        # === PARAMETERS ===
        # Stream configuration
        self.stream_name = rospy.get_param('~stream_name', 'RobotStream')
        self.aws_region = rospy.get_param('~aws_region', 'us-east-1')
        
        # Video parameters
        self.width = rospy.get_param('~width', 640)
        self.height = rospy.get_param('~height', 480)
        self.fps = rospy.get_param('~fps', 15)
        self.bitrate = rospy.get_param('~bitrate', 512)
        
        # Stability parameters
        self.retry_count = rospy.get_param('~retry_count', 3)
        self.retry_delay = rospy.get_param('~retry_delay', 5)
        self.buffer_duration = rospy.get_param('~buffer_duration', 120)
        self.connection_timeout = rospy.get_param('~connection_timeout', 30)
        self.storage_size = rospy.get_param('~storage_size', 256)
        
        # Output management
        self.output_to_screen = rospy.get_param('~output_to_screen', True)
        
        # === LOGGING SETUP ===
        self.setup_logging()
        
        # === STATE ===
        self.pipeline = None
        self.appsrc = None
        self.pts = 0
        self.frame_duration = int(1e9 / self.fps)
        self.current_retry = 0
        self.is_streaming = False
        
        # ROS Publisher
        self.status_pub = rospy.Publisher('/kvs/streaming', String, queue_size=10)

        # ROS Subscriber
        self.bridge = CvBridge()
        rospy.Subscriber('/camera/image_raw', Image, self.camera_callback)
        
        # Initialize GStreamer
        Gst.init(None)
        
        # Start streaming with retry logic
        self.start_streaming()

    def setup_logging(self):
        """Setup logging to file if in silent mode"""
        if not self.output_to_screen:
            # Create log directory if it doesn't exist
            log_dir = os.path.expanduser('~/.ros/log')
            if not os.path.exists(log_dir):
                os.makedirs(log_dir)
            
            log_file = os.path.join(log_dir, 'kvs_stream.log')
            
            # Setup file logging
            self.file_logger = logging.getLogger('kvs_streamer')
            self.file_logger.setLevel(logging.DEBUG)
            handler = logging.FileHandler(log_file)
            handler.setLevel(logging.DEBUG)
            formatter = logging.Formatter('%(asctime)s - %(levelname)s - %(message)s')
            handler.setFormatter(formatter)
            self.file_logger.addHandler(handler)
            
            # Only show errors/warnings in main terminal
            rospy.loginfo("KVS silent mode enabled. Logs written to: %s" % log_file)
        else:
            self.file_logger = None
    
    def log_debug(self, message):
        """Log debug message to file if in silent mode, or to screen otherwise"""
        if self.file_logger:
            self.file_logger.debug(message)
        else:
            rospy.logdebug(message)
    
    def log_info(self, message):
        """Log info message to file if in silent mode, or to screen otherwise"""
        if self.file_logger:
            self.file_logger.info(message)
        else:
            rospy.loginfo(message)
    
    def log_warn(self, message):
        """Always log warnings to screen"""
        if self.file_logger:
            self.file_logger.warning(message)
        rospy.logwarn(message)
    
    def log_err(self, message):
        """Always log errors to screen"""
        if self.file_logger:
            self.file_logger.error(message)
        rospy.logerr(message)
    
    def build_pipeline_string(self):
        """Build GStreamer pipeline string with stability parameters"""
        pipeline_str = (
            "appsrc name=source is-live=true format=time do-timestamp=false "
            "caps=video/x-raw,format=BGR,width={width},height={height},framerate={fps}/1 ! "
            "videoconvert ! videoscale ! video/x-raw,format=I420,width={width},height={height} ! "
            "x264enc bframes=0 key-int-max={keyframe_interval} bitrate={bitrate} "
            "tune=zerolatency speed-preset=ultrafast ! "
            "video/x-h264,stream-format=avc,alignment=au,profile=baseline ! "
            "kvssink stream-name={stream_name} storage-size={storage_size} "
            "aws-region={aws_region} connection-timeout={connection_timeout} "
            "buffer-duration={buffer_duration}"
        ).format(
            width=self.width,
            height=self.height,
            fps=self.fps,
            keyframe_interval=int(2 * self.fps),  # 2x fps as recommended
            bitrate=int(self.bitrate * 1000),  # Convert kbps to bps
            stream_name=self.stream_name,
            storage_size=self.storage_size,
            aws_region=self.aws_region,
            connection_timeout=self.connection_timeout,
            buffer_duration=self.buffer_duration
        )
        return pipeline_str
    
    def start_streaming(self):
        """Start streaming with retry logic"""
        while self.current_retry < self.retry_count and not rospy.is_shutdown():
            try:
                self.log_info("Starting KVS stream (attempt %d/%d)" % (self.current_retry + 1, self.retry_count))
                
                # Build and create pipeline
                pipeline_str = self.build_pipeline_string()
                self.log_debug("Pipeline: %s" % pipeline_str)
                
                self.pipeline = Gst.parse_launch(pipeline_str)
                self.appsrc = self.pipeline.get_by_name("source")
                
                # Attach bus watch
                bus = self.pipeline.get_bus()
                bus.add_signal_watch()
                bus.connect("message", self.on_bus_message)

                # Start pipeline
                ret = self.pipeline.set_state(Gst.State.PLAYING)
                if ret == Gst.StateChangeReturn.FAILURE:
                    raise Exception("Failed to set pipeline to PLAYING state")
                
                # Wait for pipeline to start
                time.sleep(1.0)
                
                # Check if pipeline is actually playing
                state_result = self.pipeline.get_state(Gst.CLOCK_TIME_NONE)
                if state_result[0] == Gst.StateChangeReturn.FAILURE:
                    raise Exception("Pipeline failed to start")
                
                self.is_streaming = True
                self.current_retry = 0  # Reset retry counter on success
                self.status_pub.publish(String("STREAMING_STARTED"))
                self.log_info("KVS stream started successfully")
                return
                
            except Exception as e:
                self.current_retry += 1
                self.log_err("Failed to start stream (attempt %d/%d): %s" % 
                           (self.current_retry, self.retry_count, str(e)))
                
                if self.pipeline:
                    self.pipeline.set_state(Gst.State.NULL)
                    self.pipeline = None
                    self.appsrc = None
                
                if self.current_retry < self.retry_count:
                    self.log_warn("Retrying in %d seconds..." % self.retry_delay)
                    time.sleep(self.retry_delay)
                else:
                    self.log_err("Max retry attempts reached. Stream not started.")
                    self.status_pub.publish(String("ERROR: Max retries exceeded"))
    
    def camera_callback(self, msg):
        """Callback for camera image messages"""
        if not self.is_streaming or not self.appsrc:
            return
        
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            self.push_frame(frame.tobytes())
        except CvBridgeError as e:
            self.log_err("CvBridge Error: %s" % e)
        except Exception as e:
            self.log_err("Error in camera callback: %s" % e)

    def push_frame(self, frame):
        """Push frame to GStreamer pipeline"""
        if not self.appsrc:
            return
        
        try:
            buffer = Gst.Buffer.new_allocate(None, len(frame), None)
            buffer.fill(0, frame)
            buffer.pts = self.pts
            buffer.dts = self.pts
            buffer.duration = self.frame_duration
            self.pts += self.frame_duration

            self.appsrc.emit("push-buffer", buffer)
            self.status_pub.publish(String("FRAME_PUSHED"))
        except Exception as e:
            self.log_err("Error pushing frame: %s" % e)

    def on_bus_message(self, bus, message):
        """Handle GStreamer bus messages"""
        t = message.type
        if t == Gst.MessageType.ERROR:
            err, debug = message.parse_error()
            error_msg = "GStreamer ERROR: %s, %s" % (err, debug)
            self.log_err(error_msg)
            self.status_pub.publish(String("ERROR: %s" % err))
            
            # Attempt to restart stream on error
            if self.is_streaming:
                self.is_streaming = False
                self.log_warn("Stream error detected. Attempting to restart...")
                if self.pipeline:
                    self.pipeline.set_state(Gst.State.NULL)
                    self.pipeline = None
                    self.appsrc = None
                self.current_retry = 0  # Reset retry counter
                rospy.sleep(self.retry_delay)
                self.start_streaming()
                
        elif t == Gst.MessageType.WARNING:
            warn, debug = message.parse_warning()
            warn_msg = "GStreamer WARNING: %s, %s" % (warn, debug)
            self.log_warn(warn_msg)
            self.status_pub.publish(String("WARNING: %s" % warn))
        elif t == Gst.MessageType.EOS:
            self.log_info("GStreamer EOS reached")
            self.status_pub.publish(String("EOS"))
            self.is_streaming = False
        elif t == Gst.MessageType.STATE_CHANGED:
            if message.src == self.pipeline:
                old_state, new_state, pending_state = message.parse_state_changed()
                self.log_debug("Pipeline state changed: %s -> %s" % 
                             (old_state.value_nick, new_state.value_nick))

    def shutdown(self):
        """Clean shutdown"""
        self.log_info("Shutting down KVS streamer")
        self.is_streaming = False
        if self.pipeline:
            self.pipeline.set_state(Gst.State.NULL)
            self.pipeline = None
            self.appsrc = None
        self.log_info("Pipeline stopped")

if __name__ == '__main__':
    try:
        node = KVSStreamerNode()
        rospy.on_shutdown(node.shutdown)
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
    except Exception as e:
        rospy.logerr("Fatal error in KVS streamer: %s" % str(e))