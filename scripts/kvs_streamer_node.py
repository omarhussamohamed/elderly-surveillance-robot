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
import sys

gi.require_version('Gst', '1.0')
from gi.repository import Gst, GObject

# Suppress GStreamer debug output from kvssink plugin
# Level 0 = none, 1 = ERROR only, 2 = WARNING, 3 = FIXME, 4 = INFO, 5 = DEBUG
# Set to 1 to only show errors, kvssink plugin logs are at DEBUG level
if 'GST_DEBUG' not in os.environ:
    os.environ['GST_DEBUG'] = '1'  # Only show errors (level 1)
if 'GST_DEBUG_NO_COLOR' not in os.environ:
    os.environ['GST_DEBUG_NO_COLOR'] = '1'
    os.environ['GST_DEBUG_NO_COLOR'] = '1'

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
        self.last_frame_time = None
        self.frame_count = 0
        
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
        # Calculate fragment duration in seconds (2 seconds is recommended minimum)
        fragment_duration = 2
        
        pipeline_str = (
            "appsrc name=source is-live=true format=time do-timestamp=true "
            "caps=video/x-raw,format=BGR,width={width},height={height},framerate={fps}/1 "
            "max-bytes=0 block=true ! "
            "videoconvert ! videoscale ! "
            "video/x-raw,format=I420,width={width},height={height} ! "
            "x264enc bframes=0 key-int-max={keyframe_interval} bitrate={bitrate} "
            "tune=zerolatency speed-preset=ultrafast ! "
            "video/x-h264,stream-format=avc,alignment=au,profile=baseline ! "
            "kvssink stream-name={stream_name} storage-size={storage_size} "
            "aws-region={aws_region} connection-timeout={connection_timeout} "
            "buffer-duration={buffer_duration} fragment-duration={fragment_duration} "
            "fragment-acceptance-duration={fragment_duration}"
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
            buffer_duration=self.buffer_duration,
            fragment_duration=fragment_duration
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
                
                # Wait for pipeline to transition to playing state
                state_result = self.pipeline.get_state(Gst.CLOCK_TIME_NONE)
                timeout = 0
                while state_result[0] != Gst.StateChangeReturn.SUCCESS and timeout < 10:
                    time.sleep(0.5)
                    state_result = self.pipeline.get_state(Gst.CLOCK_TIME_NONE)
                    timeout += 1
                
                if state_result[0] == Gst.StateChangeReturn.FAILURE:
                    raise Exception("Pipeline failed to start")
                
                # Wait a bit more to ensure kvssink connects to AWS
                time.sleep(2.0)
                
                # Reset frame tracking - start from 0 for proper fragment creation
                self.pts = 0
                self.frame_count = 0
                self.last_frame_time = None
                
                self.is_streaming = True
                self.current_retry = 0  # Reset retry counter on success
                self.status_pub.publish(String("STREAMING_STARTED"))
                self.log_info("KVS stream started successfully - Stream: %s, Region: %s" % 
                            (self.stream_name, self.aws_region))
                self.log_info("Pipeline configuration: %dx%d @ %dfps, bitrate=%dkbps" % 
                            (self.width, self.height, self.fps, self.bitrate))
                self.log_info("Fragment duration: 2 seconds (keyframe every %d frames)" % 
                            int(2 * self.fps))
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
            
            # Resize frame if camera resolution doesn't match pipeline resolution
            frame_height, frame_width = frame.shape[:2]
            if frame_width != self.width or frame_height != self.height:
                frame = cv2.resize(frame, (self.width, self.height), interpolation=cv2.INTER_LINEAR)
            
            self.push_frame(frame.tobytes())
        except CvBridgeError as e:
            self.log_err("CvBridge Error: %s" % e)
        except Exception as e:
            self.log_err("Error in camera callback: %s" % e)

    def push_frame(self, frame):
        """Push frame to GStreamer pipeline"""
        if not self.appsrc or not self.is_streaming:
            return
        
        try:
            # Use monotonic timestamps starting from 0
            # This ensures proper fragment creation in kvssink
            if self.last_frame_time is None:
                self.pts = 0
                self.last_frame_time = rospy.Time.now()
            else:
                # Increment PTS by frame duration for consistent timing
                self.pts += self.frame_duration
            
            # Create buffer with proper size
            buffer = Gst.Buffer.new_allocate(None, len(frame), None)
            buffer.fill(0, frame)
            
            # Set timestamps - critical for fragment creation
            buffer.pts = self.pts
            buffer.dts = self.pts
            buffer.duration = self.frame_duration
            
            # Add timestamp metadata for debugging
            buffer.offset = self.frame_count
            buffer.offset_end = self.frame_count + 1
            
            # Update last frame time for rate limiting
            self.last_frame_time = rospy.Time.now()
            
            # Push buffer
            ret = self.appsrc.emit("push-buffer", buffer)
            if ret == Gst.FlowReturn.OK:
                self.frame_count += 1
                # Only publish status every 30 frames to reduce overhead
                if self.frame_count % 30 == 0:
                    self.status_pub.publish(String("FRAME_PUSHED:%d" % self.frame_count))
            elif ret == Gst.FlowReturn.FLUSHING:
                # Pipeline is shutting down, stop pushing
                self.is_streaming = False
                self.log_warn("Pipeline is flushing, stopping frame push")
            elif ret == Gst.FlowReturn.FULL:
                # Buffer is full, drop frame (shouldn't happen with proper rate limiting)
                self.log_debug("Buffer full, frame dropped")
            else:
                self.log_warn("Push buffer returned: %s" % ret)
                
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
                if new_state == Gst.State.PLAYING:
                    self.log_info("Pipeline is now PLAYING - streaming to AWS")
        elif t == Gst.MessageType.STREAM_START:
            self.log_info("Stream started - AWS KVS connection established")
            self.status_pub.publish(String("AWS_CONNECTED"))
        elif t == Gst.MessageType.STREAM_COLLECTION:
            collection = message.parse_stream_collection()
            self.log_debug("Stream collection: %d streams" % collection.get_n_streams())
        elif t == Gst.MessageType.ELEMENT:
            # Check for kvssink-specific messages
            if message.src and hasattr(message.src, 'get_name'):
                element_name = message.src.get_name()
                if 'kvssink' in element_name.lower():
                    structure = message.get_structure()
                    if structure:
                        self.log_debug("kvssink message: %s" % structure.get_name())

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