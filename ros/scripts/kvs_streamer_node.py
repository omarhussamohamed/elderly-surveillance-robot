#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""
AWS Kinesis Video Streams Node (ROS Melodic / Jetson)

- Streams camera frames to AWS KVS
- Subscribes to /camera/image_raw
- Auto-restarts on pipeline failure
- ROS-safe (never blocks robot)
"""

import rospy
import cv2
import os
import time
import json
import logging
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

import gi
gi.require_version('Gst', '1.0')
from gi.repository import Gst

# Reduce GStreamer noise
os.environ.setdefault('GST_DEBUG', '1')
os.environ.setdefault('GST_DEBUG_NO_COLOR', '1')


class KVSStreamerNode(object):
    def __init__(self):
        rospy.init_node('kvs_streamer_node', anonymous=False)

        # ── Parameters ─────────────────────────────────────────────
        self.stream_name = rospy.get_param('~stream_name', 'RobotStream')
        self.aws_region = rospy.get_param('~aws_region', 'us-east-1')

        self.width = rospy.get_param('~width', 1280)
        self.height = rospy.get_param('~height', 720)
        self.fps = rospy.get_param('~fps', 15)
        self.bitrate = rospy.get_param('~bitrate', 2000)

        self.retry_count = rospy.get_param('~retry_count', 5)
        self.retry_delay = rospy.get_param('~retry_delay', 3)

        self.buffer_duration = rospy.get_param('~buffer_duration', 180)
        self.connection_timeout = rospy.get_param('~connection_timeout', 45)
        self.storage_size = rospy.get_param('~storage_size', 512)
        self.fragment_duration = rospy.get_param('~fragment_duration', 0)

        self.output_to_screen = rospy.get_param('~output_to_screen', True)

        # ── State ─────────────────────────────────────────────────
        self.pipeline = None
        self.appsrc = None
        self.streaming = False

        self.frame_duration = int(1e9 / self.fps)
        self.pts = 0
        self.frame_count = 0
        self.last_frame_time = None

        # ── ROS I/O ───────────────────────────────────────────────
        self.status_pub = rospy.Publisher(
            '/kvs/streaming', String, queue_size=10
        )

        self.bridge = CvBridge()
        rospy.Subscriber(
            '/camera/image_raw', Image, self._camera_cb, queue_size=1
        )

        # ── Logging ───────────────────────────────────────────────
        self._setup_logging()

        # ── GStreamer ─────────────────────────────────────────────
        Gst.init(None)

        rospy.loginfo(
            "KVS configured: %dx%d @ %dfps (%dkbps)",
            self.width, self.height, self.fps, self.bitrate
        )

        self._start_streaming()
        rospy.on_shutdown(self.shutdown)

    # ─────────────────────────────────────────────────────────────
    def _setup_logging(self):
        if self.output_to_screen:
            self.logger = None
            return

        log_dir = os.path.expanduser('~/.ros/log')
        if not os.path.exists(log_dir):
            os.makedirs(log_dir)

        log_file = os.path.join(log_dir, 'kvs_stream.log')
        self.logger = logging.getLogger('kvs_streamer')
        self.logger.setLevel(logging.DEBUG)

        handler = logging.FileHandler(log_file)
        handler.setFormatter(
            logging.Formatter('%(asctime)s - %(levelname)s - %(message)s')
        )
        self.logger.addHandler(handler)

        rospy.loginfo("KVS logs redirected to %s", log_file)

    def _log(self, level, msg):
        """Log message via file logger or rospy."""
        if self.logger:
            getattr(self.logger, level)(msg)
        else:
            # Map to rospy log methods
            log_map = {
                'debug': rospy.logdebug,
                'info': rospy.loginfo,
                'loginfo': rospy.loginfo,
                'warn': rospy.logwarn,
                'logwarn': rospy.logwarn,
                'error': rospy.logerr,
                'logerr': rospy.logerr,
            }
            log_fn = log_map.get(level, rospy.loginfo)
            log_fn(msg)

    # ─────────────────────────────────────────────────────────────
    def _build_pipeline(self):
        keyframe_interval = self.fps * max(self.fragment_duration, 1)

        return (
            "appsrc name=source is-live=true format=time do-timestamp=true "
            "caps=video/x-raw,format=BGR,width={w},height={h},framerate={fps}/1 "
            "block=true ! videoconvert ! video/x-raw,format=I420 ! "
            "omxh264enc bitrate={br}000 control-rate=1 "
            "iframeinterval={ki} insert-sps-pps=true ! "
            "h264parse config-interval=1 ! "
            "kvssink stream-name={sn} storage-size={ss} "
            "aws-region={ar} connection-timeout={ct} "
            "buffer-duration={bd} fragment-duration={fd}"
        ).format(
            w=self.width, h=self.height, fps=self.fps,
            br=self.bitrate, ki=keyframe_interval,
            sn=self.stream_name, ss=self.storage_size,
            ar=self.aws_region, ct=self.connection_timeout,
            bd=self.buffer_duration, fd=self.fragment_duration
        )

    # ─────────────────────────────────────────────────────────────
    def _start_streaming(self):
        attempts = 0
        while attempts < self.retry_count and not rospy.is_shutdown():
            try:
                pipeline_str = self._build_pipeline()
                self._log('loginfo', "Starting KVS pipeline")

                self.pipeline = Gst.parse_launch(pipeline_str)
                self.appsrc = self.pipeline.get_by_name("source")

                bus = self.pipeline.get_bus()
                bus.add_signal_watch()
                bus.connect("message", self._on_bus_message)

                self.pipeline.set_state(Gst.State.PLAYING)
                self.streaming = True
                self.pts = 0
                self.frame_count = 0

                self.status_pub.publish("STREAMING")
                rospy.loginfo("KVS streaming started")
                return

            except Exception as e:
                attempts += 1
                self._log('logwarn', "KVS start failed (%d/%d): %s" %
                          (attempts, self.retry_count, e))
                time.sleep(self.retry_delay)

        self.status_pub.publish("ERROR")
        rospy.logerr("KVS failed to start after retries")

    # ─────────────────────────────────────────────────────────────
    def _camera_cb(self, msg):
        if not self.streaming or not self.appsrc:
            return

        try:
            frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")

            if frame.shape[1] != self.width or frame.shape[0] != self.height:
                frame = cv2.resize(frame, (self.width, self.height))

            self._push_frame(frame.tobytes())

        except CvBridgeError as e:
            self._log('logwarn', "CvBridge error: %s" % e)
        except Exception as e:
            self._log('logerr', "Frame processing error: %s" % e)

    # ─────────────────────────────────────────────────────────────
    def _push_frame(self, data):
        buffer = Gst.Buffer.new_allocate(None, len(data), None)
        buffer.fill(0, data)

        buffer.pts = self.pts
        buffer.dts = self.pts
        buffer.duration = self.frame_duration
        self.pts += self.frame_duration

        ret = self.appsrc.emit("push-buffer", buffer)
        if ret != Gst.FlowReturn.OK:
            self._log('logwarn', "Push buffer returned %s" % ret)
        else:
            self.frame_count += 1

    # ─────────────────────────────────────────────────────────────
    def _on_bus_message(self, bus, message):
        if message.type == Gst.MessageType.ERROR:
            err, dbg = message.parse_error()
            self._log('logerr', "GStreamer error: %s" % err)
            self._restart_stream()

    def _restart_stream(self):
        rospy.logwarn("Restarting KVS stream...")
        self.streaming = False
        if self.pipeline:
            self.pipeline.set_state(Gst.State.NULL)
        self.pipeline = None
        self.appsrc = None
        time.sleep(self.retry_delay)
        self._start_streaming()

    # ─────────────────────────────────────────────────────────────
    def shutdown(self):
        rospy.loginfo("Shutting down KVS streamer")
        self.streaming = False
        if self.pipeline:
            self.pipeline.set_state(Gst.State.NULL)
        self.pipeline = None
        self.appsrc = None


if __name__ == '__main__':
    try:
        KVSStreamerNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
