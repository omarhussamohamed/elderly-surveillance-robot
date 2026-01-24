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

gi.require_version('Gst', '1.0')
from gi.repository import Gst, GObject

class KVSStreamerNode:
    def __init__(self):
        rospy.init_node('kvs_streamer_node', anonymous=True)

        # ROS Publisher
        self.status_pub = rospy.Publisher('/kvs/streaming', String, queue_size=10)

        # GStreamer Pipeline
        self.pipeline = Gst.parse_launch(
            "appsrc name=source is-live=true format=time do-timestamp=false "
            "caps=video/x-raw,format=BGR,width=640,height=480,framerate=30/1 ! "
            "videoconvert ! videoscale ! video/x-raw,format=I420,width=640,height=480 ! "
            "x264enc bframes=0 key-int-max=30 bitrate=500 tune=zerolatency speed-preset=ultrafast ! "
            "video/x-h264,stream-format=avc,alignment=au,profile=baseline ! "
            "kvssink stream-name=RobotStream storage-size=128 aws-region=us-east-1"
        )

        self.appsrc = self.pipeline.get_by_name("source")
        self.pts = 0
        self.frame_duration = int(1e9 / 30)  # Assuming 30 FPS

        # Attach bus watch
        bus = self.pipeline.get_bus()
        bus.add_signal_watch()
        bus.connect("message", self.on_bus_message)

        # Start pipeline
        self.pipeline.set_state(Gst.State.PLAYING)
        self.status_pub.publish(String("STREAMING_STARTED"))

    def push_frame(self, frame):
        buffer = Gst.Buffer.new_allocate(None, len(frame), None)
        buffer.fill(0, frame)
        buffer.pts = self.pts
        buffer.dts = self.pts
        buffer.duration = self.frame_duration
        self.pts += self.frame_duration

        self.appsrc.emit("push-buffer", buffer)
        self.status_pub.publish(String("FRAME_PUSHED"))

    def on_bus_message(self, bus, message):
        t = message.type
        if t == Gst.MessageType.ERROR:
            err, debug = message.parse_error()
            rospy.logerr("GStreamer ERROR: %s, %s" % (err, debug))
            self.status_pub.publish(String("ERROR: %s" % err))
        elif t == Gst.MessageType.WARNING:
            warn, debug = message.parse_warning()
            rospy.logwarn("GStreamer WARNING: %s, %s" % (warn, debug))
            self.status_pub.publish(String("WARNING: %s" % warn))
        elif t == Gst.MessageType.EOS:
            rospy.loginfo("GStreamer EOS reached")
            self.status_pub.publish(String("EOS"))

    def shutdown(self):
        self.pipeline.set_state(Gst.State.NULL)
        rospy.loginfo("Pipeline stopped")

if __name__ == '__main__':
    try:
        node = KVSStreamerNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
