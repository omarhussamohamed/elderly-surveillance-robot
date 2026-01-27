#!/usr/bin/env python2
# -*- coding: utf-8 -*-

import rospy
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus

class SystemHealthMonitor:
    def __init__(self):
        rospy.init_node('system_health_monitor')

        self.check_interval = rospy.get_param('~check_interval', 10)
        self.startup_delay = rospy.get_param('~startup_delay', 30)
        self.critical_nodes = rospy.get_param('~critical_nodes', "rplidar_node,esp32_serial_node,ekf_localization_node").split(',')
        self.optional_nodes = rospy.get_param('~optional_nodes', "mpu9250_node,cloud_bridge_node,sensors_actuators_node").split(',')

        rospy.sleep(self.startup_delay)  # Wait for startup

        self.pub = rospy.Publisher('/diagnostics', DiagnosticArray, queue_size=10)
        rospy.Timer(rospy.Duration(self.check_interval), self.check_nodes)

    def check_nodes(self, event):
        msg = DiagnosticArray()
        msg.header.stamp = rospy.Time.now()

        # Check critical
        for node in self.critical_nodes:
            status = self.get_node_status(node)
            if status.level == DiagnosticStatus.ERROR:
                rospy.logerr("Critical node %s down!" % node)
            msg.status.append(status)

        # Check optional
        for node in self.optional_nodes:
            msg.status.append(self.get_node_status(node))

        self.pub.publish(msg)

    def get_node_status(self, node):
        # Real implementation would use rosnode ping or similar
        # This is a placeholder
        try:
            # Simulate check
            return DiagnosticStatus(level=DiagnosticStatus.OK, name=node, message="OK")
        except:
            return DiagnosticStatus(level=DiagnosticStatus.ERROR, name=node, message="Down")

if __name__ == '__main__':
    try:
        SystemHealthMonitor()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass