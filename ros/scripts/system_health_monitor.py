#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""
System Health Monitor

- Publishes real diagnostics to /diagnostics
- Checks existence of critical & optional ROS nodes
- Never fakes OK status
"""

import rospy
try:
    import xmlrpc.client as xmlrpclib  # Python 3
except ImportError:
    import xmlrpclib  # Python 2
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus


class SystemHealthMonitor(object):
    def __init__(self):
        rospy.init_node('system_health_monitor', anonymous=False)

        # ── Parameters ─────────────────────────────────────────────
        self.check_interval = rospy.get_param('~check_interval', 10.0)
        self.startup_delay = rospy.get_param('~startup_delay', 30.0)

        self.critical_nodes = rospy.get_param(
            '~critical_nodes',
            ['rplidar_node', 'sensors_actuators_node']
        )

        self.optional_nodes = rospy.get_param(
            '~optional_nodes',
            ['mpu9250_node', 'cloud_bridge_node', 'kvs_streamer_node']
        )

        rospy.loginfo("System Health Monitor starting...")
        rospy.sleep(self.startup_delay)

        self.pub = rospy.Publisher(
            '/diagnostics', DiagnosticArray, queue_size=10
        )

        self.master = xmlrpclib.ServerProxy(
            rospy.get_master_uri()
        )

        self.timer = rospy.Timer(
            rospy.Duration(self.check_interval),
            self._check_nodes
        )

        rospy.on_shutdown(self.shutdown)
        rospy.loginfo("System Health Monitor running")

    # ─────────────────────────────────────────────────────────────
    def shutdown(self):
        """Clean shutdown of health monitor."""
        rospy.loginfo("Shutting down System Health Monitor")
        if hasattr(self, 'timer') and self.timer:
            self.timer.shutdown()

    # ─────────────────────────────────────────────────────────────
    def _get_active_nodes(self):
        try:
            _, _, system_state = self.master.getSystemState('')
            pubs, subs, srvs = system_state
            nodes = set()

            for lst in (pubs, subs, srvs):
                for _, node_list in lst:
                    nodes.update(node_list)

            return nodes
        except Exception as e:
            rospy.logerr("Failed to query ROS master: %s", str(e))
            return set()

    # ─────────────────────────────────────────────────────────────
    def _node_status(self, node, active_nodes, critical):
        if node in active_nodes:
            return DiagnosticStatus(
                level=DiagnosticStatus.OK,
                name=node,
                message="Running"
            )

        if critical:
            rospy.logerr("CRITICAL node down: %s", node)
            return DiagnosticStatus(
                level=DiagnosticStatus.ERROR,
                name=node,
                message="Not running"
            )

        rospy.logwarn("Optional node down: %s", node)
        return DiagnosticStatus(
            level=DiagnosticStatus.WARN,
            name=node,
            message="Not running"
        )

    # ─────────────────────────────────────────────────────────────
    def _check_nodes(self, event):
        active_nodes = self._get_active_nodes()

        msg = DiagnosticArray()
        msg.header.stamp = rospy.Time.now()

        for node in self.critical_nodes:
            msg.status.append(
                self._node_status(node, active_nodes, critical=True)
            )

        for node in self.optional_nodes:
            msg.status.append(
                self._node_status(node, active_nodes, critical=False)
            )

        self.pub.publish(msg)


if __name__ == '__main__':
    try:
        SystemHealthMonitor()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
