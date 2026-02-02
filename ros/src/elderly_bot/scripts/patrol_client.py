#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""
Patrol Client Node

- Sends navigation goals to move_base when available
- SAFE MODE by default (does NOT block system if navigation is down)
- Waypoints loaded from YAML
- Designed for future navigation integration
"""

import rospy
import actionlib
import yaml
import os
import sys
from geometry_msgs.msg import Quaternion
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from actionlib_msgs.msg import GoalStatus
from tf.transformations import quaternion_from_euler
import rospkg


class PatrolClient(object):
    def __init__(self):
        rospy.init_node('patrol_client', anonymous=False)
        rospy.loginfo("Starting Patrol Client")

        # ── Parameters ─────────────────────────────────────────────
        self.enabled = rospy.get_param('~enabled', False)
        self.goal_timeout = rospy.get_param('~goal_timeout', 300.0)
        self.inter_goal_delay = rospy.get_param('~inter_goal_delay', 2.0)
        self.max_retries = rospy.get_param('~max_retries', 3)

        patrol_file_param = rospy.get_param(
            '~patrol_file',
            '$(find elderly_bot)/config/patrol_goals.yaml'
        )
        self.patrol_file = self._resolve_path(patrol_file_param)

        if not self.enabled:
            rospy.logwarn("Patrol client DISABLED (enabled:=false)")
            rospy.logwarn("Navigation stack not required")
            return

        # ── Load patrol goals ─────────────────────────────────────
        self.goals = self._load_goals()
        if not self.goals:
            rospy.logerr("No valid patrol goals loaded — patrol aborted")
            return

        # ── Action client ─────────────────────────────────────────
        self.client = actionlib.SimpleActionClient(
            'move_base', MoveBaseAction
        )

        rospy.loginfo("Waiting for move_base action server...")
        if not self.client.wait_for_server(rospy.Duration(5.0)):
            rospy.logerr("move_base not available — patrol aborted")
            return

        rospy.loginfo("Patrol client connected to move_base")

        self._patrol_loop()

    # ─────────────────────────────────────────────────────────────
    def _resolve_path(self, path):
        if '$(find ' in path:
            try:
                pkg = path.split('$(find ')[1].split(')')[0]
                rp = rospkg.RosPack()
                pkg_path = rp.get_path(pkg)
                return path.replace('$(find %s)' % pkg, pkg_path)
            except Exception as e:
                rospy.logerr("Path resolution failed: %s", str(e))
        return os.path.expandvars(os.path.expanduser(path))

    # ─────────────────────────────────────────────────────────────
    def _load_goals(self):
        if not os.path.exists(self.patrol_file):
            rospy.logerr("Patrol file not found: %s", self.patrol_file)
            return []

        try:
            with open(self.patrol_file, 'r') as f:
                data = yaml.safe_load(f)

            goals = data.get('patrol_goals', [])
            valid = []

            for g in goals:
                if all(k in g for k in ('name', 'x', 'y', 'yaw')):
                    valid.append(g)
                else:
                    rospy.logwarn("Invalid goal entry skipped: %s", g)

            rospy.loginfo("Loaded %d patrol goals", len(valid))
            return valid

        except Exception as e:
            rospy.logerr("Failed to load patrol goals: %s", str(e))
            return []

    # ─────────────────────────────────────────────────────────────
    def _create_goal(self, goal_data):
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()

        goal.target_pose.pose.position.x = float(goal_data['x'])
        goal.target_pose.pose.position.y = float(goal_data['y'])

        q = quaternion_from_euler(0, 0, float(goal_data['yaw']))
        goal.target_pose.pose.orientation = Quaternion(*q)

        return goal

    # ─────────────────────────────────────────────────────────────
    def _send_goal(self, goal_data):
        goal = self._create_goal(goal_data)
        self.client.send_goal(goal)

        finished = self.client.wait_for_result(
            rospy.Duration(self.goal_timeout)
        )

        if not finished:
            self.client.cancel_goal()
            rospy.logwarn("Goal timeout: %s", goal_data['name'])
            return False

        state = self.client.get_state()
        if state == GoalStatus.SUCCEEDED:
            rospy.loginfo("Goal reached: %s", goal_data['name'])
            return True

        rospy.logwarn(
            "Goal failed (%s): %s",
            self._status_text(state),
            goal_data['name']
        )
        return False

    # ─────────────────────────────────────────────────────────────
    def _patrol_loop(self):
        rospy.loginfo("Starting patrol loop")
        idx = 0

        while not rospy.is_shutdown():
            goal = self.goals[idx]

            success = False
            for attempt in range(self.max_retries + 1):
                success = self._send_goal(goal)
                if success:
                    break
                rospy.sleep(self.inter_goal_delay)

            if not success:
                rospy.logerr(
                    "Goal failed after retries: %s",
                    goal['name']
                )

            idx = (idx + 1) % len(self.goals)
            rospy.sleep(self.inter_goal_delay)

    # ─────────────────────────────────────────────────────────────
    @staticmethod
    def _status_text(code):
        mapping = {
            GoalStatus.PENDING: "PENDING",
            GoalStatus.ACTIVE: "ACTIVE",
            GoalStatus.PREEMPTED: "PREEMPTED",
            GoalStatus.SUCCEEDED: "SUCCEEDED",
            GoalStatus.ABORTED: "ABORTED",
            GoalStatus.REJECTED: "REJECTED",
            GoalStatus.LOST: "LOST"
        }
        return mapping.get(code, "UNKNOWN")


if __name__ == '__main__':
    try:
        PatrolClient()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
