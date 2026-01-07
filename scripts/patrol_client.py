#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""
Elderly Bot Patrol Client

This script implements an autonomous patrol system using the ROS navigation stack.
It loads patrol waypoints from a YAML file and cycles through them indefinitely.

Features:
- ActionClient interface to move_base
- Indefinite patrol loop
- Goal status monitoring
- Recovery from navigation failures
- Configurable via ROS parameters

Usage:
    rosrun elderly_bot patrol_client.py
    
    or with custom patrol file:
    
    rosrun elderly_bot patrol_client.py _patrol_file:=/path/to/patrol.yaml
"""

import rospy
import actionlib
import yaml
import sys
import os
from geometry_msgs.msg import PoseStamped, Quaternion
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from actionlib_msgs.msg import GoalStatus
from tf.transformations import quaternion_from_euler


class PatrolClient(object):
    """
    Patrol client that sends navigation goals to move_base in a loop.
    """
    
    def __init__(self):
        """Initialize the patrol client."""
        
        rospy.init_node('patrol_client', anonymous=False)
        rospy.loginfo("Initializing Patrol Client...")
        
        # Load parameters
        self.patrol_file = rospy.get_param('~patrol_file', 
                                           '$(find elderly_bot)/config/patrol_goals.yaml')
        
        # Expand ROS package paths
        self.patrol_file = self._expand_path(self.patrol_file)
        
        # Timeout for each goal (seconds)
        self.goal_timeout = rospy.get_param('~goal_timeout', 300.0)
        
        # Delay between goals (seconds)
        self.inter_goal_delay = rospy.get_param('~inter_goal_delay', 2.0)
        
        # Maximum retries for a failed goal
        self.max_retries = rospy.get_param('~max_retries', 3)
        
        # Load patrol goals
        self.patrol_goals = self._load_patrol_goals()
        
        if not self.patrol_goals:
            rospy.logerr("No patrol goals loaded! Exiting.")
            sys.exit(1)
        
        rospy.loginfo("Loaded {} patrol goals".format(len(self.patrol_goals)))
        
        # Create action client
        self.client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        
        rospy.loginfo("Waiting for move_base action server...")
        self.client.wait_for_server()
        rospy.loginfo("Connected to move_base action server")
        
        # Statistics
        self.total_goals_sent = 0
        self.total_goals_reached = 0
        self.total_goals_failed = 0
        self.current_patrol_cycle = 0
        
        rospy.loginfo("Patrol Client initialized successfully")
    
    def _expand_path(self, path):
        """
        Expand ROS package paths and environment variables.
        
        Args:
            path: Path string that may contain $(find package_name)
            
        Returns:
            Expanded absolute path
        """
        # Handle $(find package_name) syntax
        if '$(find ' in path:
            import re
            match = re.search(r'\$\(find ([^\)]+)\)', path)
            if match:
                package_name = match.group(1)
                try:
                    from rospkg import RosPack
                    rp = RosPack()
                    package_path = rp.get_path(package_name)
                    path = path.replace('$(find {})'.format(package_name), package_path)
                except Exception as e:
                    rospy.logerr("Failed to find package {}: {}".format(package_name, e))
        
        # Expand environment variables and user home
        path = os.path.expandvars(os.path.expanduser(path))
        
        return path
    
    def _load_patrol_goals(self):
        """
        Load patrol goals from YAML file.
        
        Returns:
            List of goal dictionaries with keys: name, x, y, yaw
        """
        try:
            with open(self.patrol_file, 'r') as f:
                data = yaml.safe_load(f)
                
            if not data or 'patrol_goals' not in data:
                rospy.logerr("Invalid patrol file format. Expected 'patrol_goals' key.")
                return []
            
            goals = data['patrol_goals']
            
            # Validate goals
            valid_goals = []
            for i, goal in enumerate(goals):
                if not all(k in goal for k in ['name', 'x', 'y', 'yaw']):
                    rospy.logwarn("Goal {} missing required fields, skipping".format(i))
                    continue
                valid_goals.append(goal)
            
            return valid_goals
            
        except IOError as e:
            rospy.logerr("Failed to open patrol file {}: {}".format(self.patrol_file, e))
            return []
        except yaml.YAMLError as e:
            rospy.logerr("Failed to parse patrol file: {}".format(e))
            return []
    
    def _create_goal(self, goal_data):
        """
        Create a MoveBaseGoal from goal data.
        
        Args:
            goal_data: Dictionary with keys: name, x, y, yaw
            
        Returns:
            MoveBaseGoal message
        """
        goal = MoveBaseGoal()
        
        # Header
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()
        
        # Position
        goal.target_pose.pose.position.x = float(goal_data['x'])
        goal.target_pose.pose.position.y = float(goal_data['y'])
        goal.target_pose.pose.position.z = 0.0
        
        # Orientation (convert yaw to quaternion)
        yaw = float(goal_data['yaw'])
        q = quaternion_from_euler(0, 0, yaw)
        goal.target_pose.pose.orientation = Quaternion(*q)
        
        return goal
    
    def _send_goal(self, goal_data, retry_count=0):
        """
        Send a navigation goal to move_base.
        
        Args:
            goal_data: Dictionary with goal information
            retry_count: Current retry attempt number
            
        Returns:
            True if goal was reached, False otherwise
        """
        goal_name = goal_data['name']
        
        if retry_count > 0:
            rospy.loginfo("Retry {}/{} for goal: {}".format(
                retry_count, self.max_retries, goal_name))
        else:
            rospy.loginfo("Sending goal: {} (x={:.2f}, y={:.2f}, yaw={:.2f})".format(
                goal_name, goal_data['x'], goal_data['y'], goal_data['yaw']))
        
        # Create and send goal
        goal = self._create_goal(goal_data)
        self.client.send_goal(goal)
        self.total_goals_sent += 1
        
        # Wait for result with timeout
        finished = self.client.wait_for_result(rospy.Duration(self.goal_timeout))
        
        if not finished:
            rospy.logwarn("Goal {} timed out after {} seconds".format(
                goal_name, self.goal_timeout))
            self.client.cancel_goal()
            return False
        
        # Check result
        state = self.client.get_state()
        
        if state == GoalStatus.SUCCEEDED:
            rospy.loginfo("Goal {} reached successfully!".format(goal_name))
            self.total_goals_reached += 1
            return True
        else:
            status_text = self._get_status_text(state)
            rospy.logwarn("Goal {} failed with status: {}".format(goal_name, status_text))
            return False
    
    def _get_status_text(self, status):
        """
        Convert GoalStatus to human-readable text.
        
        Args:
            status: GoalStatus code
            
        Returns:
            Status description string
        """
        status_map = {
            GoalStatus.PENDING: "PENDING",
            GoalStatus.ACTIVE: "ACTIVE",
            GoalStatus.PREEMPTED: "PREEMPTED",
            GoalStatus.SUCCEEDED: "SUCCEEDED",
            GoalStatus.ABORTED: "ABORTED",
            GoalStatus.REJECTED: "REJECTED",
            GoalStatus.PREEMPTING: "PREEMPTING",
            GoalStatus.RECALLING: "RECALLING",
            GoalStatus.RECALLED: "RECALLED",
            GoalStatus.LOST: "LOST"
        }
        return status_map.get(status, "UNKNOWN")
    
    def patrol(self):
        """
        Main patrol loop - cycles through goals indefinitely.
        """
        rospy.loginfo("Starting patrol...")
        rospy.loginfo("Press Ctrl+C to stop")
        
        goal_index = 0
        
        try:
            while not rospy.is_shutdown():
                # Start new patrol cycle
                if goal_index == 0:
                    self.current_patrol_cycle += 1
                    rospy.loginfo("=" * 60)
                    rospy.loginfo("Starting patrol cycle {}".format(self.current_patrol_cycle))
                    rospy.loginfo("=" * 60)
                
                # Get current goal
                goal_data = self.patrol_goals[goal_index]
                
                # Try to reach goal with retries
                success = False
                for retry in range(self.max_retries + 1):
                    success = self._send_goal(goal_data, retry)
                    if success:
                        break
                    
                    if retry < self.max_retries:
                        rospy.loginfo("Retrying in {} seconds...".format(self.inter_goal_delay))
                        rospy.sleep(self.inter_goal_delay)
                
                if not success:
                    rospy.logerr("Failed to reach goal {} after {} attempts".format(
                        goal_data['name'], self.max_retries + 1))
                    self.total_goals_failed += 1
                    # Continue to next goal anyway
                
                # Move to next goal
                goal_index = (goal_index + 1) % len(self.patrol_goals)
                
                # Delay between goals
                if not rospy.is_shutdown():
                    rospy.sleep(self.inter_goal_delay)
                
        except rospy.ROSInterruptException:
            rospy.loginfo("Patrol interrupted by user")
        finally:
            self._print_statistics()
    
    def _print_statistics(self):
        """Print patrol statistics."""
        rospy.loginfo("=" * 60)
        rospy.loginfo("PATROL STATISTICS")
        rospy.loginfo("=" * 60)
        rospy.loginfo("Total patrol cycles: {}".format(self.current_patrol_cycle))
        rospy.loginfo("Total goals sent: {}".format(self.total_goals_sent))
        rospy.loginfo("Goals reached: {}".format(self.total_goals_reached))
        rospy.loginfo("Goals failed: {}".format(self.total_goals_failed))
        
        if self.total_goals_sent > 0:
            success_rate = (float(self.total_goals_reached) / self.total_goals_sent) * 100
            rospy.loginfo("Success rate: {:.1f}%".format(success_rate))
        
        rospy.loginfo("=" * 60)


def main():
    """Main entry point."""
    try:
        patrol_client = PatrolClient()
        patrol_client.patrol()
    except rospy.ROSInterruptException:
        pass
    except Exception as e:
        rospy.logerr("Patrol client error: {}".format(e))
        import traceback
        traceback.print_exc()


if __name__ == '__main__':
    main()


