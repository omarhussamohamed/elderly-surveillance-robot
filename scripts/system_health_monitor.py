#!/usr/bin/env python2
import rospy
import rosnode
import time

class SystemHealthMonitor:
    def __init__(self):
        rospy.init_node('system_health_monitor')
        
        self.check_interval = rospy.get_param('~check_interval', 5.0)
        self.critical_nodes = rospy.get_param('~critical_nodes', '').split(',')
        self.optional_nodes = rospy.get_param('~optional_nodes', '').split(',')
        
        rospy.loginfo("=== System Health Monitor Started ===")
        rospy.loginfo("Critical nodes: %s", self.critical_nodes)
        rospy.loginfo("Optional nodes: %s", self.optional_nodes)
        rospy.loginfo("Check interval: %s seconds", self.check_interval)
        
        # Wait 15 seconds before first check (let nodes start)
        rospy.sleep(15)
        
        self.monitor_loop()
    
    def monitor_loop(self):
        rate = rospy.Rate(1.0/self.check_interval)
        
        while not rospy.is_shutdown():
            try:
                all_nodes = rosnode.get_node_names()
                
                # Check critical nodes
                for node in self.critical_nodes:
                    node = node.strip()
                    if node and node not in all_nodes:
                        rospy.logwarn("Critical node %s is not running!", node)
                
                # Check optional nodes
                for node in self.optional_nodes:
                    node = node.strip()
                    if node and node not in all_nodes:
                        rospy.logdebug("Optional node %s is not running", node)
                
                # Log status (less frequent)
                if rospy.get_time() % 30 < 1:  # Every 30 seconds
                    rospy.loginfo("Active nodes: %d", len(all_nodes))
                
            except Exception as e:
                rospy.logerr("Error checking nodes: %s", str(e))
            
            rate.sleep()

if __name__ == '__main__':
    try:
        SystemHealthMonitor()
    except rospy.ROSInterruptException:
        pass