#!/usr/bin/env python2
import rospy
import rosnode
import time

class SystemHealthMonitor:
    def __init__(self):
        rospy.init_node('system_health_monitor')
        
        self.check_interval = rospy.get_param('~check_interval', 10.0)
        self.startup_delay = rospy.get_param('~startup_delay', 30.0)
        self.critical_nodes = rospy.get_param('~critical_nodes', '').split(',')
        self.optional_nodes = rospy.get_param('~optional_nodes', '').split(',')
        
        # Prepend '/' to all node names for accurate comparison
        self.critical_nodes = ['/' + node.strip() for node in self.critical_nodes if node.strip()]
        self.optional_nodes = ['/' + node.strip() for node in self.optional_nodes if node.strip()]
        
        rospy.loginfo("=== System Health Monitor Started ===")
        rospy.loginfo("Critical nodes: %s", self.critical_nodes)
        rospy.loginfo("Optional nodes: %s", self.optional_nodes)
        rospy.loginfo("Check interval: %s seconds", self.check_interval)
        rospy.loginfo("Startup delay: %s seconds", self.startup_delay)
        
        # Wait before first check (let nodes fully start)
        rospy.sleep(self.startup_delay)
        
        self.monitor_loop()
    
    def monitor_loop(self):
        rate = rospy.Rate(1.0 / self.check_interval)
        
        while not rospy.is_shutdown():
            try:
                all_nodes = rosnode.get_node_names()
                
                # Check critical nodes
                for node in self.critical_nodes:
                    if node not in all_nodes:
                        rospy.logwarn("Critical node %s is not running!", node)
                
                # Check optional nodes
                for node in self.optional_nodes:
                    if node not in all_nodes:
                        rospy.logdebug("Optional node %s is not running", node)
                
                # Log overall status less frequently (every ~60s)
                current_time = rospy.get_time()
                if int(current_time) % 60 < self.check_interval:
                    rospy.loginfo("Active nodes: %d / Total expected: %d", 
                                  len(all_nodes), len(self.critical_nodes) + len(self.optional_nodes))
                
            except Exception as e:
                rospy.logerr("Error checking nodes: %s", str(e))
            
            rate.sleep()

if __name__ == '__main__':
    try:
        SystemHealthMonitor()
    except rospy.ROSInterruptException:
        pass