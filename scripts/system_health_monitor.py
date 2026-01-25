#!/usr/bin/env python2
import rospy
import rosnode

class SystemHealthMonitor:
    def __init__(self):
        rospy.init_node('system_health_monitor')
        
        self.check_interval = rospy.get_param('~check_interval', 5.0)
        self.critical_nodes = rospy.get_param('~critical_nodes', '').split(',')
        self.optional_nodes = rospy.get_param('~optional_nodes', '').split(',')
        
        rospy.loginfo("=== System Health Monitor Started ===")
        rospy.loginfo("Critical nodes: %s", self.critical_nodes)
        rospy.loginfo("Optional nodes: %s", self.optional_nodes)
        rospy.loginfo("Check interval: %s", self.check_interval)
        
        self.monitor_loop()
    
    def check_node(self, node_name):
        try:
            node_api = rosnode.get_api_uri(rospy.get_master(), node_name, skip_cache=True)
            if node_api:
                return True
            return False
        except:
            return False
    
    def monitor_loop(self):
        rate = rospy.Rate(1.0/self.check_interval)
        
        while not rospy.is_shutdown():
            all_nodes = rosnode.get_node_names()
            
            # Check critical nodes
            for node in self.critical_nodes:
                if node and node not in all_nodes:
                    rospy.logwarn("Critical node %s is not running!", node)
            
            # Check optional nodes
            for node in self.optional_nodes:
                if node and node not in all_nodes:
                    rospy.loginfo("Optional node %s is not running", node)
            
            # Log status
            rospy.logdebug("Active nodes: %s", len(all_nodes))
            
            rate.sleep()

if __name__ == '__main__':
    try:
        SystemHealthMonitor()
    except rospy.ROSInterruptException:
        pass