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
        rospy.loginfo(f"Critical nodes: {self.critical_nodes}")
        rospy.loginfo(f"Optional nodes: {self.optional_nodes}")
        rospy.loginfo(f"Check interval: {self.check_interval}s")
        
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
                    rospy.logwarn(f"Critical node {node} is not running!")
            
            # Check optional nodes
            for node in self.optional_nodes:
                if node and node not in all_nodes:
                    rospy.loginfo(f"Optional node {node} is not running")
            
            # Log status
            rospy.logdebug(f"Active nodes: {len(all_nodes)}")
            
            rate.sleep()

if __name__ == '__main__':
    try:
        SystemHealthMonitor()
    except rospy.ROSInterruptException:
        pass