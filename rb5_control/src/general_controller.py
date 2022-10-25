#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
import numpy as np
from pid_controller import PIDcontroller

class GeneralController:

    def __init__(self, current_state, cmd_state, threshold):
        self._current_state = current_state
        self._cmd_state = cmd_state
        self.threshold = threshold
        self._pid = PIDcontroller(0.02, 0.003, 0.006)
        self._pub = rospy.Publisher("/twist", Twist, queue_size=1)
        self.reached = False
    
    def genTwistMsg(self, desired_twist):
        """
        Convert the twist to twist msg.
        """
        twist_msg = Twist()
        twist_msg.linear.x = desired_twist[0] 
        twist_msg.linear.y = desired_twist[1] 
        twist_msg.linear.z = 0
        twist_msg.angular.x = 0
        twist_msg.angular.y = 0
        twist_msg.angular.z = desired_twist[2]
        return twist_msg
    
    def coord(self, twist, current_state):
        J = np.array([[np.cos(current_state[2]), np.sin(current_state[2]), 0.0],
                    [-np.sin(current_state[2]), np.cos(current_state[2]), 0.0],
                    [0.0,0.0,1.0]])
        return np.dot(J, twist)
    
    def shutdown_controller(self, time = 2.0):
        self._pub.publish(self.genTwistMsg(np.array([0.0,0.0,0.0])))
        rospy.sleep(time) # visual debugging