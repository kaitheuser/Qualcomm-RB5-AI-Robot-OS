#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
import numpy as np
from pid_controller import PIDcontroller
from general_controller import GeneralController

class OpenLoopController(GeneralController):

    def __init__(self, current_state, cmd_state, threshold = 0.05):
        # self._current_state = current_state
        # self._cmd_state = cmd_state
        # self._pid = PIDcontroller(0.02, 0.005, 0.005)
        # self._pub = rospy.Publisher("/twist", Twist, queue_size=1)
        GeneralController.__init__(
            self, 
            current_state = current_state, 
            cmd_state = cmd_state,
            threshold = threshold,
        )
    

    def move_cb(self, data):
        self._pid.setTarget(self._cmd_state)
        update_value = self._pid.update(self._current_state)
        twist_msg = self.genTwistMsg(
            self.coord(
                twist = update_value, 
                current_state = self._current_state
                )
        )
        self._pub.publish(twist_msg)
        rospy.sleep(0.05)
        self._current_state += update_value

        if(np.linalg.norm(self._pid.getError(self._current_state, self._cmd_state)) > self.threshold): 
            update_value = self._pid.update(self._current_state)
            twist_msg = self.genTwistMsg(
                self.coord(
                    twist = update_value, 
                    current_state = self._current_state
                )
            )
            self._pub.publish(twist_msg)
            rospy.sleep(0.05)
            self._current_state += update_value
        else:
            rospy.loginfo('Open Loop Reached')
            self.reached = True
            self.shutdown_controller(0.05)