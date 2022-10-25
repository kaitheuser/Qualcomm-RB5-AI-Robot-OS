#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist, PoseArray
import numpy as np
from pid_controller import PIDcontroller
from general_controller import GeneralController
from tf.transformations import euler_from_quaternion

class ClosedLoopController(GeneralController):

    def __init__(self, current_state, cmd_state, threshold = 1.0):
        GeneralController.__init__(
            self, 
            current_state = current_state, 
            cmd_state = cmd_state,
            threshold = threshold,
        )
    

    def closed_move_cb(self, data):
        '''
        Input: 
            data: PoseArray
        '''
        # print ('in closed loop callback')
        # print (data.poses[0].position)
        if len(data.poses) == 0:
            rospy.loginfo("Robot lost")
            desired_twist = np.array([0.0, 0.0, 0.0])
            # TODO: figure out a reasonable value here, controls rotation speed
            desired_twist[2] = 0.05
            twist_msg = self.genTwistMsg(desired_twist)
            self._pub.publish(twist_msg)
            return
        curr_pose = data.poses[0].position
        curr_quat = data.poses[0].orientation
        curr_x, curr_z = curr_pose.x, curr_pose.z
        _, curr_r, _ = euler_from_quaternion(
            [
                curr_quat.w,
                curr_quat.x,
                curr_quat.y,
                curr_quat.z,
            ]
        )

        self._current_state = np.array([-curr_z, curr_x, curr_r])
        # print (self._current_state)
        # print (self._cmd_state)
        self._pid.setTarget(self._cmd_state)
        errors = self._pid.getError(self._current_state, self._cmd_state)
        print ('closed loop')
        print (self._current_state)
        print (self._cmd_state)
        print (self._pid.getError(self._current_state, self._cmd_state))
        # # update_value = self._pid.update(self._current_state)
        # # twist_msg = self.genTwistMsg(
        # #     self.coord(
        # #         twist = update_value, 
        # #         current_state = self._current_state
        # #         )
        # # )
        # # self._pub.publish(twist_msg)
        # # rospy.sleep(0.05)

        # print (np.linalg.norm(self._pid.getError(self._current_state, self._cmd_state)))
        # print (self.threshold)
        
        # TODO: tune the threshold here for future implementation
        if(abs(errors[0]) > 0.10 or abs(errors[1]) > 0.10 or abs(errors[2]) > 0.10): 
            update_value = self._pid.update(self._current_state)
            twist_msg = self.genTwistMsg(
                self.coord(
                    twist = update_value, 
                    current_state = self._current_state
                )
            )
            self._pub.publish(twist_msg)
            rospy.sleep(0.05)
        else:
            rospy.loginfo('Closed Loop Reached')
            self.reached = True
            # self.shutdown_controller(0.05)