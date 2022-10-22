#!/usr/bin/env python
import sys
import rospy
from geometry_msgs.msg import Twist, PoseArray
import numpy as np
from sensor_msgs.msg import Joy
from pid_controller import PIDcontroller

fwd_vel = 0.17
right_vel = 0.58
fwd_cmd = 0.5
right_cmd = 1.8

joy_pub = rospy.Publisher('/joy', Joy, queue_size = 10)

def movement_cb(data):
    target_state = np.array([0.3, 0.0, 0.0])
    curr_pos = data.poses[0].position
    curr_x, curr_z = curr_pos.x, curr_pos.z
    current_state = np.array([curr_z, curr_x, 0.0])

    pid = PIDcontroller(5,100,1.0)
    error = pid.getError(current_state, target_state)
    if np.all(np.abs(error) < 0.05):
        joy_pub.publish(empty_joy_msg())
        rospy.loginfo("Position Reached")
    else:
        rospy.loginfo("Current error is: ")
        rospy.loginfo(error)
        pid.setTarget(target_state)
        update_value = pid.update(current_state)
        joy_msg = gen_joy_msg(update_value)
        joy_pub.publish(joy_msg)


def gen_joy_msg(update_value):
    joy_msg = Joy()
    joy_msg.axes = [0.0 ,0.0 ,0.0 ,0.0 ,0.0 ,0.0 ,0.0 ,0.0]
    joy_msg.buttons = [0, 0, 0, 0, 0, 0, 0, 0]
    update_move, update_slide, _ = update_value
    joy_msg.axes[1] = -update_move / 1.5 * 0.5
    joy_msg.axes[0] = update_slide / 1.5 * 1.8
    return joy_msg

def empty_joy_msg():
    joy_msg = Joy()
    joy_msg.axes = [0.0 ,0.0 ,0.0 ,0.0 ,0.0 ,0.0 ,0.0 ,0.0]
    joy_msg.buttons = [0, 0, 0, 0, 0, 0, 0, 0]
    return joy_msg


if __name__ == "__main__":
    rospy.init_node('closed_loop')
    while not rospy.is_shutdown():
        pose_sub = rospy.Subscriber('/april_poses', PoseArray, movement_cb)
        rospy.spin()
        