#!/usr/bin/env python
from ast import Global
import sys
import rospy
from geometry_msgs.msg import Twist, PoseArray
import numpy as np
from sensor_msgs.msg import Joy
# from pid_controller import PIDcontroller
from single_pid_controller import SinglePIDController
from tf.transformations import euler_from_quaternion

fwd_vel = 0.17
right_vel = 0.58
fwd_cmd = 0.5
right_cmd = 1.8

joy_pub = rospy.Publisher('/joy', Joy, queue_size = 1)

def movement_cb(data):
    # TODO: handle cases where the robot gets lost
    target_state = np.array([0.3, 0.0, 0.0])
    if len(data.poses) == 0: 
        joy_pub.publish(gen_rot_msg())
        rospy.loginfo("Robot Lost")
        return
    curr_pos = data.poses[0].position
    curr_quat = data.poses[0].orientation
    curr_x, curr_z = curr_pos.x, curr_pos.z
    _, curr_r, _ = euler_from_quaternion([curr_quat.w, curr_quat.x, curr_quat.y, curr_quat.z])
    # current_state = np.array([curr_z, curr_x, curr_angle])

    pid_x = SinglePIDController(0.8, 1, 1, 1.4, 0.5)
    pid_x.setTarget(target_state[1])
    error_x = pid_x.getError(curr_x)

    pid_z = SinglePIDController(1, 5, 1.0, 0.4, 0.2)
    pid_z.setTarget(target_state[0])
    error_z = pid_z.getError(curr_z)

    pid_r = SinglePIDController(0.1, 0.5, 0.5, 0.5, 0.3, True)
    pid_r.setTarget(target_state[2])
    error_r = pid_r.getError(curr_r)

    if np.abs(error_r) > 0.25:
        update_value_r = pid_r.update(error_r)
        joy_msg = gen_cmd_msg([0, 0, update_value_r])
        rospy.loginfo("Modifying starting direction")
        rospy.sleep(0.01)
        joy_pub.publish(joy_msg)
    elif np.abs(error_x) < 0.05 and np.abs(error_z) < 0.05 and np.abs(error_r) <= 0.10:
        joy_pub.publish(empty_joy_msg())
        rospy.loginfo("Position Reached")
        rospy.sleep(0.01)
    elif np.abs(error_x) < 0.05 and np.abs(error_z) < 0.05:
        update_value_r = pid_r.update(error_r)
        # update_value_r = 0.3
        joy_msg = gen_cmd_msg([0, 0, update_value_r])
        rospy.loginfo("Modifying direction")
        joy_pub.publish(joy_msg)
        rospy.sleep(0.01)
    else:
        rospy.loginfo("Current error is: ")
        print (error_z, error_x, error_r)
        update_value_z = pid_z.update(curr_z)
        update_value_x = pid_x.update(curr_x)
        update_value_r = pid_r.update(curr_r)
        joy_msg = gen_cmd_msg([update_value_z, update_value_x, 0]) # debug
        joy_pub.publish(joy_msg)
        rospy.sleep(0.01)

def gen_cmd_msg(update_value):
    joy_msg = Joy()
    joy_msg.axes = [0.0 ,0.0 ,0.0 ,0.0 ,0.0 ,0.0 ,0.0 ,0.0]
    joy_msg.buttons = [0, 0, 0, 0, 0, 0, 0, 0]
    update_move, update_slide, update_rotation = update_value
    joy_msg.axes[1] = -update_move 
    joy_msg.axes[0] = update_slide 
    joy_msg.axes[2] = -update_rotation 
    return joy_msg

def gen_rot_msg():
    joy_msg = Joy()
    joy_msg.axes = [0.0 ,0.0 ,0.0 ,0.0 ,0.0 ,0.0 ,0.0 ,0.0]
    joy_msg.buttons = [0, 0, 0, 0, 0, 0, 0, 0]
    joy_msg.axes[2] = 0.3
    return joy_msg

def empty_joy_msg():
    joy_msg = Joy()
    joy_msg.axes = [0.0 ,0.0 ,0.0 ,0.0 ,0.0 ,0.0 ,0.0 ,0.0]
    joy_msg.buttons = [0, 0, 0, 0, 1, 0, 0, 0]
    return joy_msg


if __name__ == "__main__":
    rospy.init_node('closed_loop')
    while not rospy.is_shutdown():
        pose_sub = rospy.Subscriber('/april_poses', PoseArray, movement_cb)
        rospy.spin()
        