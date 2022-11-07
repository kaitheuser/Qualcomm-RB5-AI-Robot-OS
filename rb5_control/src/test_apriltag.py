#!/usr/bin/env python
from tf.transformations import quaternion_matrix, euler_from_quaternion
from april_detection.msg import AprilTagDetectionArray
import rospy
from math import *
import numpy as np


rospy.init_node("quat2rot")
while True:
    
    msg = rospy.wait_for_message("/apriltag_detection_array", AprilTagDetectionArray, timeout = 100)
    if len(msg.detections) != 0:
        
        for info in msg.detections:
            tag_id = info.id
            _, curr_r, _ = euler_from_quaternion(
                [
                    info.pose.orientation.w,
                    info.pose.orientation.x,
                    info.pose.orientation.y,
                    info.pose.orientation.z,
                ])
            curr_r *= 180/np.pi
            curr_pose = info.pose.position
            curr_x, curr_z = curr_pose.x, curr_pose.z
            angle = atan2(-curr_x,curr_z) * 180/np.pi
            print(tag_id, curr_r, curr_z, -curr_x, len(msg.detections), angle, np.linalg.norm(np.array([curr_x, curr_z])))
