#!/usr/bin/env python
from tf.transformations import quaternion_matrix
from april_detection.msg import AprilTagDetectionArray
import rospy

rospy.init_node("quat2rot")
msg = rospy.wait_for_message("/apriltag_detection_array", AprilTagDetectionArray, timeout = 100)

tfMat = quaternion_matrix([msg.detections[0].pose.orientation.w,
                            msg.detections[0].pose.orientation.x, 
                            msg.detections[0].pose.orientation.y,
                            msg.detections[0].pose.orientation.z])

tfMat[0][-1], tfMat[1][-1], tfMat[2][-1] = msg.detections[0].pose.position.x, msg.detections[0].pose.position.y, msg.detections[0].pose.position.z

print(tfMat)