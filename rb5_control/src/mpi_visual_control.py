#!/usr/bin/env python
import rospy
import numpy as np
from pid_controller import PIDcontroller, genTwistMsg, coord
from tf.transformations import euler_from_quaternion
import time
from geometry_msgs.msg import Twist
from april_detection.msg import AprilTagDetectionArray

def aprilTagPose_callback(poseData: AprilTagDetectionArray):

    # Set target waypoints (x, y, orientation)
    waypoint = np.array([[0.0,0.0,0.0], 
                         [0.5,0.0,0.0],
                         [0.5,1.0,np.pi],
                         [0.0,0.0,0.0]])

    waypoint_apriltag = np.array([[0.2,0.0,0.0], 
                                  [1.0,0.0,0.0],
                                  [0.7,0.0,0.0]])

    # init pid controller
    pid = PIDcontroller(0.02,0.005,0.005)

    # init current state
    current_state = np.array([0.0,0.0,0.0])

    # in this loop we will go through each way point.
    # once error between the current state and the current way point is small enough, 
    # the current way point will be updated with a new point.
    for idx, wp in enumerate(waypoint):
        # Skip the first waypoint (assume it is properly aligned)
        if idx == 0: continue
        print("move to way point", wp)
        # set wp as the target point
        pid.setTarget(wp)
        # calculate the current twist
        update_value = pid.update(current_state)
        # publish the twist
        pub_twist.publish(genTwistMsg(coord(update_value, current_state)))
        #print(coord(update_value, current_state))
        time.sleep(0.05)
        # update the current state
        current_state += update_value
        # Open loop control while moving from one way point to another waypoint
        while(np.linalg.norm(pid.getError(current_state, wp)) > 0.05): # check the error between current state and current way point
            # calculate the current twist
            update_value = pid.update(current_state)
            # publish the twist
            pub_twist.publish(genTwistMsg(coord(update_value, current_state)))
            #print(coord(update_value, current_state))
            time.sleep(0.05)
            # update the current state
            current_state += update_value

        # Closed loop control to calibrate its reached waypoint
        while(np.linalg.norm(pid.getError(current_state, waypoint_apriltag[idx])) > 0.05): # check the error between current state and current way point
            if len(poseData.poses) == 0: 
                desired_twist = np.array([0, 0, 1.18])
                pub_twist.publish(desired_twist)
                rospy.loginfo("Robot Lost")
                return
            

            
            # calculate the current twist
            update_value = pid.update(current_state)
            # publish the twist
            pub_twist.publish(genTwistMsg(coord(update_value, current_state)))
            #print(coord(update_value, current_state))
            time.sleep(0.05)
            # update the current state
            current_state += update_value

        



if __name__ == "__main__":

    rospy.init_node("vision_control")
    pub_twist = rospy.Publisher("/twist", Twist, queue_size=1)
    sub_aprilPose = rospy.Subscriber("/apriltag_detection_array", AprilTagDetectionArray, aprilTagPose_callback)
    rospy.spin()
        
    # stop the car and exit
    pub_twist.publish(genTwistMsg(np.array([0.0,0.0,0.0])))