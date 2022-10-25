#!/usr/bin/env python
import rospy
import numpy as np
from pid_controller import PIDcontroller, genTwistMsg, coord
from tf.transformations import euler_from_quaternion
import time
from geometry_msgs.msg import Twist, PoseArray
# from april_detection.msg import AprilTagDetectionArray
from single_pid_controller import SinglePIDController
from sensor_msgs.msg import Joy

# Define global variables: open loop and closed loop status
openLoop_Status = False
closeLoop_Status = False
wp_idx = 0
waypoint_apriltag = np.array([[0.59,0.0,0.0], 
                              [0.50,0.0,0.0],
                              [1.10,0.0,0.0]])

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
    # joy_msg.buttons = [0, 0, 0, 0, 1, 0, 0, 0]
    joy_msg.buttons = [0, 0, 0, 0, 0, 0, 0, 0]
    return joy_msg

# Callback function for visual closed loop control
def vehiclePose_callback(aprilTagPose):
    global openLoop_Status
    global closeLoop_Status
    global waypoint_apriltag
    global wp_idx

    if wp_idx == 3:
        openLoop_Status = False

    if openLoop_Status == True:
        if len(aprilTagPose.poses) != 0:
        #     rospy.loginfo("Searching for April Tag...")
        #     set_twist = np.array([0.0, 0.0, 0.04])
        #     msg_twist = genTwistMsg(set_twist)
        #     pub_twist.publish(msg_twist)
        #     rospy.sleep(0.5)
        #     return
        # else:

            curr_Pose = aprilTagPose.poses[0].position
            curr_Quat = aprilTagPose.poses[0].orientation

            curr_PosX = curr_Pose.x
            curr_PosZ = curr_Pose.z
            _, curr_Agl, _ = euler_from_quaternion([curr_Quat.w,
                                                    curr_Quat.x,
                                                    curr_Quat.y,
                                                    curr_Quat.z])
            # current_state = np.array([curr_PosZ, curr_PosX, curr_Agl])

            pid_x = SinglePIDController(0.8, 1, 1, 1.4, 0.5)
            pid_x.setTarget(waypoint_apriltag[wp_idx][1])
            error_x = pid_x.getError(curr_PosX)

            pid_z = SinglePIDController(1, 5, 1.0, 0.4, 0.4)
            pid_z.setTarget(waypoint_apriltag[wp_idx][0])
            error_z = pid_z.getError(curr_PosZ)

            pid_r = SinglePIDController(0.1, 0.5, 0.5, 0.5, 0.4, True)
            pid_r.setTarget(waypoint_apriltag[wp_idx][2])
            error_r = pid_r.getError(curr_Agl)

            if np.abs(error_r) > 0.10:
                update_value_r = pid_r.update(error_r)
                joy_msg = gen_cmd_msg([0, 0, update_value_r])
                rospy.loginfo("Modifying starting direction")
                rospy.sleep(0.01)
                joy_pub.publish(joy_msg)
            elif np.abs(error_x) < 0.10 and np.abs(error_z) < 0.10 and np.abs(error_r) <= 0.10:
                joy_pub.publish(empty_joy_msg())
                rospy.loginfo("Position Reached")
                rospy.sleep(0.01)
                closeLoop_Status = True
                wp_idx += 1
                rospy.loginfo('Alignment Complete.')
            elif np.abs(error_x) < 0.10 and np.abs(error_z) < 0.10:
                update_value_r = pid_r.update(error_r)
                joy_msg = gen_cmd_msg([0, 0, update_value_r])
                rospy.loginfo("Modifying direction")
                joy_pub.publish(joy_msg)
                rospy.sleep(0.01)
            else:
                rospy.loginfo("Current error is: ")
                print (error_z, error_x, error_r)
                update_value_z = pid_z.update(curr_PosZ)
                update_value_x = pid_x.update(curr_PosX)
                update_value_r = pid_r.update(curr_Agl)
                joy_msg = gen_cmd_msg([update_value_z, update_value_x, 0]) # debug
                joy_pub.publish(joy_msg)
                rospy.sleep(0.01)
            
            # # init pid controller
            # pid = PIDcontroller(0.02,0.005,0.005)
            # pid.setTarget(waypoint_apriltag[wp_idx])
            # update_value = pid.update(current_state)
            # pub_twist.publish(genTwistMsg(coord(update_value, current_state)))
            # time.sleep(0.05)
            # print(np.linalg.norm(pid.getError(current_state, waypoint_apriltag[wp_idx])))

            # if (np.linalg.norm(pid.getError(current_state, waypoint_apriltag[wp_idx])) > 0.10): # check the error between current state and current way point
            #     # calculate the current twist
            #     update_value = pid.update(current_state)
            #     # publish the twist
            #     pub_twist.publish(genTwistMsg(coord(update_value, current_state)))
            #     #print(coord(update_value, current_state))
            #     time.sleep(0.05)
            #     # update the current state
            #     current_state += update_value
            # elif (np.linalg.norm(pid.getError(current_state, waypoint_apriltag[wp_idx])) <= 0.10):
            #     closeLoop_Status = True
            #     wp_idx += 1
            #     rospy.loginfo('Alignment Complete.')


if __name__ == "__main__":

    rospy.init_node("vision_control")
    pub_twist = rospy.Publisher("/twist", Twist, queue_size=1)
    joy_pub = rospy.Publisher('/joy', Joy, queue_size = 1)
    sub_aprilTagPose = rospy.Subscriber('/april_poses', PoseArray, callback=vehiclePose_callback)
    rospy.sleep(0.5)

    # Set target waypoints (x, y, orientation)
    waypoint = np.array([[0.0,0.0,0.0], 
                         [0.5,0.0,0.0],
                         [0.5,1.0,np.pi],
                         [0.0,0.0,0.0]])

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

        # open loop and closed loop status
        openLoop_Status = False
        closeLoop_Status = False

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

        openLoop_Status=True

        # Close loop control to realign itself before moving to the next waypoint
        # After open loop control, april tag is assume to be within view of sight of the robot.
        while (openLoop_Status==True and closeLoop_Status==False):
            #rospy.loginfo("Self-aligning...")
            pass
                
    # stop the car and exit
    pub_twist.publish(genTwistMsg(np.array([0.0,0.0,0.0])))
    openLoop_Status =False