#!/usr/bin/env python
import rospy
import numpy as np
import time
from tf.transformations import euler_from_quaternion
## Controller Libraries
from pid_controller import PIDcontroller, genTwistMsg, coord
from single_pid_controller import SinglePIDController
from mpi_navigator import MegaNavigatorNode
## Messages Libraries
from sensor_msgs.msg import Joy
from april_detection.msg import AprilTagDetectionArray
from geometry_msgs.msg import Twist, PoseArray

# Define global variables:
openLoop_Status = False                         # Open Loop Completion Status
closeLoop_Status = False                        # Closed Loop Completion Status
wp_apt_idx = 0                                  # Waypoint index for Apriltag
waypoints_apriltag = np.array([[0.59,0.0,0.0],   # Each Waypoint based on its nearest
                              [0.50,0.0,0.0],   # AprilTag reference frame
                              [1.10,0.0,0.0]])

# Define global functions for the callbacks
def gen_cmd_msg(cmd_vals):
    """
    Generate Joy Command
    """
    joy_cmd = Joy()
    joy_cmd.axes = [0.0 ,0.0 ,0.0 ,0.0 ,0.0 ,0.0 ,0.0 ,0.0]
    joy_cmd.buttons = [0, 0, 0, 0, 0, 0, 0, 0]
    drive_cmd, slide_cmd, rotate_cmd = cmd_vals
    joy_cmd.axes[1] = -drive_cmd                # +ve forward
    joy_cmd.axes[0] = slide_cmd                 # +ve left
    joy_cmd.axes[2] = -rotate_cmd               # +ve ccw
    return joy_cmd

def visualControl_callback(aprilTagPose):
    """
    Visual Closed Loop Control to realign robot pose.
    """
    global openLoop_Status
    global closeLoop_Status
    global waypoints_apriltag
    global wp_apt_idx

    if wp_apt_idx == len(waypoints_apriltag):
        # Make sure does execute the visual control after completed
        openLoop_Status = False
    
    if openLoop_Status == True and closeLoop_Status == False:
        if len(aprilTagPose.detections) == 0:
            rospy.loginfo("Searching for April Tag...")
            set_twist = np.array([0.0, 0.0, 0.04])
            msg_twist = genTwistMsg(set_twist)
            pub_Twist.publish(msg_twist)
            rospy.sleep(0.5)
        else:
        # If april tag is detected
        # if len(aprilTagPose.detections) != 0:

            # Get April Tage Pose
            curr_Pose = aprilTagPose.detections[0].pose.position
            curr_Quat = aprilTagPose.detections[0].pose.orientation

            curr_PosZ, curr_PosX = curr_Pose.z, curr_Pose.x
            _, curr_Agl, _ = euler_from_quaternion([curr_Quat.w,
                                                    curr_Quat.x,
                                                    curr_Quat.y,
                                                    curr_Quat.z])
            
            # This is actually x-axis map frame
            pid_z = SinglePIDController(1.0, 5.0, 1.0, 1.0, 0.5)
            pid_z.setTarget(waypoints_apriltag[wp_apt_idx][0])
            error_z = pid_z.getError(curr_PosZ)
            # This is actually y-axis map frame
            pid_x = SinglePIDController(1.0, 1.0, 1.0, 1.0, 0.5)
            pid_x.setTarget(waypoints_apriltag[wp_apt_idx][1])
            error_x = pid_x.getError(curr_PosX)
            # Orientation
            pid_r = SinglePIDController(0.1, 0.5, 0.5, 0.5, 0.4, True)
            pid_r.setTarget(waypoints_apriltag[wp_apt_idx][2])
            error_r = pid_r.getError(curr_Agl)

            # Check waypoint reached
            if np.abs(error_x) < 0.08 and np.abs(error_z) < 0.08 and np.abs(error_r) < 0.25:
                pub_Joy.publish(gen_cmd_msg([0.0, 0.0, 0.0]))
                rospy.sleep(0.05)
                rospy.loginfo("Waypoint Reached and Alignment Complete")
                closeLoop_Status = True
                wp_apt_idx += 1
            # Check heading first
            elif np.abs(error_r) >= 0.25:         # About 15 Deg.
                update_value_r = pid_r.update(error_r)
                joy_cmd = gen_cmd_msg([0, 0, update_value_r])
                rospy.loginfo("Adjusting Heading")
                pub_Joy.publish(joy_cmd)
                rospy.sleep(0.05)
            else:
                rospy.loginfo("Current error is: ")
                print (error_z, error_x, error_r)
                update_value_z = pid_z.update(curr_PosZ)
                update_value_x = pid_x.update(curr_PosX)
                update_value_r = pid_r.update(curr_Agl)
                joy_cmd = gen_cmd_msg([update_value_z, update_value_x, 0]) # debug
                pub_Joy.publish(joy_cmd)
                rospy.sleep(0.05)


if __name__ == "__main__":

    # Initialize Node
    rospy.init_node("vision_control")

    # Publishers
    pub_Twist = rospy.Publisher("/twist", Twist, queue_size = 1)
    pub_Joy = rospy.Publisher('/joy', Joy, queue_size = 1)
    # Subscriber
    sub_aprilTagPose = rospy.Subscriber('/apriltag_detection_array', AprilTagDetectionArray, callback=visualControl_callback)

    # Set target waypoints (x, y, orientation)
    waypoints = np.array([[0.0,0.0,0.0], 
                         [0.5,0.0,0.0],
                         [0.5,1.0,np.pi],
                         [0.0,0.0,0.0]])

    # Initialize PID Controller
    pid = PIDcontroller(0.02,0.01,0.01)

    # Initialize the current state
    current_state = waypoints[0,:]

    # Loop through waypoints
    for idx, wp in enumerate(waypoints):

        # Skip the first waypoint (assume it is properly aligned)
        if idx == 0: continue

        # Show next waypoint log
        rospy.loginfo("Next Waypoint ( %s m, %s m, %s rad )", wp[0], wp[1], wp[2])

        # Set Next Waypoint
        pid.setTarget(wp)

        # Calculate the current twist
        update_value = pid.update(current_state)

        # Publish the twist
        pub_Twist.publish(genTwistMsg(coord(update_value, current_state)))
        time.sleep(0.05)

        # Update the current state
        current_state += update_value

        # Open loop control while moving from one way point to another waypoint
        while(np.linalg.norm(pid.getError(current_state, wp)) > 0.05): # check the error between current state and current way point
            
            # Calculate the current twist
            update_value = pid.update(current_state)

            # Publish the twist
            pub_Twist.publish(genTwistMsg(coord(update_value, current_state)))
            time.sleep(0.05)

            # Update the current state
            current_state += update_value

        # Mark open loop control complete
        openLoop_Status = True

        # Close loop control to realign itself before moving to the next waypoint
        # After open loop control, april tag is assume to be within view of sight of the robot.
        while (openLoop_Status==True and closeLoop_Status==False):
            # Execute in callback function
            pass

        # Reset loop status
        closeLoop_Status = False
        openLoop_Status = False

    # Stop the car and exit
    pub_Twist.publish(genTwistMsg(np.array([0.0,0.0,0.0])))
    openLoop_Status =False
            




    

    
    