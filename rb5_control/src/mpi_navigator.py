#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Joy
import numpy as np


class MegaNavigatorNode:
    def __init__(self, init_x, init_y, init_theta):

        # Initialize the ROS node
        rospy.init_node("megapi_navigator")
        # Display log
        rospy.loginfo("MBot Navigator Node Has Been Started.")

        # Define a publisher
        self.nav_pub = rospy.Publisher("/joy", Joy, queue_size=10)

        # Get current waypoint and heading
        self.curr_WPx, self.curr_WPy, self.curr_Head = init_x, init_y, init_theta

        # Initialize joystick message object
        self.joy_msg = Joy()
        # Initialize joystick axis message and buttons
        self.joy_msg.axes = [0.0 ,0.0 ,0.0 ,0.0 ,0.0 ,0.0 ,0.0 ,0.0]
        self.joy_msg.buttons = [0, 0, 0, 0, 0, 0, 0, 0]
        # Publish the joystick message.
        self.nav_pub.publish(self.joy_msg)
        rospy.sleep(0.5) # seconds

        # Parameters
        self.fwd_VEL = 0.17 # m/s
        self.rvs_VEL = 0.19 # m/s
        self.ccw_VEL = 1.20 # rad/s
        self.cw_VEL = 1.15  # rad/s
        self.right_VEL = 0.58 # m/s
        self.left_VEL = 0.52 # m/s


    # Forward / Reverse Drive
    def drive(self, distance):

        # Calculate the time required to run the input distance given the drive velocity
        runtime = (abs(distance) / self.fwd_VEL) if distance > 0 else (abs(distance) / self.rvs_VEL)
        # Set the forward/backward drive magnitude
        self.joy_msg.axes[1] = 0.5 if distance > 0 else -0.5
        # Publish to the topic
        self.nav_pub.publish(self.joy_msg)
        # Run for the calculated amount of time
        rospy.sleep(runtime)
        # Stop the vehicle
        self.joy_msg.axes[1] = 0.0
        # Publish to the topic again
        self.nav_pub.publish(self.joy_msg)

    # CCW / CW Rotation
    def rotate(self, theta):

        # Calculate the time required to run the input distance given the drive velocity
        runtime = (abs(theta) / self.ccw_VEL) if theta > 0 else (abs(theta) / self.cw_VEL)
        # Set the ccw/cw angular velocity magnitude
        self.joy_msg.axes[2] = 0.5 if theta > 0 else -0.5
        # Publish to the topic
        self.nav_pub.publish(self.joy_msg)
        # Run for the calculated amount of time
        rospy.sleep(runtime)
        # Stop the vehicle
        self.joy_msg.axes[2] = 0.0
        # Publish to the topic again
        self.nav_pub.publish(self.joy_msg)

    # Right / Left Slide
    def slide(self, distance):

        # Calculate the time required to run the input distance given the drive velocity
        runtime = (abs(distance) / self.right_VEL) if distance > 0 else (abs(distance) / self.left_VEL)
        # Set the forward/backward drive magnitude
        self.joy_msg.axes[0] = -1.8 if distance > 0 else 1.8
        # Publish to the topic
        self.nav_pub.publish(self.joy_msg)
        # Run for the calculated amount of time
        rospy.sleep(runtime)
        # Stop the vehicle
        self.joy_msg.axes[0] = 0.0
        # Publish to the topic again
        self.nav_pub.publish(self.joy_msg)


if __name__ == '__main__':

    # Load and Read waypoint text file with numpy
    waypoints = np.loadtxt('/home/rosws/src/rb5_ros/waypoints/waypoints.txt', delimiter = ',')
    waypoints = waypoints.tolist()
    # Get current coordinate and heading
    init_x, init_y, init_theta = waypoints.pop(0)

    # Create an object
    mpi_Nav = MegaNavigatorNode(init_x, init_y, init_theta)

    # While waypoints are not empty
    while waypoints:

        # Get Next Waypoint and Heading
        nxt_WPx, nxt_WPy, nxt_head = waypoints.pop(0)

        # Print status
        rospy.loginfo("Next Waypoint ( %s m, %s m )", nxt_WPx, nxt_WPy)

        # Calculate the heading of the next and current waypoints
        wps_vec = [nxt_WPx - mpi_Nav.curr_WPx, nxt_WPy - mpi_Nav.curr_WPy]
        wps_head = np.math.atan2(wps_vec[1], wps_vec[0])
        wps_head = np.floor(wps_head * 100) / 100 if wps_head > 0 else np.ceil(wps_head * 100) / 100

        # Calculate the distance between next and current waypoints.
        dist = np.linalg.norm([nxt_WPx-mpi_Nav.curr_WPx, nxt_WPy-mpi_Nav.curr_WPy])

        # Calculate the angle between two heading vectors
        delta_theta = wps_head - mpi_Nav.curr_Head

        # If the angle between the waypoints vector and the heading is equal or pi.
        if ((delta_theta == 0.0) or 
            (abs(delta_theta) == 3.14)):
            # Go forward if the headings are the same. Or else, reverse.
            if mpi_Nav.curr_Head == wps_head:
                rospy.loginfo("Moving Forward: %s m", dist)
                mpi_Nav.drive(dist)
            else:
                rospy.loginfo("Moving Backward: - %s m", dist)
                mpi_Nav.drive(-dist)
        elif ((delta_theta == 1.57) or 
              (delta_theta == -1.57)):
            # Slide left if the delta_theta is positive 90 degrees. Or else, slide right.
            if delta_theta == 1.57:
                rospy.loginfo("Slide Left: - %s m", dist)
                mpi_Nav.slide(-dist)
            else:
                rospy.loginfo("Slide Right: %s m", dist)
                mpi_Nav.slide(dist)
        else:
            # Print Status
            rospy.loginfo("Rotating CCW: %s rad", delta_theta) if delta_theta > 0 else rospy.loginfo("Rotating CW: - %s rad", delta_theta)
            # Face the next waypoint before moving forward
            mpi_Nav.rotate(delta_theta)
            # Update the MBot heading
            mpi_Nav.curr_Head = wps_head
            rospy.loginfo("Then, Moving Forward: %s m", dist)
            # Move forward
            mpi_Nav.drive(dist)

        # Calculate the angle between the updated MBot heading and the next waypoint heading
        delta_theta2 = nxt_head - mpi_Nav.curr_Head

        if delta_theta2 != 0.0:
            # Print Status
            rospy.loginfo("Rotating CCW: %s rad", delta_theta2) if delta_theta2 > 0 else rospy.loginfo("Rotating CW: - %s rad", delta_theta2)

            # Correct MBot heading to the next waypoint heading
            mpi_Nav.rotate(delta_theta2)

        # Update current position and heading
        mpi_Nav.curr_WPx, mpi_Nav.curr_WPy, mpi_Nav.curr_Head = nxt_WPx, nxt_WPy, nxt_head

        # Print Status
        rospy.loginfo("Waypoint Reached. Moving to Next Waypoint")
        rospy.loginfo("=========================================")

    # Print Status
    rospy.loginfo("***Mission Accomplished***")
