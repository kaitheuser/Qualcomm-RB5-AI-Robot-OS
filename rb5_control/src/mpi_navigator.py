#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Joy
from mpi_control import MegaPiController
import numpy as np

class MegaNavigatorNode:
    def __init__(self):

        # Initialize the ROS node
        rospy.init_node("megapi_navigator")
        # Display log
        rospy.loginfo("MBot Navigator Node Has Been Started.")

        # Define a publisher
        nav_pub = rospy.Publisher("/joy", Joy, queue_size=10)

        # Load and Read waypoint text file with numpy
        self.waypoints = np.loadtxt('/home/rosws/src/rb5_ros/waypoints/waypoints.txt', delimiter = ',')
        # Get current waypoint and heading
        self.curr_WPx, self.curr_WPy, self.curr_Head = self.waypoints[0,:]

        # Initialize joystick message object
        self.joy_msg = Joy()
        # Initialize joystick axis message and buttons
        self.joy_msg.axes = [0.0 ,0.0 ,0.0 ,0.0 ,0.0 ,0.0 ,0.0 ,0.0]
        self.joy_msg.buttons = [0, 0, 0, 0, 0, 0, 0, 0]

        # Parameters
        self.drive_Vel = 0.1554  # m/s



    def drive(self, distance):
        runtime = abs(distance) / self.drive_Vel
        self.joy_msg.axes[1] = 0.5 if distance > 0 else -0.5
        flag = True
        while not rospy.is_shutdown() or flag:
            rospy.sleep(runtime)
            self.joy_msg.axes[1] = 0.0
            flag = False











if __name__ == '__main__':

    #mpi_Nav = MegaNavigatorNode()
    #mpi_Nav.drive(1.0)

    #while not rospy.is_shutdown():
    from copy import deepcopy
    from std_msgs.msg import Int8
    from math import atan2

    # processing waypoints
    with open('/home/rosws/src/rb5_ros/waypoints/waypoints.txt') as f:
        waypoints_file = f.readlines()

    waypoints_string_ls = [ele[: -1].split(',') for ele in waypoints_file]
    waypoints_string_ls[-1][-1] = '0'
    waypoints_ls = []

    for string_ls in waypoints_string_ls:
        waypoints_ls.append([float(ele) for ele in string_ls])
    
    waypoints_ls = waypoints_ls[: 3]

    speed_ls = [0.35, 0.185, 1.4, 1.3]
    x_speed, y_speed, theta_speed_ccw, theta_speed_cw = speed_ls

    speed_cmd = [1.2, 0.5, 0.5]
    x_cmd, y_cmd, theta_cmd = speed_cmd

    rospy.init_node("auto_driving")
    pub_auto = rospy.Publisher("/auto", Joy, queue_size = 1)
    current_state = [0, 0, 1.57]

    joy_msg = Joy()
    joy_msg.axes = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    joy_msg.buttons = [0, 0, 0, 0, 0, 0, 0, 0]

    pub_auto.publish(joy_msg)
    rospy.sleep(1.0)

    def move_y(delta_y):
        delta_time = abs(delta_y) / y_speed
        curr_msg = deepcopy(joy_msg)
        direction = 1 if delta_y > 0 else -1
        curr_msg.axes[1] = direction * y_cmd
        pub_auto.publish(curr_msg)
        print (delta_time)
        rospy.sleep(delta_time)

    def move_x(delta_x):
        rospy.loginfo("in move x")
        delta_time = abs(delta_x) / x_speed
        curr_msg = deepcopy(joy_msg)
        direction = 1 if delta_x > 0 else -1
        curr_msg.axes[0] = -direction * x_cmd
        rospy.loginfo(delta_time)
        pub_auto.publish(curr_msg)
        print (delta_time)
        rospy.sleep(delta_time)

    def move_theta(delta_theta):
        curr_msg = deepcopy(joy_msg)
        if delta_theta > 0:
            delta_time = abs(delta_theta) / theta_speed_ccw
            curr_msg.axes[2] = theta_cmd
        else:
            delta_time = abs(delta_theta) / theta_speed_cw
            curr_msg.axes[2] = -theta_cmd
        # direction = 1 if delta_theta > 0 else -1
        # curr_msg.axes[2] = direction * theta_cmd
        pub_auto.publish(curr_msg)
        print (delta_time)
        rospy.sleep(delta_time)

    while waypoints_ls:

        # data = rospy.wait_for_message("/auto_stop", Int8)
        # if data.data != 1: break

        w_x, w_y, w_theta = waypoints_ls.pop(0)

        print ([w_x, w_y, w_theta])
        w_x = w_x / 2
        w_y = w_y / 2
        if w_theta < 0:
            w_theta = -w_theta + 3.14
        c_x, c_y, c_theta = current_state
        delta_x, delta_y, delta_theta = w_x - c_x, w_y - c_y, w_theta - c_theta

        print "delta_x " + str(delta_x) + " delta_y " + str(delta_y) + " delta_theta " + str(delta_theta)
        
        if delta_theta == delta_y == delta_x == 0:
            rospy.loginfo("passed")
            # current_state = [w_x, w_y, w_theta]
        # slide
        if delta_theta == delta_y == 0 and delta_x != 0:
            rospy.loginfo("slide")
            move_x(delta_x)
            # current_state = [w_x, w_y, w_theta]
        # move in single y direction:
        elif delta_x == 0:
            rospy.loginfo("head in y")
            move_y(delta_y)
        else:
            moving_direction = atan2(delta_y, delta_x)
            print ("moving direction is ", moving_direction)
            if moving_direction < 0: moving_direction += 2 * 3.14
            delta_moving_direction = moving_direction - c_theta
            rospy.loginfo("head to moving direction")
            move_theta(delta_moving_direction)
            delta_theta = w_theta - delta_moving_direction
            moving_distance = np.sqrt(delta_x ** 2 + delta_y ** 2)
            rospy.loginfo("head in y in moving direction")
            move_y(moving_distance)
        
        rospy.loginfo("adjust direction")
        if delta_theta != 0: move_theta(delta_theta)
        current_state = [w_x, w_y, w_theta]


        pub_auto.publish(joy_msg)
        rospy.sleep(1.0)

    pub_auto.publish(joy_msg)
        

            

