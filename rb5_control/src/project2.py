#!/usr/bin/env python

import rospy
import numpy as np
from open_loop_controller import OpenLoopController
from closed_loop_controller import ClosedLoopController
from geometry_msgs.msg import PoseArray


waypoints = np.array(
    [
        [0.5, 0.0, 0.0],
        [0.5, 1.0, np.pi],
        [0.0, 0.0, 0.0],
    ],
)
april_tag_refs = np.array(
    [
        [0.3, 0, 0],
        [0.5, 0, 0],
        [0.8, 0, 0],
    ]
)
current_state = np.array([0.0,0.0,0.0])
index = 0

waypoint = waypoints[index]
april_tag_ref = april_tag_refs[index]

open_controller = OpenLoopController(
    current_state = current_state,
    cmd_state = waypoint,
    threshold = 0.05,
)

closed_controller = ClosedLoopController(
    current_state = current_state,
    cmd_state = april_tag_ref,
    threshold = 0.1,
)

if __name__ == "__main__":
    rospy.init_node('project2')

    rate = rospy.Rate(10)
    
    while not rospy.is_shutdown():
        # print ('open controller ' + str(open_controller.reached))
        # print ('closed controller ' + str(closed_controller.reached))
        if not open_controller.reached:
            rospy.Subscriber('/april_poses', PoseArray, open_controller.move_cb)         
        elif not closed_controller.reached:
            rospy.Subscriber('/april_poses', PoseArray, closed_controller.move_cb)
        else:
            print ('reset')
            open_controller.reached = closed_controller.reached = False
            index += 1
            waypoint = waypoints[index]
            april_tag_ref = april_tag_refs[index]
            open_controller._cmd_state = waypoint
            closed_controller._cmd_state = april_tag_ref
    
    rate.sleep()