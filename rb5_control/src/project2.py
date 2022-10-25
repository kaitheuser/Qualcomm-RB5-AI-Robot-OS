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
        [-0.65, 0, 0],
        [-0.85, 0, 0],
        [-0.85, 0, 0],
    ]
)

# waypoints = np.array(
#     [
#         [0.5, 0.0, 0.0],
#         [0.5, 1.0, np.pi],
#     ],
# )
# april_tag_refs = np.array(
#     [
#         [-0.65, 0, 0],
#         [-0.85, 0, 0],
#     ]
# )
current_state = np.array([0.0, 0.0, 0.0])
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
    threshold = 0.05,
)

if __name__ == "__main__":
    rospy.init_node('project2')

    rate = rospy.Rate(10)
    
    while not rospy.is_shutdown():
        # print ('open controller ' + str(open_controller.reached))
        # print ('closed controller ' + str(closed_controller.reached))
        # data = rospy.wait_for_message('/april_poses', PoseArray)
        if open_controller.reached != True:
            func = open_controller.move_cb
            # print ('in open loop')
            # open_sub = rospy.Subscriber('/april_poses', PoseArray, open_controller.move_cb, queue_size=1)         
        elif not closed_controller.reached:
            func = closed_controller.closed_move_cb
            # print ('in closed loop')
            # closed_sub = rospy.Subscriber('/april_poses', PoseArray, closed_controller.closed_move_cb, queue_size=1)
        else:
            # closed_controller.shutdown_controller(2)
            print ('reset')
            open_controller.reached = closed_controller.reached = False
            index += 1
            # if index > 1:
            #     closed_controller.shutdown_controller(2)
            #     break
            waypoint = waypoints[index]
            april_tag_ref = april_tag_refs[index]
            open_controller._cmd_state = waypoint
            closed_controller._cmd_state = april_tag_ref
        rospy.Subscriber('/april_poses', PoseArray, func, queue_size = 5)
    
        rate.sleep()
    