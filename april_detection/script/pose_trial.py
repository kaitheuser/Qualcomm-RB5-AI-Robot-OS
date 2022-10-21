#!/usr/bin/env python  

import rospy
from geometry_msgs.msg import PoseArray

rospy.init_node("pose_trial")

msg = rospy.wait_for_message('/april_poses', topic_type=PoseArray)
# print (type(msg.poses))
# print (len(msg.poses))

# print (type(msg))

print ("length of pose array is " + str(len(msg.poses)))

for i in range(len(msg.poses)):
    print (i)
    print (msg.poses[i])