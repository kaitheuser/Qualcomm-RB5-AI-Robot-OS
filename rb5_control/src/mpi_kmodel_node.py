#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Joy
import sys

rospy.init_node("drive_trial")

joy_msg = Joy()
joy_msg.axes = [0.0 ,0.0 ,0.0 ,0.0 ,0.0 ,0.0 ,0.0 ,0.0]
joy_msg.buttons = [0, 0, 0, 0, 0, 0, 0, 0]
pub = rospy.Publisher('/joy', Joy, queue_size=10)
pub.publish(joy_msg)
rospy.sleep(1.0)

distance = float(sys.argv[1])
fwd_spd = 0.17
fwd_cmd = 0.5

joy_msg.axes[1] = fwd_cmd

pub.publish(joy_msg)
rospy.sleep(distance / fwd_spd)

# current_pos = 0
# fixed_timestamp = 0.01

# while not abs(current_pos - distance) < 0.005:
#     pub.publish(joy_msg)
#     rospy.sleep(fixed_timestamp)
#     current_pos += fwd_spd * fixed_timestamp
#     print (current_pos)

joy_msg.axes[1] = 0
pub.publish(joy_msg)