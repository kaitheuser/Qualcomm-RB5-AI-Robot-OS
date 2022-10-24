#!/usr/bin/env python

from tf.transformations import quaternion_matrix
from math import *

def quat_to_rad(msg):
    # if len(msg) == 0: return None
    tfMat = quaternion_matrix([msg.w, msg.x, msg.y, msg.z])
    return atan2(-tfMat[2][0], sqrt(tfMat[2][1] ** 2 + tfMat[2][2] ** 2))