#!/usr/bin/env python3

from geometry_msgs.msg import Quaternion
import tf

def yaw_from_quaternion(quaternion):
    e = tf.transformations.euler_from_quaternion(
            (quaternion.x, quaternion.y, quaternion.z, quaternion.w))
    return e[2]

def quaternion_from_yaw(yaw):
    qx,qy,qz,qw  = tf.transformations.quaternion_from_euler(0,0,yaw)
    return Quaternion(x=qx, y=qy, z=qz, w=qw)
