# Autonomous Mobile Robotics Exam - Group 30
#
# KDL helper functions for Task 2.

import math

from geometry_msgs.msg import Quaternion, TransformStamped
from PyKDL import Frame, Rotation, Vector


def yaw_to_quat(yaw: float) -> Quaternion:
    q = Quaternion()
    q.x = 0.0
    q.y = 0.0
    q.z = math.sin(yaw / 2.0)
    q.w = math.cos(yaw / 2.0)
    return q

def quat_to_yaw(qx: float, qy: float, qz: float, qw: float) -> float:
    # 2D yaw from a quaternion. Same as tf_transformations.euler_from_quaternion[2].
    siny_cosp = 2.0 * (qw * qz + qx * qy)
    cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
    return math.atan2(siny_cosp, cosy_cosp)


def transform_to_kdl_frame(tf_msg: TransformStamped) -> Frame:
    p = tf_msg.transform.translation
    r = tf_msg.transform.rotation
    return Frame(Rotation.Quaternion(r.x, r.y, r.z, r.w), Vector(p.x, p.y, p.z))


def frame_to_pos_quat(frame: Frame):
    # Split a KDL Frame into ([x, y, z], [qx, qy, qz, qw]) with the quaternion
    # normalized. pymoveit2.move_to_pose expects the quat in xyzw order.
    pos = [frame.p.x(), frame.p.y(), frame.p.z()]
    qx, qy, qz, qw = frame.M.GetQuaternion()
    n = math.sqrt(qx * qx + qy * qy + qz * qz + qw * qw)
    quat = [qx / n, qy / n, qz / n, qw / n]
    return pos, quat
