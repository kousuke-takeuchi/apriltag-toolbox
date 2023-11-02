from typing import List
import array
import math
import cv2
import numpy as np
import quaternion

import tf
from tf.transformations import quaternion_from_matrix
from geometry_msgs.msg import Vector3, Point, Quaternion, Pose


def quaternion_to_euler(quaternion):
    """Convert Quaternion to Euler Angles

    quarternion: geometry_msgs/Quaternion
    euler: geometry_msgs/Vector3
    """
    e = tf.transformations.euler_from_quaternion((quaternion.x, quaternion.y, quaternion.z, quaternion.w))
    return Vector3(x=e[0], y=e[1], z=e[2])


def euler_to_quaternion(euler):
    """Convert Euler Angles to Quaternion

    euler: geometry_msgs/Vector3
    quarternion: geometry_msgs/Quaternion
    """
    q = tf.transformations.quaternion_from_euler(euler.x, euler.y, euler.z)
    quaternion = Quaternion(x=q[0], y=q[1], z=q[2], w=q[3])
    return quaternion


def euler_from_matrix(matrix):
    quat = quaternion_from_matrix(matrix)
    q = Quaternion(x=quat[0], y=quat[1], z=quat[2], w=quat[3])
    return quaternion_to_euler(q)


# XYZ -> -Y-ZX
def transform_apriltag_coord(cv_pose):
    position = cv_pose.position
    orientation = cv_pose.orientation
    eular = quaternion_to_euler(orientation)
    new_euler = Vector3(x=eular.z, y=eular.x, z=-eular.y)
    
    new_position = Point(x=position.z, y=position.x, z=-position.y)
    new_orientation = tf.transformations.quaternion_from_euler(new_euler.x, new_euler.y, new_euler.z)
    new_pose = Pose(position=new_position, orientation=Quaternion(x=new_orientation[0], y=new_orientation[1], z=new_orientation[2], w=new_orientation[3]))
    return new_pose


def transform_ros_coord(cv_pose):
    position = cv_pose.position
    orientation = cv_pose.orientation
    eular = quaternion_to_euler(orientation)
    new_euler = Vector3(x=eular.x, y=eular.z, z=eular.y)
    
    new_position = Point(x=position.x, y=position.z, z=position.y)
    new_orientation = tf.transformations.quaternion_from_euler(new_euler.x, new_euler.y, new_euler.z)
    new_pose = Pose(position=new_position, orientation=Quaternion(x=new_orientation[0], y=new_orientation[1], z=new_orientation[2], w=new_orientation[3]))
    return new_pose


def transform_viz_coord(cv_pose):
    position = cv_pose.position
    orientation = cv_pose.orientation
    eular = quaternion_to_euler(orientation)
    new_euler = Vector3(x=-eular.x, y=eular.z+math.pi/2, z=-eular.y)
    
    new_position = Point(x=position.x, y=position.z, z=position.y)
    new_orientation = tf.transformations.quaternion_from_euler(new_euler.x, new_euler.y, new_euler.z)
    new_pose = Pose(position=new_position, orientation=Quaternion(x=new_orientation[0], y=new_orientation[1], z=new_orientation[2], w=new_orientation[3]))
    return new_pose


# XYZ -> Z-X-Y
def transform_cv_coord(ros_pose):
    position = ros_pose.position
    orientation = ros_pose.orientation
    eular = quaternion_to_euler(orientation)
    new_euler = Vector3(x=eular.y, y=-eular.z, z=eular.x)

    new_position = Point(x=position.y, y=-position.z, z=position.x)
    new_orientation = tf.transformations.quaternion_from_euler(new_euler.x, new_euler.y, new_euler.z)
    new_pose = Pose(position=new_position, orientation=Quaternion(x=new_orientation[0], y=new_orientation[1], z=new_orientation[2], w=new_orientation[3]))
    return new_pose


def is_inside_image_center(x: float, y: float, w: int, h: int, k: float) -> bool:
    return True
    return x > w / k and y > h / k and x < (w - w / k) and y < (h - h / k)


def is_ahead_center_and_nearby(tag_c, max_angle) -> bool:
    return True
    distance = math.sqrt(tag_c.pose.pose.pose.position.x**2 + tag_c.pose.pose.pose.position.y**2)
    rotation = quaternion_to_euler(tag_c.pose.pose.pose.orientation)
    angle = (rotation.z - math.pi) % (2 * math.pi)
    if tag_c.pose.pose.pose.position.z > 3 and abs(angle) < max_angle:
        return False
    return True


def mkmat(rows, cols, L):
    mat = np.zeros((rows, cols), dtype=np.float64)
    for i in range(rows):
        for j in range(cols):
            mat[i, j] = L[i*cols+j]
    return mat


def vec2skew(v):  # v∈R^3-->v_× (外積作用の行列)                           
    v = v.reshape([3,])
    return np.array([[0,-v[2],v[1]], [v[2],0,-v[0]], [-v[1],v[0],0]])


def skew2vec(a):  # v_×-->v                                                                                                                                                           
    return np.array([a[2,1], a[0,2], a[1,0]])


def rodrigues_to_quat(r):  # R-->q換算
    qr = np.sqrt(1 + np.trace(r)) / 2
    qi = skew2vec(r-r.T) / (4*qr)  # qr≒0(θ≒180度)のとき不安定                                                                                                                      
    return np.array([qr, *qi])


def set_corners(corners: List[float], pose: Pose, tag_size: float) -> List[Point]:
    w_corners: List[Point] = []
    for i in range(len(corners) // 2):
        w_corners.append(Point(x=corners[i*2], y=corners[i*2+1], z=0))
    a = tag_size / 2
    w_Q_b = np.quaternion(pose.orientation.w, pose.orientation.x, pose.orientation.y, pose.orientation.z)
    w_T_b = np.array([pose.position.x, pose.position.y, pose.position.z])
    b_cs = np.array([[-a, -a, 0], [a, -a, 0], [a, a, 0], [-a, a, 0]])
    for i, b_c in enumerate(b_cs):
        w_c = quaternion.as_rotation_matrix(w_Q_b) @ b_c + w_T_b
        w_corners[i].x = w_c[0]
        w_corners[i].y = w_c[1]
        w_corners[i].z = w_c[2]
    return w_corners