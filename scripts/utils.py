from typing import List

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


def euler_from_matrix(matrix):
    quat = quaternion_from_matrix(matrix)
    q = Quaternion(x=quat[0], y=quat[1], z=quat[2], w=quat[3])
    return quaternion_to_euler(q)


# XYZ -> -Y-ZX
def trasform_ros_coord(cv_pose):
    position = cv_pose.position
    orientation = cv_pose.orientation
    eular = quaternion_to_euler(orientation)
    new_euler = Vector3(x=eular.z, y=-eular.x, z=-eular.y)
    
    new_position = Point(x=position.z, y=-position.x, z=-position.y)
    new_orientation = tf.transformations.quaternion_from_euler(new_euler.x, new_euler.y, new_euler.z)
    new_pose = Pose(position=new_position, orientation=Quaternion(x=new_orientation[0], y=new_orientation[1], z=new_orientation[2], w=new_orientation[3]))
    return new_pose

def transform_cv_coord(ros_pose):
    pass