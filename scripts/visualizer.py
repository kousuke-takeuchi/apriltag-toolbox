#!/usr/bin/python3
# encoding: utf-8
import enum
from typing import List, Set, Tuple

import rospy

from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import ColorRGBA
from apriltag_toolbox.msg import AprilTagDetectionArray, AprilTagDetection

from utils import transform_ros_coord



class Color(enum.Enum):
    RED = (1, 0, 0)
    GREEN = (0, 1, 0)
    BLUE = (0, 0, 1)
    CYAN = (0, 1, 1)
    MAGENTA = (1, 0, 1)
    YELLOW = (1, 1, 0)


class ApriltagVisualizer:
    _color: ColorRGBA = ColorRGBA()
    old_ids: Set[int] = set()
    
    def __init__(self, topic):
        self._pub_markers = rospy.Publisher(topic, MarkerArray, queue_size=1)


    def set_color(self, color: Color):
        self._color.r = color.value[0]
        self._color.g = color.value[1]
        self._color.b = color.value[2]

    
    def set_alpha(self, alpha: float):
        self._color.a = alpha

    
    def publish_apriltags_marker(self, tags: List[AprilTagDetection], frame_id: str, stamp: float):
        # Get new ids
        new_ids: Set[int] = set()
        for tag in tags:
            new_ids.add(tag.id[0])

        # Get union of new and old ids
        union_ids: Set[int] = self.old_ids.union(new_ids)

        # Difference of new and union should be delete
        del_ids: Set[int] = union_ids.difference(new_ids)

        # Add and delete markers
        marker_array: MarkerArray = MarkerArray()
        for tag in tags:
            marker = Marker()
            marker.header.frame_id = frame_id
            marker.header.stamp = stamp
            # marker.ns = tag.family;
            marker.id = tag.id[0]
            marker.type = Marker.CUBE
            marker.action = Marker.ADD
            marker.scale.x = marker.scale.y = tag.size[0]
            marker.scale.z = marker.scale.x / 10
            marker.color = self._color
            marker.pose = transform_ros_coord(tag.pose.pose.pose)
            marker_array.markers.append(marker)

        for id in del_ids:
            marker = Marker()
            marker.header.frame_id = frame_id
            marker.header.stamp = stamp
            marker.ns = "36h11";  # super hacky
            marker.id = id
            marker.action = Marker.DELETE
            marker_array.markers.append(marker)
        
        self._pub_markers.publish(marker_array)
        self.old_ids = new_ids