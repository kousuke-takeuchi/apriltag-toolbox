#!/usr/bin/python3
# encoding: utf-8
from typing import List
import math
import os
import pathlib

import rospy
import tf

from sensor_msgs.msg import CameraInfo
from nav_msgs.msg import Odometry
from apriltag_toolbox.msg import AprilTagDetection, AprilTagDetectionArray
from apriltag_toolbox.srv import SaveMap, SaveMapResponse

from tag_map import TagMap
from mapper import Mapper
from utils import is_inside_image_center, is_ahead_center_and_nearby, mkmat, transform_cv_coord, transform_ros_coord
from visualizer import ApriltagVisualizer, Color

class MapperNode:
    def __init__(self, frame_id):
        self._world_frame_id = frame_id
        self._map = TagMap()
        self._mapper = Mapper(0.04, 1)
        self._tag_viz = ApriltagVisualizer("apriltags_map")
        self._tag_viz.set_color(Color.GREEN)
        self._tag_viz.set_alpha(0.75)
        self._tf_broadcaster = tf.TransformBroadcaster()
        
        self.current_pose = None
        
        map_filepath = rospy.get_param("~map", None)
        if map_filepath is not None:
            self._load_map(map_filepath)
        
        self.ground_mode = rospy.get_param("~ground_mode", False)
        
        self._camera_info = None
        self._K = None
        self._D = None

        self._sub_tags = rospy.Subscriber("apriltags", AprilTagDetectionArray, self._tags_callback, queue_size=1)
        self._sub_cinfo = rospy.Subscriber("camera_info", CameraInfo, self._camera_info_callback, queue_size=1)
        self._sub_odom = rospy.Subscriber("odom", Odometry, self._odom_callback, queue_size=1)
        
        self._save_map_srv = rospy.Service('save_map', SaveMap, self._save_map_callback)


    def get_good_tags(self, tags_c : List[AprilTagDetection]):
        tags_c_good : List[AprilTagDetection] = []
        
        for tag_c in tags_c:
            if is_inside_image_center(tag_c.center[0], tag_c.center[1], self._camera_info.width, self._camera_info.height, 5):
            # if is_ahead_center_and_nearby(tag_c, math.pi / 4):
                if self.ground_mode:
                    tag_c.pose.pose.pose.position.y = 0.0
                    tag_c.pose.pose.pose.orientation.x = 0.0
                    tag_c.pose.pose.pose.orientation.y = 0.0
                    tag_c.pose.pose.pose.orientation.z = 0.0
                    tag_c.pose.pose.pose.orientation.w = 1.0
                tags_c_good.append(tag_c)
        return tags_c_good


    ##########
    # private
    ##########

    def _tags_callback(self, tags_c_msg : AprilTagDetectionArray):
        # Do nothing if no detection, this prevents checking in the following steps
        if len(tags_c_msg.detections) == 0:
            rospy.logwarn("No tags detected.")
            return

        # Do nothing if camera info not received
        if self._camera_info is None:
            rospy.logwarn("No camera info received")
            return

        # Do nothing if there are no good tags close to the center of the image
        tags_c_good = self.get_good_tags(tags_c_msg.detections)
        if len(tags_c_good) == 0:
            rospy.logwarn("No good tags detected.")
            return

        # Initialize map by adding the first tag that is not on the edge of the image
        if not self._map.init():
            self._map.add_first_tag(tags_c_good[0])
            rospy.loginfo("AprilMap initialized.")

        # Do nothing if no pose can be estimated
        # pose = self._map.estimate_pose(tags_c_good, self._K, self._D)
        pose = self.current_pose
        if pose is None:
            rospy.logwarn("No 2D-3D correspondence.")
            return

        # Now that with the initial pose calculated, we can do some mapping
        self._mapper.add_pose(pose)
        self._mapper.add_factors(tags_c_good)
        estimated_pose = None
        if self._mapper.init():
            # This will only add new landmarks
            self._mapper.add_landmarks(tags_c_good)
            self._mapper.optimize()
            # Get latest estimates from mapper and put into map
            estimated_pose = self._mapper.update(self._map)
            # Prepare for next iteration
            self._mapper.clear()
        else:
            # This will add first landmark at origin and fix scale for first pose and
            # first landmark
            self._mapper.initialize(self._map.first_tag())
            
        if not estimated_pose:
            rospy.logwarn("No estimated pose.")
            return

        # Publish camera to world transform
        ros_pose = transform_ros_coord(estimated_pose)
        position = [ros_pose.position.x, ros_pose.position.y, ros_pose.position.z]
        orientation = [ros_pose.orientation.x, ros_pose.orientation.y, ros_pose.orientation.z, ros_pose.orientation.w]
        self._tf_broadcaster.sendTransform(
            position,
            orientation,
            tags_c_msg.header.stamp,
            tags_c_msg.header.frame_id,
            self._world_frame_id,
        )

        # Publish visualisation markers
        self._tag_viz.publish_apriltags_marker(self._map.tags_w(), self._world_frame_id, tags_c_msg.header.stamp)


    def _camera_info_callback(self, cinfo_msg : CameraInfo):
        if self._camera_info is not None:
            self._sub_cinfo.unregister()
            rospy.loginfo(f"{rospy.get_namespace()}: Camera initialized")
            return
        self._camera_info = cinfo_msg
        self._K = mkmat(3, 3, cinfo_msg.K)
        self._D = mkmat(len(cinfo_msg.D), 1, cinfo_msg.D)


    def _save_map_callback(self, msg):
        response = SaveMapResponse()
        filename = msg.path
        if not filename.startswith("/"):
            filename = os.path.join(os.getcwd(), filename)
        p = pathlib.Path(filename)
        if not os.path.exists(p.parent):
            rospy.logerr(f"{rospy.get_namespace()}: {p.parent} does not exist")
            return response
        if not os.path.exists(p):
            os.makedirs(p)
        self._mapper.save_map(p)
        return response
    

    def _load_map(self, filepath: str):
        self._mapper.load_map(pathlib.Path(filepath))
        
    
    def _odom_callback(self, odom: Odometry):
        self.current_pose = odom.pose.pose


if __name__ == '__main__':
    rospy.init_node('mapper')

    try:
        mapper_node = MapperNode("tag_map")
        rospy.spin()
    except Exception as e:
        rospy.logerr(f"{rospy.get_namespace()}: {e}")