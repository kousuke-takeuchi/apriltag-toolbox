#!/usr/bin/python3
# encoding: utf-8
from typing import List, Optional

import cv2
import numpy as np
import quaternion

import rospy
from geometry_msgs.msg import Pose
from apriltag_toolbox.msg import AprilTagDetection
from utils import rodrigues_to_quat, set_corners


class TagMap:
    _tags_w : List[AprilTagDetection] = []


    def add_or_update(self, tag_w : AprilTagDetection, pose : Pose):
        tag = self._find_by_id(tag_w.id[0], self._tags_w)
        if tag is None:
            # Not in map, add to map
            self._add_tag(tag_w, pose)
        else:
            # Already in map update
            self.update_tag(tag, pose)
            
    def update_tag(self, tag_w : AprilTagDetection, pose : Pose):
        tag_w.pose.pose.pose = pose
        tag_w.corners3 = set_corners(tag_w.corners, tag_w.pose.pose.pose, tag_w.size[0])
    

    def add_first_tag(self, tag_c : AprilTagDetection):
        # Create tag in world frame and set to origin
        # Set the first tag to origin
        pose = Pose()
        pose.position.x = 0
        pose.position.y = 0
        pose.position.z = 0
        pose.orientation.x = 0
        pose.orientation.y = 0
        pose.orientation.z = 0
        pose.orientation.w = 1
        self._add_tag(tag_c, pose)

    def init(self) -> bool:
        return len(self.tags_w()) > 0

    def first_tag(self) -> Optional[AprilTagDetection]:
        tags = self.tags_w()
        return tags[0] if len(tags) > 0 else None

    def tags_w(self) -> List[AprilTagDetection]:
        return self._tags_w
    

    def estimate_pose(self, tags_c : List[AprilTagDetection], K, D):
        img_pts = np.empty((1, 2), dtype=np.float64)
        obj_pts = np.empty((1, 3), dtype=np.float64)

        for tag_c in tags_c:
            # Find 2D-3D correspondences
            tag_w = self._find_by_id(tag_c.id[0], self.tags_w())
            if tag_w is not None:
                for corners in tag_w.corners3:
                    obj_pt = np.array([[corners.x, corners.y, corners.z]])
                    obj_pts = np.append(obj_pts, obj_pt, axis=0)
                img_pts = np.append(img_pts, np.array(tag_c.corners, dtype=np.float64).reshape((4, 2)), axis=0)
        obj_pts = obj_pts[1:, :]
        img_pts = img_pts[1:, :]

        assert len(img_pts) == len(obj_pts), "size mismatch!"

        if len(img_pts) == 0:
            return

        # Actual pose estimation work here
        success, c_r_w, c_t_w = cv2.solvePnP(obj_pts, img_pts, K, D, flags=0)
        
        if not success:
            rospy.logwarn("solvePnP failed")
            return
        
        c_R_w, _ = cv2.Rodrigues(c_r_w.reshape(3))
        w_T_c = -c_R_w.T * c_t_w
        
        pose = Pose()
        pose.position.x = w_T_c[0]
        pose.position.y = w_T_c[1]
        pose.position.z = w_T_c[2]

        quat = rodrigues_to_quat(c_R_w)
        quat = np.quaternion(quat[0], quat[1], quat[2], quat[3])
        w_Q_c = np.quaternion.inverse(quat)
        pose.orientation.x = w_Q_c.x
        pose.orientation.y = w_Q_c.y
        pose.orientation.z = w_Q_c.z
        pose.orientation.w = w_Q_c.w

        return pose

    
    ##########
    # private
    ##########
    def _add_tag(self, tag : AprilTagDetection, pose : Pose):
        self.update_tag(tag, pose)
        self._tags_w.append(tag)
        rospy.loginfo(f"tag {tag.id[0]} added to map")


    def _find_by_id(self, id : int, tags : List[AprilTagDetection]) -> Optional[AprilTagDetection]:
        for tag in tags:
            if tag.id[0] == id:
                return tag
        return None