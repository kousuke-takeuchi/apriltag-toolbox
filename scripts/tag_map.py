#!/usr/bin/python3
# encoding: utf-8
from typing import List, Optional

import cv2

import rospy
from geometry_msgs.msg import Pose
from apriltag_toolbox.msg import AprilTagDetection


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
        # SetCorners(&tag_w->corners, tag_w->pose, tag_w->size);
        # tag_w->center = tag_w->pose.pose.pose.position;

    def add_first_tag(self, tag_c : AprilTagDetection):
        # Create tag in world frame and set to origin
        # Set the first tag to origin
        pose = Pose()
        self._add_tag(tag_c, pose)

    def init(self) -> bool:
        return len(self.tags_w()) > 0

    def first_tag(self) -> Optional[AprilTagDetection]:
        tags = self.tags_w()
        return tags[0] if len(tags) > 0 else None

    def tags_w(self) -> List[AprilTagDetection]:
        return self._tags_w
    

    def estimate_pose(self, tags_c : List[AprilTagDetection], K, D):
        img_pts = []
        obj_pts = []

        for tag_c in tags_c:
            # Find 2D-3D correspondences
            tag_w = self.find_by_id(tag_c.id[0], self.tags_w())
            if tag_w is not None:
                for p in tag_w.corners:


        # std::for_each(tag_w.corners.begin(), tag_w.corners.end(),
        #                 [&obj_pts](const geometry_msgs::Point &p_w) {
        #     obj_pts.emplace_back(p_w.x, p_w.y, p_w.z);
        # });
        # std::for_each(tag_c.corners.begin(), tag_c.corners.end(),
        #                 [&img_pts](const geometry_msgs::Point &p_c) {
        #     img_pts.emplace_back(p_c.x, p_c.y);
        # });

        assert len(img_pts) == len(obj_pts), "size mismatch!"

        if len(img_pts) == 0:
            return

        # Actual pose estimation work here
        cv::Mat c_r_w, c_t_w, c_R_w;

        success, c_r_w, c_t_w = cv2.solvePnP(obj_pts, img_pts, K, D, flags=0)
        c_R_w, Jacob = cv2.Rodrigues(c_r_w)
        w_T_c = -w_R_c.T * c_T_w

        pose.position.x = w_T_c[0]
        pose.position.y = w_T_c[1]
        pose.position.z = w_T_c[2]

        Eigen::Quaterniond w_Q_c = RodriguesToQuat(c_r_w).inverse();
        SetOrientation(&pose->orientation, w_Q_c);
        return true;

    
    ##########
    # private
    ##########
    def _add_tag(self, tag : AprilTagDetection, pose : Pose):
        tag_w : AprilTagDetection = tag
        self.update_tag(tag_w, pose)
        self._tags_w.append(tag_w)
        rospy.loginfo(f"tag {tag.id[0]} added to map")

    def find_by_id(self, id : int, tags : List[AprilTagDetection]) -> Optional[AprilTagDetection]:
        for tag in tags:
            if tag.id[0] == id:
                return tag
        return None