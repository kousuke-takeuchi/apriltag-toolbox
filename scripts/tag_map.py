#!/usr/bin/python3
# encoding: utf-8
from typing import List, Optional

import cv2

import rospy
from geometry_msgs.msg import Pose
from apriltag_ros.msg import AprilTagDetection


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