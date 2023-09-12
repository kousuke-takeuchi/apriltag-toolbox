#!/usr/bin/python3
# encoding: utf-8
from typing import Set, Dict, List

import gtsam

import rospy
from geometry_msgs.msg import Pose, Point
from apriltag_ros.msg import AprilTagDetection

from tag_map import TagMap


class Mapper:
    _init : bool = False
    pose_cnt : int = 0
    _graph : gtsam.NonlinearFactorGraph = gtsam.NonlinearFactorGraph()
    _initial_estimates : gtsam.Values = gtsam.Values()
    _pose : gtsam.Pose3 = gtsam.Pose3()
    _all_ids : Set[int] = []
    _all_tags_c : Dict[int, AprilTagDetection] = {}

    def __init__(self, relinearize_thresh : float, relinearize_skip : int):
        self._init = False
        # self._params = gtsam.ISAM2Params(gtsam.ISAM2GaussNewtonParams(), relinearize_thresh, relinearize_skip)
        self._params = gtsam.ISAM2Params()
        self._isam2 = gtsam.ISAM2(self._params)

        tag_noise_vector = [0.2,0.2,0.2,0.1,0.1,0.1]
        small_noise_vector = [0.1,0.1,0.1,0.05,0.05,0.05]
        self._tag_noise = gtsam.noiseModel.Diagonal.Sigmas(tag_noise_vector)
        self._small_noise = gtsam.noiseModel.Diagonal.Sigmas(small_noise_vector)


    def init(self) -> bool:
        return _init

    def optimize(self, num_iterations : int = 1):
        self._isam2.update(self._graph, self._initial_estimates)
        if num_iterations > 1:
            for _ in ragne(1, num_iterations):
                self._isam2.update()

    def update(self, map : TagMap, pose : Pose):
        assert len(self._all_ids) == len(self._all_tags_c), "id and size mismatch"
        results = self._isam2.calculateEstimate()
        # Update the current pose
        cam_pose = results[Symbol('x', pose_cnt)]
        pose.position = Point(cam_pose.x, cam_pose.y, cam_pose.z)
        pose.orientation = cam_pose.rotation.toQuaternion()
        # Update the current map
        for tag_id in self._all_ids:
            tag_pose3 = results[Symbol('l', tag_id)]
            tag_pose : Pose = Pose()
            tag_pose.position = Pose(tag_pose3.x, tag_pose3.y, tag_pose3.z)
            tag_pose.orientation = tag_pose3.rotation.toQuaternion()
            # This should not change the size of all_sizes_ because all_sizes_ and
            # all_ids_ should have the same size
            tag_c = self._all_tags_c[tag_id]
            map.add_or_update(tag_c, tag_pose)

    def add_pose(self, pose : Pose):
        self.pose_cnt += 1
        self._pose = self._from_geometry_pose(pose)
        rospy.logwarn(self.pose_cnt)
        rospy.logwarn(f"{self._pose}")
        rospy.logwarn(f"{gtsam.Symbol('x', self.pose_cnt)}")
        self._initial_estimates.insert(gtsam.Symbol('x', self.pose_cnt), self._pose)

    def add_factors(self, tags_c : List[AprilTagDetection]):
        x_i = gtsam.Symbol('x', self.pose_cnt)
        for tag_c in tags_c:
            self._graph.push_back(gtsam.BetweenFactor(x_i, gtsam.Symbol('l', tag_c.id[0]), self._from_geometry_pose(tag_c.pose.pose.pose), self._tag_noise))

    def add_landmarks(self, tags_c : List[AprilTagDetection]):
        for tag_c in tags_c:
            # Only add landmark if it's not already added
            if tag_c.id[0] not in _all_ids:    
                w_T_c = _pose
                c_T_t = self._from_geometry_pose(tag_c.pose.pose.pose)
                w_T_t = w_T_c.compose(c_T_t)
                self._add_landmark(tag_c, w_T_t)

    def initialize(self, tag_w : AprilTagDetection):
        assert self.pose_cnt == 1, "Incorrect initial pose"
        self._add_landmark(tag_w, gtsam.Pose3())
        self._add_prior(tag_w.id[0])
        self._init = True

    def clear(self):
        self._graph.resize(0)
        self._initial_estimates.clear()


    ##########
    # private
    ##########

    def _add_landmark(self, tag_w : AprilTagDetection, pose : gtsam.Pose3):
        self._initial_estimates.insert(gtsam.Symbol('l', tag_c.id[0]), pose)
        self._all_ids.insert(tag_c.id[0])
        self._all_tags_c[tag_c.id[0]] = tag_c

    def _add_prior(self, landmark_id: int):
        # A very strong prior on first pose and landmark
        rospy.loginfo(f"Add pose prior on: {self.pose_cnt}")
        self._graph.push_back(gtsam.PriorFactor(gtsam.Symbol('x', self.pose_cnt), self._pose, self._small_noise))
        rospy.loginfo(f"Add landmark prior on: {landmark_id}")
        self.graph.push_back(gtsam.PriorFactor(gtsam.Symbol('l', landmark_id), gtsam.Pose3(), self._small_noise))

    def _from_geometry_pose(self, pose : Pose) -> gtsam.Pose3:
        t = gtsam.Point3(pose.position.x, pose.position.y, pose.position.z)
        r = gtsam.Rot3(pose.orientation.w, pose.orientation.x, pose.orientation.y, pose.orientation.z)
        return gtsam.Pose3(r, t)
