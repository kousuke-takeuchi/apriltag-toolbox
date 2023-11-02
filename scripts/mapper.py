#!/usr/bin/python3
# encoding: utf-8
from typing import Set, Dict, List
import pathlib
import pickle

import gtsam

import rospy
from geometry_msgs.msg import Pose, Point, Quaternion
from apriltag_toolbox.msg import AprilTagDetection

from tag_map import TagMap


L = lambda j: int(gtsam.symbol('l', j))
X = lambda i: int(gtsam.symbol('x', i))


class Mapper:
    _init : bool = False
    pose_cnt : int = 0
    _graph : gtsam.NonlinearFactorGraph = gtsam.NonlinearFactorGraph()
    _initial_estimates : gtsam.Values = gtsam.Values()
    _pose : gtsam.Pose3 = gtsam.Pose3()
    _all_ids : Set[int] = set()
    _all_tags_c : Dict[int, AprilTagDetection] = {}
    _current_results: gtsam.Values = gtsam.Values()

    def __init__(self, relinearize_thresh : float, relinearize_skip : int):
        self._init = False
        # self._params = gtsam.ISAM2Params(gtsam.ISAM2GaussNewtonParams(), relinearize_thresh, relinearize_skip)
        self._params = gtsam.ISAM2Params()
        self._isam2 = gtsam.ISAM2(self._params)

        tag_noise_vector = [0.6,0.6,0.6,0.3,0.3,0.3]
        small_noise_vector = [0.1,0.1,0.1,0.05,0.05,0.05]
        self._tag_noise = gtsam.noiseModel.Diagonal.Sigmas(tag_noise_vector)
        self._small_noise = gtsam.noiseModel.Diagonal.Sigmas(small_noise_vector)


    def init(self) -> bool:
        return self._init

    
    def load_map(self, folder: pathlib.Path):
        graph_filename = folder / "graph.g2o"
        self._graph, self._initial_estimates = gtsam.readG2o(str(graph_filename), True)
        pydata_filename = folder / "poses.pkl"
        pydata = pickle.load(open(pydata_filename, "rb"))
        self._all_ids = pydata["all_ids"]
        self._all_tags_c = pydata["all_tags_c"]
        self.pose_cnt = pydata["pose_cnt"]
        self._init = True
    
    
    def save_map(self, folder: pathlib.Path):
        graph_filename = folder / "graph.g2o"
        gtsam.writeG2o(self._graph, self._current_results, str(graph_filename))
        pydata = {
            "all_ids": self._all_ids,
            "all_tags_c": self._all_tags_c,
            "pose_cnt": self.pose_cnt
        }
        pydata_filename = folder / "poses.pkl"
        pickle.dump(pydata, open(pydata_filename, "wb"))


    def optimize(self, num_iterations : int = 1):
        self._isam2.update(self._graph, self._initial_estimates)
        if num_iterations > 1:
            for _ in range(1, num_iterations):
                self._isam2.update()

    def update(self, map : TagMap):
        assert len(self._all_ids) == len(self._all_tags_c), "id and size mismatch"
        results = self._isam2.calculateEstimate()
        self._current_results = results
        # Update the current pose
        cam_pose = results.atPose3(X(self.pose_cnt))
        pose = Pose()
        pose.position = Point(cam_pose.x(), cam_pose.y(), cam_pose.z())
        quat = cam_pose.rotation().toQuaternion()
        pose.orientation = Quaternion(x=quat.x(), y=quat.y(), z=quat.z(), w=quat.w())
        # Update the current map
        for tag_id in self._all_ids:
            tag_pose3 = results.atPose3(L(tag_id))
            tag_pose : Pose = Pose()
            tag_pose.position = Point(tag_pose3.x(), tag_pose3.y(), tag_pose3.z())
            quat = tag_pose3.rotation().toQuaternion()
            tag_pose.orientation = Quaternion(x=quat.x(), y=quat.y(), z=quat.z(), w=quat.w())
            # This should not change the size of all_sizes_ because all_sizes_ and
            # all_ids_ should have the same size
            tag_c = self._all_tags_c[tag_id]
            map.add_or_update(tag_c, tag_pose)
        return pose

    def add_pose(self, pose : Pose):
        self.pose_cnt += 1
        self._pose = self._from_geometry_pose(pose)
        self._initial_estimates.insert(X(self.pose_cnt), self._pose)

    def add_factors(self, tags_c : List[AprilTagDetection]):
        x_i = X(self.pose_cnt)
        for tag_c in tags_c:
            self._graph.push_back(gtsam.BetweenFactorPose3(x_i, L(tag_c.id[0]), self._from_geometry_pose(tag_c.pose.pose.pose), self._tag_noise))

    def add_landmarks(self, tags_c : List[AprilTagDetection]):
        for tag_c in tags_c:
            # Only add landmark if it's not already added
            if tag_c.id[0] not in self._all_ids:
                w_T_c = self._pose
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

    def _add_landmark(self, tag_c : AprilTagDetection, pose : gtsam.Pose3):
        self._initial_estimates.insert(L(tag_c.id[0]), pose)
        self._all_ids.add(tag_c.id[0])
        self._all_tags_c[tag_c.id[0]] = tag_c

    def _add_prior(self, landmark_id: int):
        # A very strong prior on first pose and landmark
        rospy.loginfo(f"Add pose prior on: {self.pose_cnt}")
        self._graph.push_back(gtsam.PriorFactorPose3(X(self.pose_cnt), self._pose, self._small_noise))
        rospy.loginfo(f"Add landmark prior on: {landmark_id}")
        self._graph.push_back(gtsam.PriorFactorPose3(L(landmark_id), gtsam.Pose3(), self._small_noise))

    def _from_geometry_pose(self, pose : Pose) -> gtsam.Pose3:
        t = gtsam.Point3(pose.position.x, pose.position.y, pose.position.z)
        r = gtsam.Rot3(pose.orientation.w, pose.orientation.x, pose.orientation.y, pose.orientation.z)
        return gtsam.Pose3(r, t)
