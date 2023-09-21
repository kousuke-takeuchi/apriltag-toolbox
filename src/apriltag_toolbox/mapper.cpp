#include "apriltag_toolbox/mapper.h"
#include "apriltag_toolbox/utils.h"
#include <gtsam/slam/PriorFactor.h>
#include <ros/ros.h>

namespace apriltag_toolbox {

using namespace gtsam;

int Mapper::pose_cnt = 0;

Mapper::Mapper(double relinearize_thresh, int relinearize_skip)
    : init_(false),
      params_(ISAM2GaussNewtonParams(), relinearize_thresh, relinearize_skip),
      isam2_(params_) {
  Vector tag_noise_vector(6);
  tag_noise_vector << 0.2,0.2,0.2,0.1,0.1,0.1;
  Vector small_noise_vector(6);
  small_noise_vector << 0.1,0.1,0.1,0.05,0.05,0.05;
  tag_noise_=noiseModel::Diagonal::Sigmas(tag_noise_vector);
  small_noise_=noiseModel::Diagonal::Sigmas(small_noise_vector);
}

void Mapper::AddPose(const geometry_msgs::Pose &pose) {
  pose_cnt++;
  pose_ = FromGeometryPose(pose);
  initial_estimates_.insert(Symbol('x', pose_cnt), pose_);
}

void Mapper::Initialize(const apriltag_ros::AprilTagDetection &tag_w) {
  ROS_ASSERT_MSG(pose_cnt == 1, "Incorrect initial pose");
  AddLandmark(tag_w, Pose3());
  AddPrior(tag_w.id[0]);
  init_ = true;
}

void Mapper::AddPrior(int landmark_id) {
  // A very strong prior on first pose and landmark
  ROS_INFO("Add pose prior on: %d", pose_cnt);
  graph_.push_back(
      PriorFactor<Pose3>(Symbol('x', pose_cnt), pose_, small_noise_));
  ROS_INFO("Add landmark prior on: %d", landmark_id);
  graph_.push_back(
      PriorFactor<Pose3>(Symbol('l', landmark_id), Pose3(), small_noise_));
}

void Mapper::AddLandmark(const apriltag_ros::AprilTagDetection &tag_c, const Pose3 &pose) {
  initial_estimates_.insert(Symbol('l', tag_c.id[0]), pose);
  all_ids_.insert(tag_c.id[0]);
  all_tags_c_[tag_c.id[0]] = tag_c;
}

void Mapper::AddLandmarks(const std::vector<apriltag_ros::AprilTagDetection> &tags_c) {
  for (const apriltag_ros::AprilTagDetection &tag_c : tags_c) {
    // Only add landmark if it's not already added
    if (all_ids_.find(tag_c.id[0]) == all_ids_.end()) {
      const Pose3 &w_T_c = pose_;
      const Pose3 c_T_t = FromGeometryPose(tag_c.pose.pose.pose);
      const Pose3 w_T_t = w_T_c.compose(c_T_t);
      AddLandmark(tag_c, w_T_t);
    }
  }
}

void Mapper::AddFactors(const std::vector<apriltag_ros::AprilTagDetection> &tags_c) {
  Symbol x_i('x', pose_cnt);
  for (const apriltag_ros::AprilTagDetection &tag_c : tags_c) {
    graph_.push_back(BetweenFactor<Pose3>(
        x_i, Symbol('l', tag_c.id[0]), FromGeometryPose(tag_c.pose.pose.pose), tag_noise_));
  }
}

void Mapper::Optimize(int num_iterations) {
  isam2_.update(graph_, initial_estimates_);
  if (num_iterations > 1) {
    for (int i = 1; i < num_iterations; ++i) {
      isam2_.update();
    }
  }
}

void Mapper::Update(TagMap *map, geometry_msgs::Pose *pose) const {
  ROS_ASSERT_MSG(all_ids_.size() == all_tags_c_.size(), "id and size mismatch");
  Values results = isam2_.calculateEstimate();
  // Update the current pose
  const Pose3 &cam_pose = results.at<Pose3>(Symbol('x', pose_cnt));
  SetPosition(&pose->position, cam_pose.x(), cam_pose.y(), cam_pose.z());
  SetOrientation(&pose->orientation, cam_pose.rotation().toQuaternion());
  // Update the current map
  for (const int tag_id : all_ids_) {
    const Pose3 &tag_pose3 = results.at<Pose3>(Symbol('l', tag_id));
    geometry_msgs::Pose tag_pose;
    SetPosition(&tag_pose.position, tag_pose3.x(), tag_pose3.y(),
                tag_pose3.z());
    SetOrientation(&tag_pose.orientation, tag_pose3.rotation().toQuaternion());
    // This should not change the size of all_sizes_ because all_sizes_ and
    // all_ids_ should have the same size
    auto it = all_tags_c_.find(tag_id);
    map->AddOrUpdate(it->second, tag_pose);
  }
}

void Mapper::Clear() {
  graph_.resize(0);
  initial_estimates_.clear();
}

Pose3 FromGeometryPose(const geometry_msgs::Pose &pose) {
  Point3 t(pose.position.x, pose.position.y, pose.position.z);
  print(pose.orientation)
  Rot3 R(Eigen::Quaterniond(pose.orientation.w, pose.orientation.x,
                            pose.orientation.y, pose.orientation.z));
  return Pose3(R, t);
}

}  // namespace apriltag_toolbox
