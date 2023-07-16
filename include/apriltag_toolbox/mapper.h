#ifndef APRILTAG_TOOLBOX_ROS_MAPPER_H_
#define APRILTAG_TOOLBOX_ROS_MAPPER_H_

#include <gtsam/inference/Symbol.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/nonlinear/ISAM2.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/slam/BetweenFactor.h>

#include <geometry_msgs/Pose.h>
#include <apriltag_toolbox/Apriltags.h>
#include "apriltag_toolbox/tag_map.h"

namespace apriltag_toolbox {

// Feel like using iSAM2?
class Mapper {
 public:
  static int pose_cnt;

  Mapper(double relinearize_thresh, int relinearize_skip);

  bool init() const { return init_; }
  void Optimize(int num_iterations = 1);
  void Update(apriltag_toolbox::TagMap* map, geometry_msgs::Pose* pose) const;
  void AddPose(const geometry_msgs::Pose& pose);
  void AddFactors(const std::vector<apriltag_toolbox::Apriltag>& tags_c);
  void AddLandmarks(const std::vector<apriltag_toolbox::Apriltag>& tags_c);
  void Initialize(const Apriltag& tag_w);
  void Clear();

 private:
  void AddLandmark(const apriltag_toolbox::Apriltag& tag_w,
                   const gtsam::Pose3& pose);
  void AddPrior(int landmark_id);

  bool init_;
  gtsam::ISAM2Params params_;
  gtsam::ISAM2 isam2_;
  gtsam::NonlinearFactorGraph graph_;
  gtsam::Values initial_estimates_;
  gtsam::Pose3 pose_;
  gtsam::noiseModel::Diagonal::shared_ptr tag_noise_;
  gtsam::noiseModel::Diagonal::shared_ptr small_noise_;
  std::set<int> all_ids_;
  std::map<int, apriltag_toolbox::Apriltag> all_tags_c_;
};

gtsam::Pose3 FromGeometryPose(const geometry_msgs::Pose& pose);
}  // namespace apriltag_toolbox

#endif  // APRILTAG_TOOLBOX_MAPPER_H_
