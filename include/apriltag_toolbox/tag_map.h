#ifndef APRILTAG_TOOLBOX_TAG_MAP_H_
#define APRILTAG_TOOLBOX_TAG_MAP_H_

#include <apriltag_ros/AprilTagDetection.h>
#include <opencv2/core/core.hpp>
#include <set>

namespace apriltag_toolbox {

class TagMap {
 public:
  TagMap() = default;

  void AddOrUpdate(const apriltag_ros::AprilTagDetection& tag_w, const geometry_msgs::Pose& pose);
  void UpdateTag(apriltag_ros::AprilTagDetection* tag_w, const geometry_msgs::Pose& pose);
  void AddFirstTag(const apriltag_ros::AprilTagDetection& tag_c);
//   bool EstimatePose(const std::vector<apriltag_ros::AprilTagDetection>& tags_c, const cv::Matx33d& K,
//                     const cv::Mat_<double>& D, geometry_msgs::Pose* pose) const;

  bool init() const { return !tags_w().empty(); }
  const apriltag_ros::AprilTagDetection& first_tag() const { return tags_w().front(); }
  const std::vector<apriltag_ros::AprilTagDetection>& tags_w() const { return tags_w_; }

 private:
  void AddTag(const apriltag_ros::AprilTagDetection& tag, const geometry_msgs::Pose& pose);

  std::vector<apriltag_ros::AprilTagDetection> tags_w_;
};

std::vector<apriltag_ros::AprilTagDetection>::const_iterator FindById(
    int id, const std::vector<apriltag_ros::AprilTagDetection>& tags);

std::vector<apriltag_ros::AprilTagDetection>::iterator FindById(
    int id, std::vector<apriltag_ros::AprilTagDetection>& tags);
}  // namespace apriltag_toolbox

#endif  // APRILTAG_TOOLBOX_TAG_MAP_H_
