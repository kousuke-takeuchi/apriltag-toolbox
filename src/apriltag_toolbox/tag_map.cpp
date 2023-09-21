#include "apriltag_toolbox/tag_map.h"
#include "apriltag_toolbox/utils.h"

#include <ros/ros.h>
// #include <opencv2/calib3d/calib3d.hpp>
namespace apriltag_toolbox {

void TagMap::AddOrUpdate(const apriltag_ros::AprilTagDetection &tag_w,
                         const geometry_msgs::Pose &pose) {
  auto it = FindById(tag_w.id[0], tags_w_);
  if (it == tags_w().end()) {
    // Not in map, add to map
    AddTag(tag_w, pose);
  } else {
    // Already in map update
    UpdateTag(&(*it), pose);
  }
}

void TagMap::UpdateTag(apriltag_ros::AprilTagDetection *tag_w, const geometry_msgs::Pose &pose) {
  tag_w->pose.pose.pose = pose;
  // SetCorners(&tag_w->corners, tag_w->pose, tag_w->size);
  // tag_w->center = tag_w->pose.pose.pose.position;
}

void TagMap::AddTag(const apriltag_ros::AprilTagDetection &tag, const geometry_msgs::Pose &pose) {
  apriltag_ros::AprilTagDetection tag_w = tag;
  UpdateTag(&tag_w, pose);
  tags_w_.push_back(tag_w);
  ROS_INFO("tag %d added to map", tag.id[0]);
}

void TagMap::AddFirstTag(const apriltag_ros::AprilTagDetection &tag_c) {
  // Creat tag in world frame and set to origin
  // Set the first tag to origin
  geometry_msgs::Pose pose;
  SetPose(&pose);
  AddTag(tag_c, pose);
}

// bool TagMap::EstimatePose(const std::vector<apriltag_ros::AprilTagDetection> &tags_c,
//                           const cv::Matx33d &K, const cv::Mat_<double> &D,
//                           geometry_msgs::Pose *pose) const {
//   std::vector<cv::Point2f> img_pts;
//   std::vector<cv::Point3f> obj_pts;

//   for (const apriltag_ros::AprilTagDetection &tag_c : tags_c) {
//     // Find 2D-3D correspondences
//     auto it = FindById(tag_c.id[0], tags_w());
//     if (it != tags_w().cend()) {
//       const apriltag_ros::AprilTagDetection &tag_w = *it;
//       std::for_each(tag_w.corners.begin(), tag_w.corners.end(),
//                     [&obj_pts](const geometry_msgs::Point &p_w) {
//         obj_pts.emplace_back(p_w.x, p_w.y, p_w.z);
//       });
//       std::for_each(tag_c.corners.begin(), tag_c.corners.end(),
//                     [&img_pts](const geometry_msgs::Point &p_c) {
//         img_pts.emplace_back(p_c.x, p_c.y);
//       });
//     }
//   }

//   ROS_ASSERT_MSG(img_pts.size() == obj_pts.size(), "size mismatch!");
//   if (img_pts.empty()) return false;
//   Actual pose estimation work here
//   cv::Mat c_r_w, c_t_w, c_R_w;
//   cv::solvePnP(obj_pts, img_pts, K, D, c_r_w, c_t_w);
//   cv::Rodrigues(c_r_w, c_R_w);
//   cv::Mat c_T_w(c_t_w);
//   cv::Mat w_R_c(c_R_w.t());
//   cv::Mat w_T_c = -w_R_c * c_T_w;

//   double *pt = w_T_c.ptr<double>();
//   SetPosition(&pose->position, pt[0], pt[1], pt[2]);

//   Eigen::Quaterniond w_Q_c = RodriguesToQuat(c_r_w).inverse();
//   SetOrientation(&pose->orientation, w_Q_c);
//   return true;
// }

std::vector<apriltag_ros::AprilTagDetection>::const_iterator FindById(
    int id, const std::vector<apriltag_ros::AprilTagDetection> &tags) {
  return std::find_if(tags.cbegin(), tags.cend(),
                      [&id](const apriltag_ros::AprilTagDetection &tag) { return id == tag.id[0]; });
}

std::vector<apriltag_ros::AprilTagDetection>::iterator FindById(
    int id, std::vector<apriltag_ros::AprilTagDetection> &tags) {
  return std::find_if(tags.begin(), tags.end(),
                      [&id](const apriltag_ros::AprilTagDetection &tag) { return id == tag.id[0]; });
}

}  // namespace apriltag_toolbox
