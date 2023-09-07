#ifndef APRILTAG_TOOLBOX_MAPPER_NODE_H_
#define APRILTAG_TOOLBOX_MAPPER_NODE_H_

#include <ros/ros.h>
#include <apriltag_ros/AprilTagDetection.h>
#include <apriltag_ros/AprilTagDetectionArray.h>
#include <sensor_msgs/CameraInfo.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/TransformStamped.h>
#include <image_geometry/pinhole_camera_model.h>
#include "apriltag_toolbox/visualizer.h"
#include <tf2_ros/transform_broadcaster.h>

#include "apriltag_toolbox/mapper.h"
#include "apriltag_toolbox/tag_map.h"

namespace apriltag_toolbox {

class MapperNode {
 public:
  MapperNode(const ros::NodeHandle& nh, const std::string& frame_id)
      : nh_(nh),
        sub_tags_(nh_.subscribe("apriltags", 1, &MapperNode::TagsCallback, this)),
        sub_cinfo_(nh_.subscribe("camera_info", 1, &MapperNode::CameraInfoCallback, this)),
        frame_id_(frame_id),
        mapper_(0.04, 1),
        tag_viz_(nh, "apriltags_map") {
    		tag_viz_.SetColor(apriltag_toolbox::GREEN);
    		tag_viz_.SetAlpha(0.75);
  	}

  bool GetGoodTags(const std::vector<apriltag_ros::AprilTagDetection> tags_c,
                   std::vector<apriltag_ros::AprilTagDetection>* tags_c_good);

 private:
  void TagsCallback(const apriltag_ros::AprilTagDetectionArrayConstPtr& tags_c_msg);
  void CameraInfoCallback(const sensor_msgs::CameraInfoConstPtr& cinfo_msg);

  ros::NodeHandle nh_;
  ros::Subscriber sub_tags_;
  ros::Subscriber sub_cinfo_;
  std::string frame_id_;
  apriltag_toolbox::TagMap map_;
  apriltag_toolbox::Mapper mapper_;
  apriltag_toolbox::ApriltagVisualizer tag_viz_;
  image_geometry::PinholeCameraModel model_;
  tf2_ros::TransformBroadcaster tf_broadcaster_;

};

}  // namespace apriltag_toolbox

#endif  // APRILTAG_TOOLBOX_MAPPER_NODE_H_
