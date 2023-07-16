#ifndef APRILTAG_TOOLBOX_TAG_MAP_H_
#define APRILTAG_TOOLBOX_TAG_MAP_H_

#include <apriltag_toolbox/Apriltags.h>
#include <opencv2/core/core.hpp>
#include <set>

namespace apriltag_toolbox {

class TagMap {
 public:
  TagMap() = default;

  void AddOrUpdate(const Apriltag& tag_w, const geometry_msgs::Pose& pose);
  void UpdateTag(Apriltag* tag_w, const geometry_msgs::Pose& pose);
  void AddFirstTag(const Apriltag& tag_c);
  bool EstimatePose(const std::vector<Apriltag>& tags_c, const cv::Matx33d& K,
                    const cv::Mat_<double>& D, geometry_msgs::Pose* pose) const;

  bool init() const { return !tags_w().empty(); }
  const apriltag_toolbox::Apriltag& first_tag() const { return tags_w().front(); }
  const std::vector<apriltag_toolbox::Apriltag>& tags_w() const { return tags_w_; }

 private:
  void AddTag(const Apriltag& tag, const geometry_msgs::Pose& pose);

  std::vector<apriltag_toolbox::Apriltag> tags_w_;
};

std::vector<apriltag_toolbox::Apriltag>::const_iterator FindById(
    int id, const std::vector<apriltag_toolbox::Apriltag>& tags);

std::vector<apriltag_toolbox::Apriltag>::iterator FindById(
    int id, std::vector<apriltag_toolbox::Apriltag>& tags);
}  // namespace apriltag_toolbox

#endif  // APRILTAG_TOOLBOX_TAG_MAP_H_
