#include <apriltag_toolbox/helpers.h>

#include <assert.h>
#include <signal.h>
#include <sys/time.h>
#include <unistd.h>

#include <ros/ros.h>
#include <tf/transform_datatypes.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <visualization_msgs/Marker.h>

#include "apriltag_ros/Fiducial.h"
#include "apriltag_ros/FiducialArray.h"
#include "apriltag_ros/FiducialTransform.h"
#include "apriltag_ros/FiducialTransformArray.h"

#include "apriltag_toolbox/map.h"

#include <opencv2/calib3d.hpp>
#include <opencv2/highgui.hpp>

#include <list>
#include <string>

using namespace std;
using namespace cv;

class FiducialSlam {
private:
    ros::Subscriber ft_sub;

    bool use_fiducial_area_as_weight;
    double weighting_scale;

    void transformCallback(const apriltag_ros::FiducialTransformArray::ConstPtr &msg);

public:
    Map fiducialMap;
    int pose_publish_rate;
    FiducialSlam(ros::NodeHandle &nh);
};

void FiducialSlam::transformCallback(const apriltag_ros::FiducialTransformArray::ConstPtr &msg) {
    vector<Observation> observations;

    for (size_t i = 0; i < msg->transforms.size(); i++) {
        const apriltag_ros::FiducialTransform &ft = msg->transforms[i];

        tf2::Vector3 tvec(ft.transform.translation.x, ft.transform.translation.y,
                          ft.transform.translation.z);

        tf2::Quaternion q(ft.transform.rotation.x, ft.transform.rotation.y, ft.transform.rotation.z,
                          ft.transform.rotation.w);

        double variance;
        if (use_fiducial_area_as_weight) {
            variance = weighting_scale / ft.fiducial_area;
        } else {
            variance = weighting_scale * ft.object_error;
        }

        Observation obs(ft.fiducial_id, tf2::Stamped<TransformWithVariance>(
                                            TransformWithVariance(ft.transform, variance),
                                            msg->header.stamp, msg->header.frame_id));
        observations.push_back(obs);
    }

    fiducialMap.update(observations, msg->header.stamp);
}

FiducialSlam::FiducialSlam(ros::NodeHandle &nh) : fiducialMap(nh) {

    // If set, use the fiducial area in pixels^2 as an indication of the
    // 'goodness' of it. This will favor fiducials that are close to the
    // camera and center of the image. The reciprical of the area is actually
    // used, in place of reprojection error as the estimate's variance
    nh.param<bool>("use_fiducial_area_as_weight", use_fiducial_area_as_weight, false);
    // Scaling factor for weighing
    nh.param<double>("weighting_scale", weighting_scale, 1e9);
    nh.param<int>("pose_publish_rate", pose_publish_rate, 20);

    ft_sub = nh.subscribe("/fiducial_transforms", 1, &FiducialSlam::transformCallback, this);

    ROS_INFO("Fiducial Slam ready");
}

auto node = unique_ptr<FiducialSlam>(nullptr);

void mySigintHandler(int sig) {
    if (node != nullptr) node->fiducialMap.saveMap();

    ros::shutdown();
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "apriltag_toolbox", ros::init_options::NoSigintHandler);
    ros::NodeHandle nh("~");

    node = std::make_unique<FiducialSlam>(nh);
    signal(SIGINT, mySigintHandler);

    ros::Rate r(node->pose_publish_rate);
    while (ros::ok()) {
        ros::spinOnce();
        r.sleep();
        node->fiducialMap.update();
    }

    return 0;
}