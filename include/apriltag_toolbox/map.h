#ifndef MAP_H
#define MAP_H

#include <ros/ros.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <visualization_msgs/Marker.h>

#include <opencv2/aruco.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/highgui.hpp>

#include <fiducial_msgs/FiducialMapEntry.h>
#include <fiducial_msgs/FiducialMapEntryArray.h>

#include <list>
#include <string>

#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <tf2/convert.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <std_srvs/Empty.h>

#include <fiducial_slam/transform_with_variance.h>

// An observation of a single fiducial in a single image
class Observation {
public:
    int fid;
    tf2::Stamped<TransformWithVariance> T_fidCam;
    tf2::Stamped<TransformWithVariance> T_camFid;

    Observation(){};

    Observation(int fid, const tf2::Stamped<TransformWithVariance> &camFid);
};

// A single fiducial that is in the map
class Fiducial {
public:
    int id;
    int numObs;
    bool visible;
    std::set<int> links;  // Stores the IDs of connected fiducials

    tf2::Stamped<TransformWithVariance> pose;
    ros::Time lastPublished;

    void update(const tf2::Stamped<TransformWithVariance> &newPose);

    Fiducial() {}

    Fiducial(int id, const tf2::Stamped<TransformWithVariance> &pose);
};

// Class containing map data
class Map {
public:
    tf2_ros::TransformBroadcaster broadcaster;
    tf2_ros::Buffer tfBuffer;
    std::unique_ptr<tf2_ros::TransformListener> listener;

    ros::Publisher markerPub;
    ros::Publisher mapPub;
    ros::Publisher robotPosePub;
    ros::Publisher cameraPosePub;

    ros::ServiceServer clearSrv;
    ros::ServiceServer addSrv;
    bool clearCallback(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res);

    std::string mapFilename;
    std::string mapFrame;
    std::string odomFrame;
    std::string cameraFrame;
    std::string baseFrame;
    double future_date_transforms;
    bool publish_6dof_pose;
    double multiErrorThreshold;

    bool isInitializingMap;
    bool readOnly;
    int frameNum;
    int initialFrameNum;
    int originFid;

    bool overridePublishedCovariance;
    std::vector<double> covarianceDiagonal;

    bool havePose;
    float tfPublishInterval;
    bool publishPoseTf;
    ros::Time tfPublishTime;
    geometry_msgs::TransformStamped poseTf;

    std::map<int, Fiducial> fiducials;
    int fiducialToAdd;

    Map(ros::NodeHandle &nh);
    void update();
    void update(std::vector<Observation> &obs, const ros::Time &time);
    void autoInit(const std::vector<Observation> &obs, const ros::Time &time);
    int updatePose(std::vector<Observation> &obs, const ros::Time &time,
                   tf2::Stamped<TransformWithVariance> &cameraPose);
    void updateMap(const std::vector<Observation> &obs, const ros::Time &time,
                   const tf2::Stamped<TransformWithVariance> &cameraPose);

    bool loadMap();
    bool loadMap(std::string filename);
    bool saveMap();
    bool saveMap(std::string filename);

    void publishTf();
    void publishMap();
    void publishMarker(Fiducial &fid);
    void publishMarkers();
    void drawLine(const tf2::Vector3 &p0, const tf2::Vector3 &p1);

    bool lookupTransform(const std::string &from, const std::string &to, const ros::Time &time,
                         tf2::Transform &T) const;
};

#endif