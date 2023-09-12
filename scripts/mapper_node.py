#!/usr/bin/python3
# encoding: utf-8
from typing import List

import rospy
from image_geometry import PinholeCameraModel
# from tf2_ros import TransformBroadcaster

from apriltag_ros.msg import AprilTagDetection, AprilTagDetectionArray
from std_msgs.msg import Header
from sensor_msgs.msg import CameraInfo
from geometry_msgs.msg import Pose, Vector3, TransformStamped

from tag_map import TagMap
from mapper import Mapper
# from visualizer import ApriltagVisualizer

class MapperNode:
    def __init__(self, frame_id):
        self._frame_id = frame_id
        self._map = TagMap()
        self._mapper = Mapper(0.04, 1)
        # self._tag_viz = ApriltagVisualizer(self, "apriltags_map")
        # self._tag_viz.set_color(apriltag_toolbox.GREEN)
        # self._tag_viz.set_alpha(0.75)
        self._model = PinholeCameraModel()
        # self._tf_broadcaster = TransformBroadcaster()

        self._sub_tags = rospy.Subscriber("apriltags", AprilTagDetectionArray, self._tags_callback, queue_size=1)
        self._sub_cinfo = rospy.Subscriber("camera_info", CameraInfo, self._camera_info_callback, queue_size=1)


    def get_good_tags(self, tags_c : List[AprilTagDetection]):
        tags_c_good : List[ApriltagDetection] = []
        # [TODO] check the angle of tags
        # if (IsInsideImageCenter(tag_c.center.x, tag_c.center.y,
        #                         model_.cameraInfo().width,
        #                         model_.cameraInfo().height, 5)) {
        for tag_c in tags_c:
            tags_c_good.append(tag_c)
        return tags_c_good


    ##########
    # private
    ##########

    def _tags_callback(self, tags_c_msg : AprilTagDetectionArray):
        # Do nothing if no detection, this prevents checking in the following steps
        if len(tags_c_msg.detections) == 0:
            rospy.logwarn("No tags detected.")
            return

        # Do nothing if camera info not received
        # if not self._model.initialized():
        #     rospy.logwarn("No camera info received")
        #     return

        # Do nothing if there are no good tags close to the center of the image
        tags_c_good = self.get_good_tags(tags_c_msg.detections)
        if len(tags_c_good) == 0:
            rospy.logwarn("No good tags detected.")
            return

        # Initialize map by adding the first tag that is not on the edge of the image
        if not self._map.init():
            self._map.add_first_tag(tags_c_good[0])
            rospy.loginfo("AprilMap initialized.")

        # Do nothing if no pose can be estimated
        pose = Pose()
        pose.orientation.w = 1.0
        # if (!map_.EstimatePose(tags_c_msg->detections, model_.fullIntrinsicMatrix(),
        #                        model_.distortionCoeffs(), &pose)) {
        #   ROS_WARN_THROTTLE(1, "No 2D-3D correspondence.");
        #   return;
        # }
        # Now that with the initial pose calculated, we can do some mapping
        self._mapper.add_pose(pose)
        self._mapper.add_factors(tags_c_good)
        if self._mapper.init():
            # This will only add new landmarks
            self._mapper.add_landmarks(tags_c_good)
            self._mapper.optimize()
            # Get latest estimates from mapper and put into map
            self._mapper.update(self._map, pose)
            # Prepare for next iteration
            self._mapper.clear()
        else:
            # This will add first landmark at origin and fix scale for first pose and
            # first landmark
            self._mapper.initialize(self._map.first_tag())

        # Publish camera to world transform
        header = Header()
        header.stamp = tags_c_msg.header.stamp
        header.frame_id = self._frame_id

        translation = Vector3()
        translation.x = pose.position.x
        translation.y = pose.position.y
        translation.z = pose.position.z

        transform_stamped = TransformStamped()
        transform_stamped.header = header
        transform_stamped.child_frame_id = tags_c_msg.header.frame_id
        transform_stamped.transform.translation = translation
        transform_stamped.transform.rotation = pose.orientation

        self._tf_broadcaster.sendTransform(transform_stamped)

        # Publish visualisation markers
        self._tag_viz.PublishApriltagsMarker(self._map.tags_w(), self._frame_id, tags_c_msg.header.stamp)


    def _camera_info_callback(self, cinfo_msg : CameraInfo):
        # if self._model.initialized():
        #     self._sub_cinfo.shutdown()
        #     rospy.loginfo(f"{rospy.get_namespace()}: Camera initialized")
        #     return
        # self._model.from_camera_info(cinfo_msg)
        pass


if __name__ == '__main__':
    rospy.init_node('mapper')

    mapper_node = MapperNode("world")
    rospy.spin()
    # try:
    #     mapper_node = MapperNode("world")
    #     rospy.spin()
    # except Exception as e:
    #     rospy.logerr(f"{rospy.get_namespace()}: {e}")