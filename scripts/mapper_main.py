#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import copy
import time
import argparse
from typing import List

import cv2
from pupil_apriltags import Detector

import rospy
from image_geometry import PinholeCameraModel
from tf.transformations import quaternion_from_matrix
import ros_numpy
from std_msgs.msg import Header
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import Point, Quaternion, Pose, PoseWithCovarianceStamped, Vector3, TransformStamped

from apriltag_toolbox.msg import AprilTagDetectionArray, AprilTagDetection
from tag_map import TagMap
from mapper import Mapper


class ApriltagNode(object):
    def __init__(self):
        rospy.init_node("apriltag_ros")

        self.camera_frame = rospy.get_param("~camera_frame", default="camera")
        self.publish_tf = rospy.get_param("~publish_tf", default=False)

        families = rospy.get_param("~families", default='tag36h11')
        nthreads = rospy.get_param("~nthreads", default=1)
        quad_decimate = rospy.get_param("~quad_decimate", default=2.0)
        quad_sigma = rospy.get_param("~quad_sigma", default=0.0)
        refine_edges = rospy.get_param("~refine_edges", default=1)
        decode_sharpening = rospy.get_param("~decode_sharpening", default=0.25)
        self.tag_size = rospy.get_param("~tag_size", default=0.25)

        self.at_detector = Detector(
            families=families,
            nthreads=nthreads,
            quad_decimate=quad_decimate,
            quad_sigma=quad_sigma,
            refine_edges=refine_edges,
            decode_sharpening=decode_sharpening,
            debug=False,
        )

        self._frame_id = 'world'
        self._map = TagMap()
        self._mapper = Mapper(0.04, 1)
        # self._tag_viz = ApriltagVisualizer(self, "apriltags_map")
        # self._tag_viz.set_color(apriltag_toolbox.GREEN)
        # self._tag_viz.set_alpha(0.75)
        # self._tf_broadcaster = TransformBroadcaster()

        self.current_image = None
        self.camera_info = None
        self.camera_params = None

        self.tag_detections_pub = rospy.Publisher("tag_detections", AprilTagDetectionArray, queue_size=1)
        self.tag_detections_image_pub = rospy.Publisher("tag_detections_image", Image, queue_size=1)

        self._image_sub = rospy.Subscriber("image_rect", Image, self.image_callback, queue_size=1)
        self._camera_info_sub = rospy.Subscriber("camera_info", CameraInfo, self.camera_info_callback, queue_size=1)
        self._image_sub
        self._camera_info_sub


    def image_callback(self, image_msg):
        self.current_image = image_msg


    def camera_info_callback(self, camera_info_msg):
        self.camera_params = [camera_info_msg.K[0], camera_info_msg.K[4], camera_info_msg.K[2], camera_info_msg.K[5]]
        self.camera_info = camera_info_msg

    
    def update(self):
        if self.current_image is None:
            return
        
        if self.self.camera_info is None:
            return

        try:
            image = ros_numpy.numpify(self.current_image)
            gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        except Exception as err:
            rospy.logerr(err)
            return

        # detect tags
        tags = self.at_detector.detect(
            gray,
            estimate_tag_pose=True,
            camera_params=self.camera_params,
            tag_size=self.tag_size,
        )

        # publish tag
        detections = []
        for tag in tags:
            det = AprilTagDetection()
            det.id = [tag.tag_id]
            det.size = [self.tag_size]
            det.decision_margin = tag.decision_margin
            det.center = tag.center.tolist()
            corners = tag.corners
            corners = corners[0] + corners[1] + corners[2] + corners[3]
            det.corners = corners.tolist()
            det.R = tag.pose_R[0, :].tolist() + tag.pose_R[1, :].tolist() + tag.pose_R[2, :].tolist()
            t = tag.pose_t.tolist()
            det.t = tag.pose_t[0].tolist() + tag.pose_t[1].tolist() + tag.pose_t[2].tolist()
            det.err = tag.pose_err

            R = tag.pose_R.tolist()
            R[0] = R[0] + [0.0]
            R[1] = R[1] + [0.0]
            R[2] = R[2] + [0.0]
            R.append([0.0, 0.0, 0.0, 1.0])
            
            p = Point(x=det.t[0], y=det.t[1], z=det.t[2])
            quat = quaternion_from_matrix(R)
            quat = quat.tolist()
            q = Quaternion(x=quat[0], y=quat[1], z=quat[2], w=quat[3])
            pose = Pose(position=p, orientation=q)
            det.pose.pose.pose = pose
            det.pose.header = self.current_image.header
            det.pose.header.frame_id = self.camera_frame

            detections.append(det)
        
        detections_msg = AprilTagDetectionArray()
        detections_msg.header = self.current_image.header
        detections_msg.header.frame_id = self.camera_frame
        detections_msg.detections = detections
        self.tag_detections_pub.publish(detections_msg)


        # publish tf
        if self.publish_tf:
            pass


        # draw image and publish
        image = self.draw_tags(image, tags)
        try:
            imgMsg = ros_numpy.msgify(Image, image, encoding= 'bgr8')
            self.tag_detections_image_pub.publish(imgMsg)
        except Exception as err:
            rospy.logerr(err)
            return

        
        # Do nothing if no detection, this prevents checking in the following steps
        if len(detections_msg.detections) == 0:
            rospy.logwarn("No tags detected.")
            return

        # Do nothing if camera info not received
        if self.camera_info is None:
            rospy.logwarn("No camera info received")
            return

        # Do nothing if there are no good tags close to the center of the image
        tags_c_good = self.get_good_tags(detections_msg.detections)
        if len(tags_c_good) == 0:
            rospy.logwarn("No good tags detected.")
            return

        # Initialize map by adding the first tag that is not on the edge of the image
        if not self._map.init():
            self._map.add_first_tag(tags_c_good[0])
            rospy.loginfo("AprilMap initialized.")

        # Do nothing if no pose can be estimated
        pose = Pose()
        pose = self._map.EstimatePose(detections_msg.detections, self.camera_info.K, self.camera_info.D):
        if not pose:
            ROS_WARN_THROTTLE(1, "No 2D-3D correspondence.") 
            return

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
        header.stamp = detections_msg.header.stamp
        header.frame_id = self._frame_id

        translation = Vector3()
        translation.x = pose.position.x
        translation.y = pose.position.y
        translation.z = pose.position.z

        transform_stamped = TransformStamped()
        transform_stamped.header = header
        transform_stamped.child_frame_id = detections_msg.header.frame_id
        transform_stamped.transform.translation = translation
        transform_stamped.transform.rotation = pose.orientation

        self._tf_broadcaster.sendTransform(transform_stamped)

        # Publish visualisation markers
        # self._tag_viz.PublishApriltagsMarker(self._map.tags_w(), self._frame_id, detections_msg.header.stamp)


    def draw_tags(self, image, tags):
        for tag in tags:
            tag_family = tag.tag_family
            tag_id = tag.tag_id
            center = tag.center
            corners = tag.corners

            center = (int(center[0]), int(center[1]))
            corner_01 = (int(corners[0][0]), int(corners[0][1]))
            corner_02 = (int(corners[1][0]), int(corners[1][1]))
            corner_03 = (int(corners[2][0]), int(corners[2][1]))
            corner_04 = (int(corners[3][0]), int(corners[3][1]))

            # 中心
            cv2.circle(image, (center[0], center[1]), 5, (0, 0, 255), 2)

            # 各辺
            cv2.line(image, (corner_01[0], corner_01[1]),
                    (corner_02[0], corner_02[1]), (255, 0, 0), 2)
            cv2.line(image, (corner_02[0], corner_02[1]),
                    (corner_03[0], corner_03[1]), (255, 0, 0), 2)
            cv2.line(image, (corner_03[0], corner_03[1]),
                    (corner_04[0], corner_04[1]), (0, 255, 0), 2)
            cv2.line(image, (corner_04[0], corner_04[1]),
                    (corner_01[0], corner_01[1]), (0, 255, 0), 2)

            # タグファミリー、タグID
            # cv2.putText(image,
            #            str(tag_family) + ':' + str(tag_id),
            #            (corner_01[0], corner_01[1] - 10), cv2.FONT_HERSHEY_SIMPLEX,
            #            0.6, (0, 255, 0), 1, cv2.LINE_AA)
            cv2.putText(image, str(tag_id), (center[0] - 10, center[1] - 10),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.75, (0, 0, 255), 2, cv2.LINE_AA)

        return image

    def get_good_tags(self, tags_c : List[AprilTagDetection]):
        tags_c_good : List[ApriltagDetection] = []
        # [TODO] check the angle of tags
        # if (IsInsideImageCenter(tag_c.center.x, tag_c.center.y,
        #                         model_.cameraInfo().width,
        #                         model_.cameraInfo().height, 5)) {
        for tag_c in tags_c:
            tags_c_good.append(tag_c)
        return tags_c_good



if __name__ == '__main__':
    node = ApriltagNode()
    r = rospy.Rate(rospy.get_param("~publish_rate", 10))

    while not rospy.is_shutdown():
        try:
            node.update()
            r.sleep()
        except rospy.ROSInterruptException:
            pass