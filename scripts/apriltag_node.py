#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import math

import cv2
from pupil_apriltags import Detector

import rospy
import tf
from tf.transformations import quaternion_from_matrix
import ros_numpy
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import Point, Quaternion, Pose, PoseWithCovarianceStamped

from apriltag_toolbox.msg import AprilTagDetectionArray, AprilTagDetection
from utils import transform_apriltag_coord, quaternion_to_euler, euler_to_quaternion, is_ahead_center_and_nearby


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
        self.br = tf.TransformBroadcaster()

        self.current_image = None
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

    
    def update(self):
        if self.current_image is None:
            return
        
        if self.camera_params is None:
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
            corners = corners[0].tolist() + corners[1].tolist() + corners[2].tolist() + corners[3].tolist()
            det.corners = corners
            det.R = tag.pose_R[0, :].tolist() + tag.pose_R[1, :].tolist() + tag.pose_R[2, :].tolist()
            det.t = tag.pose_t[0].tolist() + tag.pose_t[1].tolist() + tag.pose_t[2].tolist()
            det.err = tag.pose_err

            R = tag.pose_R.tolist()
            R[0] = R[0] + [0.0]
            R[1] = R[1] + [0.0]
            R[2] = R[2] + [0.0]
            R.append([0.0, 0.0, 0.0, 1.0])
            
            p = Point(x=-det.t[0], y=det.t[1], z=det.t[2])
            quat = quaternion_from_matrix(R)
            quat = quat.tolist()
            q = Quaternion(x=quat[0], y=quat[1], z=quat[2], w=quat[3])
            euler = quaternion_to_euler(q)
            euler.x = euler.x + math.pi
            euler.z = euler.z + math.pi
            q = euler_to_quaternion(euler)
            pose = Pose(position=p, orientation=q)
            det.pose.pose.pose = transform_apriltag_coord(pose)
            det.pose.header = self.current_image.header
            det.pose.header.frame_id = tag.tag_id
            
            if not is_ahead_center_and_nearby(det, math.pi / 4):
                continue

            detections.append(det)
        
        detections_msg = AprilTagDetectionArray()
        detections_msg.header = self.current_image.header
        detections_msg.header.frame_id = self.camera_frame
        detections_msg.detections = detections
        self.tag_detections_pub.publish(detections_msg)


        # publish tf
        if self.publish_tf:
            for detection in detections:
                position = [detection.pose.pose.pose.position.x, detection.pose.pose.pose.position.y, detection.pose.pose.pose.position.z]
                orientation = [detection.pose.pose.pose.orientation.x, detection.pose.pose.pose.orientation.y, detection.pose.pose.pose.orientation.z, detection.pose.pose.pose.orientation.w]
                self.br.sendTransform(
                    position,
                    orientation,
                    detection.pose.header.stamp,
                    str(detection.id[0]),
                    self.camera_frame
                )

        # draw image and publish
        image = self.draw_tags(image, tags)
        try:
            imgMsg = ros_numpy.msgify(Image, image, encoding= 'bgr8')
            self.tag_detections_image_pub.publish(imgMsg)
        except Exception as err:
            rospy.logerr(err)
            return


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


if __name__ == '__main__':
    node = ApriltagNode()
    r = rospy.Rate(rospy.get_param("~publish_rate", 10))

    while not rospy.is_shutdown():
        try:
            node.update()
            r.sleep()
        except rospy.ROSInterruptException:
            pass