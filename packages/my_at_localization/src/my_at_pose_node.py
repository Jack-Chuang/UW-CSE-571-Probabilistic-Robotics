#!/usr/bin/env python3
from typing import Tuple, Optional, List

import cv2
import numpy as np
import rospy
import tf2_ros
import tf_conversions
from cv_bridge import CvBridge
from dt_apriltags import Detector
from duckietown.dtros import DTROS, NodeType, TopicType
from geometry_msgs.msg import TransformStamped
from image_processing.rectification import Rectify
from nav_msgs.msg import Odometry
from sensor_msgs.msg import CameraInfo
from sensor_msgs.msg import CompressedImage
from sensor_msgs.msg import Image
from std_msgs.msg import Int32MultiArray 
from tf.transformations import *


__TAG_ID_DICT = {32: 32, 33: 31, 65: 61, 31: 33, 57: 57, 61: 65, 10: 11, 11: 10, 9: 9, 24: 26, 25: 25, 26: 24}


def fetch_tag_id(tag):
    return __TAG_ID_DICT[tag.tag_id]


class ATPoseNode(DTROS):
    """
        Computes an estimate of the Duckiebot pose using the wheel encoders.
        Args:
            node_name (:obj:`str`): a unique, descriptive name for the ROS node
        Configuration:

        Publisher:
            ~/rectified_image (:obj:`Image`): The rectified image
            ~at_localization (:obj:`PoseStamped`): The computed position broadcasted in TFs
        Subscribers:
            ~/camera_node/image/compressed (:obj:`CompressedImage`): Observation from robot
       
    """
    def __init__(self, node_name):
        # Initialize the DTROS parent class
        super(ATPoseNode, self).__init__(
            node_name=node_name,
            node_type=NodeType.LOCALIZATION
        )
        # get the name of the robot
        self.veh = rospy.get_namespace().strip("/")
        self.bridge = CvBridge()
        self.camera_info = None
        self.Rectify = None

        self.at_detector = Detector(families='tag36h11',
                                    nthreads=4,
                                    quad_decimate=4.0,
                                    quad_sigma=0.0,
                                    refine_edges=1,
                                    decode_sharpening=0.25,
                                    debug=0)

        # Construct publishers
        self.at_estimated_pose = rospy.Publisher(
            f'/{self.veh}/at_localization',
            Odometry,
            queue_size=1,
            dt_topic_type=TopicType.LOCALIZATION
        )

        # Camera subscribers:
        camera_topic = f'/{self.veh}/camera_node/image/compressed'
        self.camera_feed_sub = rospy.Subscriber(
            camera_topic,
            CompressedImage,
            self.detectATPose,
            queue_size=1
        )

        camera_info_topic = f'/{self.veh}/camera_node/camera_info'
        self.camera_info_sub = rospy.Subscriber(
            camera_info_topic,
            CameraInfo,
            self.getCameraInfo,
            queue_size=1
        )

        self.image_pub = rospy.Publisher(f'/{self.veh}/rectified_image', Image, queue_size=10)

        self.tag_pub = rospy.Publisher(f'/{self.veh}/detected_tags', Int32MultiArray, queue_size=5)
        
        self.log("Initialized!")

    def getCameraInfo(self, cam_msg):
        if (self.camera_info == None):
            self.camera_info = cam_msg
            self.Rectify = Rectify(self.camera_info)
        return

    def _broadcast_tf(
            self,
            parent_frame_id: str,
            child_frame_id: str,
            stamp: Optional[float] = None,
            translations: Optional[Tuple[float, float, float]] = None,
            euler_angles: Optional[Tuple[float, float, float]] = None,
            quarternion: Optional[List[float]] = None):
        br = tf2_ros.StaticTransformBroadcaster()
        ts = TransformStamped()

        ts.header.frame_id = parent_frame_id
        ts.child_frame_id = child_frame_id

        if stamp is None:
            stamp = rospy.Time.now()
        ts.header.stamp = stamp

        if translations is None:
            translations = 0, 0, 0
        ts.transform.translation.x = translations[0]
        ts.transform.translation.y = translations[1]
        ts.transform.translation.z = translations[2]

        if euler_angles is not None:
            q = tf_conversions.transformations.quaternion_from_euler(*euler_angles)
        elif quarternion is not None:
            q = quarternion
        else:
            q = 0, 0, 0, 1
        ts.transform.rotation.x = q[0]
        ts.transform.rotation.y = q[1]
        ts.transform.rotation.z = q[2]
        ts.transform.rotation.w = q[3]

        br.sendTransform(ts)

    def _broadcast_detected_tag(self, image_msg, tag_id, tag):
        # inverse the rotation and tranformation comming out of the AT detector.
        # The AP detector's output is the camera frame relative to the TAG
        # According to: https://duckietown.slack.com/archives/C6ZHPV212/p1619876209086300?thread_ts=1619751267.084600&cid=C6ZHPV212
        # While the transformation below needs TAG relative to camera
        # Therefore we need to reverse it first.
        pose_R = np.identity(4)
        pose_R[:3, :3] = tag.pose_R
        inv_pose_R = np.transpose(pose_R)
        pose_t = np.ones((4, 1)) 
        pose_t[:3, :] = tag.pose_t
        inv_pose_t = -np.matmul(inv_pose_R, pose_t)
        out_q = quaternion_from_matrix(inv_pose_R)
        out_t = inv_pose_t[:3, :]

        tag_frame_id = 'april_tag_{}'.format(tag_id)
        tag_cam_frame_id = 'april_tag_cam_{}'.format(tag_id)
        camera_rgb_link_frame_id = 'at_{}_camera_rgb_link'.format(tag_id)
        camera_link_frame_id = 'at_{}_camera_link'.format(tag_id)
        base_link_frame_id = 'at_{}_base_link'.format(tag_id)

        # ATag to ATag cam (this is because AprilTAG pose is different from physical)
        self._broadcast_tf(
            parent_frame_id=tag_frame_id,
            child_frame_id=tag_cam_frame_id,
            euler_angles=(-90 * math.pi / 180, 0, -90 * math.pi / 180)
        )

        # ATag cam to camera_rgb_link (again this is internal cam frame)
        self._broadcast_tf(
            parent_frame_id=tag_cam_frame_id,
            child_frame_id=camera_rgb_link_frame_id,
            stamp=image_msg.header.stamp,
            translations=out_t,
            quarternion=out_q
        )

        # camera_rgb_link to camera_link (internal cam frame to physical cam frame)
        self._broadcast_tf(
            parent_frame_id=camera_rgb_link_frame_id,
            child_frame_id=camera_link_frame_id,
            stamp=image_msg.header.stamp,
            euler_angles=(0, -90 * math.pi / 180, 90 * math.pi / 180)
        )

        # camera_link to base_link of robot
        self._broadcast_tf(
            parent_frame_id=camera_link_frame_id,
            child_frame_id=base_link_frame_id,
            stamp=image_msg.header.stamp,
            translations=(-0.066, 0, -0.106),
            euler_angles=(0, -15 * math.pi / 180, 0)
        )

    def detectATPose(self, image_msg):
        """
            Image callback.
            Args:
                msg_encoder (:obj:`Compressed`) encoder ROS message.
        """
        try:
            cv_image = self.bridge.compressed_imgmsg_to_cv2(image_msg)
        except ValueError as e:
            self.logerr('Could not decode image: %s' % e)
            return

        new_cam, rectified_img = self.Rectify.rectify_full(cv_image)

        try:
            self.image_pub.publish(self.bridge.cv2_to_imgmsg(rectified_img, "bgr8"))
        except ValueError as e:
            self.logerr('Could not decode image: %s' % e)
            return

        gray_img = cv2.cvtColor(rectified_img, cv2.COLOR_RGB2GRAY)

        camera_params = (new_cam[0, 0], new_cam[1, 1], new_cam[0, 2], new_cam[1, 2])
        detected_tags = self.at_detector.detect(gray_img, estimate_tag_pose=True, camera_params=camera_params, tag_size=0.065)
        detected_tag_ids = list(map(lambda x: fetch_tag_id(x), detected_tags))
        array_for_pub = Int32MultiArray(data=detected_tag_ids)
        self.tag_pub.publish(array_for_pub)
        for tag_id, tag in zip(detected_tag_ids, detected_tags):
            print('detected {}: ({}, {})'.format(tag_id, image_msg.header.stamp.to_time(), rospy.Time.now().to_time()))
            self._broadcast_detected_tag(image_msg, tag_id, tag)


if __name__ == "__main__":
    # Initialize the node
    at_pose_node = ATPoseNode(node_name='my_at_pose_node')
    # Keep it spinning
    rospy.spin()
