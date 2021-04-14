#!/usr/bin/env python3
import numpy as np
import rospy

from duckietown.dtros import DTROS, NodeType, TopicType
from duckietown_msgs.msg import WheelEncoderStamped
from sensor_msgs.msg import CompressedImage
from sensor_msgs.msg import Image
from sensor_msgs.msg import CameraInfo
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry

# import odometry_activity
import tf2_ros
import tf_conversions
from tf.transformations import *
import cv2
from cv_bridge import CvBridge
from image_processing.calibration_utils import get_camera_info_for_robot
from image_processing.rectification import Rectify
# from image_processing import DecoderNode
from dt_apriltags import Detector
# import geometry_msgs

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
 
        self.image_pub = rospy.Publisher(f'/{self.veh}/rectified_image',Image,  queue_size=10)

 
        self.log("Initialized!")

    def getCameraInfo(self, cam_msg):
        if(self.camera_info==None):
            self.camera_info = cam_msg
            self.Rectify = Rectify(self.camera_info)
        return

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

        
        camera_params = ( new_cam[0,0], new_cam[1,1], new_cam[0,2], new_cam[1,2] )
        tags = self.at_detector.detect(gray_img, estimate_tag_pose=True, camera_params=camera_params, tag_size=0.065)
        
        

        if(len(tags)>0):
            print("Detected: Tag ID ", tags[0].tag_id)
            
            pose = np.identity(4)
            pose[0:3, 0:3] = tags[0].pose_R
            
            q_tag = quaternion_from_matrix(pose)

            # Map to ATag
            br0 = tf2_ros.StaticTransformBroadcaster()
            t0 = TransformStamped()

            t0.header.stamp = rospy.Time.now()
            t0.header.frame_id = "map"
            t0.child_frame_id = f'april_tag_{tags[0].tag_id}'
            t0.transform.translation.x = 0
            t0.transform.translation.y = 0
            t0.transform.translation.z = 0.075
            t0.transform.rotation.x = 0
            t0.transform.rotation.y = 0
            t0.transform.rotation.z = 0
            t0.transform.rotation.w = 1

            br0.sendTransform(t0)

            # ATag to ATag cam (this is because AprilTAG pose is different from physical)
            br1 = tf2_ros.StaticTransformBroadcaster()
            t1 = TransformStamped()

            t1.header.stamp = rospy.Time.now()
            t1.header.frame_id = f'april_tag_{tags[0].tag_id}'
            t1.child_frame_id = f'april_tag_cam_{tags[0].tag_id}'
            t1.transform.translation.x = 0
            t1.transform.translation.y = 0
            t1.transform.translation.z = 0
            quat1 = tf_conversions.transformations.quaternion_from_euler(-90*math.pi/180, 0, 90*math.pi/180)
            t1.transform.rotation.x = quat1[0]
            t1.transform.rotation.y = quat1[1]
            t1.transform.rotation.z = quat1[2]
            t1.transform.rotation.w = quat1[3]

            br1.sendTransform(t1)


            # ATag cam to camera_rgb_link (again this is internal cam frame)
            br2 = tf2_ros.TransformBroadcaster()
            t2 = TransformStamped()

            t2.header.stamp = image_msg.header.stamp
            t2.header.frame_id = f'april_tag_cam_{tags[0].tag_id}'
            t2.child_frame_id = "camera_rgb_link"
            t2.transform.translation.x = tags[0].pose_t[0]
            t2.transform.translation.y = tags[0].pose_t[1]
            t2.transform.translation.z = tags[0].pose_t[2]            
            t2.transform.rotation.x = q_tag[0]
            t2.transform.rotation.y = q_tag[1]
            t2.transform.rotation.z = q_tag[2]
            t2.transform.rotation.w = q_tag[3]

            br2.sendTransform(t2)

            # camera_rgb_link to camera_link (internal cam frame to physical cam frame)
            br3 = tf2_ros.StaticTransformBroadcaster()
            t3 = TransformStamped()

            t3.header.stamp = image_msg.header.stamp
            t3.header.frame_id = "camera_rgb_link"
            t3.child_frame_id = "camera_link"
            t3.transform.translation.x = 0
            t3.transform.translation.y = 0
            t3.transform.translation.z = 0
            quat3 = tf_conversions.transformations.quaternion_from_euler(0, 90*math.pi/180, -90*math.pi/180)
            t3.transform.rotation.x = quat3[0]
            t3.transform.rotation.y = quat3[1]
            t3.transform.rotation.z = quat3[2]
            t3.transform.rotation.w = quat3[3]

            br3.sendTransform(t3)

            # camera_link to base_link of robot 
            br4 = tf2_ros.StaticTransformBroadcaster()
            t4 = TransformStamped()

            t4.header.stamp = image_msg.header.stamp
            t4.header.frame_id = "camera_link"
            t4.child_frame_id = "at_base_link"
            t4.transform.translation.x = -0.066
            t4.transform.translation.y = 0
            t4.transform.translation.z = -0.106
            quat4 = tf_conversions.transformations.quaternion_from_euler(0, -15*math.pi/180, 0)
            t4.transform.rotation.x = quat4[0]
            t4.transform.rotation.y = quat4[1]
            t4.transform.rotation.z = quat4[2]
            t4.transform.rotation.w = quat4[3]

            br4.sendTransform(t4)
        else:
            print("Detected: Tag ID - None")


    def onShutdown(self):
        super(EncoderPoseNode, self).onShutdown()
     


if __name__ == "__main__":
    # Initialize the node
    at_pose_node = ATPoseNode(node_name='my_at_pose_node')
    # Keep it spinning
    rospy.spin()
