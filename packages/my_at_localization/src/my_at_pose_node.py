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
            ~encoder_localization (:obj:`PoseStamped`): The computed position
        Subscribers:
            ~/left_wheel_encoder_node/tick (:obj:`WheelEncoderStamped`):
                encoder ticks
            ~/right_wheel_encoder_node/tick (:obj:`WheelEncoderStamped`):
                encoder ticks
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
        # print("new_cam", new_cam)

        # cameraMatrix = numpy.array(new_cam['K']).reshape((3,3))
        camera_params = ( new_cam[0,0], new_cam[1,1], new_cam[0,2], new_cam[1,2] )
        tags = self.at_detector.detect(gray_img, estimate_tag_pose=True, camera_params=camera_params, tag_size=0.065)
        
        
        # print("Detected pose", tags[0].pose_R)

        if(len(tags)>0):
            print("Detected: ID ", tags[0].tag_id)
            # pose = PoseStamped()

            # pose.header.stamp = rospy.Time.now()
            # pose.header.frame_id = 'map'

            # pose.pose.position.x = tags[0].pose_t[0]
            # pose.pose.position.y = tags[0].pose_t[1]
            # pose.pose.position.z = tags[0].pose_t[2]
            pose = np.identity(4)
            pose[0:3, 0:3] = tags[0].pose_R
            
            q_tag = quaternion_from_matrix(pose)
            # pose.pose.orientation.x = q_tag[0]
            # pose.pose.orientation.y = q_tag[1]
            # pose.pose.orientation.z = q_tag[2]
            # pose.pose.orientation.w = q_tag[3]

            br1 = tf2_ros.StaticTransformBroadcaster()
            t1 = TransformStamped()

            t1.header.stamp = image_msg.header.stamp
            t1.header.frame_id = "map"
            t1.child_frame_id = f'april_tag_{tags[0].tag_id}'
            t1.transform.translation.x = 0
            t1.transform.translation.y = 0
            t1.transform.translation.z = 0.075
            t1.transform.rotation.x = 0
            t1.transform.rotation.y = 0
            t1.transform.rotation.z = 0
            t1.transform.rotation.w = 1

            br1.sendTransform(t1)


            br2 = tf2_ros.TransformBroadcaster()
            t2 = TransformStamped()

            t2.header.stamp = image_msg.header.stamp
            t2.header.frame_id = f'april_tag_{tags[0].tag_id}'
            t2.child_frame_id = "camera_link"
            t2.transform.translation.x = tags[0].pose_t[0]
            t2.transform.translation.y = tags[0].pose_t[1]
            t2.transform.translation.z = tags[0].pose_t[2]            
            t2.transform.rotation.x = q_tag[0]
            t2.transform.rotation.y = q_tag[1]
            t2.transform.rotation.z = q_tag[2]
            t2.transform.rotation.w = q_tag[3]

            br2.sendTransform(t2)


            br3 = tf2_ros.StaticTransformBroadcaster()
            t3 = TransformStamped()

            t3.header.stamp = image_msg.header.stamp
            t3.header.frame_id = "camera_link"
            t3.child_frame_id = "at_base_link"
            t3.transform.translation.x = -0.066
            t3.transform.translation.y = 0
            t3.transform.translation.z = -0.106
            quat = tf_conversions.transformations.quaternion_from_euler(0, -15*math.pi/180, 0)
            t3.transform.rotation.x = quat[0]
            t3.transform.rotation.y = quat[1]
            t3.transform.rotation.z = quat[2]
            t3.transform.rotation.w = quat[3]

            br3.sendTransform(t3)



            # broadcaster = tf2_ros.StaticTransformBroadcaster()
            # static_transformStamped = TransformStamped()

            # static_transformStamped.header.stamp = image_msg.header.stamp
            # static_transformStamped.header.frame_id = "camera_link"
            # static_transformStamped.child_frame_id = f'april_tag_{tags[0].tag_id}'

            # static_transformStamped.transform.translation.x = tags[0].pose_t[0]
            # static_transformStamped.transform.translation.y = tags[0].pose_t[1]
            # static_transformStamped.transform.translation.z = tags[0].pose_t[2]

            # # quat = tf.transformations.quaternion_from_euler(
            # #        float(sys.argv[5]),float(sys.argv[6]),float(sys.argv[7]))
            # static_transformStamped.transform.rotation.x = q_tag[0]
            # static_transformStamped.transform.rotation.y = q_tag[1]
            # static_transformStamped.transform.rotation.z = q_tag[2]
            # static_transformStamped.transform.rotation.w = q_tag[3]

            # broadcaster.sendTransform(static_transformStamped)
        # print("Detected pose_R:", tags[0].pose_R)
        # print("Detected pose_t:", tags[0].pose_t)
        # newCameraMatrix = cv2.getOptimalNewCameraMatrix(cameraMatrix, 
        #                                         distCoeffs, 
        #                                         (640, 480), 
        #                                         1.0)
        # cv2.initUndistortRectifyMap(cameraMatrix, 
        #                             distCoeffs, 
        #                             np.eye(3), 
        #                             newCameraMatrix, 
        #                             (640, 480), 
        #                             cv2.CV_32FC1)
        # cv2.remap(compressed_image, map1, map2, cv2.INTER_LINEAR)
        

        # ticks = msg_encoder.data

        # delta_ticks = ticks-self.left_tick_prev
        # self.left_tick_prev = ticks

        # total_ticks = msg_encoder.resolution

        # # Assuming no wheel slipping
        # self.delta_phi_left = 2*np.pi*delta_ticks/total_ticks

        # compute the new pose
        # self.posePublisher(msg_encoder.header.stamp)

    # def cbRightEncoder(self, msg_encoder):
    #     """
    #         Wheel encoder callback, the rotation of the wheel.
    #         Args:
    #             msg_encoder (:obj:`WheelEncoderStamped`) encoder ROS message.
    #     """
    #     ticks = msg_encoder.data

    #     delta_ticks = ticks-self.right_tick_prev
    #     self.right_tick_prev = ticks

    #     total_ticks = msg_encoder.resolution

    #     # Assuming no wheel slipping
    #     self.delta_phi_right = 2*np.pi*delta_ticks/total_ticks

    #     # compute the new pose
    #     self.posePublisher(msg_encoder.header.stamp)

    # def posePublisher(self, time_stamp):
    #     """
    #         Publish the pose of the Duckiebot given by the kinematic model
    #             using the encoders.
    #         Publish:
    #             ~/encoder_localization (:obj:`PoseStamped`): Duckiebot pose.
    #     """
    #     self.x_curr, self.y_curr, self.theta_curr = odometry_activity.poseEstimation(
    #         self.R, self.L,
    #         self.x_prev, self.y_prev, self.theta_prev,
    #         self.delta_phi_left, self.delta_phi_right)

    #     self.x_prev = self.x_curr
    #     self.y_prev = self.y_curr
    #     self.theta_prev = self.theta_curr

    #     pose = PoseStamped()

    #     # pose.header.stamp = rospy.Time.now()
    #     # pose.header.frame_id = 'map'

    #     # pose.pose.position.x = self.x_curr
    #     # pose.pose.position.y = self.y_curr
    #     # pose.pose.position.z = 0

    #     # pose.pose.orientation.x = 0
    #     # pose.pose.orientation.y = 0
    #     # pose.pose.orientation.z = np.sin(self.theta_curr/2)
    #     # pose.pose.orientation.w = np.cos(self.theta_curr/2)

    #     # self.db_estimated_pose.publish(pose)

    #     odom = Odometry()
    #     odom.header.frame_id = "map"
    #     odom.header.stamp = rospy.Time.now()

    #     odom.pose.pose.position.x = self.x_curr
    #     odom.pose.pose.position.y = self.y_curr
    #     odom.pose.pose.position.z = 0

    #     odom.pose.pose.orientation.x = 0
    #     odom.pose.pose.orientation.y = 0
    #     odom.pose.pose.orientation.z = np.sin(self.theta_curr/2)
    #     odom.pose.pose.orientation.w = np.cos(self.theta_curr/2)

    #     #odom.pose.covariance = np.diag([1e-2, 1e-2, 1e-2, 1e3, 1e3, 1e-1]).ravel()
    #     #odom.twist.twist.linear.x = self._lin_vel
    #     #odom.twist.twist.angular.z = self._ang_vel
    #     #odom.twist.covariance = np.diag([1e-2, 1e3, 1e3, 1e3, 1e3, 1e-2]).ravel()
        
    #     self.db_estimated_pose.publish(odom)

    #     br = tf2_ros.TransformBroadcaster()
    #     t = TransformStamped()

    #     t.header.stamp = time_stamp
    #     t.header.frame_id = "map"
    #     t.child_frame_id = "encoder_base_link"
    #     t.transform.translation.x = self.x_curr
    #     t.transform.translation.y = self.y_curr
    #     t.transform.translation.z = 0.0
    #     # q = tf_conversions.transformations.quaternion_from_euler(0, 0, msg.theta)
    #     t.transform.rotation.x = 0
    #     t.transform.rotation.y = 0
    #     t.transform.rotation.z = np.sin(self.theta_curr/2)
    #     t.transform.rotation.w = np.cos(self.theta_curr/2)

    #     br.sendTransform(t) 


    #
    # Pose estimation is the function that is created by the user.
    #

    def onShutdown(self):
        super(EncoderPoseNode, self).onShutdown()
     


if __name__ == "__main__":
    # Initialize the node
    at_pose_node = ATPoseNode(node_name='my_at_pose_node')
    # Keep it spinning
    rospy.spin()
    # try:
        # rospy.spin()
    # except KeyboardInterrupt:
        # print("Shutting down")
    # cv2.destroyAllWindows()