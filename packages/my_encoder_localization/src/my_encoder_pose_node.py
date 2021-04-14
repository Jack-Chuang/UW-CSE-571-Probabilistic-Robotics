#!/usr/bin/env python3
import numpy as np
import rospy
import math

from duckietown.dtros import DTROS, NodeType, TopicType
from duckietown_msgs.msg import WheelEncoderStamped
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry

import odometry_activity
import tf2_ros
import tf_conversions
# import geometry_msgs

class EncoderPoseNode(DTROS):
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
        super(EncoderPoseNode, self).__init__(
            node_name=node_name,
            node_type=NodeType.LOCALIZATION
        )
        # get the name of the robot
        self.veh = rospy.get_namespace().strip("/")

        # Add the node parameters to the parameters dictionary
        self.delta_phi_left = 0
        self.left_tick_prev = 0

        self.delta_phi_right = 0
        self.right_tick_prev = 0

        self.x_prev = 0
        self.y_prev = 0
        self.theta_prev = 0

        self.x_curr = 0
        self.y_curr = 0
        self.theta_curr = 0

        # nominal R and L:
        self.R = rospy.get_param(f'/{self.veh}/kinematics_node/radius', 100)
        self.L = rospy.get_param(f'/{self.veh}/kinematics_node/baseline', 100)

        # Construct publishers
        self.db_estimated_pose = rospy.Publisher(
            f'/{self.veh}/encoder_localization',
            Odometry,
            queue_size=1,
            dt_topic_type=TopicType.LOCALIZATION
        )

        # Wheel encoders subscribers:
        left_encoder_topic = f'/{self.veh}/left_wheel_encoder_node/tick'
        self.enco_left = rospy.Subscriber(
            left_encoder_topic,
            WheelEncoderStamped,
            self.cbLeftEncoder,
            queue_size=1
        )

        right_encoder_topic = f'/{self.veh}/right_wheel_encoder_node/tick'
        self.enco_right = rospy.Subscriber(
            right_encoder_topic,
            WheelEncoderStamped,
            self.cbRightEncoder,
            queue_size=1
        )



        self.log("Initialized!")

    def cbLeftEncoder(self, msg_encoder):
        """
            Wheel encoder callback, the rotation of the wheel.
            Args:
                msg_encoder (:obj:`WheelEncoderStamped`) encoder ROS message.
        """
        ticks = msg_encoder.data

        delta_ticks = ticks-self.left_tick_prev
        self.left_tick_prev = ticks

        total_ticks = msg_encoder.resolution

        # Assuming no wheel slipping
        self.delta_phi_left = 2*np.pi*delta_ticks/total_ticks

        # compute the new pose
        self.posePublisher(msg_encoder.header.stamp)

    def cbRightEncoder(self, msg_encoder):
        """
            Wheel encoder callback, the rotation of the wheel.
            Args:
                msg_encoder (:obj:`WheelEncoderStamped`) encoder ROS message.
        """
        ticks = msg_encoder.data

        delta_ticks = ticks-self.right_tick_prev
        self.right_tick_prev = ticks

        total_ticks = msg_encoder.resolution

        # Assuming no wheel slipping
        self.delta_phi_right = 2*np.pi*delta_ticks/total_ticks

        # compute the new pose
        self.posePublisher(msg_encoder.header.stamp)

    def posePublisher(self, time_stamp):
        """
            Publish the pose of the Duckiebot given by the kinematic model
                using the encoders.
            Publish:
                ~/encoder_localization (:obj:`PoseStamped`): Duckiebot pose.
        """
        self.x_curr, self.y_curr, self.theta_curr = odometry_activity.poseEstimation(
            self.R, self.L,
            self.x_prev, self.y_prev, self.theta_prev,
            self.delta_phi_left, self.delta_phi_right)

        self.x_prev = self.x_curr
        self.y_prev = self.y_curr
        self.theta_prev = self.theta_curr


        odom = Odometry()
        odom.header.frame_id = "map"
        odom.header.stamp = time_stamp

        odom.pose.pose.position.x = self.x_curr
        odom.pose.pose.position.y = self.y_curr
        odom.pose.pose.position.z = 0

        odom.pose.pose.orientation.x = 0
        odom.pose.pose.orientation.y = 0
        odom.pose.pose.orientation.z = np.sin(self.theta_curr/2)
        odom.pose.pose.orientation.w = np.cos(self.theta_curr/2)
        
        self.db_estimated_pose.publish(odom)
        # Map - Baselinke TF
        br = tf2_ros.TransformBroadcaster()
        t = TransformStamped()

        t.header.stamp = time_stamp
        t.header.frame_id = "map"
        t.child_frame_id = "encoder_base_link"
        t.transform.translation.x = self.x_curr
        t.transform.translation.y = self.y_curr
        t.transform.translation.z = 0.0
        # q = tf_conversions.transformations.quaternion_from_euler(0, 0, msg.theta)
        t.transform.rotation.x = 0
        t.transform.rotation.y = 0
        t.transform.rotation.z = np.sin(self.theta_curr/2)
        t.transform.rotation.w = np.cos(self.theta_curr/2)

        # Baselink - Cameralink TF
        br2 = tf2_ros.StaticTransformBroadcaster()
        t2 = TransformStamped()

        t2.header.stamp = time_stamp
        t2.header.frame_id = "encoder_base_link"
        t2.child_frame_id = "camera_link"
        t2.transform.translation.x = 0.066
        t2.transform.translation.y = 0
        t2.transform.translation.z = 0.106
        quat = tf_conversions.transformations.quaternion_from_euler(0, 15*math.pi/180, 0)
        t2.transform.rotation.x = quat[0]
        t2.transform.rotation.y = quat[1]
        t2.transform.rotation.z = quat[2]
        t2.transform.rotation.w = quat[3]

        br.sendTransform(t) 
        br2.sendTransform(t2)



    def onShutdown(self):
        super(EncoderPoseNode, self).onShutdown()
     


if __name__ == "__main__":
    # Initialize the node
    encoder_pose_node = EncoderPoseNode(node_name='my_encoder_pose_node')
    # Keep it spinning
    rospy.spin()