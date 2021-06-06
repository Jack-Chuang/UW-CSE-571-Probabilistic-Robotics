#!/usr/bin/env python3
from __future__ import absolute_import, unicode_literals

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

from sensor_msgs.msg import Joy

# imports for AMQ

import datetime

from kombu import Connection
from kombu import Exchange
from kombu import Producer
from kombu import Queue
import sys
import time

from sensor_msgs.msg import Range
from duckietown_msgs.msg import Twist2DStamped, LanePose, WheelsCmdStamped, BoolStamped, FSMState, StopLineReading



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
        
        # Default RabbitMQ server URI
        self.rabbit_url = 'amqp://user4:rmq2021@usw-gp-vm.westus.cloudapp.azure.com:5672//'

        # Kombu Connection
        self.conn = Connection(self.rabbit_url)
        self.channel = self.conn.channel()
        
        self.curr_steering_angle = 0
        
        # Kombu Exchange
        # - set delivery_mode to transient to prevent disk writes for faster delivery
        self.exchange = Exchange("xubot-exchange", type="direct", delivery_mode=1)
        self.video_producer = Producer(exchange=self.exchange, channel=self.channel, routing_key="xubot-stream")
        
        # Kombu Queue
        self.video_queue = Queue(name="xubot-stream", exchange=self.exchange, routing_key="xubot-stream") 
        self.video_queue.maybe_bind(self.conn)
        self.video_queue.declare()
        
        self.encode_param = [int(cv2.IMWRITE_JPEG_QUALITY),90]

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
            self.lane_detection,
            queue_size=1,
            buff_size=2**24
        )

        camera_info_topic = f'/{self.veh}/camera_node/camera_info'
        self.camera_info_sub = rospy.Subscriber(
            camera_info_topic,
            CameraInfo,
            self.getCameraInfo,
            queue_size=1
        )
        
        tof_info_topic = f'/{self.veh}/front_center_tof_driver_node/range'
        self.tof_info_sub = rospy.Subscriber(
            tof_info_topic,
            Range,
            self.getTofInfo,
            queue_size=1
        )
        
        self.image_pub = rospy.Publisher(f'/{self.veh}/rectified_image', Image, queue_size=10)

        self.tag_pub = rospy.Publisher(f'/{self.veh}/detected_tags', Int32MultiArray, queue_size=5)
        
        self.motion_pub = rospy.Publisher(f'/{self.veh}/joy', Joy, queue_size=1)
        
        # variable for controlling wheels' speed
        self.msg_wheels_cmd = WheelsCmdStamped()
        
        self.pub_wheels_cmd = rospy.Publisher(
            "~wheels_cmd_executed",
            WheelsCmdStamped,
            queue_size=1,
            dt_topic_type=TopicType.DRIVER
        )
        
        self.pose_msg = LanePose()
        
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
        
        # Construct publishers
        self.pub_car_cmd = rospy.Publisher("~car_cmd",
            Twist2DStamped,
            queue_size=1,
            dt_topic_type=TopicType.CONTROL
        )
        
    
    def publishCmd(self, car_cmd_msg):
        """Publishes a car command message.
        Args:
            car_cmd_msg (:obj:`Twist2DStamped`): Message containing the requested control action.
        """
        self.pub_car_cmd.publish(car_cmd_msg)
        
    
        
        
    
    def getTofInfo(self, tof_msg):
        """
            TOF callback
        """
        # print(tof_msg)
        pass

    def region_of_interests(self, edges):
        """
        helper function for clipping useful parts
        """
        height, width = edges.shape
        mask = np.zeros_like(edges)
        
        # keep the bottom half of the screen
        polygon = np.array([[
            (0, height * 1 / 2),
            (width, height * 1 / 2),
            (width, height),
            (0, height),
        ]], np.float32)
        
        cv2.fillPoly(mask, np.int32([polygon]), 255)
        
        cropped_edges = cv2.bitwise_and(edges, mask)
        return cropped_edges
    
    def detect_line_segments(self, cropped_edges):
        rho = 1
        angle = np.pi / 180
        min_threshold = 10
        line_segnments = cv2.HoughLinesP(cropped_edges, rho, angle, min_threshold, np.array([]), minLineLength=8, maxLineGap=4)
        
        return line_segnments
    
    
    def make_points(self, frame, line):
        height, width, _ = frame.shape
        slope, intercept = line
        y1 = height  # bottom of the frame
        y2 = int(y1 * 1 / 2)  # make points from middle of the frame down

        # bound the coordinates within the frame
        x1 = max(-width, min(2 * width, int((y1 - intercept) / slope)))
        x2 = max(-width, min(2 * width, int((y2 - intercept) / slope)))
        return [[x1, y1, x2, y2]]

    def average_slope_intercept(self, frame, line_segments):
        """
        This function combines line segments into one or two lane lines
        If all line slopes are < 0: then we only have detected left lane
        If all line slopes are > 0: then we only have detected right lane
        """
        lane_lines = []
        if line_segments is None:
            return lane_lines

        height, width, _ = frame.shape
        left_fit = []
        right_fit = []

        boundary = 1/3
        left_region_boundary = width * (1 - boundary)  # left lane line segment should be on left 2/3 of the screen
        right_region_boundary = width * boundary # right lane line segment should be on left 2/3 of the screen

        for line_segment in line_segments:
            for x1, y1, x2, y2 in line_segment:
                if x1 == x2:
                    continue
                fit = np.polyfit((x1, x2), (y1, y2), 1)
                slope = fit[0]
                intercept = fit[1]
                if slope < 0:
                    if x1 < left_region_boundary and x2 < left_region_boundary:
                        left_fit.append((slope, intercept))
                else:
                    if x1 > right_region_boundary and x2 > right_region_boundary:
                        right_fit.append((slope, intercept))

        left_fit_average = np.average(left_fit, axis=0)
        if len(left_fit) > 1:
            lane_lines.append(self.make_points(frame, left_fit_average))

        right_fit_average = np.average(right_fit, axis=0)
        if len(right_fit) > 1:
            lane_lines.append(self.make_points(frame, right_fit_average))



        return lane_lines


    def display_heading_line(self, frame, steering_angle, line_color=(0, 0, 255), line_width=5 ):
        heading_image = np.zeros_like(frame)
        height, width, _ = frame.shape

        # figure out the heading line from steering angle
        # heading line (x1,y1) is always center bottom of the screen
        # (x2, y2) requires a bit of trigonometry

        # Note: the steering angle of:
        # 0-89 degree: turn left
        # 90 degree: going straight
        # 91-180 degree: turn right 
        steering_angle_radian = steering_angle / 180.0 * math.pi
        x1 = int(width / 2)
        y1 = height
        x2 = int(x1 - height / 2 / math.tan(steering_angle_radian))
        y2 = int(height / 2)

        cv2.line(heading_image, (x1, y1), (x2, y2), line_color, line_width)
        heading_image = cv2.addWeighted(frame, 0.8, heading_image, 1, 1)

        return heading_image


    def get_xy_offset(self, lines_avg, width, height):
        if len(lines_avg) == 1:
            x1, _, x2, _ = lines_avg[0][0]
            x_offset = x2 - x1
            y_offset = int(height / 2)
            
            return x_offset, y_offset
        _, _, left_x2, _ = lines_avg[0][0]
        _, _, right_x2, _ = lines_avg[1][0]
        mid = int(width / 2)
        x_offset = (left_x2 + right_x2) / 2 - mid
        y_offset = int(height / 2)
        
        return x_offset, y_offset
            
    def stabilize_steering_angle(
          new_steering_angle, 
          num_of_lane_lines, 
          max_angle_deviation_two_lines=5, 
          max_angle_deviation_one_lane=1):
        """
        Using last steering angle to stabilize the steering angle
        if new angle is too different from current angle, 
        only turn by max_angle_deviation degrees
        """
        if num_of_lane_lines == 2:
            # if both lane lines detected, then we can deviate more
            max_angle_deviation = max_angle_deviation_two_lines
        else:
            # if only one lane detected, don't deviate too much
            max_angle_deviation = max_angle_deviation_one_lane
        
        angle_deviation = new_steering_angle - self.curr_steering_angle
        if abs(angle_deviation) > max_angle_deviation:
            stabilized_steering_angle = int(self.curr_steering_angle
                + max_angle_deviation * angle_deviation / abs(angle_deviation))
        else:
            stabilized_steering_angle = new_steering_angle
        return stabilized_steering_angle
    


    def lane_detection(self, image_msg):
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


        img_hsv = cv2.cvtColor(rectified_img, cv2.COLOR_BGR2HSV)
        
        lower_yello = np.array([20, 40, 40])
        upper_yello = np.array([45, 255, 255])
        
        
        # image dilation
        mask_yellow = cv2.inRange(img_hsv, lower_yello, upper_yello)
        
        # repair broken center lane detection
        kernel = np.ones((20,20), np.uint8)
        
        d_im = cv2.dilate(mask_yellow, kernel, iterations = 1)
        e_im = cv2.erode(d_im, kernel, iterations = 1)
        
        # smoothing the image
        kernel_smooth = np.ones((5,5), np.float32) / 25
        dist = cv2.filter2D(e_im, -1, kernel)
        
        # RGB filter for white lane
        (B, G, R) = cv2.split(rectified_img)
        R[R < 230] = 0
        R[R >= 230] = 255
        
        G[G < 170] = 0
        G[G >= 170] = 255
        
        B[B < 115] = 0
        B[B >= 115] = 255
        
        res_rg = np.maximum(R, G)
        
        res_rgb = np.maximum(res_rg, B)
        
        # repair the white lane detection
        kernel_white = np.ones((25,25), np.uint8)
        d_res_rgb = cv2.dilate(res_rgb, kernel_white, iterations = 1)
        e_res_rgb = cv2.erode(d_res_rgb, kernel_white, iterations = 1)
        
        res_center_lane = np.maximum(dist, e_res_rgb)
        
        res_center_lane = cv2.blur(res_center_lane, (9, 9))
        
        # lane detection
        edges = cv2.Canny(res_center_lane, 50,200,apertureSize = 3)
        
        cropped_edges = self.region_of_interests(edges)
        
        line_segments = self.detect_line_segments(cropped_edges)
        
        lines_avg = self.average_slope_intercept(rectified_img, line_segments)

        if lines_avg is None:
            return
        
        if len(lines_avg) == 0:
            return
        
        # _, _, left_x2, _ = lines_avg[0][0]
        # _, _, right_x2, _ = lines_avg[1][0]
        # mid = int(width / 2)
        # x_offset = (left_x2 + right_x2) / 2 - mid
        # y_offset = int(height / 2)
        
        # display detected lines on the image
        for x in range(0, len(lines_avg)):
            for x1, y1, x2, y2 in lines_avg[x]:
                cv2.line(rectified_img, (x1, y1), (x2, y2), (0, 255, 0), 4)
        
        height, width, _ = rectified_img.shape
        
        x_offset, y_offset = self.get_xy_offset(lines_avg, width, height)
        
        angle_to_mid_radian = math.atan(x_offset / y_offset)  # angle (in radian) to center vertical line
        angle_to_mid_deg = int(angle_to_mid_radian * 180.0 / math.pi)  # angle (in degrees) to center vertical line
        
        
        steering_angle = angle_to_mid_deg + 90  # this is the steering angle needed by Duckiebot
        print(steering_angle)
        
        
        # self.msg_wheels_cmd.header = "~wheels_cmd"
        # self.msg_wheels_cmd.header.stamp = rospy.get_rostime()
        
        # Initialize car control msg, add header from input message
        car_control_msg = Twist2DStamped()
        car_control_msg.header = self.pose_msg.header
        
        # Add commands to car message
        car_control_msg.v = 0
        car_control_msg.omega = 0
        
        axes = [0.0, 0.5, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        buttons = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
        
        if angle_to_mid_deg > 8:
            car_control_msg.omega = 0.4 * angle_to_mid_deg / 90
            axes = [0.0, 0.1, 0.0, -0.35 * angle_to_mid_deg / 90, 0.0, 0.0, 0.0, 0.0] 
            print('steer left')
        elif angle_to_mid_deg < -8:
            print('steer right')
            car_control_msg.omega = 0.4 * angle_to_mid_deg / 90
            axes = [0.0, 0.1, 0.0, 0.35 * angle_to_mid_deg / 90, 0.0, 0.0, 0.0, 0.0] 
        
        print(angle_to_mid_deg)
        print(axes)


        heading_img = self.display_heading_line(rectified_img, steering_angle)
        
        
        frame = cv2.resize(heading_img, None, fx=0.6, fy=0.6)
        # Encode into JPEG
        result, imgencode = cv2.imencode('.jpg', frame, self.encode_param)
        # Send JPEG-encoded byte array
        
        
        # publish image frame to Azure cloud AMQ server
        print('res streamed')
        self.video_producer.publish(imgencode.tobytes(), content_type='image/jpeg', content_encoding='binary')



        msg = Joy(header=None, axes=axes, buttons=None)
        
        self.motion_pub.publish(msg)
        rospy.sleep(0.1)
        


if __name__ == "__main__":
    # Initialize the node
    at_pose_node = ATPoseNode(node_name='my_at_pose_node')
    # Keep it spinning
    rospy.spin()
