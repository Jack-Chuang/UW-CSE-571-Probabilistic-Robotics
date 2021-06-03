#!/usr/bin/env python3

import numpy as np
import rospy
import tf
import matplotlib.pyplot as plt
from geometry_msgs.msg import PointStamped, PoseStamped
from tf.transformations import euler_from_quaternion, quaternion_from_euler
# import roslib; roslib.load_manifest('visualization_marker_tutorials')
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray

from duckietown_msgs.msg import Twist2DStamped, Pose2DStamped
from std_msgs.msg import Int32MultiArray

from duckietown.dtros import DTROS, NodeType, TopicType

import math
import random
import threading



class SensorFusionNode(DTROS):
    """
    Much of this code block is lifted from the official Duckietown Github:
    https://github.com/duckietown/dt-car-interface/blob/daffy/packages/dagu_car/src/velocity_to_pose_node.py

    The goal of this node is to provide a state estimate using one of the two filtering methods we have covered in class: the Extended Kalman Filter
    and the Particle Filter. You will be fusing the estimates from a motion model with sensor readings from the cameras.
    We have left some code here from the official Duckietown repo, but you should feel free to discard it
    if you so choose to use a different approach.

    The motion model callback as listed here will provide you with motion estimates, but you will need to figure out the covariance matrix yourself.
    Additionally, for the PF, you will need to figure out how to sample (based on the covariance matrix),
    and for the EKF, you will need to figure out how to Linearize. Our expectation is that this is a group project, so we are not providing
    skeleton code for these parts.

    Likewise, you will need to implement your own sensor model and figure out how to manage the sensor readings. We have implemented a subscriber below
    that will fire the `sensor_fusion_callback` whenever a new sensor reading comes in. You will need to figure out how to unpack the sensor reading and
    what to do with them. To do this, you might use the [tf](https://docs.ros.org/en/melodic/api/tf/html/python/tf_python.html) package to get the transformation from the tf tree
    at the appropriate time. Just like in the motion model, you will need to consider the covariance matrix for the sensor model.

    Args:
        node_name (:obj:`str`): a unique, descriptive name for the node that ROS will use

    Subscriber:
        ~velocity (:obj:`Twist2DStamped`): The robot velocity, typically obtained from forward kinematics

    Publisher:
        ~pose (:obj:`Pose2DStamped`): The integrated pose relative to the pose of the robot at node initialization

    """
    def __init__(self, node_name):
        # Initialize the DTROS parent class
        super(SensorFusionNode, self).__init__(
            node_name=node_name,
            node_type=NodeType.LOCALIZATION
        )
        
        self.msg_velocity = None

        # Get the vehicle name
        self.veh_name = rospy.get_namespace().strip("/")

        # Keep track of the last known pose
        self.last_pose = Pose2DStamped()
        self.last_theta_dot = 0
        self.last_v = 0

        # TODO Feel free to use a flag like this to set which type of filter you're currently using.
        # You can also define these as parameters in the launch file for this node if you're feeling fancy
        # (or if this was a system you wanted to use in the real world), but a hardcoded flag works just as well
        # for a class project like this.
        self.FUSION_TYPE = "PF"

        # Setup the publisher for marker
        topic = 'visualization_marker_array'
        self.marker_publisher = rospy.Publisher(
            topic, 
            MarkerArray, 
            queue_size=1
        )

        # Setup the publisher
        self.pub_pose = rospy.Publisher(
            "~pose",
            Pose2DStamped,
            queue_size=1,
            dt_topic_type=TopicType.LOCALIZATION
        )

        # Setup the publisher for visulize
        self.pub_loc = rospy.Publisher(
            "/localization",
            PoseStamped,
            queue_size=1
        )


        # Setup the subscriber to the motion of the robot
        self.sub_velocity = rospy.Subscriber(
            f"/{self.veh_name}/kinematics_node/velocity",
            Twist2DStamped,
            self.motion_model_callback,
            queue_size=1
        )


        # Setup the subscriber for when sensor readings come in
        self.sub_sensor = rospy.Subscriber(
            f"/{self.veh_name}/detected_tags",
            Int32MultiArray,
            self.sensor_fusion_callback,
            queue_size=1
        )

        # experimental subscriber
        self.sub_sensor = rospy.Subscriber(
            f"/{self.veh_name}/front_center_tof_driver_node/range",
            Int32MultiArray,
            self.distance_measurement,
            queue_size=1
        )

        # create a marker array
        self.markerArray = MarkerArray()

        # Setup the subscriber for apriltag
        self.tag = tf.TransformListener()


        # define the motion model noise
        self.R = np.diag([
            0.05,  # variance of x-coordinate
            0.05,  # variance of y-coordinate
            np.deg2rad(10.0) # variance of yaw angle +- 5 degree
        ])

        # observation covariance matrix (sigma_r, sigma_phi, sigma_s)
        self.Q = np.array([
            [0.05, 0, 0],
            [0, np.deg2rad(10.0), 0],
            [0, 0, 0.05]
        ])

        # define the motion model jacobian
        self.jacob_G = np.array([
            [0.5, 0, 0],
            [0, 0.5, 0],
            [0, 0, 0.5]
        ])
        
        # a list of April tags with their corresponding coordinates
        self.marker_list = {"tag_31": [0, 0], "tag_32": [0, 0.5], "tag_65": [0.5, 0.25]}

        # define the motion model covariance matrix
        self.cov_m = np.eye(3)
        
        self.dt = 0.0

        # the selected set of particles for PF
        self.X_bar = []

        # iteration of PF
        self.PF_iter = 0

        # ---
        self.log("Initialized.")

        print(self.distance_measurement)


    def sensor_fusion_callback(self, msg_sensor):
        """
        This function should handle the sensor fusion. It will fire whenever
        the robot observes an AprilTag
        """
        # preprocess apriltag info
        # print(self.tag.getFrameStrings())
        
        # perform motion update
        if self.msg_velocity is None:
            return
        
        
        

        tf_msg = []
        at_ids = msg_sensor.data
        for at_id in at_ids:
            tag_frame_id = 'april_tag_{}'.format(at_id)
            base_link = "at_%d_base_link" % at_id
            transform = self.tag.lookupTransform(base_link,tag_frame_id, rospy.Time(0))
            angle = np.arctan2(transform[0][1], transform[0][0])
            radius = np.sqrt(transform[0][0]**2 + transform[0][1]**2)
            tf_msg.append([radius, angle, at_id])
            # print([radius, angle, at_id])
            

        # offset = [-0.5,0.25]
        offset = [0,0.25]

        if self.FUSION_TYPE == "EKF":
            # perform EKF filtering

            # motion update
            # update the robot pose according to motion model
            self.motion_model_update(self.msg_velocity)
            # update the motion model jacobian G
            if self.last_theta_dot != 0:
                G = np.array([
                    [1, 0, self.last_v / self.last_theta_dot * np.cos(self.theta_old) - self.last_v / self.last_theta_dot * np.cos(self.last_pose.theta)],
                    [0, 1, self.last_v / self.last_theta_dot * np.sin(self.theta_old) - self.last_v / self.last_theta_dot * np.sin(self.last_pose.theta)],
                    [0, 0, 1]
                ])

                self.jacob_G = G
            mu_pred = np.array([self.last_pose.x, self.last_pose.y, self.last_pose.theta]).T
            jacob_G = self.jacob_G
            cov_t_pred = jacob_G @ self.cov_m  @ jacob_G.T + self.R

            # measurement update
            # print(msg_sensor.header.stamp)

            mu_t = mu_pred
            cov_t = cov_t_pred
            q = None

            for z in tf_msg:
                # replace msg_sensor.features with the correctly unpacked
                # april tags measurements (r, phi, s) corresponding to radius, bearing and signature/tag index
                j = "tag_%s" % z[2]
                tag_frame_id = 'april_tag_{}'.format(z[2])
                tag_tf = self.tag.lookupTransform(tag_frame_id, 'map', rospy.Time(0))
                # print(j,tag_tf)
                delta_x = tag_tf[0][0] - mu_pred[0]
                delta_y = tag_tf[0][1] - mu_pred[1]
                delta = np.array([delta_x, delta_y]).T

                q = delta.T @ delta
                
                if q == 0:
                    continue

                z_pred = np.array([np.sqrt(q), np.arctan2(delta_y, delta_x) - mu_pred[-1], z[2]]).T

                jacob_H = 1 / q * np.array([
                    [np.sqrt(q)*delta_x, -np.sqrt(q)*delta_y, 0],
                    [delta_y, delta_x, -1],
                    [0, 0, 0]
                ])
                K_i = cov_t_pred @ jacob_H.T @ np.linalg.inv(jacob_H @ cov_t_pred @ jacob_H.T + self.Q)
                
                mu_t += K_i @ (np.array(z).T - z_pred)
                cov_t -= K_i @ jacob_H @ cov_t_pred

            if True:
            # if mu_t[0] - self.last_pose :
                # publish the pose
                msg_pose = Pose2DStamped()
                msg_pose.header.stamp = rospy.Time.now() 
                msg_pose.header.frame_id = self.veh_name
                msg_pose.theta = mu_t[2]
                msg_pose.x = mu_t[0]
                msg_pose.y = mu_t[1]

                # update covariance matrix for motion model
                self.cov_m = cov_t
                self.pub_pose.publish(msg_pose)
                # plt.plot(msg_pose.x, msg_pose.y)
                # plt.savefig()
                # plt.show()


                pose = PoseStamped()
                pose.header.frame_id = "map"
                x,y,z,w = quaternion_from_euler(0,0,msg_pose.theta)
                pose.pose.orientation.x = x
                pose.pose.orientation.y = y
                pose.pose.orientation.z = z
                pose.pose.orientation.w = w

                pose.header.stamp = rospy.Time.now()
                pose.pose.position.x = msg_pose.x+offset[0]
                pose.pose.position.y = msg_pose.y+offset[1]
                # print("publish")
                self.pub_loc.publish(pose)
                # point = PointStamped()
                # point.header.stamp = rospy.Time.now()
                # point.header.frame_id = "map"
                # point.point.x = msg_pose.x
                # point.point.y = msg_pose.y
                # point.
                # self.pub_loc.publish(point)
        
        if self.FUSION_TYPE == "PF":
            # parameter for the sample motion model velocity
            alpha_1 = 1.0
            alpha_2 = 0.5
            alpha_3 = 0.5
            alpha_4 = 1.0
            alpha_5 = 0.5
            alpha_6 = 1.0

            M = 5000
            
            su_x, su_y, su_theta, avg_x, avg_y, avg_theta, count = 0, 0, 0, 0, 0, 0, 0
            
            mw = 0

            self.PF_iter += 1


            if self.PF_iter == 1:
                for m in range(M):
                    x_t = self.first_sample_motion_model(M)
                    # likelyhood field range finder measurement model
                    w_t = self.measurement_model(M, tf_msg, x_t)
                    # add particles to self.X_bar
                    self.X_bar.append([x_t, w_t])

                    if self.X_bar[m][1] > mw:
                        mw = self.X_bar[m][1]
            if self.PF_iter > 1:
                for m in range(len(self.X_bar)):
                    
                    x_t = self.sample_motion_model(len(self.X_bar), m)
                    # likelyhood field range finder measurement model
                    w_t = self.measurement_model(len(self.X_bar), tf_msg, x_t)
                    # add particles to self.X_bar
                    self.X_bar.append([x_t, w_t])
                    self.X_bar.pop(m)
                    if self.X_bar[m][1] > mw:
                        mw = self.X_bar[m][1]
            # print(len(self.X_bar))
            if len(self.X_bar) != 0:
                # resampling of the particles
                #if variance < some number:
                r = np.random.rand() * mw
                c = self.X_bar[0][1]
                i = 0
                for m in range(M):
                    u = r + (m - 1) * 1 / M
                    while u > c:
                        i += 1
                        if i == len(self.X_bar):
                            i -= 1
                            break
                        c = c + self.X_bar[i][1]
                    su_x += self.X_bar[i][0][0]
                    su_y += self.X_bar[i][0][1]
                    su_theta += self.X_bar[i][0][2]
                    count += 1
                    #X.append(self.X_bar[i][0]) 

                    marker = Marker()
                    marker.header.frame_id = "map"
                    marker.type = marker.SPHERE
                    marker.action = marker.ADD
                    marker.scale.x = 0.1
                    marker.scale.y = 0.1
                    marker.scale.z = 0.1
                    marker.color.a = 1.0
                    marker.color.r = 1.0
                    marker.color.g = 1.0
                    marker.color.b = 0.0
                    marker_quaternion = quaternion_from_euler(self.X_bar[i][0][0], self.X_bar[i][0][1], self.X_bar[i][0][2])
                    marker.pose.orientation.x = marker_quaternion[0]
                    marker.pose.orientation.y = marker_quaternion[1]
                    marker.pose.orientation.z = marker_quaternion[2]
                    marker.pose.orientation.w = marker_quaternion[3]
                    marker.pose.position.x = self.X_bar[i][0][0]
                    marker.pose.position.y = self.X_bar[i][0][1] 
                    marker.pose.position.z = 0 
                    marker.id = m
                    self.markerArray.markers.append(marker)

                
                self.marker_publisher.publish(self.markerArray)
                self.markerArray.markers.clear()

                avg_x = su_x / count
                avg_y = su_y / count
                avg_theta = su_theta / count

                # publish the pose
                msg_pose = Pose2DStamped()
                msg_pose.header.stamp = rospy.Time.now() 
                msg_pose.header.frame_id = self.veh_name
                msg_pose.theta = avg_theta
                msg_pose.x = avg_x
                msg_pose.y = avg_y

                self.pub_pose.publish(msg_pose)

                pose = PoseStamped()
                pose.header.frame_id = "map"
                x,y,z,w = quaternion_from_euler(0,0,msg_pose.theta)
                pose.pose.orientation.x = x
                pose.pose.orientation.y = y
                pose.pose.orientation.z = z
                pose.pose.orientation.w = w

                pose.header.stamp = rospy.Time.now()
                pose.pose.position.x = msg_pose.x+offset[0]
                pose.pose.position.y = msg_pose.y+offset[1]
                self.pub_loc.publish(pose)
        
        # self.lock.release()
    '''
    helper function for calculating the gaussian probability
    '''
    def calc_gauss(self, x, sigma):
        p = 1.0 / math.sqrt(2.0 * math.pi * sigma ** 2) * math.exp(-x ** 2 / (2 * sigma ** 2))
        return p

    def first_sample_motion_model(self, M):
        # alpha_1 = 1.0
        # alpha_2 = 0.5
        # alpha_3 = 0.5
        # alpha_4 = 1.0
        # alpha_5 = 0.5
        # alpha_6 = 1.0
        # w_t = 1/M
        # # sample motion model velocity with sample triangular distribution
        # v_pred = self.last_v + (alpha_1 * self.last_v + alpha_2 * self.last_theta_dot) * (np.random.rand() - 0.5) * 2 * (np.random.rand() - 0.5) * 2 + np.random.rand() * 0.1
        # omega_pred = self.last_theta_dot + (alpha_3 * self.last_v + alpha_4 * self.last_theta_dot) * (np.random.rand() - 0.5) * 2 * (np.random.rand() - 0.5) * 2 + (np.random.rand() - 0.5) * 0.1 * np.pi
        # gama = (alpha_5 * self.last_v+ alpha_6 * self.last_theta_dot) * (np.random.rand() - 0.5) * 2 * (np.random.rand() - 0.5) * 2
        # # update the x, y, and theta
        # x_prime = 0.0
        # y_prime = 0.0
        # theta_prime = 0.0
        # if omega_pred == 0:
        #     x_prime = self.last_pose.x + np.random.rand() * 0.1
        #     y_prime = self.last_pose.y + np.random.rand() * 0.1
        #     theta_prime = self.last_pose.theta + (np.random.rand() - 0.5) * 0.1 * np.pi
        # else:
        #     x_prime = self.last_pose.x - v_pred/omega_pred * np.sin(self.last_pose.theta) + v_pred/omega_pred * np.sin(self.last_pose.theta + omega_pred * self.dt) + np.random.rand() * 0.1
        #     y_prime = self.last_pose.y + v_pred/omega_pred * np.cos(self.last_pose.theta) - v_pred/omega_pred * np.cos(self.last_pose.theta + omega_pred * self.dt) + + np.random.rand() * 0.1
        #     theta_prime = self.last_pose.theta + omega_pred * self.dt + gama * self.dt + (np.random.rand() - 0.5) * 0.1 * np.pi
        
        x_prime = random.uniform(-2, 2)
        y_prime = random.uniform(-2, 2)
        theta_prime = random.uniform(-np.pi, np.pi)

        x_t = np.array([x_prime, y_prime, theta_prime]).T
        return x_t

    def sample_motion_model(self, M, m):
        alpha_1 = 1.0
        alpha_2 = 0.5
        alpha_3 = 0.5
        alpha_4 = 1.0
        alpha_5 = 0.5
        alpha_6 = 1.0
        w_t = 1 / M
        # sample motion model velocity with sample triangular distribution
        # v_pred = self.last_v + (alpha_1 * self.last_v + alpha_2 * self.last_theta_dot) * (np.random.rand() - 0.5) * 2 * (np.random.rand() - 0.5) * 2
        # omega_pred = self.last_theta_dot + (alpha_3 * self.last_v + alpha_4 * self.last_theta_dot) * (np.random.rand() - 0.5) * 2 * (np.random.rand() - 0.5) * 2
        # gama = (alpha_5 * self.last_v + alpha_6 * self.last_theta_dot) * (np.random.rand() - 0.5) * 2 * (np.random.rand() - 0.5) * 2
        v_pred = self.last_v
        omega_pred = self.last_theta_dot
        gama = (alpha_5 * self.last_v + alpha_6 * self.last_theta_dot) * (np.random.rand() - 0.5) * 2 * (np.random.rand() - 0.5) * 2
        # update the x, y, and theta
        x_prime = 0.0
        y_prime = 0.0
        theta_prime = 0.0
        if omega_pred == 0:
            x_prime = self.X_bar[m][0][0]
            y_prime = self.X_bar[m][0][1]
            theta_prime = self.X_bar[m][0][2]
        else:
            x_prime = self.X_bar[m][0][0] - v_pred/omega_pred * np.sin(self.X_bar[m][0][2]) + v_pred/omega_pred * np.sin(self.X_bar[m][0][2] + omega_pred * self.dt)
            y_prime = self.X_bar[m][0][1] + v_pred/omega_pred * np.cos(self.X_bar[m][0][2]) - v_pred/omega_pred * np.cos(self.X_bar[m][0][2] + omega_pred * self.dt)
            theta_prime = self.X_bar[m][0][2] + omega_pred * self.dt + gama * self.dt
        
        x_t = np.array([x_prime, y_prime, theta_prime]).T
        return x_t

    def measurement_model(self, total, tf_msg, x_t):
        w_t = 0
        for z in tf_msg:
            w_t = 1 / total
            # j = "tag_%s" % z[2]
            # delta_x = self.marker_list[j][0] - x_prime
            # delta_y = self.marker_list[j][1] - y_prime

            tag_frame_id = 'april_tag_{}'.format(z[2])
            tag_tf = self.tag.lookupTransform(tag_frame_id, 'map', rospy.Time(0))
            # print(j,tag_tf)
            delta_x = tag_tf[0][0] - x_t[0]
            delta_y = tag_tf[0][1] - x_t[1]

            r_pred = np.sqrt((delta_x) ** 2 + (delta_y) ** 2)

            w_t_r = self.calc_gauss(r_pred - z[0], 0.1)
            w_t_theta = self.calc_gauss(x_t[2] - z[1], np.deg2rad(10.0))
            w_t *= w_t_r * w_t_theta
        return w_t

    def motion_model_update(self, msg_velocity):
        
        # check the validity of sensor readings
        if abs(msg_velocity.v) > 1.0 or abs(msg_velocity.omega) > 1.0:
            return
        
        if self.last_pose.header.stamp.to_sec() > 0:  # skip first frame

            dt = (msg_velocity.header.stamp - self.last_pose.header.stamp).to_sec()
            
            self.dt = dt

            # record the last theta
            self.theta_old = self.last_pose.theta

            # Integrate the relative movement between the last pose and the current
            theta_delta = self.last_theta_dot * dt
            # to ensure no division by zero for radius calculation:
            if np.abs(self.last_theta_dot) < 0.000001:
                # straight line
                x_delta = self.last_v * dt
                y_delta = 0
            else:
                # arc of circle
                radius = self.last_v / self.last_theta_dot
                x_delta = radius * np.sin(theta_delta)
                y_delta = radius * (1.0 - np.cos(theta_delta))

            # Add to the previous to get absolute pose relative to the starting position
            theta_res = self.last_pose.theta + theta_delta
            x_res = self.last_pose.x + x_delta * np.cos(self.last_pose.theta) - y_delta * np.sin(self.last_pose.theta)
            y_res = self.last_pose.y + y_delta * np.cos(self.last_pose.theta) + x_delta * np.sin(self.last_pose.theta)

            # Update the stored last pose
            self.last_pose.theta = theta_res
            self.last_pose.x = x_res
            self.last_pose.y = y_res
            # print(self.last_pose)
            # TODO Note how this puts the motion model estimate into a message and publishes the pose.
            # You will also need to publish the pose coming from sensor fusion when you correct
            # the estimates from the motion model
            # msg_pose = Pose2DStamped()
            # msg_pose.header = msg_velocity.header
            # msg_pose.header.frame_id = self.veh_name
            # msg_pose.theta = theta_res
            # msg_pose.x = x_res
            # msg_pose.y = y_res
            # self.pub_pose.publish(msg_pose)

        self.last_pose.header.stamp = msg_velocity.header.stamp
        self.last_theta_dot = msg_velocity.omega
        self.last_v = msg_velocity.v


    def motion_model_callback(self, msg_velocity):
        """

        This function will use robot velocity information to give a new state
        Performs the calclulation from velocity to pose and publishes a messsage with the result.


        Feel free to modify this however you wish. It's left more-or-less as-is
        from the official duckietown repo

        Args:
            msg_velocity (:obj:`Twist2DStamped`): the current velocity message

        """
        # update the odometry message
        self.msg_velocity = msg_velocity

    def distance_measurement(self, msg_distance):
        self.msg_distance = msg_distance

if __name__ == '__main__':
    # Initialize the node
    sensor_fusion_node = SensorFusionNode(node_name='sensor_fusion_node')
    # Keep it spinning to keep the node alive
    rospy.spin()