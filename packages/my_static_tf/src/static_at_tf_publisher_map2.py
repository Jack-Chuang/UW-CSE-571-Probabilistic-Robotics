#!/usr/bin/env python3
from typing import Optional, Tuple, List

import rospy
import tf2_ros
import tf_conversions
from duckietown.dtros import DTROS, NodeType
from geometry_msgs.msg import TransformStamped

__TAG_ID_DICT = {32: 32, 33: 31, 65: 61, 31: 33, 57: 57, 61: 65, 10: 11, 11: 10, 9: 9, 24: 26, 25: 25, 26: 24}


def fetch_tag_id(tag) -> int:
    if tag.tag_id not in __TAG_ID_DICT:
        return -1
    return __TAG_ID_DICT[tag.tag_id]


MAP_TRANSFORM_DICT = {
    31: ((0.01, 0.01, 0.075), (0, 0, 3.9269875)),
    32: ((0.01, 0.56, 0.075), (0, 0, 3.9269875)),
    61: ((0.0225, 0.86, 0.075), (0, 0, 3.14159)),
    65: ((0.545, 0.0275, 0.075), (0, 0, 4.712385)),
    57: ((1.1325, 0.1625, 0.075), (0, 0, 0)),
    33: ((1.1325, 0.4075, 0.075), (0, 0, 0)),
    9: ((0.6125, 0.7525, 0.075), (0, 0, 3.14159)),
    11: ((0.6125, 0.9975, 0.075), (0, 0, 3.14159)),
    24: ((1.205, 1.1325, 0.075), (0, 0, 1.570795)),
    10: ((1.74, 1.15, 0.075), (0, 0, 0.7853975)),
    26: ((1.74, 0.6, 0.075), (0, 0, 0.7853975)),
    25: ((1.7225, 0.3, 0.075), (0, 0, 0)),
}


class StaticATTFPublisherNode(DTROS):
    def __init__(self, node_name):
        DTROS.__init__(
            self,
            node_name=node_name,
            node_type=NodeType.GENERIC)
        self.veh = rospy.get_namespace().strip('/')
        # self.pub = rospy.Publisher(
        #     '/{}/at_tf_static',
        #     Odometry,
        #     queue_size=1,
        #     dt_topic_type=TopicType.LOCALIZATION)

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

    def run(self):
        # publish message every 1 second
        rate = rospy.Rate(1)  # 1Hz
        while not rospy.is_shutdown():
            map_frame_id = 'map'
            for tag_id, (translations, euler_angles) in MAP_TRANSFORM_DICT.items():
                tag_frame_id = 'april_tag_{}'.format(tag_id)
                self._broadcast_tf(
                    parent_frame_id=map_frame_id,
                    child_frame_id=tag_frame_id,
                    translations=translations,
                    euler_angles=euler_angles)
            rate.sleep()


if __name__ == '__main__':
    # create the node
    node = StaticATTFPublisherNode(node_name='static_at_tf_publisher_map2')
    # run node
    node.run()
    # keep spinning
    rospy.spin()
