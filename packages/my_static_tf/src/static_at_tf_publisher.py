#!/usr/bin/env python3
from typing import Optional, Tuple, List

import rospy
import tf2_ros
import tf_conversions
from duckietown.dtros import DTROS, NodeType
from geometry_msgs.msg import TransformStamped

__TAG_ID_DICT = {32: 32, 33: 31, 65: 61, 31: 33, 57: 57, 61: 65, 10: 11, 11: 10, 9: 9, 24: 26, 25: 25, 26: 24}


def fetch_tag_id(tag):
    return __TAG_ID_DICT[tag.tag_id]


transform_dict = {
    (31, 32): ((0, 0.5, 0), (0, 0, 0)),
    (31, 65): ((0.5, 0.25, 0), (0, 0, 0))
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
            for cand_tag_ids, transform_params in transform_dict.items():
                tag_frame_ids = list(map(lambda x: 'april_tag_{}'.format(x), cand_tag_ids))
                self._broadcast_tf(
                    parent_frame_id=tag_frame_ids[0],
                    child_frame_id=tag_frame_ids[1],
                    translations=transform_params[0],
                    euler_angles=transform_params[1]
                )
            rate.sleep()


if __name__ == '__main__':
    # create the node
    node = StaticATTFPublisherNode(node_name='static_at_tf_publisher')
    # run node
    node.run()
    # keep spinning
    rospy.spin()
