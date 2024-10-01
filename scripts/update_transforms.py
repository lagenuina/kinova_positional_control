#!/usr/bin/env python
import rospy
import numpy as np
import tf
import transformations as transform
from Scripts.srv import (SendFloat32MultiArray)
from std_msgs.msg import (
    Float32,
)


class UpdateTransforms:

    def __init__(
        self,
        robot_name,
        anchor_id,
    ):

        self.listener = tf.TransformListener()
        self.br = tf.TransformBroadcaster()
        self.rate = rospy.Rate(10)

        self.ROBOT_NAME = robot_name
        self.anchor_id = str(anchor_id)
        self.__anchor_found = False
        self.anchor_frame = self.anchor_id
        self.__chest_position = 0.44

        self.__tf_from_odom_to_camera = {
            'position': np.array([0.0, 0.0, 0.0]),
            'orientation': np.array([1.0, 0.0, 0.0, 0.0]),
        }

        self.__tf_from_odom_to_anchor = {
            'position': np.array([0.0, 0.0, 0.0]),
            'orientation': np.array([0.0, 0.0, 0.0, 1.0]),
        }

        rospy.Service(
            '/set_anchor',
            SendFloat32MultiArray,
            self.__set_anchor,
        )

        rospy.Subscriber(
            'chest_logger/current_position',
            Float32,
            self.__update_chest_callback,
        )

    def __set_anchor(self, request):

        anchor_pose = request.data.data

        transformation_matrix = transform.quaternion_matrix(
            [
                self.__tf_from_odom_to_camera['orientation'][3],
                self.__tf_from_odom_to_camera['orientation'][0],
                self.__tf_from_odom_to_camera['orientation'][1],
                self.__tf_from_odom_to_camera['orientation'][2]
            ]
        )
        transformation_matrix[:3, 3] = self.__tf_from_odom_to_camera['position']

        transformation_matrix_target = transform.quaternion_matrix([1, 0, 0, 0])
        transformation_matrix_target[:3, 3] = anchor_pose

        odom_to_anchor = np.dot(
            transformation_matrix,
            transformation_matrix_target,
        )

        self.__tf_from_odom_to_anchor['position'] = odom_to_anchor[:3, 3]

        self.__anchor_found = True
        return True

    def __update_chest_callback(self, message):

        self.__chest_position = (message.data)

    def main_loop(self):

        if self.__anchor_found:

            self.br.sendTransform(
                self.__tf_from_odom_to_anchor['position'],
                self.__tf_from_odom_to_camera['orientation'],
                rospy.Time.now(),
                self.anchor_frame,
                '/odom',
            )

        self.br.sendTransform(
            (0.0, 0.05, 0.855 + self.__chest_position),
            (-0.5, 0.5, -0.5, 0.5),
            rospy.Time.now(),
            '/chest_cam',
            '/base_link',
        )

        try:

            (
                self.__tf_from_odom_to_camera['position'],
                self.__tf_from_odom_to_camera['orientation']
            ) = self.listener.lookupTransform(
                '/odom',
                '/chest_cam',
                rospy.Time(0),
            )

        except (
            tf.LookupException, tf.ConnectivityException,
            tf.ExtrapolationException
        ):

            self.rate.sleep()


if __name__ == '__main__':

    rospy.init_node('tf_updates',)

    kinova_name = rospy.get_param(
        param_name=f'{rospy.get_name()}/robot_name',
        default='my_gen3',
    )

    anchor_id = rospy.get_param(param_name=f'{rospy.get_name()}/anchor_id')

    transformations = UpdateTransforms(
        robot_name=kinova_name,
        anchor_id=anchor_id,
    )

    while not rospy.is_shutdown():
        transformations.main_loop()
        transformations.rate.sleep()
