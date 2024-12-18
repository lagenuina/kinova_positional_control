#!/usr/bin/env python
import rospy
import numpy as np
import tf
import transformations as transform
from Scripts.srv import (SendFloat32MultiArray)
from std_msgs.msg import (
    Float32,
)
from geometry_msgs.msg import (Point)


class UpdateTransforms:

    def __init__(
        self,
        robot_name,
    ):

        self.listener = tf.TransformListener()
        self.br = tf.TransformBroadcaster()
        self.rate = rospy.Rate(10)

        self.ROBOT_NAME = robot_name
        self.__anchor_frame = 'anchor'

        self.__calibrated_anchor = False
        self.__chest_position = 0.44

        self.__anchor_set = False

        self.__calibrated_anchor_pos = [0, 0, 0]

        self.__tf_from_odom_to_camera = {
            'position': np.array([0.0, 0.0, 0.0]),
            'orientation': np.array([1.0, 0.0, 0.0, 0.0]),
        }

        self.__tf_from_odom_to_anchor = {
            'position': np.array([0.0, 0.0, 0.0]),
            'orientation': np.array([1.0, 0.0, 0.0, 0.0]),
        }

        self.__tf_from_anchor_to_toolframe = {
            'position': np.array([0.0, 0.0, 0.0]),
            'orientation': np.array([0.0, 0.0, 0.0, 1.0]),
        }

        rospy.Subscriber(
            'chest_logger/current_position',
            Float32,
            self.__update_chest_callback,
        )

        rospy.Subscriber(
            f'/my_gen3/calibrate_anchor',
            Point,
            self.__calibration_anchor_callback,
        ),

        self.__tf_tool_frame_pub = rospy.Publisher(
            '/tf_anchor_tool_frame',
            Point,
            queue_size=1,
        )

    def __calibration_anchor_callback(self, message):

        self.__calibrated_anchor_pos[0] += message.x
        self.__calibrated_anchor_pos[1] += message.y
        self.__calibrated_anchor_pos[2] += -message.z

        self.__anchor_frame = 'new_anchor'
        self.__calibrated_anchor = True

    def __update_chest_callback(self, message):

        self.__chest_position = (message.data)

    def main_loop(self):

        if not self.__anchor_set:

            try:

                (
                    self.__tf_from_odom_to_anchor['position'],
                    self.__tf_from_odom_to_anchor['orientation']
                ) = self.listener.lookupTransform(
                    '/odom',
                    '/kortex/tool_frame',
                    rospy.Time(0),
                )

                self.__anchor_set = True

            except (
                tf.LookupException, tf.ConnectivityException,
                tf.ExtrapolationException
            ):

                self.rate.sleep()

        if self.__calibrated_anchor:

            self.br.sendTransform(
                self.__calibrated_anchor_pos,
                (0, 0, 0, 1),
                rospy.Time.now(),
                self.__anchor_frame,
                'anchor',
            )

        self.br.sendTransform(
            self.__tf_from_odom_to_anchor['position'],
            self.__tf_from_odom_to_anchor['orientation'],
            rospy.Time.now(),
            'anchor',
            'odom',
        )

        self.br.sendTransform(
            (0.02, -0.14, 0.93 + self.__chest_position),
            (0.2887332, 0.2887332, -0.6454712, 0.6454712),
            rospy.Time.now(),
            'kortex/base_link',
            'base_link',
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

            (
                self.__tf_from_anchor_to_toolframe['position'],
                self.__tf_from_anchor_to_toolframe['orientation']
            ) = self.listener.lookupTransform(
                self.__anchor_frame,
                '/kortex/tool_frame',
                rospy.Time(0),
            )

            tool_frame_point = Point()
            tool_frame_point.x = self.__tf_from_anchor_to_toolframe['position'][
                0]
            tool_frame_point.y = self.__tf_from_anchor_to_toolframe['position'][
                1]
            tool_frame_point.z = self.__tf_from_anchor_to_toolframe['position'][
                2]

            self.__tf_tool_frame_pub.publish(tool_frame_point)

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

    transformations = UpdateTransforms(robot_name=kinova_name,)

    while not rospy.is_shutdown():
        transformations.main_loop()
        transformations.rate.sleep()
