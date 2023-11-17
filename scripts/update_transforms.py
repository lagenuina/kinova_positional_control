#!/usr/bin/env python
import rospy
import numpy as np
import tf
import transformations as transform
import math
from std_msgs.msg import (Bool, Float64MultiArray)
from geometry_msgs.msg import (Pose, Point, Quaternion)
from gopher_ros_clearcore.msg import (Position)


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
        self.anchor_id = anchor_id

        self.anchor_frame = self.anchor_id

        self.chest_position = 0.44
        self.calibrated_anchor = False
        self.calibrated_anchor_pos = [0, 0, 0]
        self.trans_tool_frame = [0, 0, 0]
        self.rot_kortex_tool_frame = [0, 0, 0, 1]
        self.trans_kortex_baselink = [0, 0, 0]
        self.rot_kortex_baselink = [0, 0, 0, 1]
        self.trans_baselink = [0, 0, 0]
        self.rot_baselink = [0, 0, 0, 1]
        self.trans_anchor = [0, 0, 0]
        self.rot_anchor = [0, 0, 0, 1]
        self.world_rotation_anchor = [0, 0, 0, 1]

        self.__tf_toolframe_anchor = rospy.Publisher(
            f'/{self.ROBOT_NAME}/tf_toolframe',
            Point,
            queue_size=1,
        )

        self.__tf_baselink_anchor = rospy.Publisher(
            f'/{self.ROBOT_NAME}/tf_baselink',
            Pose,
            queue_size=1,
        )

        rospy.Subscriber(
            f'/{self.ROBOT_NAME}/calibrate_anchor',
            Point,
            self.__calibration_anchor_callback,
        ),

        rospy.Subscriber(
            'z_chest_pos',
            Position,
            self.__update_chest_callback,
        )

        rospy.Subscriber(
            'unity_anchor_rotation',
            Quaternion,
            self.__unity_anchor_rotation_callback,
        )

    def __unity_anchor_rotation_callback(self, message):

        self.world_rotation_anchor = [
            message.x,
            message.y,
            message.z,
            message.w,
        ]

    def __update_chest_callback(self, message):

        self.chest_position = (message.position) / 1000

    def __calibration_anchor_callback(self, message):

        self.calibrated_anchor_pos[0] += message.x
        self.calibrated_anchor_pos[1] += message.y
        self.calibrated_anchor_pos[2] += -message.z

        self.anchor_frame = 'new_anchor'
        self.calibrated_anchor = True

    def main_loop(self):

        if self.calibrated_anchor:

            self.br.sendTransform(
                self.calibrated_anchor_pos, (0, 0, 0, 1), rospy.Time.now(),
                self.anchor_frame, self.anchor_id
            )

        self.br.sendTransform(
            (0.02, -0.14, 0.93 + self.chest_position),
            (0.2887332, 0.2887332, -0.6454712, 0.6454712), rospy.Time.now(),
            'kortex/base_link', 'base_link'
        )

        try:

            (self.trans_tool_frame,
             self.rot_tool_frame) = self.listener.lookupTransform(
                 self.anchor_frame, 'kortex/tool_frame', rospy.Time(0)
             )

            (self.trans_kortex_baselink,
             self.rot_kortex_baselink) = self.listener.lookupTransform(
                 self.anchor_frame, 'kortex/base_link', rospy.Time(0)
             )

            # Calculate world rotation of base link Kortex frame
            self.rot_kortex_baselink = transform.quaternion_multiply(
                self.rot_kortex_baselink,
                self.world_rotation_anchor,
            )

            # Transform to Left coordinate syste
            self.rot_kortex_baselink = transform.quaternion_multiply(
                transform.quaternion_about_axis(
                    np.deg2rad(90),
                    (
                        1,
                        0,
                        0,
                    ),
                ),
                self.rot_kortex_baselink,
            )

        except (
            tf.LookupException, tf.ConnectivityException,
            tf.ExtrapolationException
        ):

            self.rate.sleep()

        tool_frame_position = Point()
        tool_frame_position.x = self.trans_tool_frame[0]
        tool_frame_position.y = self.trans_tool_frame[1]
        tool_frame_position.z = self.trans_tool_frame[2]

        self.__tf_toolframe_anchor.publish(tool_frame_position)

        base_link_position = Pose()
        base_link_position.position.x = self.trans_kortex_baselink[0]
        base_link_position.position.y = self.trans_kortex_baselink[1]
        base_link_position.position.z = self.trans_kortex_baselink[2]
        base_link_position.orientation.x = self.rot_kortex_baselink[0]
        base_link_position.orientation.y = self.rot_kortex_baselink[1]
        base_link_position.orientation.z = self.rot_kortex_baselink[2]
        base_link_position.orientation.w = self.rot_kortex_baselink[3]

        self.__tf_baselink_anchor.publish(base_link_position)


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