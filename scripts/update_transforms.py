#!/usr/bin/env python
import rospy
import numpy as np
import tf
import transformations as transform
import math
from std_msgs.msg import (Bool, Float32MultiArray)
from geometry_msgs.msg import (Pose, Point, Quaternion)
from gopher_ros_clearcore.msg import (Position)
from Scripts.srv import ConvertTargetPosition


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

        self.world_rotation_anchor = [0, 0, 0, 1]

        self.anchor_tool_frame_tf = {
            'position': np.array([0.0, 0.0, 0.0]),
            'orientation': np.array([0.0, 0.0, 0.0, 1.0]),
        }

        self.kortex_baselink_tf = {
            'position': np.array([0.0, 0.0, 0.0]),
            'orientation': np.array([0.0, 0.0, 0.0, 1.0]),
        }

        self.chest_cam_anchor_tf = {
            'position': np.array([0.0, 0.0, 0.0]),
            'orientation': np.array([0.0, 0.0, 0.0, 1.0]),
        }

        self.target_cam_base_tf = {
            'position': np.array([0.0, 0.0, 0.0]),
            'orientation': np.array([0.0, 0.0, 0.0, 1.0]),
        }

        self.camera_target_tf = [0, 0, 0]
        self.target_cam_anchor_tf = {
            'position': np.array([0.0, 0.0, 0.0]),
            'orientation': np.array([0.0, 0.0, 0.0, 1.0]),
        }

        self.trans_tool_base = {
            'position': np.array([0.0, 0.0, 0.0]),
            'orientation': np.array([1.0, 0.0, 0.0, 0.0]),
        }

        self.__tf_toolframe_anchor = rospy.Publisher(
            f'/{self.ROBOT_NAME}/tf_toolframe',
            Pose,
            queue_size=1,
        )

        self.__tf_baselink_anchor = rospy.Publisher(
            f'/{self.ROBOT_NAME}/tf_baselink',
            Pose,
            queue_size=1,
        )

        self.__tf_baselink_toolframe = rospy.Publisher(
            f'/{self.ROBOT_NAME}/tf_base_tool_frame',
            Pose,
            queue_size=1,
        )

        self.__tf_target_camera_pub = rospy.Publisher(
            '/my_gen3/target_hologram',
            Pose,
            queue_size=1,
        )

        self.__tf_target_camera_base_pub = rospy.Publisher(
            '/my_gen3/tf_base_target_cam',
            Pose,
            queue_size=1,
        )

        self.__tf_chest_cam_anchor_tf_pub = rospy.Publisher(
            '/my_gen3/tf_chest_cam_anchor',
            Pose,
            queue_size=1,
        )

        rospy.Subscriber(
            f'/{self.ROBOT_NAME}/calibrate_anchor',
            Point,
            self.__calibration_anchor_callback,
        ),

        rospy.Subscriber(
            f'/{self.ROBOT_NAME}/target_workspace_cam',
            Point,
            self.__camera_target_callback,
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

        self.convert_target_service = rospy.Service(
            '/from_chest_to_anchor',
            ConvertTargetPosition,
            self.convert_target_position,
        )

    def convert_target_position(self, request):

        transformation_matrix = transform.quaternion_matrix(
            [
                self.chest_cam_anchor_tf['orientation'][3],
                self.chest_cam_anchor_tf['orientation'][0],
                self.chest_cam_anchor_tf['orientation'][1],
                self.chest_cam_anchor_tf['orientation'][2]
            ]
        )
        transformation_matrix[:3, 3] = self.chest_cam_anchor_tf['position']

        transformation_matrix_target = transform.quaternion_matrix([1, 0, 0, 0])
        transformation_matrix_target[:3, 3] = request.fromchest.data

        anchor_to_target = np.dot(
            transformation_matrix, transformation_matrix_target
        )

        translation = Float32MultiArray()
        translation.data = anchor_to_target[:3, 3]

        return translation

    def __unity_anchor_rotation_callback(self, message):

        self.world_rotation_anchor = [
            message.x,
            message.y,
            message.z,
            message.w,
        ]

    def __camera_target_callback(self, message):

        self.camera_target_tf[0] = message.x
        self.camera_target_tf[1] = message.y
        self.camera_target_tf[2] = message.z

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

        self.br.sendTransform(
            (0.0, 0.05, 0.855 + self.chest_position), (-0.5, 0.5, -0.5, 0.5),
            rospy.Time.now(), '/chest_cam', '/base_link'
        )

        self.br.sendTransform(
            self.camera_target_tf,
            (0, 0, 0, 1),
            rospy.Time.now(),
            '/target_cam',
            self.anchor_frame,
        )

        try:

            (
                self.anchor_tool_frame_tf['position'],
                self.anchor_tool_frame_tf['orientation']
            ) = self.listener.lookupTransform(
                self.anchor_frame,
                'kortex/tool_frame',
                rospy.Time(0),
            )

            (
                self.trans_tool_base['position'],
                self.trans_tool_base['orientation']
            ) = self.listener.lookupTransform(
                '/base_link', 'kortex/tool_frame', rospy.Time(0)
            )

            (
                self.kortex_baselink_tf['position'],
                self.kortex_baselink_tf['orientation']
            ) = self.listener.lookupTransform(
                self.anchor_frame, 'kortex/base_link', rospy.Time(0)
            )

            (
                self.target_cam_anchor_tf['position'],
                self.target_cam_anchor_tf['orientation']
            ) = self.listener.lookupTransform(
                self.anchor_frame,
                '/target_cam',
                rospy.Time(0),
            )

            (
                self.target_cam_base_tf['position'],
                self.target_cam_base_tf['orientation']
            ) = self.listener.lookupTransform(
                '/base_link',
                '/target_cam',
                rospy.Time(0),
            )

            # Calculate world rotation of base link Kortex frame
            self.kortex_baselink_tf['orientation'
                                   ] = transform.quaternion_multiply(
                                       self.kortex_baselink_tf['orientation'],
                                       self.world_rotation_anchor,
                                   )

            # Transform to Left coordinate system
            self.kortex_baselink_tf['orientation'
                                   ] = transform.quaternion_multiply(
                                       transform.quaternion_about_axis(
                                           np.deg2rad(90),
                                           (
                                               1,
                                               0,
                                               0,
                                           ),
                                       ),
                                       self.kortex_baselink_tf['orientation'],
                                   )

            self.chest_cam_anchor_tf['position'], self.chest_cam_anchor_tf[
                'orientation'] = self.listener.lookupTransform(
                    self.anchor_frame,
                    '/chest_cam',
                    rospy.Time(0),
                )

        except (
            tf.LookupException, tf.ConnectivityException,
            tf.ExtrapolationException
        ):

            self.rate.sleep()

        anchor_to_tool_frame = self.__compose_pose_message(
            self.anchor_tool_frame_tf
        )
        self.__tf_toolframe_anchor.publish(anchor_to_tool_frame)

        anchor_to_kortex_base = self.__compose_pose_message(
            self.kortex_baselink_tf
        )
        self.__tf_baselink_anchor.publish(anchor_to_kortex_base)

        # base_to_toolframe = Pose()
        # base_to_toolframe.position.x = self.trans_tool_base['position'][0]
        # base_to_toolframe.position.y = self.trans_tool_base['position'][1]
        # base_to_toolframe.position.z = self.trans_tool_base['position'][2]
        # base_to_toolframe.orientation.x = self.trans_tool_base['orientation'][0]
        # base_to_toolframe.orientation.y = self.trans_tool_base['orientation'][1]
        # base_to_toolframe.orientation.z = self.trans_tool_base['orientation'][2]
        # base_to_toolframe.orientation.w = self.trans_tool_base['orientation'][3]

        base_to_toolframe = self.__compose_pose_message(self.trans_tool_base)
        self.__tf_baselink_toolframe.publish(base_to_toolframe)

        # target_cam_position = Pose()
        # target_cam_position.position.x = self.target_cam_anchor_tf['position'][0
        #                                                                       ]
        # target_cam_position.position.y = self.target_cam_anchor_tf['position'][1
        #                                                                       ]
        # target_cam_position.position.z = self.target_cam_anchor_tf['position'][2
        #                                                                       ]
        # target_cam_position.orientation.x = self.target_cam_anchor_tf[
        #     'orientation'][0]
        # target_cam_position.orientation.y = self.target_cam_anchor_tf[
        #     'orientation'][1]
        # target_cam_position.orientation.z = self.target_cam_anchor_tf[
        #     'orientation'][2]
        # target_cam_position.orientation.w = self.target_cam_anchor_tf[
        #     'orientation'][3]
        # self.__tf_target_camera_pub.publish(target_cam_position)

        anchor_to_target = self.__compose_pose_message(
            self.target_cam_anchor_tf
        )
        self.__tf_target_camera_pub.publish(anchor_to_target)

        # target_cam_base = Pose()
        # target_cam_base.position.x = self.target_cam_base_tf['position'][0]
        # target_cam_base.position.y = self.target_cam_base_tf['position'][1]
        # target_cam_base.position.z = self.target_cam_base_tf['position'][2]
        # target_cam_base.orientation.x = self.target_cam_base_tf['orientation'][0
        #                                                                       ]
        # target_cam_base.orientation.y = self.target_cam_base_tf['orientation'][1
        #                                                                       ]
        # target_cam_base.orientation.z = self.target_cam_base_tf['orientation'][2
        #                                                                       ]
        # target_cam_base.orientation.w = self.target_cam_base_tf['orientation'][3
        #                                                                       ]

        target_to_base = self.__compose_pose_message(self.target_cam_base_tf)
        self.__tf_target_camera_base_pub.publish(target_to_base)

        # cam_anchor = Pose()
        # cam_anchor.position.x = self.chest_cam_anchor_tf['position'][0]
        # cam_anchor.position.y = self.chest_cam_anchor_tf['position'][1]
        # cam_anchor.position.z = self.chest_cam_anchor_tf['position'][2]
        # cam_anchor.orientation.x = self.chest_cam_anchor_tf['orientation'][0]
        # cam_anchor.orientation.y = self.chest_cam_anchor_tf['orientation'][1]
        # cam_anchor.orientation.z = self.chest_cam_anchor_tf['orientation'][2]
        # cam_anchor.orientation.w = self.chest_cam_anchor_tf['orientation'][3]

        cam_to_anchor = self.__compose_pose_message(self.chest_cam_anchor_tf)
        self.__tf_chest_cam_anchor_tf_pub.publish(cam_to_anchor)

    def __compose_pose_message(self, target_pose):
        """
        target_pose: dict
            'position': np.array([0.0, 0.0, 0.0]),
            'orientation': np.array([1.0, 0.0, 0.0, 0.0])
        
        """

        # NOTE: These two checks might not be needed, check function usage.
        if not isinstance(target_pose, dict):
            raise TypeError('target_pose is not a dictionary.')

        for key in ['position', 'orientation']:
            if key not in target_pose:
                raise KeyError(f'key {key} not found in target_pose.')

        pose_message = Pose()
        pose_message.position.x = target_pose['position'][0]
        pose_message.position.y = target_pose['position'][1]
        pose_message.position.z = target_pose['position'][2]

        pose_message.orientation.w = target_pose['orientation'][0]
        pose_message.orientation.x = target_pose['orientation'][1]
        pose_message.orientation.y = target_pose['orientation'][2]
        pose_message.orientation.z = target_pose['orientation'][3]

        return pose_message


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
