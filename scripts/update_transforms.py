#!/usr/bin/env python
import rospy
import numpy as np
import tf
import transformations as transform
from std_msgs.msg import (
    Float32,
    Float32MultiArray,
)
from geometry_msgs.msg import (Pose, Point)
from gopher_ros_clearcore.msg import (Position)
from Scripts.srv import (ConvertTargetPosition)


class UpdateTransforms:

    def __init__(
        self,
        robot_name,
        anchor_id,
    ):

        # # Private CONSTANTS:
        self.__LISTENER = tf.TransformListener()
        self.__BR = tf.TransformBroadcaster()

        # # Public CONSTANTS:
        self.RATE = rospy.Rate(10)
        self.ROBOT_NAME = robot_name
        self.ANCHOR_ID = anchor_id

        # # Private variables:
        self.__anchor_frame = self.ANCHOR_ID
        self.__chest_position = 0.44

        self.__calibrated_anchor = False
        self.__calibrated_anchor_pos = [0, 0, 0]

        self.__tf_from_camera_to_object = [0, 0, 0]

        self.__tf_from_anchor_to_toolframe = {
            'position': np.array([0.0, 0.0, 0.0]),
            'orientation': np.array([0.0, 0.0, 0.0, 1.0]),
        }

        self.__tf_from_anchor_to_kortexbase = {
            'position': np.array([0.0, 0.0, 0.0]),
            'orientation': np.array([0.0, 0.0, 0.0, 1.0]),
        }

        self.__tf_from_anchor_to_camera = {
            'position': np.array([0.0, 0.0, 0.0]),
            'orientation': np.array([0.0, 0.0, 0.0, 1.0]),
        }

        self.__tf_from_anchor_to_object = {
            'position': np.array([0.0, 0.0, 0.0]),
            'orientation': np.array([0.0, 0.0, 0.0, 1.0]),
        }

        self.__tf_from_mobilebase_to_object = {
            'position': np.array([0.0, 0.0, 0.0]),
            'orientation': np.array([0.0, 0.0, 0.0, 1.0]),
        }

        self.__tf_from_mobilebase_to_toolframe = {
            'position': np.array([0.0, 0.0, 0.0]),
            'orientation': np.array([1.0, 0.0, 0.0, 0.0]),
        }

        # # Public variables:

        # # Service provider:
        rospy.Service(
            '/from_chest_to_anchor',
            ConvertTargetPosition,
            self.__convert_target_position,
        )

        # # Topic publisher:
        self.__anchor_to_toolframe_pub = rospy.Publisher(
            f'/{self.ROBOT_NAME}/tf_toolframe',
            Point,
            queue_size=1,
        )

        self.__anchor_to_kortexbase_pub = rospy.Publisher(
            f'/{self.ROBOT_NAME}/tf_baselink',
            Pose,
            queue_size=1,
        )

        self.__mobilebase_to_toolframe_pub = rospy.Publisher(
            f'/{self.ROBOT_NAME}/tf_base_tool_frame',
            Pose,
            queue_size=1,
        )

        self.__camera_to_object_pub = rospy.Publisher(
            f'/{self.ROBOT_NAME}/target_hologram',
            Pose,
            queue_size=1,
        )

        self.__base_to_object_pub = rospy.Publisher(
            f'/{self.ROBOT_NAME}/tf_base_target_cam',
            Pose,
            queue_size=1,
        )

        self.__anchor_to_camera_pub = rospy.Publisher(
            f'/{self.ROBOT_NAME}/tf_chest_cam_anchor',
            Pose,
            queue_size=1,
        )

        # # Topic subscriber:
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
            'chest_logger/current_position',
            Float32,
            self.__update_chest_callback,
        )

    def __convert_target_position(self, request):

        transformation_matrix = transform.quaternion_matrix(
            [
                self.__tf_from_anchor_to_camera['orientation'][3],
                self.__tf_from_anchor_to_camera['orientation'][0],
                self.__tf_from_anchor_to_camera['orientation'][1],
                self.__tf_from_anchor_to_camera['orientation'][2]
            ]
        )
        transformation_matrix[:3,
                              3] = self.__tf_from_anchor_to_camera['position']

        transformation_matrix_target = transform.quaternion_matrix([1, 0, 0, 0])
        transformation_matrix_target[:3, 3] = request.fromchest.data

        anchor_to_target = np.dot(
            transformation_matrix, transformation_matrix_target
        )

        translation = Float32MultiArray()
        translation.data = anchor_to_target[:3, 3]

        return translation

    def __camera_target_callback(self, message):

        self.__tf_from_camera_to_object[0] = message.x
        self.__tf_from_camera_to_object[1] = message.y
        self.__tf_from_camera_to_object[2] = message.z

    def __update_chest_callback(self, message):

        self.__chest_position = (message.data)

    def __calibration_anchor_callback(self, message):

        self.__calibrated_anchor_pos[0] += message.x
        self.__calibrated_anchor_pos[1] += message.y
        self.__calibrated_anchor_pos[2] += -message.z

        self.__anchor_frame = 'new_anchor'
        self.__calibrated_anchor = True

    def __send_transformations(self):

        if self.__calibrated_anchor:

            self.__BR.sendTransform(
                self.__calibrated_anchor_pos,
                (0, 0, 0, 1),
                rospy.Time.now(),
                self.__anchor_frame,
                self.ANCHOR_ID,
            )

        self.__BR.sendTransform(
            (0.02, -0.14, 0.93 + self.__chest_position),
            (0.2887332, 0.2887332, -0.6454712, 0.6454712),
            rospy.Time.now(),
            'kortex/base_link',
            'base_link',
        )

        self.__BR.sendTransform(
            (0.0, 0.05, 0.855 + self.__chest_position),
            (-0.5, 0.5, -0.5, 0.5),
            rospy.Time.now(),
            '/chest_cam',
            '/base_link',
        )

        self.__BR.sendTransform(
            self.__tf_from_camera_to_object,
            (0, 0, 0, 1),
            rospy.Time.now(),
            '/target_cam',
            self.__anchor_frame,
        )

    def __listen_to_transformations(self):

        try:

            (
                self.__tf_from_anchor_to_toolframe['position'],
                self.__tf_from_anchor_to_toolframe['orientation']
            ) = self.__LISTENER.lookupTransform(
                self.__anchor_frame,
                'kortex/tool_frame',
                rospy.Time(0),
            )

            (
                self.__tf_from_mobilebase_to_toolframe['position'],
                self.__tf_from_mobilebase_to_toolframe['orientation']
            ) = self.__LISTENER.lookupTransform(
                '/base_link', 'kortex/tool_frame', rospy.Time(0)
            )

            (
                self.__tf_from_anchor_to_kortexbase['position'],
                self.__tf_from_anchor_to_kortexbase['orientation']
            ) = self.__LISTENER.lookupTransform(
                self.__anchor_frame, 'kortex/base_link', rospy.Time(0)
            )

            (
                self.__tf_from_anchor_to_object['position'],
                self.__tf_from_anchor_to_object['orientation']
            ) = self.__LISTENER.lookupTransform(
                self.__anchor_frame,
                '/target_cam',
                rospy.Time(0),
            )

            self.__tf_from_anchor_to_camera[
                'position'], self.__tf_from_anchor_to_camera[
                    'orientation'] = self.__LISTENER.lookupTransform(
                        self.__anchor_frame,
                        '/chest_cam',
                        rospy.Time(0),
                    )

            (
                self.__tf_from_mobilebase_to_object['position'],
                self.__tf_from_mobilebase_to_object['orientation']
            ) = self.__LISTENER.lookupTransform(
                '/base_link',
                '/target_cam',
                rospy.Time(0),
            )

            # Calculate world rotation of base link Kortex frame
            self.__tf_from_anchor_to_kortexbase[
                'orientation'] = transform.quaternion_multiply(
                    self.__tf_from_anchor_to_kortexbase['orientation'],
                    [0, 0, 0, 1],
                )

            # Transform to Left coordinate system
            self.__tf_from_anchor_to_kortexbase[
                'orientation'] = transform.quaternion_multiply(
                    transform.quaternion_about_axis(
                        np.deg2rad(90),
                        (
                            1,
                            0,
                            0,
                        ),
                    ),
                    self.__tf_from_anchor_to_kortexbase['orientation'],
                )

        except (
            tf.LookupException,
            tf.ConnectivityException,
            tf.ExtrapolationException,
        ):

            self.RATE.sleep()

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

    def main_loop(self):

        self.__send_transformations()
        self.__listen_to_transformations()

        anchor_to_tool_frame = Point()
        anchor_to_tool_frame.x = self.__tf_from_anchor_to_toolframe['position'][
            0]
        anchor_to_tool_frame.y = self.__tf_from_anchor_to_toolframe['position'][
            1]
        anchor_to_tool_frame.z = self.__tf_from_anchor_to_toolframe['position'][
            2]
        self.__anchor_to_toolframe_pub.publish(anchor_to_tool_frame)

        anchor_to_kortexbase = self.__compose_pose_message(
            self.__tf_from_anchor_to_kortexbase
        )
        self.__anchor_to_kortexbase_pub.publish(anchor_to_kortexbase)

        base_to_toolframe = self.__compose_pose_message(
            self.__tf_from_mobilebase_to_toolframe
        )
        self.__mobilebase_to_toolframe_pub.publish(base_to_toolframe)

        anchor_to_target = self.__compose_pose_message(
            self.__tf_from_anchor_to_object
        )
        self.__camera_to_object_pub.publish(anchor_to_target)

        base_to_target = self.__compose_pose_message(
            self.__tf_from_mobilebase_to_object
        )
        self.__base_to_object_pub.publish(base_to_target)

        cam_to_anchor = self.__compose_pose_message(
            self.__tf_from_anchor_to_camera
        )
        self.__anchor_to_camera_pub.publish(cam_to_anchor)


def main():
    """
    """

    rospy.init_node(
        'pose_updates',
        log_level=rospy.INFO,
    )

    rospy.loginfo('\n\n\n\n\n')

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
        transformations.RATE.sleep()


if __name__ == '__main__':
    main()
