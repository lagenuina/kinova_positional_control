#!/usr/bin/env python
import rospy
import numpy as np
import tf
import transformations as transform
import roslaunch
import math

from std_msgs.msg import (Bool, Float32MultiArray)
from geometry_msgs.msg import (Pose, Point)

from kortex_driver.msg import (
    JointAngles,
)

from relaxed_ik_ros1.msg import (EEPoseGoals)


class UpdateRoboHologram:

    def __init__(self):

        self.listener = tf.TransformListener()
        self.br = tf.TransformBroadcaster()
        self.rate = rospy.Rate(10)

        self.__goal_absolute_positions = [0, 0, 0, 0, 0, 0, 0]

        # Rotation matrices from Global Coordinate System (parallel to the
        # floor, X facing forward globally) to Relaxed IK Coordinate System
        # (parallel to Kinova Arm base, X facing according to Kinova Arm) and
        # back. If the arm is mounted on the table, no rotations will be
        # applied.
        # Rotation matrices around axes. If the arm is mounted on the table,
        # mounting angles should all be zeros.
        self.ROTATE_X = transform.rotation_matrix(
            math.radians(0.0),
            (1, 0, 0),
        )
        self.ROTATE_Y = transform.rotation_matrix(
            math.radians(-48.2),
            (0, 1, 0),
        )
        self.ROTATE_Z = transform.rotation_matrix(
            math.radians(90.0),
            (0, 0, 1),
        )

        self.ROTATE_GCS_TO_RIKCS = transform.concatenate_matrices(
            self.ROTATE_X,
            self.ROTATE_Y,
            self.ROTATE_Z,
        )
        self.ROTATE_RIKCS_TO_GCS = transform.inverse_matrix(
            self.ROTATE_GCS_TO_RIKCS
        )

        self.input_pose = {
            'position': np.array([0.0, 0.0, 0.0]),
            'orientation': np.array([1.0, 0.0, 0.0, 0.0]),
        }

        self.target = {
            'position': np.array([-0.12, 0.25, -0.36]),
            'orientation': np.array([-0.288733, -0.288733, 0.64547, 0.64547]),
        }

        rospy.Subscriber(
            '/holorobot/relaxed_ik/joint_angle_solutions',
            JointAngles,
            self.__absolute_setpoint_callback,
        )

        rospy.Subscriber(
            '/holorobot/positional_control/input_pose',
            Pose,
            self.__input_pose_callback,
        )

        # # Topic publisher:
        self.__relaxed_ik_target_rikcs = rospy.Publisher(
            '/holorobot/relaxed_ik/ee_pose_goals',
            EEPoseGoals,
            queue_size=1,
        )

        # # Topic publisher:
        self.__joints_state_hologram = rospy.Publisher(
            '/my_gen3/joints',
            Float32MultiArray,
            queue_size=1,
        )

        ee_pose_goals = self.compose_pose_message(self.target)
        self.__relaxed_ik_target_rikcs.publish(ee_pose_goals)

    def __input_pose_callback(self, msg):
        """
        
        """

        self.input_pose['position'][0] = msg.position.x
        self.input_pose['position'][1] = msg.position.y
        self.input_pose['position'][2] = msg.position.z

        self.input_pose['orientation'][0] = msg.orientation.w
        self.input_pose['orientation'][1] = msg.orientation.x
        self.input_pose['orientation'][2] = msg.orientation.y
        self.input_pose['orientation'][3] = msg.orientation.z

        self.target['position'] = np.matmul(
            self.ROTATE_GCS_TO_RIKCS[0:3, 0:3], self.input_pose['position']
        )

        self.target['orientation'] = (
            transform.quaternion_multiply(
                transform.quaternion_from_matrix(self.ROTATE_GCS_TO_RIKCS),
                self.input_pose['orientation'],
            ),
        )[0]

    def __absolute_setpoint_callback(self, msg):
        """
        
        """

        for joint_index in range(7):
            self.__goal_absolute_positions[joint_index] = (
                msg.joint_angles[joint_index].value
            )

    def compose_pose_message(self, target):

        pose_message = Pose()
        pose_message.position.x = target['position'][0]
        pose_message.position.y = target['position'][1]
        pose_message.position.z = target['position'][2]

        pose_message.orientation.w = target['orientation'][0]
        pose_message.orientation.x = target['orientation'][1]
        pose_message.orientation.y = target['orientation'][2]
        pose_message.orientation.z = target['orientation'][3]

        ee_pose_goals = EEPoseGoals()
        ee_pose_goals.ee_poses.append(pose_message)
        ee_pose_goals.ee_poses.append(pose_message)
        ee_pose_goals.header.seq = 0

        return ee_pose_goals

    def main_loop(self):

        ee_pose_goals = self.compose_pose_message(self.target)

        self.__relaxed_ik_target_rikcs.publish(ee_pose_goals)

        joints_target = Float32MultiArray()
        joints_target.data = self.__goal_absolute_positions

        self.__joints_state_hologram.publish(joints_target)


if __name__ == '__main__':

    rospy.init_node('robot_hologram',)

    transformations = UpdateRoboHologram()

    while not rospy.is_shutdown():
        transformations.main_loop()
        transformations.rate.sleep()