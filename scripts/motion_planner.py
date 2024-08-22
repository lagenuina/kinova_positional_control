#!/usr/bin/env python
"""Implements teleoperation module for positional control.

TODO: Add detailed description.

Author (s):
    1. Lorena Genua (lorena.genua@gmail.com), Human-Inspired Robotics (HiRo)
       lab, Worcester Polytechnic Institute (WPI), 2023.

"""

import rospy
import numpy as np
import time
import transformations
from ast import (literal_eval)
from std_msgs.msg import (Bool, Float32, Int32)
from std_srvs.srv import (Empty, SetBool)
from geometry_msgs.msg import (Pose)
from kortex_driver.srv import (ApplyEmergencyStop)


class MotionPlanner:
    """
    
    """

    def __init__(
        self,
        robot_name,
        compensate_orientation,
        maximum_input_change,
        convenience_compensation,
    ):
        """
        
        """
        # # Private constants:

        # # Public constants:
        self.ROBOT_NAME = robot_name
        self.COMPENSATE_ORIENTATION = compensate_orientation
        self.MAXIMUM_INPUT_CHANGE = maximum_input_change
        self.CONVENIENCE_COMPENSATION = convenience_compensation
        self.RATE = rospy.Rate(100)

        # # Private variables:
        self.__input_pose = {
            'position': np.array([0.0, 0.0, 0.0]),
            'orientation': np.array([1.0, 0.0, 0.0, 0.0]),
        }

        self.__target_pose = {
            'position': np.array([0.0, 0.0, 0.0]),
            'orientation': np.array([1.0, 0.0, 0.0, 0.0]),
        }

        self.__last_tool_frame = {
            'position': np.array([0.0, 0.0, 0.0]),
            'orientation': np.array([1.0, 0.0, 0.0, 0.0]),
        }

        self.__last_norm_value = 0
        self.__norm_value_stable_since = None
        self.__move_to = 1
        self.__state = 0
        self.__waypoint_index = 0
        self.__waypoints = []
        self.__target_has_changed = True
        self.__last_pose_tracking = False
        self.__pose_tracking = False
        self.__enable_compensation = False
        self.__shut_down = False
        self.__state_updated = False
        self.__chest_position = 0.44
        self.__last_chest_position = 0.44

        # # Public variables:
        # Last commanded Relaxed IK pose is required to compensate controller
        # input.
        self.last_relaxed_ik_pose = {
            'position': np.array([0.0, 0.0, 0.0]),
            'orientation': np.array([1.0, 0.0, 0.0, 0.0]),
        }

        # This difference is calculated each time the tracking is started and
        # subracted from future inputs during current tracking to compensate for
        # linear misalignment between the global and relaxed_ik coordinate
        # systems.
        self.input_relaxed_ik_difference = {
            'position': np.array([0.0, 0.0, 0.0]),
            'orientation': np.array([1.0, 0.0, 0.0, 0.0]),
        }

        self.__tool_frame = {
            'position': np.array([0.0, 0.0, 0.0]),
            'orientation': np.array([1.0, 0.0, 0.0, 0.0]),
        }

        # # Initialization and dependency status topics:
        self.__is_initialized = False
        self.__dependency_initialized = False

        self.__node_is_initialized = rospy.Publisher(
            f'/{self.ROBOT_NAME}/motion_planner/is_initialized',
            Bool,
            queue_size=1,
        )

        self.__dependency_status = {
            'positional_control': False,
            'gripper_control': False,
        }

        self.__dependency_status_topics = {
            'positional_control':
                rospy.Subscriber(
                    f'/{self.ROBOT_NAME}/positional_control/is_initialized',
                    Bool,
                    self.__positional_control_callback,
                ),
            'gripper_control':
                rospy.Subscriber(
                    f'/{self.ROBOT_NAME}/gripper_control/is_initialized',
                    Bool,
                    self.__gripper_control_callback,
                ),
        }

        # # Service provider:
        rospy.Service(
            f'/{self.ROBOT_NAME}/pose_tracking',
            SetBool,
            self.__tracking_state,
        )

        # # Service subscriber:
        self.__estop_arm_srv = rospy.ServiceProxy(
            f'/{self.ROBOT_NAME}/base/apply_emergency_stop',
            ApplyEmergencyStop,
        )
        self.__update_task_state = rospy.ServiceProxy(
            f'/{self.ROBOT_NAME}/robot_control/update_state',
            Empty,
        )

        # # Topic publisher:
        self.__node_is_initialized = rospy.Publisher(
            f'/{self.ROBOT_NAME}/motion_planner/is_initialized',
            Bool,
            queue_size=1,
        )
        self.__kinova_pose = rospy.Publisher(
            f'/{self.ROBOT_NAME}/positional_control/input_pose',
            Pose,
            queue_size=1,
        )
        self.__holorobot_pose = rospy.Publisher(
            '/holorobot/positional_control/input_pose',
            Pose,
            queue_size=1,
        )

        # # Topic subscriber:
        rospy.Subscriber(
            f'/{self.ROBOT_NAME}/robot_control/target_pose',
            Pose,
            self.__target_pose_callback,
        )
        rospy.Subscriber(
            f'/{self.ROBOT_NAME}/relaxed_ik/commanded_pose_gcs',
            Pose,
            self.__commanded_pose_callback,
        )
        rospy.Subscriber(
            f'/{self.ROBOT_NAME}/tf_base_tool_frame',
            Pose,
            self.__toolframe_transform_callback,
        )
        rospy.Subscriber(
            '/chest_logger/current_velocity',
            Float32,
            self.__chest_velocity_callback,
        )
        rospy.Subscriber(
            '/chest_logger/current_position',
            Float32,
            self.__chest_position_callback,
        )
        rospy.Subscriber(
            f'/{self.ROBOT_NAME}/robot_control/current_task_state',
            Int32,
            self.__current_task_state_callback,
        )

    # # Dependency status callbacks:
    def __positional_control_callback(self, message):
        """Monitors positional_control is_initialized topic.
        
        """

        self.__dependency_status['positional_control'] = message.data

    def __gripper_control_callback(self, message):
        """Monitors gripper_control is_initialized topic.
        
        """

        self.__dependency_status['gripper_control'] = message.data

    # # Topic callbacks:
    def __tracking_state(self, request):

        self.__pose_tracking = request.data

        return [True, ""]

    def __toolframe_transform_callback(self, message):

        self.__tool_frame['position'][0] = message.position.x
        self.__tool_frame['position'][1] = message.position.y
        self.__tool_frame['position'][2] = message.position.z

    def __chest_position_callback(self, message):
        """

        """

        self.__chest_position = message.data

    def __chest_velocity_callback(self, message):
        """

        """

        self.__chest_velocity = message.data

        if self.__chest_velocity != 0.0:
            if not self.__enable_compensation:
                self.__last_tool_frame['position'] = self.__tool_frame[
                    'position'].copy()

                self.__last_chest_position = self.__chest_position

                self.__enable_compensation = True

        else:
            if self.__enable_compensation:

                self.__target_has_changed = True
                # self.__calculate_compensation()

                self.__enable_compensation = False

    def __current_task_state_callback(self, message):

        self.__state = message.data

    def __target_pose_callback(self, message):
        """
        
        """

        self.__input_pose['position'][0] = message.position.x
        self.__input_pose['position'][1] = message.position.y
        self.__input_pose['position'][2] = message.position.z

        self.__input_pose['orientation'][0] = message.orientation.w
        self.__input_pose['orientation'][1] = message.orientation.x
        self.__input_pose['orientation'][2] = message.orientation.y
        self.__input_pose['orientation'][3] = message.orientation.z

        position_difference = np.linalg.norm(
            self.__input_pose['position'] - self.__target_pose['position']
        )

        if position_difference > 0.10:

            self.__target_pose['position'] = self.__input_pose['position'].copy(
            )
            self.__target_pose['orientation'] = self.__input_pose['orientation'
                                                                 ].copy()

            self.__target_has_changed = True
            self.__state_updated = False

    def __commanded_pose_callback(self, message):
        """
        
        """

        self.last_relaxed_ik_pose['position'][0] = message.position.x
        self.last_relaxed_ik_pose['position'][1] = message.position.y
        self.last_relaxed_ik_pose['position'][2] = message.position.z

        self.last_relaxed_ik_pose['orientation'][0] = message.orientation.w
        self.last_relaxed_ik_pose['orientation'][1] = message.orientation.x
        self.last_relaxed_ik_pose['orientation'][2] = message.orientation.y
        self.last_relaxed_ik_pose['orientation'][3] = message.orientation.z

    # # Private methods:
    def __check_initialization(self):
        """Monitors required criteria and sets is_initialized variable.

        Monitors nodes' dependency status by checking if dependency's
        is_initialized topic has at most one publisher (this ensures that
        dependency node is alive and does not have any duplicates) and that it
        publishes True. If dependency's status was True, but get_num_connections
        is not equal to 1, this means that the connection is lost and emergency
        actions should be performed.

        Once all dependencies are initialized and additional criteria met, the
        nodes is_initialized status changes to True. This status can change to
        False any time to False if some criteria are no longer met.
        
        """

        self.__dependency_initialized = True

        for key in self.__dependency_status:
            if self.__dependency_status_topics[key].get_num_connections() != 1:
                if self.__dependency_status[key]:
                    rospy.logerr(
                        (
                            f'/{self.ROBOT_NAME}/motion_planner: '
                            f'lost connection to {key}!'
                        )
                    )

                    # # Emergency actions on lost connection:
                    # NOTE (optionally): Add code, which needs to be executed if
                    # connection to any of dependencies was lost.

                self.__dependency_status[key] = False

            if not self.__dependency_status[key]:
                self.__dependency_initialized = False

        if not self.__dependency_initialized:
            waiting_for = ''
            for key in self.__dependency_status:
                if not self.__dependency_status[key]:
                    waiting_for += f'\n- waiting for {key}...'

            rospy.logwarn_throttle(
                15,
                (
                    f'/{self.ROBOT_NAME}/motion_planner:'
                    f'{waiting_for}'
                    # f'\nMake sure those dependencies are running properly!'
                ),
            )

        # NOTE: Add more initialization criterea if needed.
        if (self.__dependency_initialized):
            if not self.__is_initialized:
                rospy.loginfo(
                    f'\033[92m/{self.ROBOT_NAME}/motion_planner: ready.\033[0m',
                )

                self.__is_initialized = True

        else:
            if self.__is_initialized:
                # NOTE (optionally): Add code, which needs to be executed if the
                # nodes's status changes from True to False.
                pass

            self.__is_initialized = False

        self.__node_is_initialized.publish(self.__is_initialized)

    def __calculate_compensation(self):
        """Calculates the compensation for coordinate systems misalignment.
        
        """

        self.input_relaxed_ik_difference['position'] = (
            self.__tool_frame['position']
            - self.last_relaxed_ik_pose['position']
        )

        self.input_relaxed_ik_difference['orientation'] = (
            transformations.quaternion_multiply(
                transformations.quaternion_inverse(
                    self.__input_pose['orientation']
                ),
                self.last_relaxed_ik_pose['orientation'],
            )
        )

        self.__last_chest_position = self.__chest_position

    def __has_reached_waypoint(self, waypoints, type):

        if type == "target":
            norm = np.linalg.norm(
                self.__tool_frame['position'] - self.__target_pose['position']
            )
        else:
            norm = np.linalg.norm(
                self.__tool_frame['position'] - waypoints[self.__waypoint_index]
            )

        if self.__check_if_stuck(norm) or norm < 0.03:
            return True
        else:
            return False

    def __check_if_stuck(self, current_norm_value):

        has_arm_stopped = False

        if current_norm_value < 0.15 and abs(
            self.__last_norm_value - current_norm_value
        ) < 0.01:

            if self.__norm_value_stable_since is None:
                self.__norm_value_stable_since = time.time()

            elif time.time() - self.__norm_value_stable_since >= 2:
                # If the value has been stable for 3 seconds, switch to the next state
                has_arm_stopped = True

        else:
            self.__norm_value_stable_since = None

        self.__last_norm_value = current_norm_value

        return has_arm_stopped

    def __move_to_next_waypoint(self, waypoints):

        if self.__waypoint_index < len(waypoints):

            if self.__has_reached_waypoint(waypoints, "waypoint"):

                waypoint = waypoints[self.__waypoint_index]

                self.__command_pose(waypoint)
                self.__waypoint_index += 1

        elif self.__waypoint_index == len(waypoints):

            if self.__has_reached_waypoint(
                waypoints, "target"
            ) and not self.__state_updated:

                rospy.sleep(0.3)
                self.__update_task_state()
                self.__state_updated = True

    def __command_pose(self, desired_pose):

        compensated_input_pose = {
            'position': np.array([0.0, 0.0, 0.0]),
            'orientation': np.array([1.0, 0.0, 0.0, 0.0]),
        }

        compensated_input_pose['position'] = (
            desired_pose - self.input_relaxed_ik_difference['position']
        )

        # Use fixed (last commanded) orientation.
        compensated_input_pose['orientation'] = (
            self.last_relaxed_ik_pose['orientation']
        )

        pose_message = Pose()
        pose_message.position.x = compensated_input_pose['position'][0]
        pose_message.position.y = compensated_input_pose['position'][1]
        pose_message.position.z = compensated_input_pose['position'][2]

        pose_message.orientation.w = compensated_input_pose['orientation'][0]
        pose_message.orientation.x = compensated_input_pose['orientation'][1]
        pose_message.orientation.y = compensated_input_pose['orientation'][2]
        pose_message.orientation.z = compensated_input_pose['orientation'][3]

        self.__kinova_pose.publish(pose_message)

    # # Public methods:
    def main_loop(self):
        """
        
        """
        self.__check_initialization()

        if self.__shut_down:

            rospy.signal_shutdown("Service was called.")

            rospy.loginfo_once("\n E-Stop! Robot control node has shutdown.\n")

            self.__shut_down = False

        if not self.__is_initialized:
            return

        if (self.__last_pose_tracking != self.__pose_tracking):
            self.__calculate_compensation()

        self.__last_pose_tracking = self.__pose_tracking

        if self.__target_has_changed:

            self.__calculate_compensation()

            self.__waypoints = self.generate_waypoints(
                self.__tool_frame['position'],
                self.__target_pose['position'],
                max_distance=0.1,
                waypoints_number=45
            )

            # Reset waypoint index and target flag
            self.__waypoint_index = 0
            self.__target_has_changed = False

        if self.__pose_tracking:

            if self.__enable_compensation:
                chest_delta = self.__last_chest_position - self.__chest_position

                compensated_pose = self.__last_tool_frame['position'].copy()
                compensated_pose[2] += chest_delta

                self.__command_pose(compensated_pose)

            else:
                self.__move_to_next_waypoint(self.__waypoints)

    def get_axis_order(self):

        if self.__state == 1 or self.__state == 3:
            return [2, 1, 0]
        else:
            if self.__move_to == 2:
                return [1, 2, 0]
            else:
                return [0, 1, 2]

    def generate_waypoints(
        self,
        current_pose,
        target_pose,
        max_distance=0.05,
        waypoints_number=30
    ):

        # Check if the distance between current and target poses is greater than max_distance
        distance = np.sqrt(
            (current_pose[0] - target_pose[0])**2
            + (current_pose[1] - target_pose[1])**2
            + (current_pose[2] - target_pose[2])**2
        )

        waypoints = []

        if distance > max_distance:

            # Determine the number of waypoints based on the desired resolution
            num_waypoints = int(
                (distance / max_distance) * waypoints_number
            ) + 1

            axis_order = self.get_axis_order()

            for axis in axis_order:

                waypoints.extend(
                    self.generate_axis_waypoints(
                        current_pose, target_pose, num_waypoints, axis
                    )
                )

                if waypoints:
                    current_pose = waypoints[-1]

            return waypoints

        else:

            waypoints.append(target_pose)
            return waypoints

    def generate_axis_waypoints(
        self, current_pose, target_pose, num_waypoints, axis
    ):
        waypoints = []

        for i in range(1, num_waypoints + 1):
            ratio = i / (num_waypoints + 1)
            intermediate_pose = current_pose.copy()
            intermediate_pose[
                axis] += ratio * (target_pose[axis] - current_pose[axis])
            waypoints.append(intermediate_pose)

        return waypoints

    def node_shutdown(self):
        """
        
        """

        rospy.loginfo_once(
            f'/{self.ROBOT_NAME}/motion planner node is shutting down...',
        )

        # TODO: Stop the arm motion.
        # self.__stop_arm()

        rospy.loginfo_once(
            f'/{self.ROBOT_NAME}/motion planner: node has shut down.',
        )


def main():
    """

    """

    rospy.init_node(
        'motion_planner',
        log_level=rospy.INFO,  # TODO: Make this a launch file parameter.
    )

    rospy.loginfo('\n\n\n\n\n')  # Add whitespaces to separate logs.

    # # ROS parameters:

    kinova_name = rospy.get_param(
        param_name=f'{rospy.get_name()}/robot_name',
        default='my_gen3',
    )

    compensate_orientation = rospy.get_param(
        param_name=f'{rospy.get_name()}/compensate_orientation',
        default=True,
    )

    maximum_input_change = rospy.get_param(
        param_name=f'{rospy.get_name()}/maximum_input_change',
        default=0.1,
    )

    convenience_compensation = literal_eval(
        rospy.get_param(
            param_name=f'{rospy.get_name()}/convenience_compensation',
            default='[0.0, 0.0, 0.0]',
        )
    )

    motion_planner = MotionPlanner(
        robot_name=kinova_name,
        compensate_orientation=compensate_orientation,
        maximum_input_change=maximum_input_change,
        convenience_compensation=convenience_compensation,
    )

    rospy.on_shutdown(motion_planner.node_shutdown)

    while not rospy.is_shutdown():

        motion_planner.main_loop()
        motion_planner.RATE.sleep()


if __name__ == '__main__':
    main()
