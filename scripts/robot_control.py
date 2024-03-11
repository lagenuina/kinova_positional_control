#!/usr/bin/env python
"""Implements teleoperation module for positional control.

TODO: Add detailed description.

Author (s):
    1. Lorena Genua (lorena.genua@gmail.com), Human-Inspired Robotics (HiRo)
       lab, Worcester Polytechnic Institute (WPI), 2023.
    2. Nikita Boguslavskii (bognik3@gmail.com), Human-Inspired Robotics (HiRo)
       lab, Worcester Polytechnic Institute (WPI), 2023.

"""

import rospy
import numpy as np
import time
import transformations
from ast import (literal_eval)
from std_msgs.msg import (Bool, Int32)
from std_srvs.srv import (Empty, SetBool)
from geometry_msgs.msg import (Pose)
from kortex_driver.srv import (ApplyEmergencyStop, Base_ClearFaults)
from kinova_positional_control.srv import (GripperForceGrasping, GripperPosition)
from Scripts.srv import (UpdateState, UpdateChest)


class KinovaTeleoperation:
    """
    
    """

    def __init__(
        self,
        robot_name,
        tracking_mode,
        compensate_orientation,
        maximum_input_change,
        convenience_compensation,
    ):
        """
        
        """

        if tracking_mode not in ['hold', 'toggle']:
            raise ValueError(
                'tracking_mode should be either "hold" or "press".'
            )

        # # Private constants:

        # # Public constants:
        self.ROBOT_NAME = robot_name
        self.TRACKING_MODE = tracking_mode
        self.COMPENSATE_ORIENTATION = compensate_orientation
        self.MAXIMUM_INPUT_CHANGE = maximum_input_change
        self.CONVENIENCE_COMPENSATION = convenience_compensation
        self.RATE = rospy.Rate(5)

        # # Private variables:
        self.__input_pose = {
            'position': np.array([0.0, 0.0, 0.0]),
            'orientation': np.array([1.0, 0.0, 0.0, 0.0]),
        }

        self.__tray_pose = {
            'position': np.array([0.0, 0.0, 0.0]),
            'orientation': np.array([1.0, 0.0, 0.0, 0.0]),
        }

        self.__target_grasped = {
            'position': np.array([0.0, 0.0, 0.0]),
            'orientation': np.array([1.0, 0.0, 0.0, 0.0]),
        }

        self.__target_moved = {
            'position': np.array([0.0, 0.0, 0.0]),
            'orientation': np.array([1.0, 0.0, 0.0, 0.0]),
        }

        self.__counter = None
        self.__last_norm_value = None
        self.__norm_value_stable_since = None
        self.__move_to = 1
        self.__chest_position = 440.0
        self.__state = 0
        self.__previous_state = 0

        self.__has_grasped = False
        self.__on_startup = True
        self.__last_pose_tracking = False
        self.__pose_tracking = False
        self.__is_remote_controlling = False
        self.__compensate_depth = False
        self.__compensate_height = False
        self.__move_medicine_bool = False
        self.__shut_down = False
        self.__chest_adjusted = False
        self.__new_target_received = False
        self.__rh_help = False

        self.__control_mode = 'position'

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

        self.tool_frame_position = [0, 0, 0]
        self.target_relaxed_ik = [0, 0, 0]

        # # Initialization and dependency status topics:
        self.__is_initialized = False
        self.__dependency_initialized = False

        self.__node_is_initialized = rospy.Publisher(
            f'/{self.ROBOT_NAME}/teleoperation/is_initialized',
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
            '/change_task_state_service',
            UpdateState,
            self.__change_state,
        )
        rospy.Service(
            '/update_task_state',
            Empty,
            self.__update_state,
        )
        rospy.Service(
            '/move_medicine',
            UpdateState,
            self.__move_medicine,
        )
        rospy.Service(
            f'/{self.ROBOT_NAME}/stop_task',
            Empty,
            self.__stop_service,
        )
        rospy.Service(
            '/remote_handling',
            SetBool,
            self.__remote_control,
        )
        rospy.Service(
            f'/{self.ROBOT_NAME}/robot_control/shut_down',
            Empty,
            self.__shutdown_node_service,
        )

        # # Service subscriber:
        self.__gripper_force_grasping = rospy.ServiceProxy(
            f'/{self.ROBOT_NAME}/gripper/force_grasping',
            GripperForceGrasping,
        )
        self.__gripper_position = rospy.ServiceProxy(
            f'/{self.ROBOT_NAME}/gripper/position',
            GripperPosition,
        )
        self.__update_target_service = rospy.ServiceProxy(
            '/update_target',
            SetBool,
        )
        self.__update_chest_service = rospy.ServiceProxy(
            '/update_chest',
            UpdateChest,
        )
        self.__estop_arm_srv = rospy.ServiceProxy(
            f'/{self.ROBOT_NAME}/base/apply_emergency_stop',
            ApplyEmergencyStop,
        )

        # # Topic publisher:
        self.__node_is_initialized = rospy.Publisher(
            f'/{self.ROBOT_NAME}/teleoperation/is_initialized',
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
        self.__robot_pick_and_place = rospy.Publisher(
            f'/{self.ROBOT_NAME}/pick_and_place',
            Int32,
            queue_size=1,
        )

        # # Topic subscriber:
        rospy.Subscriber(
            f'/{self.ROBOT_NAME}/tf_base_target_cam',
            Pose,
            self.__input_pose_callback,
        )
        rospy.Subscriber(
            '/target_counter',
            Int32,
            self.__target_counter_callback,
        )
        rospy.Subscriber(
            f'/{self.ROBOT_NAME}/grasping',
            Int32,
            self.__grasping_feedback_callback,
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
    def __grasping_feedback_callback(self, message):

        self.__has_grasped = message.data

    def __target_counter_callback(self, message):

        self.__counter = message.data

    def __input_pose_callback(self, message):
        """

        """

        if self.__pose_tracking:

            if self.__state == 1:

                self.__input_pose['position'][0] = message.position.x + 0.02
                self.__input_pose['position'][1] = message.position.y
                self.__input_pose['position'][2] = message.position.z + 0.02

                self.__input_pose['orientation'][0] = message.orientation.w
                self.__input_pose['orientation'][1] = message.orientation.x
                self.__input_pose['orientation'][2] = message.orientation.y
                self.__input_pose['orientation'][3] = message.orientation.z

                self.__target_grasped['position'] = self.__input_pose['position'
                                                                     ].copy()

                if self.__compensate_depth:
                    self.__input_pose['position'][0] += 0.05

                if self.__chest_position == 440:
                    self.__input_pose['position'][2] -= 0.02

            elif self.__state == 2 and self.__has_grasped == 1 and self.__is_remote_controlling:

                self.__input_pose['position'][
                    0] = self.__target_grasped['position'][0] - 0.2
                self.__input_pose['position'][1] = self.__target_grasped[
                    'position'][1]
                self.__input_pose['position'][
                    2] = self.__target_grasped['position'][2] + 0.03

            elif self.__state == 3:

                if self.__move_medicine_bool and self.__move_to in [2, 3]:

                    if self.__move_to == 3:
                        self.__input_pose['position'][0] = self.__tray_pose[
                            'position'][0]
                        self.__input_pose['position'][
                            1] = message.position.y - 0.08
                        self.__input_pose['position'][
                            2] = self.__tray_pose['position'][2] - 0.12

                    elif self.__move_to == 2:
                        self.__input_pose['position'][
                            0] = message.position.x - 0.08
                        self.__input_pose['position'][1] = message.position.y
                        self.__input_pose['position'][
                            2] = message.position.z + 0.05

                        self.__target_moved['position'] = self.__input_pose[
                            'position'].copy()

                else:
                    if self.__counter is not None:

                        if self.__counter == 0 or self.__on_startup:

                            self.__tray_pose['position'][
                                0] = message.position.x - 0.15
                            self.__tray_pose['position'][
                                1] = message.position.y - 0.35
                            self.__tray_pose['position'][
                                2] = message.position.z - 0.15

                            self.__input_pose['position'] = self.__tray_pose[
                                'position'].copy()

                            self.__on_startup = False

                        else:
                            row_width = 0.07  # Width between medicines in a row
                            row_height = 0.04  # Height between rows
                            max_per_row = 4  # Maximum medicines per row

                            row = self.__counter // max_per_row  # Calculate the current row
                            col = self.__counter % max_per_row  # Calculate the current column

                            # Update the position based on row and column

                            self.__input_pose['position'][0] = self.__tray_pose[
                                'position'][0] - (row * row_height)
                            self.__input_pose['position'][1] = self.__tray_pose[
                                'position'][1] + (col * row_width)
                            self.__input_pose['position'][2] = self.__tray_pose[
                                'position'][2]

                if self.__compensate_height:
                    self.__input_pose['position'][2] += 0.24

            elif self.__state == 4 and self.__is_remote_controlling and self.__move_to == 2:
                self.__input_pose['position'][
                    0] = self.__target_moved['position'][0] - 0.15
                self.__input_pose['position'][1] = self.__target_moved[
                    'position'][1]
                self.__input_pose['position'][2] = self.__target_moved[
                    'position'][2]

        else:
            self.__input_pose['position'][0] = self.tool_frame_position[0]
            self.__input_pose['position'][1] = self.tool_frame_position[1]
            self.__input_pose['position'][2] = self.tool_frame_position[2]

        self.task_state_machine()

    def __stop_service(self, request):

        # E-stop arm motion
        self.__estop_arm_srv()

        # Reset the flag
        self.__pose_tracking = False

        self.__state = 0

        return []

    def __remote_control(self, request):

        self.__is_remote_controlling = request.data

        return True

    def __shutdown_node_service(self, request):

        self.__shut_down = True

        return []

    def __change_state(self, request):

        self.__state = 0

        if request.state in [1, 2, 3]:
            self.__rh_help = True

        elif request.state == 0:
            self.__update_chest_service()

            self.__new_target_received = True
            self.__rh_help = False

        return True

    def __update_state(self, request):

        self.__gripper_force_grasping(0.0)

        if self.__state == 4:
            self.__state = 0
        else:
            self.__state += 1

        return []

    def __move_medicine(self, request):

        self.__move_to = request.state

        self.__move_medicine_bool = True

        return True

    def tracking_state(self, request):

        self.__pose_tracking = request

        return True

    def __toolframe_transform_callback(self, message):

        self.tool_frame_position[0] = message.position.x
        self.tool_frame_position[1] = message.position.y
        self.tool_frame_position[2] = message.position.z

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
                            f'/{self.ROBOT_NAME}/teleoperation: '
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
                    f'/{self.ROBOT_NAME}/teleoperation:'
                    f'{waiting_for}'
                    # f'\nMake sure those dependencies are running properly!'
                ),
            )

        # NOTE: Add more initialization criterea if needed.
        if (self.__dependency_initialized):
            if not self.__is_initialized:
                rospy.loginfo(
                    f'\033[92m/{self.ROBOT_NAME}/teleoperation: ready.\033[0m',
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
            self.tool_frame_position - self.last_relaxed_ik_pose['position']
        )

        self.input_relaxed_ik_difference['orientation'] = (
            transformations.quaternion_multiply(
                transformations.quaternion_inverse(
                    self.__input_pose['orientation']
                ),
                self.last_relaxed_ik_pose['orientation'],
            )
        )

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

        compensated_input_pose = {
            'position': np.array([0.0, 0.0, 0.0]),
            'orientation': np.array([1.0, 0.0, 0.0, 0.0]),
        }

        compensated_input_pose['position'] = (
            self.__input_pose['position']
            - self.input_relaxed_ik_difference['position']
        )

        self.target_relaxed_ik = compensated_input_pose['position']

        # Protection against too big positional input changes.
        # Controller loses connection, out-of-sight, goes into a sleep mode etc.
        input_position_difference = np.linalg.norm(
            compensated_input_pose['position']
            - self.last_relaxed_ik_pose['position']
        )

        # if (input_position_difference > self.MAXIMUM_INPUT_CHANGE):

        #     # Stop tracking.
        #     self.__tracking_state_machine_state = 0
        #     self.__pose_tracking = False

        #     rospy.logerr(
        #         (
        #             f'/{self.ROBOT_NAME}/teleoperation: '
        #             f'\nChange in input position exceeded maximum allowed value! '
        #             f'\n- Current input: {np.round(compensated_input_pose["position"], 3)}'
        #             f'\n- Previous input: {np.round(self.last_relaxed_ik_pose["position"], 3)}'
        #             f'\n- Difference (absolute): {np.round(input_position_difference, 3)}'
        #             f'\n- Allowed difference threshold: {np.round(self.MAXIMUM_INPUT_CHANGE, 3)}'
        #             '\nStopped input tracking.'
        #         ),
        #     )

        #     return

        # Use fixed (last commanded) orientation.
        compensated_input_pose['orientation'] = (
            self.last_relaxed_ik_pose['orientation']
        )

        # Use oculus orientation.
        if self.__control_mode == 'full':
            compensated_input_pose['orientation'] = (
                self.__input_pose['orientation']
            )

            if self.COMPENSATE_ORIENTATION:
                compensated_input_pose['orientation'] = (
                    transformations.quaternion_multiply(
                        self.__input_pose['orientation'],
                        self.input_relaxed_ik_difference['orientation'],
                    )
                )

            else:
                # # Convenience orientation corrections:
                compensated_input_pose['orientation'] = (
                    transformations.quaternion_multiply(
                        compensated_input_pose['orientation'],
                        transformations.quaternion_about_axis(
                            np.deg2rad(self.CONVENIENCE_COMPENSATION[1]),
                            (0, 1, 0),  # Around Y.
                        ),
                    )
                )
                compensated_input_pose['orientation'] = (
                    transformations.quaternion_multiply(
                        compensated_input_pose['orientation'],
                        transformations.quaternion_about_axis(
                            np.deg2rad(self.CONVENIENCE_COMPENSATION[2]),
                            (0, 0, 1),  # Around Z.
                        ),
                    )
                )
                compensated_input_pose['orientation'] = (
                    transformations.quaternion_multiply(
                        compensated_input_pose['orientation'],
                        transformations.quaternion_about_axis(
                            np.deg2rad(self.CONVENIENCE_COMPENSATION[0]),
                            (1, 0, 0),  # Around X.
                        ),
                    )
                )

        waypoints = self.generate_waypoints(
            self.last_relaxed_ik_pose['position'],
            compensated_input_pose['position'],
            max_distance=0.1
        )

        for waypoint in waypoints:

            pose_message = Pose()
            pose_message.position.x = waypoint[0]
            pose_message.position.y = waypoint[1]
            pose_message.position.z = waypoint[2]

            pose_message.orientation.w = compensated_input_pose['orientation'][0
                                                                              ]
            pose_message.orientation.x = compensated_input_pose['orientation'][1
                                                                              ]
            pose_message.orientation.y = compensated_input_pose['orientation'][2
                                                                              ]
            pose_message.orientation.z = compensated_input_pose['orientation'][3
                                                                              ]

            self.__holorobot_pose.publish(pose_message)
            if self.__pose_tracking:

                self.__kinova_pose.publish(pose_message)
                rospy.sleep(0.05)

    def generate_waypoints(self, current_pose, target_pose, max_distance=0.05):

        # Check if the distance between current and target poses is greater than max_distance
        distance = np.sqrt(
            (current_pose[0] - target_pose[0])**2
            + (current_pose[1] - target_pose[1])**2
            + (current_pose[2] - target_pose[2])**2
        )

        waypoints = []

        if distance > max_distance:

            # Determine the number of waypoints based on the desired resolution
            num_waypoints = int((distance / max_distance) * 10) + 1

            if self.__state == 1:

                if self.__counter == 0 or self.__on_startup:
                    # Generate waypoints along Z-axis
                    waypoints.extend(
                        self.generate_axis_waypoints(
                            current_pose, target_pose, num_waypoints, axis=2
                        )
                    )

                    # Generate waypoints along Y-axis using the last commanded X position
                    halfway_pose = waypoints[-1].copy()
                    halfway_pose[0] += (target_pose[0] - current_pose[0]) / 2

                    # Generate waypoints along X-axis
                    waypoints.extend(
                        self.generate_axis_waypoints(
                            waypoints[-1], halfway_pose, num_waypoints, axis=0
                        )
                    )

                    waypoints.extend(
                        self.generate_axis_waypoints(
                            waypoints[-1], target_pose, num_waypoints, axis=1
                        )
                    )

                    waypoints.extend(
                        self.generate_axis_waypoints(
                            waypoints[-1], target_pose, num_waypoints, axis=0
                        )
                    )

                else:
                    # Generate waypoints along X-axis
                    waypoints.extend(
                        self.generate_axis_waypoints(
                            current_pose, target_pose, num_waypoints, axis=2
                        )
                    )

                    # Generate waypoints along Y-axis using the last commanded X position
                    waypoints.extend(
                        self.generate_axis_waypoints(
                            waypoints[-1], target_pose, num_waypoints, axis=1
                        )
                    )

                    # Generate waypoints along Z-axis
                    waypoints.extend(
                        self.generate_axis_waypoints(
                            waypoints[-1], target_pose, num_waypoints, axis=0
                        )
                    )
            else:

                if self.__move_to == 2:

                    # Generate waypoints along Y-axis using the last commanded X position
                    waypoints.extend(
                        self.generate_axis_waypoints(
                            current_pose, target_pose, num_waypoints, axis=1
                        )
                    )

                    # Generate waypoints along Z-axis
                    waypoints.extend(
                        self.generate_axis_waypoints(
                            waypoints[-1], target_pose, num_waypoints, axis=2
                        )
                    )

                    # Generate waypoints along X-axis
                    waypoints.extend(
                        self.generate_axis_waypoints(
                            waypoints[-1], target_pose, num_waypoints, axis=0
                        )
                    )

                else:
                    # Generate waypoints along X-axis
                    waypoints.extend(
                        self.generate_axis_waypoints(
                            current_pose, target_pose, num_waypoints, axis=0
                        )
                    )

                    # Generate waypoints along Y-axis using the last commanded X position
                    waypoints.extend(
                        self.generate_axis_waypoints(
                            waypoints[-1], target_pose, num_waypoints, axis=1
                        )
                    )

                    # Generate waypoints along Z-axis
                    waypoints.extend(
                        self.generate_axis_waypoints(
                            waypoints[-1], target_pose, num_waypoints, axis=2
                        )
                    )

            return waypoints
        else:
            waypoints.append(target_pose)
            # rospy.loginfo("No need to generate waypoints. The distance is within the threshold.")
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

    def task_state_machine(self):

        self.__previous_state = self.__state

        if self.__state == 0:

            self.__pose_tracking = False

            if self.__new_target_received and not self.__rh_help:

                response = self.__update_chest_service(True)
                self.__chest_position = response.response

                if self.__previous_state == 3:
                    self.__state = 3

                else:
                    self.__gripper_position(0.0)
                    self.__state = 1

                self.__pose_tracking = True

                self.__new_target_received = False

        # Grasping
        elif self.__state == 1:

            current_norm_value = np.linalg.norm(
                self.tool_frame_position - self.__input_pose['position']
            )

            if current_norm_value < 0.005:
                # Close gripper
                self.__gripper_force_grasping(0.0)
                self.__state = 2

            # Check if the norm value is stable
            elif current_norm_value < 0.1 and abs(
                self.__last_norm_value - current_norm_value
            ) < 0.01:

                if self.__norm_value_stable_since is None:
                    self.__norm_value_stable_since = time.time()
                elif time.time() - self.__norm_value_stable_since >= 3:

                    # If the value has been stable for 3 seconds, switch to state 2
                    self.__gripper_force_grasping(0.0)
                    self.__state = 2

            else:
                self.__norm_value_stable_since = None

            self.__last_norm_value = current_norm_value

        # Grasp
        elif self.__state == 2:

            if self.__has_grasped == 1:

                # self.grasped = 1

                rospy.sleep(1)

                if not self.__is_remote_controlling:
                    self.__state = 3
                    self.__chest_adjusted = False
                    self.__compensate_depth = False

                else:

                    if self.__move_medicine_bool:

                        self.__state = 3
                        self.__chest_adjusted = False
                        self.__compensate_depth = False

                if self.__chest_position == 440.0:

                    self.__compensate_height = True

            elif self.__has_grasped == 2:

                self.__gripper_position(0.0)

                rospy.sleep(1)
                self.__compensate_depth = True

                self.__state = 0
                self.__new_target_received = True

        # Placing
        elif self.__state == 3:

            same_shelf = False

            if self.__compensate_height:

                if self.__move_to == 2 and abs(
                    self.__target_grasped['position'][2]
                    - self.__target_moved['position'][2]
                ) < 0.10:

                    current_norm_value = np.linalg.norm(
                        self.tool_frame_position - self.__input_pose['position']
                    )

                    same_shelf = True
                else:

                    current_norm_value = np.linalg.norm(
                        self.tool_frame_position - np.array(
                            [
                                self.__input_pose['position'][0],
                                self.__input_pose['position'][1],
                                self.__input_pose['position'][2] - 0.24
                            ]
                        )
                    )

            else:
                current_norm_value = np.linalg.norm(
                    self.tool_frame_position - self.__input_pose['position']
                )

            current_norm_value_x = np.linalg.norm(
                self.tool_frame_position[0] - self.__input_pose['position'][0]
            )

            if current_norm_value_x < 0.03 and not self.__chest_adjusted:

                if not same_shelf:
                    response = self.__update_chest_service(True)
                    self.__chest_position = response.response

                self.__chest_adjusted = True

            if current_norm_value < 0.005:

                self.__state = 4
                self.__chest_adjusted = False
                self.__compensate_height = False

            # Check if the norm value is stable
            elif current_norm_value < 0.08 and abs(
                self.__last_norm_value - current_norm_value
            ) < 0.001:
                if self.__norm_value_stable_since is None:
                    self.__norm_value_stable_since = time.time()
                elif time.time() - self.__norm_value_stable_since >= 3:

                    self.__state = 4
                    self.__chest_adjusted = False
                    self.__compensate_height = False

            else:
                self.__norm_value_stable_since = None

            self.__last_norm_value = current_norm_value

        # Place
        elif self.__state == 4:

            # Open gripper
            self.__gripper_position(0.0)

            current_norm_value = np.linalg.norm(
                self.tool_frame_position - self.__input_pose['position']
            )

            if self.__move_to in [1, 2, 3]:

                self.__rh_help = True
                self.__move_medicine_bool = False

                if self.__move_to == 1:
                    self.__update_target_service(True)
                    self.__is_remote_controlling = False

                    self.__move_to = 0
                    self.__state = 0

                if self.__move_to in [2, 3] and current_norm_value < 0.08:
                    self.__state = 0

            else:
                self.__update_target_service(True)
                self.__move_to = 0
                self.__state = 0

        pick_and_place_state = Int32()
        pick_and_place_state.data = self.__state
        self.__robot_pick_and_place.publish(pick_and_place_state)

    def node_shutdown(self):
        """
        
        """

        rospy.loginfo_once(
            f'/{self.ROBOT_NAME}/teleoperation node is shutting down...',
        )

        # TODO: Stop the arm motion.
        # self.__stop_arm()

        rospy.loginfo_once(
            f'/{self.ROBOT_NAME}/teleoperation: node has shut down.',
        )


def main():
    """

    """

    rospy.init_node(
        'robot_control',
        log_level=rospy.INFO,  # TODO: Make this a launch file parameter.
    )

    rospy.loginfo('\n\n\n\n\n')  # Add whitespaces to separate logs.

    # # ROS parameters:

    kinova_name = rospy.get_param(
        param_name=f'{rospy.get_name()}/robot_name',
        default='my_gen3',
    )

    tracking_mode = rospy.get_param(
        param_name=f'{rospy.get_name()}/tracking_mode',
        default='toggle',
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

    kinova_teleoperation = KinovaTeleoperation(
        robot_name=kinova_name,
        tracking_mode=tracking_mode,
        compensate_orientation=compensate_orientation,
        maximum_input_change=maximum_input_change,
        convenience_compensation=convenience_compensation,
    )

    rospy.on_shutdown(kinova_teleoperation.node_shutdown)

    while not rospy.is_shutdown():

        kinova_teleoperation.main_loop()
        kinova_teleoperation.RATE.sleep()


if __name__ == '__main__':
    main()
