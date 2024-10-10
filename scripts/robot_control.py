#!/usr/bin/env python
"""Implements teleoperation module for positional control.

TODO: Add detailed description.

Author (s):
    1. Lorena Genua (lorena.genua@gmail.com), Human-Inspired Robotics (HiRo)
       lab, Worcester Polytechnic Institute (WPI), 2023.
"""

import rospy
import numpy as np
from ast import (literal_eval)
from std_msgs.msg import (Bool, Int32, String)
from std_srvs.srv import (Empty, SetBool)
from geometry_msgs.msg import (Pose)
from kortex_driver.srv import (ApplyEmergencyStop)
from kinova_positional_control.srv import (
    GripperForceGrasping, GripperPosition
)
from Scripts.srv import (UpdateState)
from holo_project.msg import (TargetInfo)


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
        task,
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
        self.RATE = rospy.Rate(100)

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
        self.__grasping_attempts = 0

        self.__move_to = 1
        self.__state = 0
        self.__previous_state = 0
        self.__marker_id = None

        if task:
            self.__ids_grasp_failure = [8, 1]

        else:
            self.__ids_grasp_failure = [5]

        self.__increment = 0

        self.__update_state = False
        self.__update_to_grasping = False
        self.__force_grasping_status = False
        self.__pose_tracking = False
        self.__is_remote_controlling = False
        self.__move_medicine_bool = False
        self.__shut_down = False
        self.__new_target_received = False
        self.__rh_help = False

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
            '/move_medicine',
            UpdateState,
            self.__move_medicine,
        )
        rospy.Service(
            '/robot_control/move_arm',
            UpdateState,
            self.__move_arm,
        )
        rospy.Service(
            '/remote_handling',
            SetBool,
            self.__remote_control,
        )
        rospy.Service(
            f'/{self.ROBOT_NAME}/robot_control/update_state',
            Empty,
            self.__update_task_state,
        )
        rospy.Service(
            '/robot_control/grasp_command',
            Empty,
            self.__grasp_command,
        )

        # # Service subscriber:
        self.__gripper_force_grasping = rospy.ServiceProxy(
            f'/{self.ROBOT_NAME}/gripper_control/force_grasping',
            GripperForceGrasping,
        )
        self.__gripper_position = rospy.ServiceProxy(
            f'/{self.ROBOT_NAME}/gripper_control/position',
            GripperPosition,
        )
        self.__start_tracking = rospy.ServiceProxy(
            f'/{self.ROBOT_NAME}/pose_tracking',
            SetBool,
        )
        self.__update_target_service = rospy.ServiceProxy(
            '/update_target',
            Empty,
        )
        self.__update_chest_service = rospy.ServiceProxy(
            '/chest_handler/adjust_chest',
            Empty,
        )
        self.__estop_arm_srv = rospy.ServiceProxy(
            f'/{self.ROBOT_NAME}/base/apply_emergency_stop',
            ApplyEmergencyStop,
        )
        self.__grasp_failure_service = rospy.ServiceProxy(
            '/grasp_failure',
            Empty,
        )
        self.__failure_resolved_service = rospy.ServiceProxy(
            '/failure_resolved',
            Empty,
        )

        # # Topic publisher:
        self.__node_is_initialized = rospy.Publisher(
            f'/{self.ROBOT_NAME}/teleoperation/is_initialized',
            Bool,
            queue_size=1,
        )
        self.__kinova_pose = rospy.Publisher(
            f'/{self.ROBOT_NAME}/robot_control/target_pose',
            Pose,
            queue_size=1,
        )
        self.__holorobot_pose = rospy.Publisher(
            '/holorobot/positional_control/input_pose',
            Pose,
            queue_size=1,
        )
        self.__task_state = rospy.Publisher(
            f'/{self.ROBOT_NAME}/robot_control/current_task_state',
            Int32,
            queue_size=1,
        )
        self.__move_to_publisher = rospy.Publisher(
            f'/{self.ROBOT_NAME}/robot_control/move_to',
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
            '/task_manager/target_counter',
            Int32,
            self.__target_counter_callback,
        )
        rospy.Subscriber(
            '/task_manager/target_identifier',
            TargetInfo,
            self.__target_identifier_callback,
        ),
        rospy.Subscriber(
            f'/{self.ROBOT_NAME}/gripper_control/force_grasping_status',
            String,
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

        self.__force_grasping_status = message.data

    def __target_counter_callback(self, message):

        self.__counter = message.data

    def __target_identifier_callback(self, message):

        self.__marker_id = message.id

    def __input_pose_callback(self, message):
        """

        """

        self.task_state_machine()

        if self.__pose_tracking:

            if self.__state == 1:
                if self.__marker_id in self.__ids_grasp_failure:
                    # To simulate a failure in grasp, set target to be 10 cm away (depth) from the actual one
                    offset = -0.10
                else:
                    offset = 0

                self.__update_pose(
                    message.position,
                    offset_x=offset,
                    offset_z=0.0,
                )

                self.__target_grasped['position'] = (
                    self.__input_pose['position'].copy()
                )

            # If the remote operator is controlling it, after the object is grasped,
            # move arm out of the shelf and up
            elif self.__state == 3:
                self.__adjust_position(
                    self.__target_grasped['position'],
                    dx=-0.2,
                    dz=0.07,
                )

            elif self.__state == 4:

                # Place it in the bin
                if self.__move_to == 1:
                    self.__place_in_bin(message.position)

                # If the placing location is selected on the screen by the operator
                elif self.__move_to == 2:
                    self.__update_pose(
                        message.position,
                        offset_x=-0.04,
                        offset_z=0.05,
                    )
                    self.__target_moved['position'] = (
                        self.__input_pose['position'].copy()
                    )

                # If the medicine needs to be set aside
                elif self.__move_to == 3:

                    self.__set_aside(
                        message.position.y - 0.08,
                        offset_z=-0.31,
                    )

            # If the option was "Move here", after the object was placed in the position selected on the screen,
            # move it out of the shelves
            elif self.__state == 4 and self.__is_remote_controlling and self.__move_to == 2:
                self.__adjust_position(
                    self.__target_moved['position'],
                    dx=-0.15,
                )

            elif self.__state == 6:
                self.__adjust_position(
                    self.__target_moved['position'],
                    dx=-0.2,
                    dz=0.07,
                )

            elif self.__state == 7:

                self.__update_pose(
                    message.position,
                    offset_x=-0.1 + self.__increment,
                    offset_z=0.0,
                )

            elif self.__state == 2:
                self.__input_pose['position'][0] = self.tool_frame_position[0]
                self.__input_pose['position'][1] = self.tool_frame_position[1]
                self.__input_pose['position'][2] = self.tool_frame_position[2]

        # Don't move the robot, current end effector position
        else:
            self.__input_pose['position'][0] = self.tool_frame_position[0]
            self.__input_pose['position'][1] = self.tool_frame_position[1]
            self.__input_pose['position'][2] = self.tool_frame_position[2]

        pose_message = Pose()
        pose_message.position.x = self.__input_pose['position'][0]
        pose_message.position.y = self.__input_pose['position'][1]
        pose_message.position.z = self.__input_pose['position'][2]

        pose_message.orientation.w = self.__input_pose['orientation'][0]
        pose_message.orientation.x = self.__input_pose['orientation'][1]
        pose_message.orientation.y = self.__input_pose['orientation'][2]
        pose_message.orientation.z = self.__input_pose['orientation'][3]
        self.__kinova_pose.publish(pose_message)

    def __remote_control(self, request):

        if not self.__marker_id in self.__ids_grasp_failure:
            self.__is_remote_controlling = request.data

        return [True, ""]

    def __change_state(self, request):

        self.__state = 0

        if request.state in [1, 2, 3]:
            self.__rh_help = True

        elif request.state == 0:
            self.__new_target_received = True
            self.__rh_help = False

        return True

    def __move_medicine(self, request):

        self.__move_to = request.state

        self.__move_medicine_bool = True

        move_state = Int32()
        move_state.data = request.state
        self.__move_to_publisher.publish(move_state)

        return True

    def __move_arm(self, request):

        self.__direction = request.state

        # Make sure the arm doesn't move too far inside in the shelf
        if self.__direction == 0 and self.__increment < 0.10:
            self.__increment += 0.05
        elif self.__direction == 1:
            self.__increment -= 0.05

        return True

    def __update_task_state(self, request):

        self.__update_state = True

        return []

    def __grasp_command(self, request):

        self.__update_to_grasping = True
        return []

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

    def __update_pose(self, position, offset_x=0, offset_y=0, offset_z=0):
        self.__input_pose['position'][0] = position.x + offset_x
        self.__input_pose['position'][1] = position.y + offset_y
        self.__input_pose['position'][2] = position.z + offset_z

    def __adjust_position(self, base_position, dx=0, dy=0, dz=0):
        self.__input_pose['position'][0] = base_position[0] + dx
        self.__input_pose['position'][1] = base_position[1] + dy
        self.__input_pose['position'][2] = base_position[2] + dz

    def __set_aside(self, y_offset, offset_z):
        self.__input_pose['position'][0] = self.__tray_pose['position'][0]
        self.__input_pose['position'][1] = y_offset
        self.__input_pose['position'][
            2] = self.__tray_pose['position'][2] + offset_z

    def __place_in_bin(self, position):
        if self.__counter == 0:
            self.__update_pose(
                position, offset_x=-0.15, offset_y=-0.40, offset_z=-0.35
            )
            self.__tray_pose['position'] = self.__input_pose['position'].copy()
        else:
            row_width, row_height, max_per_row = 0.04, 0.01, 6
            row, col = divmod(self.__counter, max_per_row)
            self.__adjust_position(
                self.__tray_pose['position'],
                dx=-(row * row_height),
                dy=(col * row_width)
            )

    def task_state_machine(self):

        self.__previous_state = self.__state

        if self.__state == 0:

            if self.__new_target_received and not self.__rh_help:

                if self.__previous_state == 4:
                    self.__state = 4

                else:
                    self.__gripper_position(0.0)
                    self.__state = 1

                self.__pose_tracking = True
                self.__new_target_received = False

                self.__update_chest_service()
                self.__start_tracking(self.__pose_tracking)

        # Grasping
        elif self.__state == 1 and self.__update_state:

            rospy.sleep(1)

            # Close gripper
            self.__gripper_force_grasping(0.0)
            self.__state = 2
            self.__grasping_attempts += 1

            self.__update_state = False

        # Grasp
        elif self.__state == 2:

            # If it grasped
            if self.__force_grasping_status == 'grasped':

                rospy.sleep(1)
                self.__state = 3
                self.__update_state = False
                self.__grasping_attempts = 0

            # If it didn't grasp
            elif self.__force_grasping_status == 'failed':

                self.__gripper_position(0.0)
                rospy.sleep(2)

                if not self.__marker_id in self.__ids_grasp_failure:

                    if self.__grasping_attempts >= 2:
                        self.__grasping_attempts = 0
                        self.__state = 3
                        self.__update_state = False
                    else:
                        # Go back to previous state
                        self.__state = 1
                        self.__update_state = True

                else:
                    self.__grasp_failure_service()
                    self.__state = 7
                    self.__grasping_attempts = 0

        # Move grasped object up and out of the shelves
        elif self.__state == 3 and self.__update_state:

            if not self.__is_remote_controlling:
                self.__state = 4

                self.__update_chest_service()
                self.__update_state = False
            else:
                if self.__move_medicine_bool:
                    self.__state = 4

                    self.__update_chest_service()
                    self.__update_state = False

        # Placing
        elif self.__state == 4 and self.__update_state:

            if self.__move_to == 2:
                self.__gripper_position(0.0)

                rospy.sleep(1)
                self.__state = 6

            else:
                self.__state = 5

            self.__update_state = False

        # Place
        elif self.__state == 5:

            # Open gripper
            self.__gripper_position(0.0)

            self.__move_medicine_bool = False

            if self.__move_to == 1:

                if self.__is_remote_controlling or (
                    self.__marker_id in self.__ids_grasp_failure
                ):
                    self.__failure_resolved_service()
                else:
                    self.__update_target_service()

                self.__is_remote_controlling = False
                self.__rh_help = False

            self.__move_to = 1
            self.__state = 0
            self.__pose_tracking = False
            self.__start_tracking(self.__pose_tracking)

        elif self.__state == 6 and self.__update_state:

            self.__state = 5
            self.__update_state = False

        elif self.__state == 7:
            if self.__update_to_grasping:
                # Close gripper
                self.__gripper_force_grasping(0.0)

                rospy.sleep(2)
                self.__state = 2
                self.__update_to_grasping = False

                self.__increment = 0

                self.__is_remote_controlling = False

        current_state = Int32()
        current_state.data = self.__state
        self.__task_state.publish(current_state)

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

    task = rospy.get_param(
        param_name=f'{rospy.get_name()}/study',
        default='false',
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
        task=task,
    )

    rospy.on_shutdown(kinova_teleoperation.node_shutdown)

    while not rospy.is_shutdown():

        kinova_teleoperation.main_loop()
        kinova_teleoperation.RATE.sleep()


if __name__ == '__main__':
    main()
