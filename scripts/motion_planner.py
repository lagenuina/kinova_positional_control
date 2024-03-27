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
from kortex_driver.srv import (ApplyEmergencyStop)
from kinova_positional_control.srv import (
    GripperForceGrasping, GripperPosition
)
from Scripts.srv import (UpdateState, UpdateChest)


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
        self.__state = 0
        self.__waypoint_index = 0

        self.__target_has_changed = True
        self.__last_pose_tracking = False
        self.__pose_tracking = False
        self.__shut_down = False

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
        self.target_relaxed_ik = {
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
            '/chest_handler/adjust_chest',
            Empty,
        )
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

        self.tool_frame_position[0] = message.position.x
        self.tool_frame_position[1] = message.position.y
        self.tool_frame_position[2] = message.position.z

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
        
        compensated_input_pose = {
            'position': np.array([0.0, 0.0, 0.0]),
            'orientation': np.array([1.0, 0.0, 0.0, 0.0]),
        }

        compensated_input_pose['position'] = (
            self.__input_pose['position']
            - self.input_relaxed_ik_difference['position']
        )

        # Use fixed (last commanded) orientation.
        compensated_input_pose['orientation'] = (
            self.last_relaxed_ik_pose['orientation']
        )

        position_difference = np.linalg.norm(compensated_input_pose['position'] - self.target_relaxed_ik['position'])

        if position_difference > 0.03:

            self.target_relaxed_ik['position'] = compensated_input_pose['position']
            self.target_relaxed_ik['orientation'] = compensated_input_pose['orientation']

            self.__target_has_changed = True


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

    def __move_to_next_waypoint(self, waypoints):

        if self.__waypoint_index < len(waypoints):

            waypoint = waypoints[self.__waypoint_index]

            pose_message = Pose()
            pose_message.position.x = waypoint[0]
            pose_message.position.y = waypoint[1]
            pose_message.position.z = waypoint[2]

            pose_message.orientation.w = self.target_relaxed_ik['orientation'][0]
            pose_message.orientation.x = self.target_relaxed_ik['orientation'][1]
            pose_message.orientation.y = self.target_relaxed_ik['orientation'][2]
            pose_message.orientation.z = self.target_relaxed_ik['orientation'][3]

            # self.__holorobot_pose.publish(pose_message)
            self.__kinova_pose.publish(pose_message)
            rospy.sleep(0.05)

            self.__waypoint_index += 1

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

            # Generate new waypoints
            waypoints = self.generate_waypoints(
                self.last_relaxed_ik_pose['position'],
                self.target_relaxed_ik['position'],
                max_distance=0.1
            )

            # Reset waypoint index and target flag
            self.__waypoint_index = 0
            self.__target_has_changed = False

        if self.__pose_tracking:
            self.__check_norm_value()
            self.__move_to_next_waypoint(waypoints)

    def __check_norm_value(self):

        # Either this, or after the last waypoint is reached?

        current_norm_value = np.linalg.norm(self.tool_frame_position - self.__input_pose['position'])

        if current_norm_value < 0.005:
            
            # Call service to switch state
            self.__update_task_state()

        # Check if the norm value is stable
        elif current_norm_value < 0.1 and abs(self.__last_norm_value - current_norm_value) < 0.01:

            if self.__norm_value_stable_since is None:
                self.__norm_value_stable_since = time.time()
            
            elif time.time() - self.__norm_value_stable_since >= 3:
                # If the value has been stable for 3 seconds, switch to the next state
                self.__update_task_state()

        else:
            self.__norm_value_stable_since = None

        self.__last_norm_value = current_norm_value

    def get_axis_order(self):

        if self.__state == 1:
            return [2, 1, 0]
        else:
            if self.__move_to == 2:
                return [1, 2, 0]
            else:
                return [0, 1, 2]
            
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
            axis_order = self.get_axis_order()

            if self.__state == 1 and self.__counter == 0:
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

                for axis in axis_order:
                    waypoints.extend(self.generate_axis_waypoints(current_pose, target_pose, num_waypoints, axis))
                    
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
