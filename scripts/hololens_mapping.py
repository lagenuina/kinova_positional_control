#!/usr/bin/env python
import rospy
import numpy as np
import copy
import tf
from pynput.keyboard import Key, Listener
from std_msgs.msg import (Bool)
from geometry_msgs.msg import (Pose)
from Scripts.srv import BoolUpdate, UpdateState
from std_srvs.srv import Empty


class HoloLensMapping:
    """
    
    """

    def __init__(
        self,
        anchor_id,
        robot_name,
        task,
    ):
        """
        
        """

        # # Private constants:

        # # Public constants:
        self.ROBOT_NAME = robot_name
        self.anchor_id = anchor_id
        self.task = task
        self.listener = tf.TransformListener()
        self.br = tf.TransformBroadcaster()
        self.rate = rospy.Rate(5)

        # # Private variables:
        self.__input_pose = {
            'position': np.array([0.0, 0.0, 0.0]),
            'orientation': np.array([1.0, 0.0, 0.0, 0.0]),
        }

        # # Public variables:
        self.is_initialized = True
        # self.tracking_button = False

        # # Initialization and dependency status topics:
        self.__is_initialized = False
        self.__dependency_initialized = False

        self.__node_is_initialized = rospy.Publisher(
            f'/{self.ROBOT_NAME}/hololens_mapping/is_initialized',
            Bool,
            queue_size=1,
        )

        self.__dependency_status = {
            'teleoperation': False,
        }

        self.__dependency_status_topics = {
            'teleoperation':
                rospy.Subscriber(
                    f'/{self.ROBOT_NAME}/teleoperation/is_initialized',
                    Bool,
                    self.__teleoperation_callback,
                ),
        }

        # # Service provider:
        self.restart_nodes = rospy.Service(
            f'/{self.ROBOT_NAME}/restart_nodes',
            BoolUpdate,
            self.__restart_nodes_service,
        )

        # # Service subscriber:
        self.stop_robot_control_node = rospy.ServiceProxy(
            f'/{self.ROBOT_NAME}/robot_control/shut_down',
            BoolUpdate,
        )

        self.stop_hololens_chest_node = rospy.ServiceProxy(
            f'/{self.ROBOT_NAME}/hololens_chest/shut_down',
            BoolUpdate,
        )

        self.stop_positional_control_node = rospy.ServiceProxy(
            f'/{self.ROBOT_NAME}/positional_control/shut_down',
            BoolUpdate,
        )

        self.stop_task = rospy.ServiceProxy(
            f'/{self.ROBOT_NAME}/stop_task',
            BoolUpdate,
        )

        self.change_task_state_service = rospy.ServiceProxy(
            '/change_task_state_service',
            UpdateState,
        )

        if self.task == 'study':
            rospy.wait_for_service('/data_writer/resume_recording')

            self.resume_image_recording = rospy.ServiceProxy(
                '/chest_cam/image_writer/resume_recording',
                Empty,
            )

            self.pause_image_recording = rospy.ServiceProxy(
                '/chest_cam/image_writer/pause_recording',
                Empty,
            )

            self.stop_image_recording = rospy.ServiceProxy(
                '/chest_cam/image_writer/finish_recording',
                Empty,
            )

            self.resume_recording = rospy.ServiceProxy(
                '/data_writer/resume_recording',
                Empty,
            )

            self.pause_recording = rospy.ServiceProxy(
                '/data_writer/pause_recording',
                Empty,
            )

            self.stop_recording = rospy.ServiceProxy(
                '/data_writer/finish_recording',
                Empty,
            )

        # # Topic publisher:
        self.__node_is_initialized = rospy.Publisher(
            f'/{self.ROBOT_NAME}/hololens_mapping/is_initialized',
            Bool,
            queue_size=1,
        )

        self.__teleoperation_pose = rospy.Publisher(
            f'/{self.ROBOT_NAME}/teleoperation/input_pose',
            Pose,
            queue_size=1,
        )

        # # Topic subscriber:
        rospy.Subscriber(
            '/hologram_feedback/pose',
            Pose,
            self.__hololens_pose_callback,
        )

    # # Dependency status callbacks:
    def __teleoperation_callback(self, message):
        """Monitors teleoperation is_initialized topic.
        
        """

        self.__dependency_status['teleoperation'] = message.data

    def __restart_nodes_service(self, request):

        self.stop_positional_control_node(True)
        self.stop_robot_control_node(True)

        return True

    # # Topic callbacks:
    def __hololens_pose_callback(self, message):
        """

        """

        input_pose = [
            message.position.x,
            message.position.y,
            -message.position.z,
        ]

        input_orientation = [
            message.orientation.w,
            message.orientation.x,
            message.orientation.y,
            message.orientation.z,
        ]

        try:
            self.br.sendTransform(
                input_pose,
                input_orientation,
                rospy.Time.now(),
                'target',
                self.anchor_id,
            )

            (translation, rotation) = self.listener.lookupTransform(
                '/base_link', '/target', rospy.Time(0)
            )

            self.__input_pose['position'][0] = translation[0]
            self.__input_pose['position'][1] = translation[1]
            self.__input_pose['position'][2] = translation[2]

            self.__input_pose['orientation'][0] = message.orientation.w
            self.__input_pose['orientation'][1] = message.orientation.x
            self.__input_pose['orientation'][2] = message.orientation.y
            self.__input_pose['orientation'][3] = message.orientation.z

        except (
            tf.LookupException,
            tf.ConnectivityException,
            tf.ExtrapolationException,
        ):
            self.rate.sleep()

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
                            f'/{self.ROBOT_NAME}/hololens_mapping: '
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
                    f'/{self.ROBOT_NAME}/hololens_mapping:'
                    f'{waiting_for}'
                    # f'\nMake sure those dependencies are running properly!'
                ),
            )

        # NOTE: Add more initialization criterea if needed.
        if (self.__dependency_initialized):
            if not self.__is_initialized:
                rospy.loginfo(
                    f'\033[92m/{self.ROBOT_NAME}/hololens_mapping: ready.\033[0m',
                )

                self.__is_initialized = True

        else:
            if self.__is_initialized:
                # NOTE (optionally): Add code, which needs to be executed if the
                # nodes's status changes from True to False.
                pass

            self.__is_initialized = False

        self.__node_is_initialized.publish(self.__is_initialized)

    def __publish_teleoperation_pose(self):
        """
        
        """

        corrected_input_pose = copy.deepcopy(self.__input_pose)

        pose_message = Pose()
        pose_message.position.x = corrected_input_pose['position'][0]
        pose_message.position.y = corrected_input_pose['position'][1]
        pose_message.position.z = corrected_input_pose['position'][2]

        pose_message.orientation.w = corrected_input_pose['orientation'][0]
        pose_message.orientation.x = corrected_input_pose['orientation'][1]
        pose_message.orientation.y = corrected_input_pose['orientation'][2]
        pose_message.orientation.z = corrected_input_pose['orientation'][3]

        self.__teleoperation_pose.publish(pose_message)

    # # Public methods:
    def main_loop(self):
        """
        
        """

        self.__check_initialization()

        if not self.__is_initialized:
            return

        self.__publish_teleoperation_pose()
        # self.__teleoperation_tracking_button.publish(self.tracking_button)
        # self.__teleoperation_gripper_button.publish(
        #     self.__hololens_buttons.trigger_button
        # )
        # self.__teleoperation_mode_button.publish(
        #     self.__hololens_buttons.primary_button
        # )

    def node_shutdown(self):
        """
        
        """

        rospy.loginfo_once(
            f'/{self.ROBOT_NAME}/hololens_mapping: node is shutting down...',
        )

        rospy.loginfo_once(
            f'/{self.ROBOT_NAME}/hololens_mapping: node has shut down.',
        )

        if self.task == 'study':
            self.stop_recording()
            self.stop_image_recording()

    def on_press(self, key):
        try:
            if key == Key.f5:

                if self.task == 'study':
                    self.pause_recording()
                    self.pause_image_recording()

                self.stop_task(True)

            elif key == Key.f6:
                self.stop_positional_control_node(True)
                self.stop_robot_control_node(True)

            elif key == Key.f7:

                if self.task == 'study':
                    self.resume_recording()
                    self.resume_image_recording()

                self.change_task_state_service(0)

        except AttributeError:
            pass


def main():
    """

    """

    rospy.init_node(
        'hololens_mapping',
        log_level=rospy.INFO,  # TODO: Make this a launch file parameter.
    )

    rospy.loginfo('\n\n\n\n\n')  # Add whitespaces to separate logs.

    # # ROS parameters:
    kinova_name = rospy.get_param(
        param_name=f'{rospy.get_name()}/robot_name',
        default='my_gen3',
    )

    anchor_id = rospy.get_param(param_name=f'{rospy.get_name()}/anchor_id')

    task = rospy.get_param(
        param_name=f'{rospy.get_name()}/task',
        default='study',
    )

    hololens_kinova_mapping = HoloLensMapping(
        robot_name=kinova_name,
        anchor_id=anchor_id,
        task=task,
    )

    rospy.on_shutdown(hololens_kinova_mapping.node_shutdown)

    while not rospy.is_shutdown():

        with Listener(on_press=hololens_kinova_mapping.on_press) as listener:
            listener.join()

        hololens_kinova_mapping.main_loop()
        hololens_kinova_mapping.rate.sleep()


if __name__ == '__main__':
    main()
