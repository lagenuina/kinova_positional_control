#!/usr/bin/env python

import rospy
import numpy as np

from ROS_TCP_Endpoint_msgs.msg import ControllerInput
from std_msgs.msg import *
from utils import *

class Headset:
    
    def __init__(self):
        
        self.onTrackingStart = True
        self.isTracking_state = 0
        self.isTracking = False
        self.gripperButtonState = False
        self.gripperState = "open"
        self.gripperButtonState = False
        self.gripperButtonReleased = True
        
        # Initialize controller input position and end-effector target position
        self.input_pos_gcs = np.array([0.0, 0.0, 0.0])
        self.target_pos = np.array([0.0, -0.1, 0.5])
        
        # Initialize dictionary for right controller variables
        self.right_controller = {'primaryButton': False, 
                            'secondaryButton': False,
                            'triggerButton': False,
                            'triggerValue': 0.0,
                            'gripButton': False,
                            'joystickButton': False,
                            'joystick_pos_x': 0.0, 'joystick_pos_y': 0.0,
                            'controller_pos_x': 0.0, 'controller_pos_y': 0.0, 'controller_pos_z': 0.0,
                            'controller_rot_w': 0.0, 'controller_rot_x': 0.0, 'controller_rot_y': 0.0, 'controller_rot_z': 0.0   
                            }

        # Subscribers      
        rospy.Subscriber("rightControllerInfo", ControllerInput, self.right_callback)
        rospy.Subscriber("leftControllerInfo", ControllerInput, self.left_callback)

        #Service
        self.gripper_command_srv = rospy.ServiceProxy('my_gen3/base/send_gripper_command', SendGripperCommand)


    def right_callback(self, data):
        """Callback function for right controller info. Transform the controller input position from 
        Left-Handed Coordinate system to Right-handed Coordinate system: 
        1) swap y and z axis;
        2) swap x and new y (which was z) to have x facing forward;
        3) negate new y (which is x) to make it align with global coordinate system.
        """
        
        # Update dictionary
        self.right_controller['primaryButton'] = data.primaryButton
        self.right_controller['secondaryButton'] = data.secondaryButton
        self.right_controller['triggerButton'] = data.triggerButton
        self.right_controller['triggerValue'] = data.triggerValue
        self.right_controller['gripButton'] = data.gripButton
        self.right_controller['joystickButton'] = data.joystickButton
        self.right_controller['joystick_pos_x'] = data.joystick_pos_x
        self.right_controller['joystick_pos_y'] = data.joystick_pos_y
        self.right_controller['controller_pos_x'] = data.controller_pos_x
        self.right_controller['controller_pos_y'] = data.controller_pos_y
        self.right_controller['controller_pos_z'] = data.controller_pos_z
        self.right_controller['controller_rot_w'] = data.controller_rot_w
        self.right_controller['controller_rot_x'] = data.controller_rot_x
        self.right_controller['controller_rot_y'] = data.controller_rot_y
        self.right_controller['controller_rot_z'] = data.controller_rot_z
    
        # Transition from Left-handed CS (Unity) to Right-handed CS (Global) for controller position
        self.input_pos_gcs = np.array([ -1* data.controller_pos_z, data.controller_pos_x, data.controller_pos_y ])


    def gripper_control(self, mode, value):
        """Control the gripper by sending a gripper command to the Kinova robot arm.

        Args:
            mode (int): The mode of the gripper command (1 for force, 2 for velocity, 3 for position)
            value (float): The value of the gripper command (force, velocity, or position)
        """
        
        # https://github.com/Kinovarobotics/ros_kortex/blob/noetic-devel/kortex_driver/msg/generated/base/Finger.msg
        finger = Finger()
        finger.finger_identifier = 0
        finger.value = value

        # https://github.com/Kinovarobotics/ros_kortex/blob/noetic-devel/kortex_driver/msg/generated/base/Gripper.msg
        gripper = Gripper()
        gripper.finger.append(finger)

        # https://github.com/Kinovarobotics/ros_kortex/blob/noetic-devel/kortex_driver/msg/generated/base/GripperCommand.msg
        gripper_command = SendGripperCommand()
        gripper_command.mode = mode
        gripper_command.duration = 0
        gripper_command.gripper = gripper

        self.gripper_command_srv(gripper_command)


    def gripper_sm(self):
        """Control the gripper using a state machine.
        """
        
        if self.gripperState == "open" and self.gripperButtonState and self.gripperButtonReleased:
            
            # Change gripper state to close
            self.gripperState = "close"

            # Close the gripper
            self.gripper_control(3, 0.6)

        elif self.gripperState == "close" and self.gripperButtonState and self.gripperButtonReleased:
            
            # Change gripper state
            self.gripperState = "open"

            # Open the gripper
            self.gripper_control(3, 0.0)