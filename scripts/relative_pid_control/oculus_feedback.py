#!/usr/bin/env python

import rospy
import transformations as T
import numpy as np
import math

from ROS_TCP_Endpoint_msgs.msg import ControllerInput
import geometry_msgs.msg as geom_msgs
from std_msgs.msg import *
import gopher_ros_clearcore.msg as clearcore_msg
import gopher_ros_clearcore.srv as clearcore_srv

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
        
        self.input_pos_gcs = np.array([0.0, 0.0, 0.0])
        self.target_pos = np.array([0.0, -0.1, 0.5])
        
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
        
        self.left_controller = {'primaryButton': False, 
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

        # Publishers
        self.chest_vel_pub = rospy.Publisher('z_chest_vel', geom_msgs.Twist, queue_size=1)
        self.gripper_command_srv = rospy.ServiceProxy('my_gen3/base/send_gripper_command', SendGripperCommand)

    def right_callback(self, data):
        
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
    
        # # POSITION
        # Transition from Left-handed CS (Unity) to Right-handed CS (Global): swap y and z axis
        # Then swap x and new y (which was z) to have x facing forward
        # Negate new y (which is x) to make it align with a global coordinate system
        self.input_pos_gcs = np.array([ -1* data.controller_pos_z, data.controller_pos_x, data.controller_pos_y ])

        # Convert from Global to Kinova CS
        self.input_pos_kcs = global_to_kinova(self.input_pos_gcs)

        # # ORIENTATION
        # Raw quaternion input (Left-handed CS)
        input_rot_gcs = np.array([data.controller_rot_x, data.controller_rot_y, data.controller_rot_z, data.controller_rot_w])
        
        # Transition from Left-handed CS (Unity) to Right-handed CS (Global)
        input_rot_gcs = left_to_right_handed(input_rot_gcs)

        # More comfortable position (compensation)
        Qy = T.quaternion_about_axis(math.radians(-45), (0, 1, 0))
        input_rot_gcs = T.quaternion_multiply(input_rot_gcs, Qy)

        # Transition from Global CS to Kinova CS: rotate around y and z axis
        self.input_rot_kcs = global_to_kinova(input_rot_gcs)
    
    # Left controller topic callback function
    def left_callback(self, data):

        # Update dictionary
        self.left_controller['primaryButton'] = data.primaryButton
        self.left_controller['secondaryButton'] = data.secondaryButton
        self.left_controller['triggerButton'] = data.triggerButton
        self.left_controller['triggerValue'] = data.triggerValue
        self.left_controller['gripButton'] = data.gripButton
        self.left_controller['joystickButton'] = data.joystickButton
        self.left_controller['joystick_pos_x'] = data.joystick_pos_x
        self.left_controller['joystick_pos_y'] = data.joystick_pos_y
        self.left_controller['controller_pos_x'] = data.controller_pos_x
        self.left_controller['controller_pos_y'] = data.controller_pos_y
        self.left_controller['controller_pos_z'] = data.controller_pos_z
        self.left_controller['controller_rot_w'] = data.controller_rot_w
        self.left_controller['controller_rot_x'] = data.controller_rot_x
        self.left_controller['controller_rot_y'] = data.controller_rot_y
        self.left_controller['controller_rot_z'] = data.controller_rot_z

    # Gripper control: mode=1 - force, mode=2 - velocity, mode=3 - position
    def gripper_control(self, mode, value):

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

    # Gripper control state machine
    def gripper_sm(self):

        if self.gripperState == "open" and self.gripperButtonState and self.gripperButtonReleased:
            # Change gripper state
            self.gripperState = "close"

            # Close the gripper
            self.gripper_control(3, 0.6)

        elif self.gripperState == "close" and self.gripperButtonState and self.gripperButtonReleased:
            # Change gripper state
            self.gripperState = "open"

            # Open the gripper
            self.gripper_control(3, 0.0)