#!/usr/bin/env python

import rospy
import roslaunch
import transformations as T
import numpy as np
import math
import argparse

from utils import *
from kortex_feedback import Kinova
from oculus_feedback import Headset
from ros_tcp_endpoint_msgs.msg import ControllerInput
from relaxed_ik_ros1.msg import EEPoseGoals
import geometry_msgs.msg as geom_msgs
from std_msgs.msg import *
from kortex_driver.msg import *
from kortex_driver.srv import *
import kinova_positional_control.srv as posctrl_srv

class Kinova_Oculus_control():

    def __init__(self):

        # Initialize the node
        rospy.init_node("coordinate_converter", anonymous=True)
        rospy.on_shutdown(self.node_shutdown)

        # Variables
        self.isInitialized = False
        self.motionFinished = False
        self.initButton_state = False
        self.estopButton_state = False
        self.clearfaultButton_state = False  

        # Launch files
        self.roslaunch_pid()
        self.roslaunch_relaxed_ik()
        rospy.sleep(2)

        # Publishing
        self.angles_pub = rospy.Publisher('/relaxed_ik/joint_angle_solutions', JointAngles, queue_size=10)
        self.relaxed_ik_target_wcs_pub = rospy.Publisher('/relaxed_ik/position_wcs', Float32MultiArray, queue_size=1)
        self.relaxed_ik_target_gcs_pub = rospy.Publisher('/relaxed_ik/position_gcs', Float32MultiArray, queue_size=1)

        # Subscribing
        rospy.Subscriber('/pid/motion_finished', Bool, self.pid_motion_finished_callback)

        # Service
        self.pid_setpoint_srv = rospy.ServiceProxy('pid_setpoint', posctrl_srv.pid_setpoint)
        self.pid_vel_limit_srv = rospy.ServiceProxy('pid_vel_limit', posctrl_srv.pid_vel_limit)
        self.stop_arm_srv = rospy.ServiceProxy('my_gen3/base/stop', Stop)


    # Callback function that updates motionFinished flag
    def pid_motion_finished_callback(self, data):

        self.motionFinished = data.data

    # Block code execution until a motionFinished flag is set or a node is shutdown
    def wait_motion_finished(self):

        # Allow a motion to start
        rospy.sleep(1)

        # Block the code execution
        while not self.motionFinished and not rospy.is_shutdown():
            pass

    # Roslaunch PID
    def roslaunch_pid(self):

        uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
        roslaunch.configure_logging(uuid)
        self.pid_launch = roslaunch.parent.ROSLaunchParent(uuid, ['/home/fetch/catkin_workspaces/oculus_relaxedik_ws/src/kinova_pid/launch/arm_controller.launch'])
        self.pid_launch.start()

    # Roslaunch relaxed IK
    def roslaunch_relaxed_ik(self):

        uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
        roslaunch.configure_logging(uuid)
        self.relaxed_ik_launch = roslaunch.parent.ROSLaunchParent(uuid, ['/home/fetch/catkin_workspaces/oculus_relaxedik_ws/src/relaxed_ik_ros1/launch/relaxed_ik.launch'])
        self.relaxed_ik_launch.start()

    # This function is called when the node is shutting down
    def node_shutdown(self):

        print("\nNode is shutting down...")

        # Stop arm motion
        self.stop_arm_srv()

        # Shutdown PID and relaxed IK nodes
        self.pid_launch.shutdown()
        self.relaxed_ik_launch.shutdown()

        print("\nNode has shut down.")
            
    def initialization(self):

        # Set 20% velocity
        self.pid_vel_limit_srv(0.2)

        # Homing position
        print("\nHoming has started...\n")

        # Let the node initialized
        rospy.sleep(2)

        # Home using relaxedIK
        relaxedik_publish([0, 0, 0], [1, 0, 0, 0])

        # Block until the motion is finished
        self.wait_motion_finished()

        print("\nHoming has finished.\n") 

        self.pid_vel_limit_srv(1.0)

        # Set the flag and finish initialization
        self.isInitialized = True

        print("\nSystem is ready.\n")

def tracking_sm():

    # Pressed
    if controller.isTracking_state == 0 and controller.right_controller['gripButton'] == True:

        controller.isTracking_state = 1

    # Released
    elif controller.isTracking_state == 1 and controller.right_controller['gripButton'] == False:

        # Activate tracking mode
        controller.isTracking = True
        controller.isTracking_state = 2
        if system.isInitialized: print("\nTracking is ON.\n")

    # Pressed
    elif controller.isTracking_state == 2 and controller.right_controller['gripButton'] == True:

        controller.isTracking_state = 3

    # Released
    elif controller.isTracking_state == 3 and controller.right_controller['gripButton'] == False:

        # Deactivate tracking mode
        controller.isTracking = False
        controller.isTracking_state = 0
        if system.isInitialized: print("\nTracking is OFF.\n")

def tracking():

    if system.isInitialized:

        tracking_sm()
        controller.gripper_sm()

        # Start tracking if gripButton was pressed
        if controller.isTracking:

            # On first press recalculate transition (difference) from oculus to 
            if controller.onTrackingStart:

                controller.oculus_kinova_diff_pos = calculate_controller_ee_diff(controller.input_pos_gcs, controller.target_pos)

                # Remove the flag
                controller.onTrackingStart = False

            # Target position for relaxedIK: oculus input in KCS + differences
            controller.target_pos = controller.input_pos_gcs - controller.oculus_kinova_diff_pos

            relaxedik_publish(controller.target_pos, [1, 0, 0, 0])
            #relaxed_ik_publish(target_pos_kcs, target_rot_kcs)

        # Stop tracking if gripButton was released
        else:
            # Reset the flag
            controller.onTrackingStart = True

        if controller.right_controller['triggerButton']:
            controller.gripperButtonState = True
            
            # Call gripper state machine
            controller.gripper_sm()

            controller.gripperButtonReleased = False

        else:
            controller.gripperButtonState = False
            
            # Call gripper state machine
            controller.gripper_sm()

            controller.gripperButtonReleased = True

if __name__ == '__main__':
            
    Arm = Kinova()
    controller = Headset()
    system = Kinova_Oculus_control()

    # Initialize the robot
    system.initialization() 

    # Open the gripper
    controller.gripper_control(3, 0.0)
    
    # Main loop
    while not rospy.is_shutdown(): 
        
        tracking()
