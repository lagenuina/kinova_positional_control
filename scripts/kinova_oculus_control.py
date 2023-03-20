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
        """Initializes the Kinova Oculus Control class, including ROS node,
        flag variables, ROS publishers and subscribers, and ROS service proxies.
        """

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

        # Publishers
        self.angles_pub = rospy.Publisher('/relaxed_ik/joint_angle_solutions', JointAngles, queue_size=10)
        self.relaxed_ik_target_wcs_pub = rospy.Publisher('/relaxed_ik/position_wcs', Float32MultiArray, queue_size=1)
        self.relaxed_ik_target_gcs_pub = rospy.Publisher('/relaxed_ik/position_gcs', Float32MultiArray, queue_size=1)

        # Subscribers
        rospy.Subscriber('/pid/motion_finished', Bool, self.pid_motion_finished_callback)

        # Services
        self.pid_setpoint_srv = rospy.ServiceProxy('pid_setpoint', posctrl_srv.pid_setpoint)
        self.pid_vel_limit_srv = rospy.ServiceProxy('pid_vel_limit', posctrl_srv.pid_vel_limit)
        self.stop_arm_srv = rospy.ServiceProxy('my_gen3/base/stop', Stop)

    def pid_motion_finished_callback(self, data):
        """Callback function for the '/pid/motion_finished' topic that updates the
        'motionFinished' flag variable.
        """
        
        self.motionFinished = data.data

    def wait_motion_finished(self):
        """Blocks code execution until the 'motionFinished' flag is set or the ROS node
        is shutdown.
        """
        
        # Allow motion to start
        rospy.sleep(1)

        # Block code execution
        while not self.motionFinished and not rospy.is_shutdown():
            pass

    def roslaunch_pid(self):
        """Launches the 'arm_controller.launch' file from the 'kinova_pid' package. 
        This function initializes the Kinova PID controller node.
        """
        
        uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
        roslaunch.configure_logging(uuid)
        self.pid_launch = roslaunch.parent.ROSLaunchParent(uuid, ['/home/fetch/catkin_workspaces/oculus_relaxedik_ws/src/kinova_pid/launch/arm_controller.launch'])
        self.pid_launch.start()

    def roslaunch_relaxed_ik(self):
        """Launches the 'relaxed_ik.launch' file from the 'relaxed_ik_ros1' package. 
        This function initializes the RelaxedIK node.
        """
        
        uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
        roslaunch.configure_logging(uuid)
        self.relaxed_ik_launch = roslaunch.parent.ROSLaunchParent(uuid, ['/home/fetch/catkin_workspaces/oculus_relaxedik_ws/src/relaxed_ik_ros1/launch/relaxed_ik.launch'])
        self.relaxed_ik_launch.start()

    def node_shutdown(self):
        """This function is called when the node is shutting down. It stops the arm motion and shuts down the PID and 
        relaxed IK nodes.
        """
        
        print("\nNode is shutting down...")

        # Stop arm motion by calling the stop_arm_srv() service
        self.stop_arm_srv()

        # Shutdown PID and relaxed IK nodes
        self.pid_launch.shutdown()
        self.relaxed_ik_launch.shutdown()

        print("\nNode has shut down.")
            
    def initialization(self):
        """Initializes the system by homing the robotic arm, with velocity limit set to 20%, 
        to a pre-specified initial configuration (starting_config in kortex_info.yaml) with relaxedIK.
        """
        
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

        # Set the flag to indicate that the system has been initialized
        self.isInitialized = True

        print("\nSystem is ready.\n")

def tracking_sm():
    """State machine for tracking.
    """
    
    # If the grip Button is pressed
    if controller.isTracking_state == 0 and controller.right_controller['gripButton'] == True:

        # Set state to indicate that the button is pressed
        controller.isTracking_state = 1

    # If the grip Button is released after being pressed
    elif controller.isTracking_state == 1 and controller.right_controller['gripButton'] == False:

        # Activate tracking mode
        controller.isTracking = True
        controller.isTracking_state = 2
        
        # Print message to indicate that tracking is on
        if system.isInitialized: print("\nTracking is ON.\n")

    # If the grip Button is pressed again while tracking is on
    elif controller.isTracking_state == 2 and controller.right_controller['gripButton'] == True:

        # Set state to indicate that the button is pressed again
        controller.isTracking_state = 3

    # If the grip Button is released after being pressed again
    elif controller.isTracking_state == 3 and controller.right_controller['gripButton'] == False:

        # Deactivate tracking mode
        controller.isTracking = False
        controller.isTracking_state = 0
        
        # Print message to indicate that tracking is off
        if system.isInitialized: print("\nTracking is OFF.\n")

def tracking():
    """Publishes the desired end-effector position and orientation to relaxedIK and provides gripper control.
    It tracks the target position by calculating the difference between the oculus input and controller compensation.
    """
    
    if system.isInitialized:

        # Call the state machine for tracking
        tracking_sm()
        
        # Call the state machine for the gripper control
        controller.gripper_sm()

        # If tracking mode is on
        if controller.isTracking:

            # On first press recalculate transition (difference) from oculus input to target position of kinova end-effector
            if controller.onTrackingStart:

                controller.oculus_kinova_diff_pos = calculate_controller_ee_diff(controller.input_pos_gcs, controller.target_pos)

                # Remove the flag
                controller.onTrackingStart = False

            # Target position for relaxedIK: oculus input in GCS (global coordinate system) + difference previously calculated
            controller.target_pos = controller.input_pos_gcs - controller.oculus_kinova_diff_pos

            # Publish the target position to relaxedIK
            relaxedik_publish(controller.target_pos, [1, 0, 0, 0])

        # If tracking mode is off
        else:
            
            # Reset the flag
            controller.onTrackingStart = True

        # Control the gripper if the trigger button is pressed
        if controller.right_controller['triggerButton']:
            
            controller.gripperButtonState = True
            
            # Call gripper state machine
            controller.gripper_sm()

            controller.gripperButtonReleased = False

        # If the trigger button is released
        else:
            controller.gripperButtonState = False
            
            # Call gripper state machine
            controller.gripper_sm()

            controller.gripperButtonReleased = True

if __name__ == '__main__':
            
    controller = Headset()
    system = Kinova_Oculus_control()

    # Initialize the robot to home position
    system.initialization() 

    # Open the gripper
    controller.gripper_control(3, 0.0)
    
    while not rospy.is_shutdown(): 
        
        tracking()