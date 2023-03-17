import rospy
import numpy as np
import transformations as T
import math
from kortex_driver.msg import *
from kortex_driver.srv import *
from relaxed_ik_ros1.msg import EEPoseGoals
import geometry_msgs.msg as geom_msgs

# Publishing
setpoint_pub = rospy.Publisher('/relaxed_ik/ee_pose_goals', EEPoseGoals, queue_size=1)

# Publish target position and orientation in relaxed IK Coordinate system (Kinova)
def relaxedik_publish(target_position, target_orientation):

    # Form a message for relaxedIK (right arm)
    pose_r = geom_msgs.Pose()
    pose_r.position.x = target_position[0]
    pose_r.position.y = target_position[1]
    pose_r.position.z = target_position[2]

    pose_r.orientation.w = target_orientation[0]
    pose_r.orientation.x = target_orientation[1]
    pose_r.orientation.y = target_orientation[2]
    pose_r.orientation.z = target_orientation[3] 

    # TODO: Form a message for relaxedIK (left arm)
    pose_l = geom_msgs.Pose()
    pose_l.position.x = 0
    pose_l.position.y = 0
    pose_l.position.z = 0

    pose_l.orientation.w = 1
    pose_l.orientation.x = 0
    pose_l.orientation.y = 0
    pose_l.orientation.z = 0  

    # Form full message for relaxedIK
    ee_pose_goals = EEPoseGoals()
    ee_pose_goals.ee_poses.append(pose_r)
    ee_pose_goals.ee_poses.append(pose_l)
    ee_pose_goals.header.seq = 0

    # Publish
    setpoint_pub.publish(ee_pose_goals)

def left_to_right_handed(input_rot_gcs):
    
    input_rot_gcs = T.euler_from_quaternion(input_rot_gcs)
    input_rot_gcs = T.euler_matrix(-1 * input_rot_gcs[1], -1 * input_rot_gcs[2], input_rot_gcs[0])
    input_rot_gcs = T.quaternion_from_matrix(input_rot_gcs)
    
    return input_rot_gcs

def calculate_controller_ee_diff(input_pos_gcs, target_pos):
    
    return input_pos_gcs - target_pos

def global_to_kinova(input_pos_gcs):
    
    # Transition from Global CS to Kinova CS: rotate around y and z axis
    xaxis, yaxis, zaxis = (1, 0, 0), (0, 1, 0), (0, 0, 1)
    Rx = T.rotation_matrix(math.radians(0.0), xaxis)
    Ry = T.rotation_matrix(math.radians(-45.0), yaxis)
    Rz = T.rotation_matrix(math.radians(90.0), zaxis)

    R_gcs_to_kcs = T.concatenate_matrices(Rx, Ry, Rz)

    if len(input_pos_gcs) == 3:
        input_kcs = np.matmul(R_gcs_to_kcs[0:3,0:3], input_pos_gcs)

    elif len(input_pos_gcs) == 4:
        input_kcs = np.matmul(R_gcs_to_kcs, input_pos_gcs)

    return input_kcs