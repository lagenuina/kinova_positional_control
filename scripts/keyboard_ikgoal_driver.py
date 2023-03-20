#!/usr/bin/python

import numpy as np
import readchar
import rospy
from geometry_msgs.msg import PoseStamped, Vector3Stamped, QuaternionStamped, Pose
from std_msgs.msg import Bool
from relaxed_ik_ros1.msg import EEPoseGoals
import transformations as T
import kinova_positional_control.srv as posctrl_srv
from utils import relaxedik_publish

# Callback function that updates motionFinished flag
def pid_motion_finished_callback(data):
    """Callback function for the '/pid/motion_finished' topic that updates the
    'motionFinished' flag variable.
    """
    global motionFinished
    
    motionFinished = data.data
    
    
def wait_motion_finished():
    """Block code execution until the 'motionFinished' flag is set or the ROS node
    is shutdown.
    """
    global motionFinished
    
    # Allow motion to start
    rospy.sleep(1)

    # Block code execution
    while not motionFinished and not rospy.is_shutdown():
        pass


if __name__ == '__main__':
    
    rospy.init_node('keyboard_ikgoal_driver')
    
    #Publishers
    ee_pose_goals_pub = rospy.Publisher('/relaxed_ik/ee_pose_goals', EEPoseGoals, queue_size=5)
    quit_pub = rospy.Publisher('/relaxed_ik/quit',Bool,queue_size=5)

    # Service
    pid_vel_limit_srv = rospy.ServiceProxy('pid_vel_limit', posctrl_srv.pid_vel_limit)

    # Subscribers
    rospy.Subscriber('/pid/motion_finished', Bool, pid_motion_finished_callback)
        
    # Set 20% velocity
    pid_vel_limit_srv(0.2)

    # Homing position
    print("\nHoming has started...\n")

    # Let the node initialized
    rospy.sleep(2)

    # Home using relaxedIK
    relaxedik_publish([0, 0, 0], [1, 0, 0, 0])

    # Block until the motion is finished
    wait_motion_finished()

    print("\nHoming has finished.\n") 

    pid_vel_limit_srv(1.0)

    print("\nSystem is ready.\n")

    pos_stride = 0.015
    rot_stride = 0.055

    position_r = [0,0,0]
    rotation_r = [1,0,0,0]

    position_l = [0,0,0]
    rotation_l = [1,0,0,0]

    seq = 1
    rate = rospy.Rate(1000)
    
    while not rospy.is_shutdown():

        print("Pos R: {}, Pos L: {}".format(position_r, position_l))

        key = readchar.readkey()
        if key == 'w':
            position_r[0] += pos_stride
        elif key == 'x':
            position_r[0] -= pos_stride
        elif key == 'a':
            position_r[1] += pos_stride
        elif key == 'd':
            position_r[1] -= pos_stride
        elif key == 'q':
            position_r[2] += pos_stride
        elif key == 'z':
            position_r[2] -= pos_stride
        elif key == '1':
            euler = list(T.euler_from_quaternion(rotation_r))
            euler[0] += rot_stride
            rotation_r = T.quaternion_from_euler(euler[0],euler[1],euler[2])
        elif key == '2':
            euler = list(T.euler_from_quaternion(rotation_r))
            euler[0] -= rot_stride
            rotation_r = T.quaternion_from_euler(euler[0],euler[1],euler[2])
        elif key == '3':
            euler = list(T.euler_from_quaternion(rotation_r))
            euler[1] += rot_stride
            rotation_r = T.quaternion_from_euler(euler[0],euler[1],euler[2])
        elif key == '4':
            euler = list(T.euler_from_quaternion(rotation_r))
            euler[1] -= rot_stride
            rotation_r = T.quaternion_from_euler(euler[0],euler[1],euler[2])
        elif key == '5':
            euler = list(T.euler_from_quaternion(rotation_r))
            euler[2] += rot_stride
            rotation_r = T.quaternion_from_euler(euler[0],euler[1],euler[2])
        elif key == '6':
            euler = list(T.euler_from_quaternion(rotation_r))
            euler[2] -= rot_stride
            rotation_r = T.quaternion_from_euler(euler[0],euler[1],euler[2])

        elif key == 'i':
            position_l[0] += pos_stride # left
        elif key == 'm':
            position_l[0] -= pos_stride
        elif key == 'j':
            position_l[1] += pos_stride
        elif key == 'l':
            position_l[1] -= pos_stride
        elif key == 'u':
            position_l[2] += pos_stride
        elif key == 'n':
            position_l[2] -= pos_stride
        elif key == '=':
            euler = list(T.euler_from_quaternion(rotation_l))
            euler[0] += rot_stride
            rotation_l = T.quaternion_from_euler(euler[0],euler[1],euler[2])
        elif key == '-':
            euler = list(T.euler_from_quaternion(rotation_l))
            euler[0] -= rot_stride
            rotation_l = T.quaternion_from_euler(euler[0],euler[1],euler[2])
        elif key == '0':
            euler = list(T.euler_from_quaternion(rotation_l))
            euler[1] += rot_stride
            rotation_l = T.quaternion_from_euler(euler[0],euler[1],euler[2])
        elif key == '9':
            euler = list(T.euler_from_quaternion(rotation_l))
            euler[1] -= rot_stride
            rotation_l = T.quaternion_from_euler(euler[0],euler[1],euler[2])
        elif key == '8':
            euler = list(T.euler_from_quaternion(rotation_l))
            euler[2] += rot_stride
            rotation_l = T.quaternion_from_euler(euler[0],euler[1],euler[2])
        elif key == '7':
            euler = list(T.euler_from_quaternion(rotation_l))
            euler[2] -= rot_stride
            rotation_l = T.quaternion_from_euler(euler[0],euler[1],euler[2])
        elif key == 'q':
            q = Bool()
            q.data = True
            quit_pub.publish(q)
        elif key == 'c':
            rospy.signal_shutdown()

        target_pos = np.array([position_r[0], position_r[1], position_r[2]])
    
        relaxedik_publish(target_pos, [1, 0, 0, 0])

        q = Bool()
        q.data = False
        quit_pub.publish(q)
