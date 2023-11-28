#!/usr/bin/env python
import rospy
import numpy as np
import tf

from std_msgs.msg import (Bool, Float64MultiArray)
from geometry_msgs.msg import (Pose, Point)
from gopher_ros_clearcore.msg import (Position)


class WorkspaceCameraFeedback:

    def __init__(
        self,
        anchor_id,
    ):

        self.listener = tf.TransformListener()
        self.br = tf.TransformBroadcaster()
        self.rate = rospy.Rate(10)

        self.anchor_id = anchor_id

        self.__tf_toolframe_anchor = rospy.Publisher(
            f'/{self.ROBOT_NAME}/tf_toolframe',
            Point,
            queue_size=1,
        )

        rospy.Subscriber(
            f'/{self.ROBOT_NAME}/calibrate_anchor',
            Point,
            self.__calibration_anchor_callback,
        ),

    def main_loop(self):
        self.rate.sleep()


if __name__ == '__main__':

    rospy.init_node('tf_updates',)

    anchor_id = rospy.get_param(param_name=f'{rospy.get_name()}/anchor_id',)

    workspacecam = WorkspaceCameraFeedback(anchor_id=anchor_id,)

    while not rospy.is_shutdown():
        workspacecam.main_loop()
