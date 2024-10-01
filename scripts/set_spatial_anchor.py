#!/usr/bin/env python
import rospy
import numpy as np
from std_msgs.msg import Float32MultiArray
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import Point
from Scripts.srv import ConvertTargetPosition, SendFloat32MultiArray


class SpatialAnchor:

    def __init__(self, node_name, anchor_id):
        self.__BRIDGE = CvBridge()
        self.__NODE_NAME = node_name
        self.__image = None
        self.ANCHOR_ID = anchor_id
        self.__is_anchor_set = False

        self.__set_anchor_service = rospy.ServiceProxy(
            '/set_anchor', SendFloat32MultiArray
        )

        # Private CONSTANTS:
        aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_250)
        aruco_params = cv2.aruco.DetectorParameters()
        self.__ARUCO_DETECTOR = cv2.aruco.ArucoDetector(
            aruco_dict,
            aruco_params,
        )
        self.__CAMERA_MATRIX = np.array(
            [
                [605.3272705078125, 0.0, 312.21490478515625],
                [0.0, 605.3390502929688, 253.79823303222656],
                [0.0, 0.0, 1.0],
            ]
        )
        self.__DIST_COEFFS = np.array([0, 0, 0, 0, 0])
        self.__MARKER_SIZE = 0.03

        rospy.Subscriber(
            '/chest_cam/camera/color/image_raw', Image, self.__image_callback
        )

        self.RATE = rospy.Rate(10)

        rospy.sleep(20)

    def __image_callback(self, data):
        try:
            self.__image = self.__BRIDGE.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)

    def __detect_and_store(self):

        if self.__image is not None:
            corners, ids, _ = self.__ARUCO_DETECTOR.detectMarkers(self.__image)

            if ids is not None:

                for i in range(len(ids)):

                    if ids[i] == self.ANCHOR_ID:

                        rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(
                            corners[i], self.__MARKER_SIZE,
                            self.__CAMERA_MATRIX, self.__DIST_COEFFS
                        )

                        position_target = Float32MultiArray()
                        position_target.data = [
                            tvecs[0][0][0], tvecs[0][0][1],
                            tvecs[0][0][2] + 0.12
                        ]

                        if not self.__is_anchor_set:
                            self.__set_anchor_service(position_target)
                            self.__is_anchor_set = True
                            rospy.sleep(5)
                            rospy.loginfo(
                                f'\033[92mAnchor set. Ready to start!\033[0m'
                            )

    def main_loop(self):
        self.__detect_and_store()


def main():
    rospy.init_node('set_spatial_anchor', log_level=rospy.INFO)
    node_name = rospy.get_name()
    id = rospy.get_param(param_name=f'{rospy.get_name()}/anchor_id')
    object_tracker = SpatialAnchor(node_name=node_name, anchor_id=id)
    while not rospy.is_shutdown():
        object_tracker.main_loop()
        object_tracker.RATE.sleep()


if __name__ == "__main__":
    main()
