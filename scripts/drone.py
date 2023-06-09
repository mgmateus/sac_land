import rospy
import os
import cv2

import numpy as np

from std_msgs.msg import *
from sensor_msgs.msg import *
from nav_msgs.msg import *
from airsim_ros_pkgs.msg import *

from airsim_ros_pkgs.srv import *

from vision_briedge.image_processing import ImageProcessing

LINEAR_VEL = (0.0, 0.5)
ANGULAR_VEL = (-0.25, 0.25)


class Hydrone:
    def __init__(self) -> None:
        rospy.Subscriber("/airsim_node/Hydrone/Bottom/Scene", Image, self.__call_bottom)
        rospy.Subscriber("/yolov7/yolov7/visualization", Image, self.__call_inference)

        self.__vel_pub = rospy.Publisher("/airsim_node/Hydrone/vel_cmd_world_frame", VelCmd, queue_size=1)
        self.__info_pub = rospy.Publisher("drone_info", String, queue_size=10)

        self.__img_process = ImageProcessing()
        self.__vel = VelCmd()

    @property
    def img_process(self):
        return self.__img_process

    def __call_bottom(self, data):
        if data:
            cv_bottom = self.__img_process.image_transport(data)
            cv2.imshow("Cam", cv_bottom)
            cv2.waitKey(0)
        else:
            info = "Error in Bottom cam!"
            self.__info_pub.publish(info)

    def __call_inference(self, data):
        if data:
            rospy.logwarn('Inference')
            cv_inference = self.__img_process.image_transport(data)
            cv2.imshow("Inference", cv_inference)
            cv2.waitKey(0)
        else:
            info = "Error in Bottom cam!"
            self.__info_pub.publish(info)

    

    def get_state(self, action):
        self.__vel.twist.linear.x = np.clip(action[0], LINEAR_VEL[0], LINEAR_VEL[1])
        self.__vel.twist.angular.z = np.clip(action[1], ANGULAR_VEL[0], ANGULAR_VEL[1])
        self.__vel_pub.publish()

    def __str__(self) -> str:
        return 'Hydrone'


