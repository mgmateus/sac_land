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
        rospy.Subscriber("/airsim_node/Hydrone/Stereo_Cam/Scene", Image, self.__call_stereo_rgb)
        rospy.Subscriber("/airsim_node/Hydrone/Stereo_Cam/DepthPerspective", Image, self.__call_stereo_depth)
        #rospy.Subscriber("/airsim_node/Hydrone/DepthMap/DepthVis", Image, self.__call_depth)
        rospy.Subscriber("/airsim_node/Hydrone/Segmentation_Image/Segmentation", Image, self.__call_segmentation)
        rospy.Subscriber("/airsim_node/Hydrone/Bottom/Scene", Image, self.__call_bottom)

        self.__vel_pub = rospy.Publisher("/airsim_node/Hydrone/vel_cmd_world_frame", VelCmd, queue_size=1)
        self.__pub_info = rospy.Publisher("drone_info", String, queue_size=10)

        self.__img_process = ImageProcessing()
        self.__vel = VelCmd()

    @property
    def img_process(self):
        return self.__img_process
    
    def __call_stereo_rgb(self, data):
        if data:
            cv_rgb = self.__img_process.image_transport(data)
            self.__img_process.store_images("rgb", cv_rgb)
        else:
            info = "Error in RGB cam!"
            self.__pub_info.publish(info)

    def __call_stereo_depth(self, data):
        if data:
            cv_depth = self.__img_process.image_transport(data)
            self.__img_process.store_images("depth", cv_depth)
        else:
            info = "Error in Depth cam!"
            self.__pub_info.publish(info)

    def __call_segmentation(self, data):
        if data:
            cv_segmentation = self.__img_process.image_transport(data)
            self.__img_process.store_images("segmentation", cv_segmentation)
        else:
            info = "Error in Segmentation cam!"
            self.__pub_info.publish(info)

    def __call_bottom(self, data):
        if data:
            cv_bottom = self.__img_process.image_transport(data)
        else:
            info = "Error in Bottom cam!"
            self.__pub_info.publish(info)

    def get_state(self, action):
        self.__vel.twist.linear.x = np.clip(action[0], LINEAR_VEL[0], LINEAR_VEL[1])
        self.__vel.twist.angular.z = np.clip(action[1], ANGULAR_VEL[0], ANGULAR_VEL[1])
        self.__vel_pub.publish()

    def __str__(self) -> str:
        return 'Hydrone'


