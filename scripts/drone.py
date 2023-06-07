import rospy
import os
import cv2

from std_msgs.msg import *
from sensor_msgs.msg import *
from nav_msgs.msg import *
from airsim_ros_pkgs.msg import *

from airsim_ros_pkgs.srv import *

from sac_land_pkg.image_processing import ImageProcessing



class Hydrone:
    def __init__(self) -> None:
        rospy.Subscriber("/airsim_node/Hydrone/Stereo_Cam/Scene", Image, self.__call_stereo_rgb)
        rospy.Subscriber("/airsim_node/Hydrone/Stereo_Cam/DepthPerspective", Image, self.__call_stereo_depth)
        #rospy.Subscriber("/airsim_node/Hydrone/DepthMap/DepthVis", Image, self.__call_depth)
        rospy.Subscriber("/airsim_node/Hydrone/Segmentation_Image/Segmentation", Image, self.__call_segmentation)
        rospy.Subscriber("/airsim_node/Hydrone/Bottom/Scene", Image, self.__call_bottom)


        self.__pub_info = rospy.Publisher("drone_info", String, queue_size=10)

        self.__img_process = ImageProcessing()

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
        info = "Bottom cam ok!"
        if data:
            info = "Bottom cam is ok!"
            cv_segmentation = self.__img_process.image_transport(data)
            self.__img_process.store_images("bottom", cv_segmentation)
        else:
            info = "Error in Bottom cam!"
        self.__pub_info.publish(info)

    def __str__(self) -> str:
        return 'Hydrone'


