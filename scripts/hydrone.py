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


class UAV:
    def __init__(self, dir= '') -> None:

        #Subscribers
        rospy.Subscriber("/airsim_node/Hydrone/Bottom/Scene", Image, self.__call_bottom)

        #Publishers
        self.__info_pub = rospy.Publisher("vehicle_info", String, queue_size=10)

        self.__img_process = ImageProcessing(dir)
        self.__save_images = True if dir else False

        self._visualize_cam = False

    @property
    def visualize_cam(self) -> bool:
        return self._visualize_cam
    
    @visualize_cam.setter
    def visualize_cam(self, visualize):
        self._visualize_cam = visualize

    def __call_bottom(self, data): #Mudar para try excepet e definir as exceções
        if data:
            cv_bottom = self.__img_process.image_transport(data)

            if self.__save_images:
                self.__img_process.save_image(cv_bottom)

            if self._visualize_cam:
                cv2.imshow("Cam", cv_bottom)
                cv2.waitKey(3)
        else:
            info = "Error in Bottom cam!"
            self.__info_pub.publish(info)

    def goto(self, x, y, z, yaw, vehicle_name='Hydrone'):
        try:
            service = rospy.ServiceProxy("/airsim_node/local_position_goal", SetLocalPosition)
            rospy.wait_for_service("/airsim_node/local_position_goal")

            service(x, y, z, np.radians(yaw), vehicle_name)

        except rospy.ServiceException as e:
            print ('Service call failed: %s' % e)

    def __str__(self) -> str:
        return 'Hydrone' 


