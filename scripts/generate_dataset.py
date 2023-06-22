#!/usr/bin/env python3

import rospy
import os
import time

from hydrone import UAV
from simulation import Simulation

DIR_LOG = os.path.dirname(os.path.realpath(__file__)).replace("scripts", "log/dataset")
START_POSITION = [0, 0, 65, 0]




if __name__ =='__main__':
    rospy.init_node("detection_generate_dataset", anonymous=False)

    visualize_cam = bool(rospy.get_param("~visualize_cam"))

    path = [[5, 5, 0, 0],[-5, 5, 0, 0],[0, -5, 5, 0]]

    uav = UAV(DIR_LOG)
    uav.visualize_cam = visualize_cam

    simu = Simulation(START_POSITION)

    
    for position in path:
        x, y, z, yaw = position
        uav.goto(x, y, z, yaw)
        time.sleep(10)

    rospy.spin()
