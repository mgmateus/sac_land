#!/usr/bin/env python3

import rospy
import time
import os

from environment import Env

from std_msgs.msg import *
from sensor_msgs.msg import *
from nav_msgs.msg import *
from airsim_ros_pkgs.msg import *
from airsim_ros_pkgs.srv import *

# Componentes que vao para treinamento e teste
MAX_EPS = 5           #Maximo de episodeos
CURRENT_EP = 0        #Episodeo atual
MAX_STEPS = 0         #Maximo de steps em um episodeo    


SPACE_STATE_DIM = 0     #Tamanho do espaco de estados
SPACE_ACTION_DIM = 0    #Tamanho do espaco de acao
PATH = os.path.dirname(os.path.realpath(__file__)).replace("scripts", "log")

if __name__ =='__main__':
    rospy.init_node("simulation", anonymous=False)
    
    env = Env(SPACE_STATE_DIM, SPACE_ACTION_DIM, MAX_STEPS, MAX_EPS)
    #env.huauv.img_process.save_stored_images(PATH, 'rgb')

    rospy.spin()