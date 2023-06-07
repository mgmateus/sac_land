import numpy as np
import os

from std_msgs.msg import *
from sensor_msgs.msg import *
from nav_msgs.msg import *
from airsim_ros_pkgs.msg import *
from airsim_ros_pkgs.srv import *

from drone import Hydrone
from simulation import Simulation

START_POSITION = [-5, 5, 55, 0] #Posicao inicial antes do loop
RANGE_X = [-19, 19] #Variacao maxima em X
RANGE_Y = [-21, 21] #variacao maxima em Y
RANGE_Z_AIR = [55, 75]  #Variacao maxima em Z no ar
RANGE_Z_WATER = [75, 88] #Variacao maxima em Z na agua

FREQ = 1 #Frequencia de episodeos que deve haver modificacao na simulacao

AIR = 1.0  #Percentual de spawns no ar
WATER = 0  #Percentual de spawns na agua
LAND = 0   #Percentual de spawns no pousado 


class Env:
    def __init__(self, space_state_dim, space_action_dim, max_steps, max_eps) -> None:
        

        self.__current_step = 0                     #Define o episodeo atual     
        self.__max_steps = max_steps
        self.__abs_path = os.path.dirname(os.path.realpath(__file__)).replace("scripts", "log")

        self.__simulation = Simulation(START_POSITION)
        self.__huauv = Hydrone()

        self.__simulation.set_ranges(RANGE_X, RANGE_Y, RANGE_Z_AIR, RANGE_Z_WATER)
        self.__simulation.set_modes(max_eps, FREQ, AIR, WATER, LAND)

    @property
    def simulation(self):
        return self.__simulation
    
    @property
    def huauv(self):
        return self.__huauv
    

        



