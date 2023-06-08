import numpy as np
import os

from std_msgs.msg import *
from sensor_msgs.msg import *
from nav_msgs.msg import *
from airsim_ros_pkgs.msg import *
from airsim_ros_pkgs.srv import *

from drone import Hydrone
from simulation import Simulation

START_POSITION = [-10, -10, 45, 0] #Posicao inicial antes do loop
RANGE_X = [-19, 19] #Variacao maxima em X
RANGE_Y = [-21, 21] #variacao maxima em Y

FREQ = 1 #Frequencia de episodeos que deve haver modificacao na simulacao


class Env:
    def __init__(self, state_dim, action_dim, max_steps, max_eps) -> None:
        self.__observation_space = np.zeros(shape=(state_dim,))
        self.__action_dim = action_dim

        self.__n_steps = 0
        self.__max_steps = max_steps
        self.__past_distance = None

        self.__current_step = 0                     #Define o episodeo atual     
        self.__max_steps = max_steps
        self.__abs_path = os.path.dirname(os.path.realpath(__file__)).replace("scripts", "log")

        self.__simulation = Simulation(START_POSITION)
        self.__huauv = Hydrone()

        self.__simulation.set_ranges_to_target_pose(RANGE_X, RANGE_Y)
        self.__simulation.freq = FREQ

    @property
    def simulation(self):
        return self.__simulation
    
    @property
    def huauv(self):
        return self.__huauv

    def reset(self):
        self.__simulation.reset_vehicle_pose()
        self.__n_steps = 0
        _ = self.__huauv.get_state(np.zeros(shape=(self.__action_dim,)))
        
        return self.__observation_space
    

        



