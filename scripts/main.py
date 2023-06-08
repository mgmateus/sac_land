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

def action_unnormalized(action, high, low):
    action = low + (action + 1.0) * 0.5 * (high - low)
    action = np.clip(action, low, high)
    return action

is_training = True

max_episodes  = 10001
max_steps   = 10000#1000
rewards     = []
batch_size  = 256#512

action_dim = 2
state_dim  = 545
hidden_dim = 1635

ACTION_V_MIN = 0.0 # m/s
ACTION_W_MIN = -0.25 # rad
ACTION_V_MAX = 0.5 # m/s
ACTION_W_MAX = 0.25 # rad

world = 'hull'
buffer_size = 1000000#50000

print(" ")
print("---------------------------------")
print('State Dimensions: ' + str(state_dim))
print('Action Dimensions: ' + str(action_dim))
print('Action Max: ' + str(ACTION_V_MAX) + ' m/s and ' + str(ACTION_W_MAX) + ' rad')
print("---------------------------------")
print(" ")

if __name__ =='__main__':
    rospy.init_node("simulation", anonymous=False)
    status_pub = rospy.Publisher("status", String, queue_size=10)
    reward_pub = rospy.Publisher("reward", String, queue_size=10)
    
    env = Env(state_dim, action_dim, max_steps, max_episodes)
    time.sleep(15)
    env.reset()
    time.sleep(15)
    env.reset()
    #env.simulation.random_target_quadrant_pose()
    #env.huauv.img_process.save_stored_images(PATH, 'rgb')

    rospy.spin()
