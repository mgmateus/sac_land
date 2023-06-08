#!/usr/bin/env python3

import airsim
import rospy
import os
import random
import numpy as np

from std_msgs.msg import String


class ConnectionUe4:
    def __init__(self, host= '172.18.0.3'):
        '''
        For set ip address from host, use ipconfig result's on host
        For set ip address from docker network, use a ip from ping result's between containers on host
        For set ip address from WSL, os.environ['WSL_HOST_IP'] on host
        '''
        self._client = airsim.MultirotorClient(host)
        self._client.confirmConnection()
        rospy.logwarn(f"Connection: {self._client.ping()}")
        
    @property
    def client(self):
        """The client property."""
        return self._client

    def restart_connection(self): 
        self._client.reset()

class Simulation(ConnectionUe4):
    def __init__(self, start_position):
        ConnectionUe4.__init__(self)
        self.__modes = [] 
        self.__pub_info = rospy.Publisher("simulation_info", String, queue_size=10)
        
        self._start_position = start_position
        self._freq = 0

    @property
    def start_position(self):
        return self._start_position
    
    @start_position.setter
    def start_position(self, pose):
        self._start_position = pose

    @property
    def freq(self):
        return self._freq

    @freq.setter
    def freq(self, value):
        self._freq = value

    def __set_quadrant(self, px, py):
        """
                 X
        --------------------
        |        |         |
        |   Q3   |    Q2   |                  
        |        |         |   
        -------(0,0)-------- Y              
        |        |         |   
        |   Q1   |    Q0   |   
        |        |         |   
        --------------------   

        """
        
        if px < 0 and py > 0:
            info = \
            """
                    X
            --------------------
            |        |         |
            |        |         |                  
            |        |         |   
            -------(0,0)-------- Y              
            |        |         |   
            |        |    Q0   |   
            |        |         |   
            --------------------   

            """
            self.__pub_info.publish(info)
            return -90, 0
        
        elif px < 0 and py < 0:
            info = \
            """
                    X
            --------------------
            |        |         |
            |        |         |                  
            |        |         |   
            -------(0,0)-------- Y              
            |        |         |   
            |   Q1   |         |   
            |        |         |   
            --------------------   

            """
            self.__pub_info.publish(info)
            return -90, 0
        
        elif px > 0 and py > 0:
            info = \
            """
                    X
            --------------------
            |        |         |
            |        |    Q2   |                  
            |        |         |   
            -------(0,0)-------- Y              
            |        |         |   
            |        |         |   
            |        |         |   
            --------------------   

            """
            self.__pub_info.publish(info)
            return -90, -180
        
        else: 
            info = \
            """
                    X
            --------------------
            |        |         |
            |   Q3   |         |                  
            |        |         |   
            -------(0,0)-------- Y              
            |        |         |   
            |        |         |   
            |        |         |   
            --------------------   

            """
            self.__pub_info.publish(info)
            return 90, 180

    def set_vehicle_pose(self, vehicle_name='', px=float(), py=float(), pz=float(), yaw=float()):
        #All elements in pose need to have the type float if is None
        pose = airsim.Pose() 
        
        pose.orientation = airsim.utils.to_quaternion(roll=np.deg2rad(0), pitch=np.deg2rad(0), yaw= np.deg2rad(yaw))
        pose.position.x_val = px
        pose.position.y_val = py
        pose.position.z_val = pz
        rospy.logwarn("Vehicle pose are setted: {} \n".format(self.client.simGetVehiclePose('Hydrone')))
        self.client.simSetVehiclePose(pose=pose, ignore_collision=True, vehicle_name=vehicle_name)
        
        info = "New Vehicle pose are setted: {} - ({:.2f} {:.2f} {:.2f})\n".format(vehicle_name, px, py, pz) 
        self.__pub_info.publish(info) 

    def set_object_pose(self, object_name='', px=float(), py=float(), pz=float(), yaw=float()):
        #All elements in pose need to have the type float if is None
        pose = airsim.Pose() 
        pose.orientation = airsim.utils.to_quaternion(roll=np.deg2rad(0), pitch=np.deg2rad(-90), yaw= np.deg2rad(yaw))
        pose.position.x_val = px 
        pose.position.y_val = py 
        pose.position.z_val = pz     

        self.client.simSetObjectPose(object_name, pose)  
        info = "New Object pose are setted: {} - ({:.2f} {:.2f} {:.2f})\n".format(object_name, px, py, pz) 
        self.__pub_info.publish(info) 

    def set_ranges_to_vehicle_pose(self, range_x, range_y, range_z_air, range_z_water=[]):
        """
                 X
        --------------------
        |        |         |
        |   Q3   |    Q2   |                  
        |        |         |   
        -------(0,0)-------- Y              
        |        |         |   
        |   Q1   |    Q0   |   
        |        |         |   
        --------------------   

        """
        self.__vehicle_pose_ranges = np.array([range_x, range_y, range_z_air, range_z_water]).T 

        
    def set_ranges_to_target_pose(self, range_x, range_y):
        """
                 X
        --------------------
        |        |         |
        |   Q3   |    Q2   |                  
        |        |         |   
        -------(0,0)-------- Y              
        |        |         |   
        |   Q1   |    Q0   |   
        |        |         |   
        --------------------   

        """
        self.__target_pose_ranges = np.array([range_x, range_y]).T

    def set_modes(self, num_eps, freq, air=1, water=0, land=0):
        self.__freq = freq
        normalize_modes = lambda num_eps, freq, mode, str_mode: round(num_eps/freq*mode)*[str_mode]
        self.__modes += normalize_modes(num_eps, freq, air, "air")
        self.__modes += normalize_modes(num_eps, freq, water, "water")
        self.__modes += normalize_modes(num_eps, freq, land, "land")

        self.__modes = list(filter(None, self.__modes))
        self.restart_connection()


    def random_vehicle_quadrant_pose(self, current_ep, vehicle_name="Hydrone", object_name="AUV", use_base= True, _3d= False):
        random.seed()
        if current_ep % self.__freq == 0:
            px = random.uniform(self.__vehicle_pose_ranges[0][0], self.__vehicle_pose_ranges[0][1])  
            py = random.uniform(self.__vehicle_pose_ranges[1][0], self.__vehicle_pose_ranges[1][1]) 
        
            a, b = self.__set_quadrant(px, py)
            yaw = random.uniform(a, b)

            if self.__modes[0] == "air":
                info = "Mode: air - 3D: {}".format(_3d)
                pz = random.uniform(self.__vehicle_pose_ranges[2][0], self.__vehicle_pose_ranges[2][1]) if _3d else self.__vehicle_pose_ranges[2][0]
                self.restart_connection()
                self.client.simDestroyObject(object_name) if not use_base else self.set_object_pose(object_name, px, py+13, 76)
                self.set_vehicle_pose(vehicle_name, px, py, pz, yaw)

            elif self.__modes[0] == "water":
                info = "Mode: water - 3D: {}".format(_3d)
                pz = random.uniform(self.__vehicle_pose_ranges[3][0], self.__vehicle_pose_ranges[3][1]) if _3d else self.__vehicle_pose_ranges[3][0]
                self.restart_connection()
                self.client.simDestroyObject(object_name) if not use_base else self.set_object_pose(object_name, px, py+13, 76)
                self.set_vehicle_pose(vehicle_name, px, py, pz, yaw)

            else:
                info = "Mode: land - 3D: {}".format(_3d)
                self.restart_connection()
                self.set_object_pose(object_name, px, py+13, 76)
                self.set_vehicle_pose(vehicle_name, px, py, 60, yaw)

            self.__pub_info.publish(info)


    def random_target_quadrant_pose(self, object_name="AUV"):
        random.seed()
        px = random.uniform(self.__target_pose_ranges[0][0], self.__target_pose_ranges[0][1])  
        py = random.uniform(self.__target_pose_ranges[1][0], self.__target_pose_ranges[1][1]) 
        a, b = self.__set_quadrant(px, py)
        yaw = random.uniform(a, b)

        self.restart_connection()
        self.set_object_pose(object_name, px, py, 76, yaw)

    def reset_vehicle_pose(self, vehicle_name='Hydrone'):
        self.restart_connection()
        x, y, z, yaw = self._start_position
        self.set_vehicle_pose(vehicle_name, x, y, z, yaw)

        info = "Position reseted: ({:.2f} {:.2f} {:.2f})\n".format(x, y, z) 
        self.__pub_info.publish(info)


