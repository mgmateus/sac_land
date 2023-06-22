#!/usr/bin/env python3
import cv2
import rospy
from datetime import datetime

from cv_bridge import CvBridge, CvBridgeError


# Processa e faz um "storage" de imagens
class ImageProcessing:
    #Converte a mensagem de imagem ros para opencv
    @staticmethod
    def image_transport(img_msg):
        try:
            return CvBridge().imgmsg_to_cv2(img_msg, "passthrough")

        except CvBridgeError as e:
            rospy.logerr("CvBridge Error: {0}".format(e))

    def __init__(self, dir):
        self.__dir = dir

        self._rgb = []
        self._depth = []
        self._segmentation = []

    @property
    def rgb(self):
        return self._rgb
    
    @property
    def depth(self):
        return self._depth
    
    @property
    def segmentation(self):
        return self._segmentation
    
    @rgb.setter
    def rgb(self, store):
        self._rgb = store
    
    @depth.setter
    def depth(self, store):
        self._depth = store

    @segmentation.setter
    def segmentation(self, store):
        self._segmentation = store

    #Armazena as imagens em seus respectivos vetores (rgb, depth e segmentation)
    def store_images(self, type, img, store_size=3):            
        if len(self.__getattribute__(type)) == 0:     #Se o vetor eestiver vazio, replica a imagem 3x
            self.__setattr__(type, store_size * [img])

        #Reorganiza como uma fila onde [0] e o frame mais recente
        else:
            imgs = self.__getattribute__(type)
            imgs.insert(0, img)
            del imgs[-1]

            self.__setattr__(type, None) #Limpa o vetor de imagens
            self.__setattr__(type, imgs) #Redefine com a nova fila

    def save_stored_images(self, path, type):
        count = 0
        for img in self.__getattribute__(type):
            cv2.imwrite(path+"/"+type+"/t_"+str(count)+".png", img)
            count += 1
    
    def save_image(self, img, dir= "", name= ""):
        _name = str(datetime.now().strftime("%d/%m/%Y-%H:%M:%S")).replace("/", "_")
        _name = f"{(dir or self.__dir)}/{(name or _name)}.png"
        cv2.imwrite(_name, img)
            




