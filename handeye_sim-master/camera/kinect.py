#-*- coding:utf-8 -*-
# @time: 
# @author:张新新
# @email: 1262981714@qq.com
from camera import camera_base
from socket import socket
from PIL import Image
import numpy as np
import sys
ros_cv2_path = '/opt/ros/kinetic/lib/python2.7/dist-packages'
if ros_cv2_path in sys.path:
    sys.path.remove(ros_cv2_path)
    import cv2
    sys.path.append(ros_cv2_path)
else:
    import cv2
import os
class Kinect(camera_base.camera):
    def __init__(self,port,config):
        self.port = port
        self.socket = socket()
        self.temp_color_file_name = "../temp/rgb.bmp"
        self.temp_depth_file_name = "../temp/depth.png"
        self.imgsize = (1920, 1080)
        fs2 = cv2.FileStorage(config, cv2.FileStorage_READ)
        self.intrinsic = fs2.getNode("intrinsic").mat()
        self.dist = fs2.getNode("dist").mat()
        fs2.release()
        self.socket.connect(("127.0.0.1", self.port))
    def get_rgb_image(self):
        while(True):
            try:
                self.socket.send("capture".encode())
                flag = self.socket.recv(1024).decode()
                if flag == 'True':
                    image = cv2.imread(self.temp_color_file_name)
                    os.remove(self.temp_color_file_name)
                    return True, image
            except Exception:
                flag = "False"
                print("fail to connect")
                self.socket.close()
                a = input("connect right then put")
                self.socket = socket()
                self.socket.connect(("127.0.0.1", self.port))
    def get_rgb_depth_image(self):
        while (True):
            try:
                self.socket.send("capture".encode())
                flag = self.socket.recv(1024).decode()
                if flag == 'True':
                    image = cv2.imread(self.temp_color_file_name)
                    os.remove(self.temp_color_file_name)
                    depth_image = Image.open(self.temp_depth_file_name)
                    depth_image = np.array(depth_image)
                    os.remove(self.temp_depth_file_name)
                    # img = Image.fromarray(depth_image)
                    # img.save("../temp/test_save_depth.png")
                    return True, image,depth_image
            except Exception:
                print("fail to connect")
                self.socket.close()
                a = input("connect right then put")

                try:
                    self.socket = socket()
                    self.socket.connect(("127.0.0.1", self.port))
                except Exception:
                    print("error")
                    continue


    def realease(self):
        self.socket.send("end".encode())
        self.socket.close()
        return



