#-*- coding:utf-8 -*-
# @time: 
# @author:张新新
# @email: 1262981714@qq.com

from robot import robot_base
from socket import socket
import transforms3d
import numpy as np
import time
class robot(robot_base.robot):
    def __init__(self,port):
        self.port = port
        self.socket = socket()
        self.socket.connect(("127.0.0.1", self.port))
    def move(self,pose):
        while(True):
            try:
                order = "move"
                self.socket.send(order.encode())
                q = transforms3d.quaternions.mat2quat(pose[:3,:3])
                t = pose[:3,3]
                self.socket.send("{},{},{},{},{},{},{}".format(t[0],t[1],t[2],q[0],q[1],q[2],q[3]).encode())
                recv = self.socket.recv(1024).decode()
                break
            except Exception:
                a = input("please restart robot and input")
                self.socket.close()
                self.socket = socket()
                self.socket.connect(("127.0.0.1", self.port))

        if recv == "False" or recv == "":
            return False, None

        pose_str_list = recv.split(',')
        flag = pose_str_list[0]
        t1 = np.array([float(pose_str_list[1]),float(pose_str_list[2]),float(pose_str_list[3])])
        q1 = np.array([float(pose_str_list[4]),float(pose_str_list[5]),float(pose_str_list[6]),float(pose_str_list[7])])
        r = transforms3d.quaternions.quat2mat(q1)
        pose = np.identity(4)
        pose[:3,:3] = r[:,:]
        pose[:3,3] = t1[:]
        time.sleep(5)
        return True,pose

    def moveable(self,pose):
        t = pose[:3, 3]
        distance = np.linalg.norm(t)
        xy_distance  = np.linalg.norm(t[:2])
        x = t[0]
        y = t[1]
        z = t[2]

        if distance > 0.8:
            return False
        if xy_distance <0.2:
            return False
        return True

    def realease(self):
        #self.socket.connect(("127.0.0.1", 1024))
        order = "end"
        self.socket.send(order.encode())
        self.socket.close()


    def get_pose(self):
        order = "cur"
        self.socket.send(order.encode())
        recv = self.socket.recv(1024).decode()
        if recv == "False" or recv == '':
            return False, None

        pose_str_list = recv.split(',')
        flag = pose_str_list[0]
        t1 = np.array([float(pose_str_list[1]), float(pose_str_list[2]), float(pose_str_list[3])])
        q1 = np.array(
            [float(pose_str_list[4]), float(pose_str_list[5]), float(pose_str_list[6]), float(pose_str_list[7])])
        r = transforms3d.quaternions.quat2mat(q1)
        pose = np.identity(4)
        pose[:3, :3] = r[:, :]
        pose[:3, 3] = t1[:]
        time.sleep(5)
        return True, pose
    def build_board(self,x,y,z,r_w,r_x,r_y,r_z,width,height):
        order = "board"
        self.socket.send(order.encode())
        self.socket.send("{},{},{},{},{},{},{},{},{}".format(x, y, z, r_w,r_x,r_y,r_z,width,height).encode())
        time.sleep(2)


