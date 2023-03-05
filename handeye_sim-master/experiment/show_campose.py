# _*_ coding:utf-8 _*_
# @time: 2020/10/8 下午6:53
# @author: 张新新
# @email: 1262981714@qq.com

from auto import init_handeye
from auto import score
from auto import simple_campose
from auto import select_simple_pose
from auto import auto_calibration

from Vrep import vrep_connect
from Vrep import UR5
from Vrep import Kinect
from Vrep import Kinect_test

from board import apriltagboard

from handineye import motion
from handineye import rx
from handineye import rz

from method import dual
from method import li
from method import tsai

import cv2
import transforms3d
import numpy as np
import time

import random
import os
import string
import matplotlib.pyplot as plt
fs2 = cv2.FileStorage("../config/intrinsic_gt.yml",cv2.FileStorage_READ)
intrinsic = fs2.getNode("intrinsic").mat()
dist = fs2.getNode("dist").mat()
fs2.release()

fs = cv2.FileStorage("../config/handineye_gt.yml", cv2.FileStorage_READ)
Hcamera2end = fs.getNode("Hcamera2end").mat()
Hobj2base = fs.getNode("Hobj2base").mat()
fs.release()
board = apriltagboard.AprilTagBoard("../config/apriltag.yml", "../config/tagId.csv")
fs = cv2.FileStorage("../config/init_sample.yml",cv2.FileStorage_READ)
init_pose = fs.getNode("initpose").mat()

q = init_pose[:4].flatten()
pose_r = transforms3d.quaternions.quat2mat(q)

pose = np.identity(4)

pose[:3,:3]= pose_r[:,:]
pose[0, 3] = init_pose[4]
pose[1, 3] = init_pose[5]
pose[2, 3] = init_pose[6]
imgsize = (640,480)
fs.release()


cliend = vrep_connect.getVrep_connect()
camera = Kinect.Camera(cliend)
camera2 = Kinect_test.Camera(cliend)
robot = UR5.Robot(cliend)
while (True):
    flag = robot.move(pose)
    if flag:
        break
    robot.go_home()
    robot.go_guodudian()


rgb_image = camera.get_rgb_image()
flag,objpoint,imgpoint = board.getObjImgPointList(rgb_image,verbose=0)
campose = board.extrinsic(imgpoint,objpoint,intrinsic,dist)

candidate_camera_pose = simple_campose.generate_simple_pose("../config/auto_sample.yml", board,
                                                                campose[:3, :3], 0 )
board_max_x = board.marker_X * (board.markerSeparation + board.tag_size)
board_max_y = board.marker_Y * (board.markerSeparation + board.tag_size)
mid_x = board_max_x / 2
mid_y = board_max_y / 2
#board_max_y = board_max_y / 2
mid_points = np.array([[0, mid_x, mid_x, board_max_x],
                       [mid_y, 0, board_max_y, mid_y],
                       [0, 0, 0, 0],
                       [1, 1, 1, 1]])
select_pose = []
all = 0
camera_intrinsic = np.append(intrinsic, np.zeros([3, 1]), 1)


for pose in candidate_camera_pose:

    points = np.dot(camera_intrinsic, np.dot(pose, mid_points))
    folder = ""
    if (points[2, 0] < 0):
        folder = "/home/zhangxinxin/data/campose_test/False"
        print("False")
    else:
        points[:, :] = points[:, :] / points[2, :]
        t = 0
        for j in range(4):
            if points[0, j] > 0 and points[0, j] < imgsize[0] and points[1, j] > 0 and points[1, j] < imgsize[1]:
                t = t + 1
        if t >= 3:
            folder = "/home/zhangxinxin/data/campose_test/True"
            print("True")
        else:
            folder = "/home/zhangxinxin/data/campose_test/False"
            print("false")
        if t == 4:
            all = all + 1
        x = points[0,:]
        y = points[1,:]

    Hcam2base = np.dot(Hobj2base, np.linalg.inv(pose))
    camera2.move(Hcam2base)
    image = camera2.get_rgb_image()
    ran_str = ''.join(random.sample(string.ascii_letters + string.digits, 8))
    cv2.imwrite(os.path.join(folder,ran_str+".png"),image)

