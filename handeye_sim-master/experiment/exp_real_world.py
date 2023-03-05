#-*- coding:utf-8 -*-
# @time: 
# @author:张新新
# @email: 1262981714@qq.com
import sys
import os
rootpath = str("/home/speedbot/code/handeye_sim")
syspath = sys.path
sys.path.append(rootpath)
dirs = os.listdir(rootpath)
for dir in dirs:
    sys.path.append(os.path.join(rootpath,dir))

import transforms3d
from auto import auto_handeye_real_world
from auto import auto_handeye_real_world_depth
from auto import auto_handeye_real_world_depth2
from auto import auto_handeye_calibration
from auto import utils

import random
from camera import kinect
from robot import aubo

from board import apriltagboard



import time
if __name__ == '__main__':
    board = apriltagboard.AprilTagBoard("../config/apriltag_real.yml", "../config/tagId.csv")
    camera = kinect.Kinect(1026,"../real_data/intrinisc_data/intrinsic.yml")
    #camera = kinect.Kinect(1026,"../rea/intrinsic_1217.yml")
    robot = aubo.robot(port=1025)
    auto_handeye = auto_handeye_real_world_depth2.auto_handeye_calibration(board, robot, camera,
                                                                            "../config/auto_set_real.yml")
    timestamp = time.time()
    timestruct = time.localtime(timestamp)
    time_str = time.strftime('%m_%d_%H_%M', timestruct)
    # pose = robot.move(auto_handeye.init_robot_pose)
    # flag,image = camera.get_rgb_image()
    auto_handeye.set_select_method(3)
    auto_handeye.init_handeye()
    auto_handeye.handeye_cali()
    #auto_handeye.save_result("../temp/init_result.json")
    auto_handeye.run()
    #auto_handeye.save_result("../temp/init_result.json")
    timeArray = time.localtime()
    otherStyleTime = time.strftime("%Y_%m_%d_%H_%M", timeArray)
    save_dir = os.path.join("../real_data",otherStyleTime)
    os.mkdir(save_dir)
    auto_handeye.save_result(os.path.join(save_dir,"result.json"))
    auto_handeye.save_image(save_dir)
    auto_handeye.save_depth(save_dir)
    auto_handeye.save_robot_pose(os.path.join(save_dir,"robot_pose.json"))

    camera.realease()
    robot.realease()