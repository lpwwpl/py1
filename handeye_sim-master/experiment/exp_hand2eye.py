# _*_ coding:utf-8 _*_
# @time: 2020/11/2 下午3:53
# @author: 张新新
# @email: 1262981714@qq.com

from auto import auto_handeye_calibration_thread

from auto import utils

from Vrep import vrep_connect
from Vrep import LBR4p

from Vrep import Kinect_test

from board import apriltagboard
import os

import random
import cv2
import transforms3d
import numpy as np

import time
import threading

class lock(object):
    lock = 0
    def __init__(self,robot,camera):
        self.robot=robot
        self.camera=camera

    def move_and_getimage(self,pose):
        while(self.lock==1):
            time.sleep(2)
            # print("lock")
        self.lock=1
        flag = self.robot.move(pose)
        if not flag:
            self.lock=0
            return False,None

        pic = camera.get_rgb_image()
        self.lock = 0
        return True,pic
method={
        "no_Lock":0,
        "std":1,
        "no_lock_std":3,
        "rme":2,
        "no_lock_rme":4,
        "random":5
    }
def task(board,robot,camera,move_lock,methodstr,auto_handeye):
    save_dir = "../result/10_30"
    #print("method {} start {} iters".format(methodstr, i))
    # auto_handeye = auto_handeye_calibration_thread.auto_handeye_calibration(board, robot, camera,
    #                                                                         "../config/auto_set.yml", move_lock)
    timestamp = time.time()
    timestruct = time.localtime(timestamp)
    time_str = time.strftime('%m_%d_%H_%M', timestruct)
    auto_handeye.set_select_method(method[methodstr])
    if methodstr == 'ias':
        auto_handeye.ias_run()
    else:
        auto_handeye.run()
    auto_handeye.save_result(os.path.join(save_dir, "{0}_{1}.json".format(methodstr, time_str)))



if __name__ == '__main__':
    method={
        "no_Local":0,
        "std":1,
        "no_local_std":3,
        #"rme":2,
        #"no_lock_rme":4,
        "random":5,
        "ias":6
    }
    board = apriltagboard.AprilTagBoard("../config/apriltag.yml", "../config/tagId.csv")
    cliend = vrep_connect.getVrep_connect()
    camera = Kinect_test.Camera(cliend,"../config/intrinsic_gt.yml")
    robot = LBR4p.Robot(cliend)
    move_lock = lock(robot,camera)
    threads = []
    fs = cv2.FileStorage("../config/init_handtoeye_robot_pose.yml", cv2.FileStorage_READ)
    init_pose = fs.getNode("initpose").mat()

    q = init_pose[:4].flatten()
    pose_r = transforms3d.quaternions.quat2mat(q)
    move_lock = lock(robot, camera)
    pose = np.identity(4)

    pose[:3, :3] = pose_r[:, :]
    pose[0, 3] = init_pose[4]
    pose[1, 3] = init_pose[5]
    pose[2, 3] = init_pose[6]
    for i in range(100):
        print("start to cal {} iters data".format(i))
        start = time.time()
        # 中间写上代码块

        auto_handeye = auto_handeye_calibration_thread.auto_handeye_calibration(board, robot, camera,
                                                                                "../config/auto_set.yml", move_lock)
        auto_handeye.cali_type=1
        auto_handeye.set_init_robot_pose(pose)
        auto_handeye.init_handeye()
        auto_handeye.handeye_cali()
        Hx = auto_handeye.Hx
        Hy = auto_handeye.Hy
        for key in method:
            threadA = threading.Thread(target=task,args=(board,robot,camera,move_lock,key,auto_handeye.copy()))
            threadA.start()
            threads.append(threadA)
        for thread in threads:
            thread.join()
        end = time.time()
        print('Running time: %s Seconds' % (end - start))

