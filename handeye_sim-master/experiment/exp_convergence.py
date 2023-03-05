# _*_ coding:utf-8 _*_
# @time: 2020/10/5 上午9:22
# @author: 张新新
# @email: 1262981714@qq.com

from auto import init_handeye
from auto import score
from auto import simple_campose
from auto import select_simple_pose
from auto import auto_calibration

from Vrep import vrep_connect
from Vrep import LBR4p
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
if __name__ == '__main__':
        board = apriltagboard.AprilTagBoard("../config/apriltag.yml", "../config/tagId.csv")
        cliend = vrep_connect.getVrep_connect()
        camera = Kinect.Camera(cliend,"../config/intrinsic_gt.yml")
        camera2 = Kinect_test.Camera(cliend,"../config/intrinsic_gt.yml")
        robot = LBR4p.Robot(cliend)

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
        fs2 = cv2.FileStorage("../config/intrinsic_gt.yml",cv2.FileStorage_READ)
        intrinsic = fs2.getNode("intrinsic").mat()
        dist = fs2.getNode("dist").mat()
        for i in range(30):
            timestamp = time.time()
            timestruct = time.localtime(timestamp)
            time_str = time.strftime('%m_%d_%H_%M', timestruct)
            # Hcamera2end, Hobj2base, max_rme = auto_calibration.auto_calibration(robot, camera, board, pose, imgsize, intrinsic,
            #                                                                     dist, max_step=30,savepath="../result/noLocal_std_{0}.yml".format(time_str))
            Hcamera2end, Hobj2base, max_rme = auto_calibration.auto_calibration_random(robot, camera, board, pose, imgsize,
                                                                                intrinsic,
                                                                                dist, max_step=30,
                                                                                savepath="../result/random_{0}.yml".format(time_str))
            robot.go_home()


    # fs = cv2.FileStorage("../config/handineye_gt.yml", cv2.FileStorage_READ)
    # Hcamera2end_gt = fs.getNode("Hcamera2end").mat()
    # Hobj2base_gt = fs.getNode("Hobj2base").mat()
    # fs.release()
    # q_camera2end_gt = transforms3d.quaternions.mat2quat(Hcamera2end_gt[:3, :3])
    # t_camera2end_gt = Hcamera2end_gt[:3, 3]
    # q_obj2base_gt = transforms3d.quaternions.mat2quat(Hobj2base_gt[:3, :3])
    # t_obj2base_gt = Hobj2base_gt[:3, 3]
    #
    # q_camera2end_error = np.array([])
    # t_camera2end_error = np.array([])
    # q_obj2base_error = np.array([])
    # t_obj2base_error = np.array([])
    # rme = np.array([])
    # for i in range(5,30):
    #     q_camera2end_error_temp = np.array([])
    #     t_camera2end_error_temp = np.array([])
    #     q_obj2base_error_temp = np.array([])
    #     t_obj2base_error_temp = np.array([])
    #     max_rme_temp = np.array([])
    #     for j in range(20):
    #         print("pose number:{0} repeat number:{1}".format(i,j))
    #         robot.go_home()
    #         robot.go_guodudian()
    #         Hcamera2end,Hobj2base,max_rme = auto_calibration.auto_calibration(robot,camera,board,pose,imgsize,intrinsic,dist,max_step=i)
    #
    #         q_camera2end = transforms3d.quaternions.mat2quat(Hcamera2end[:3, :3])
    #         t_camera2end = Hcamera2end[:3, 3]
    #         q_obj2base = transforms3d.quaternions.mat2quat(Hobj2base[:3, :3])
    #         t_obj2base = Hobj2base[:3, 3]
    #
    #         if np.linalg.norm(q_camera2end - q_camera2end_gt) > np.linalg.norm(q_camera2end + q_camera2end_gt):
    #             q_camera2end = -q_camera2end
    #         if np.linalg.norm(q_obj2base - q_obj2base_gt) > np.linalg.norm(q_obj2base + q_obj2base_gt):
    #             q_obj2base = -q_obj2base
    #         q_camera2end_error_temp = np.append(q_camera2end_error_temp, np.abs(q_camera2end - q_camera2end_gt))
    #         t_camera2end_error_temp = np.append(t_camera2end_error_temp, np.abs(t_camera2end - t_camera2end_gt))
    #         q_obj2base_error_temp = np.append(q_obj2base_error_temp, np.abs(q_obj2base - q_obj2base_gt))
    #         t_obj2base_error_temp = np.append(t_obj2base_error_temp, np.abs(t_obj2base - t_obj2base_gt))
    #         max_rme_temp = np.append(max_rme_temp,max_rme)
    #
    #     q_camera2end_error_temp= q_camera2end_error_temp.reshape([-1,4])
    #     t_camera2end_error_temp= t_camera2end_error_temp.reshape([-1,3])
    #     q_obj2base_error_temp= q_obj2base_error_temp.reshape([-1,4])
    #     t_obj2base_error_temp= t_obj2base_error_temp.reshape([-1,3])
    #     fs2 = cv2.FileStorage("../result/convergence_result{0}.yml".format(i), cv2.FileStorage_WRITE)
    #     fs2.write("q_camera2end_error_temp", q_camera2end_error_temp)
    #     fs2.write("t_camera2end_error_temp", t_camera2end_error_temp)
    #     fs2.write("q_obj2base_error_temp", q_obj2base_error_temp)
    #     fs2.write("t_obj2base_error_temp", t_obj2base_error_temp)
    #     fs2.write("max_rme_temp", max_rme_temp)
    #     fs2.release()
    #     q_camera2end_mean = np.mean(q_camera2end_error_temp,0)
    #     t_camera2end_mean = np.mean(t_camera2end_error_temp,0)
    #     q_obj2base_mean = np.mean(q_obj2base_error_temp,0)
    #     t_obj2base_mean = np.mean(t_obj2base_error_temp,0)
    #     q_camera2end_error = np.append(q_camera2end_error,q_camera2end_mean)
    #     t_camera2end_error = np.append(t_camera2end_error,t_camera2end_mean)
    #     q_obj2base_error = np.append(q_obj2base_error,q_obj2base_mean)
    #     t_obj2base_error = np.append(t_obj2base_error,t_obj2base_mean)
    # q_camera2end_error = q_camera2end_error.reshape([-1,4])
    # t_camera2end_error =t_camera2end_error.reshape([-1,3])
    # q_obj2base_error = q_obj2base_error.reshape([-1,4])
    # t_obj2base_error = t_obj2base_error.reshape([-1,3])
    # fs = cv2.FileStorage("../result/convergence_result.yml",cv2.FileStorage_WRITE)
    # fs.write("q_camera2end_error",q_camera2end_error)
    # fs.write("t_camera2end_error",t_camera2end_error)
    # fs.write("q_obj2base_error",q_obj2base_error)
    # fs.write("t_obj2base_error",t_obj2base_error)
    # fs.release()

