# _*_ coding:utf-8 _*_
# @time: 2020/10/4 下午4:47
# @author: 张新新
# @email: 1262981714@qq.com
from auto import init_handeye
from auto import score
from auto import simple_campose
from auto import select_simple_pose
from auto import utils

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

import sys
ros_cv2_path = '/opt/ros/kinetic/lib/python2.7/dist-packages'
if ros_cv2_path in sys.path:
    sys.path.remove(ros_cv2_path)
    import cv2
    sys.path.append(ros_cv2_path)
else:
    import cv2
import transforms3d
import numpy as np
import random
def auto_calibration(robot,camera,board,initpose,imgsize, intrinsic,dist, max_error=-1,max_step=50,savepath=None):
    print("start to init cal init handeye")
    fs = None
    if not savepath is None:
        fs = cv2.FileStorage(savepath,cv2.FileStorage_WRITE)
    Hcamera2end, Hobj2base, objpoint_list, imgpoint_list, pose_list, cam_pose_list = init_handeye.init_handeye(initpose,
                                                                                                               camera,
                                                                                                               robot,
                                                                                                               board,
                                                                                                               intrinsic,
                                                                                                               dist)


    candidate_camera_pose = simple_campose.generate_simple_pose("../config/auto_sample.yml", board,
                                                                cam_pose_list[0][:3, :3], 0 )


    select_pose = select_simple_pose.select_pose_by_view(candidate_camera_pose, intrinsic, imgsize, board)


    board_point = board.GetBoardAllPoints()
    board_point = np.append(np.append(board_point.T, np.zeros([1, board_point.shape[0]]), 0),
                            np.ones([1, board_point.shape[0]]), 0)
    max_rme = 0
    print("start to find best pose")
    for j in range(max_step-len(cam_pose_list)):
        random_select_pose = select_pose
        print("move to {0} step".format(len(cam_pose_list)))
        sco_list = []
        for i in range(len(random_select_pose)):
            sco, random_robot_pose = score.score_expect_robot_pose(pose_list, cam_pose_list, random_select_pose[i], Hcamera2end)

            if not robot.moveable(random_robot_pose):
                continue
            sco_list.append([i, sco, random_robot_pose, random_select_pose[i]])
        sco_list.sort(key=lambda x: x[1])
        delete_pose = []
        for i in range(1, len(random_select_pose) + 1):

            flag = robot.move(sco_list[-i][2])
            if flag:
                robot_pose = sco_list[-i][2]
                rgb_image = camera.get_rgb_image()
                flag, objpoint, imgpoint = board.getObjImgPointList(rgb_image, verbose=0)
                if not flag:
                    continue
                robot.go_home()
                robot.go_guodudian()
                camerapose = board.extrinsic(imgpoint, objpoint, intrinsic, dist)
                cam_pose_list.append(camerapose)
                objpoint_list.append(objpoint)
                imgpoint_list.append(imgpoint)
                pose_list.append(robot_pose)
                break
            else:
                delete_pose.append(sco_list[-i][0])
        delete_pose.sort()
        for i in range(len(delete_pose)-1,-1,-1):
            del select_pose[delete_pose[i]]
            # robot.go_home()
            # robot.go_guodudian()

        A, B = motion.motion_axxb(pose_list, cam_pose_list)
        Hcamera2end = tsai.calibration(A, B)
        Hcamera2end_refine = rx.refine(Hcamera2end, pose_list, cam_pose_list, board.GetBoardAllPoints())
        q = np.array([])
        t = np.array([])
        for i in range(len(cam_pose_list)):

            Hobj2base = np.dot(pose_list[i], np.dot(Hcamera2end_refine, cam_pose_list[i]))
            q_temp = transforms3d.quaternions.mat2quat(Hobj2base[:3, :3])

            if i == 0:
                q0 = q_temp.copy()
            else:
                if np.linalg.norm(q0 - q_temp) > np.linalg.norm(q0 + q_temp):
                    q_temp = -q_temp
            q = np.append(q, q_temp)
            t = np.append(t, Hobj2base[:3, 3])
        q = q.reshape([-1, 4])
        t = t.reshape([-1, 3])
        q_mean = np.mean(q, 0)
        t_mean = np.mean(t, 0)
        q = q_mean / np.linalg.norm(q)
        Hobj2base_r = transforms3d.quaternions.quat2mat(q)
        Hobj2base = np.identity(4)
        Hobj2base[:3, :3] = Hobj2base_r[:, :]
        Hobj2base[:3, 3] = t_mean[:]
        Hcamera2end = Hcamera2end_refine
        rme = rz.proj_error(Hcamera2end,Hobj2base,pose_list,cam_pose_list,board.GetBoardAllPoints())
        max_rme = np.max(np.abs(rme))
        if not fs is None:
            fs.write("Hcamera2end{0}".format(j),Hcamera2end)
            fs.write("Hobj2base{0}".format(j),Hobj2base)
            fs.write("mean-rme{0}".format(j),np.mean(np.abs(rme)))
            fs.write("max-rme{0}".format(j),max_rme)
        print("rme:",np.mean(np.abs(rme)))
    if not fs is None:
        fs.release()
    return Hcamera2end,Hobj2base,max_rme

def auto_calibration_random(robot,camera,board,initpose,imgsize, intrinsic,dist, max_error=-1,max_step=50,savepath=None):
    print("start to init cal init handeye")
    fs = None
    if not savepath is None:
        fs = cv2.FileStorage(savepath,cv2.FileStorage_WRITE)
    Hcamera2end, Hobj2base, objpoint_list, imgpoint_list, pose_list, cam_pose_list = init_handeye.init_handeye(initpose,
                                                                                                               camera,
                                                                                                               robot,
                                                                                                               board,
                                                                                                               intrinsic,
                                                                                                               dist)
    candidate_camera_pose = simple_campose.generate_simple_pose("../config/auto_sample.yml", board,
                                                                cam_pose_list[0][:3, :3], 0 )


    select_pose = select_simple_pose.select_pose_by_view(candidate_camera_pose, intrinsic, imgsize, board)
    random_select_pose = select_pose
    max_rme = 0

    for j in range(max_step-len(cam_pose_list)):
        print("move to {0} step".format(len(cam_pose_list)))
        while(True):
            expect_campose = random.choice(random_select_pose)
            robot_pose = utils.get_Expect_robot_pose(cam_pose_list,pose_list,Hcamera2end,expect_campose)
            if not robot.moveable(robot_pose):
                continue
            flag = robot.move(robot_pose)
            if flag:
                rgb_image = camera.get_rgb_image()
                flag, objpoint, imgpoint = board.getObjImgPointList(rgb_image, verbose=0)
                if not flag:
                    continue
                robot.go_home()
                robot.go_guodudian()
                camerapose = board.extrinsic(imgpoint, objpoint, intrinsic, dist)
                cam_pose_list.append(camerapose)
                objpoint_list.append(objpoint)
                imgpoint_list.append(imgpoint)
                pose_list.append(robot_pose)
                break
        A, B = motion.motion_axxb(pose_list, cam_pose_list)
        Hcamera2end = tsai.calibration(A, B)
        Hcamera2end_refine = rx.refine(Hcamera2end, pose_list, cam_pose_list, board.GetBoardAllPoints())
        q = np.array([])
        t = np.array([])
        for i in range(len(cam_pose_list)):

            Hobj2base = np.dot(pose_list[i], np.dot(Hcamera2end_refine, cam_pose_list[i]))
            q_temp = transforms3d.quaternions.mat2quat(Hobj2base[:3, :3])

            if i == 0:
                q0 = q_temp.copy()
            else:
                if np.linalg.norm(q0 - q_temp) > np.linalg.norm(q0 + q_temp):
                    q_temp = -q_temp
            q = np.append(q, q_temp)
            t = np.append(t, Hobj2base[:3, 3])
        q = q.reshape([-1, 4])
        t = t.reshape([-1, 3])
        q_mean = np.mean(q, 0)
        t_mean = np.mean(t, 0)
        q = q_mean / np.linalg.norm(q)
        Hobj2base_r = transforms3d.quaternions.quat2mat(q)
        Hobj2base = np.identity(4)
        Hobj2base[:3, :3] = Hobj2base_r[:, :]
        Hobj2base[:3, 3] = t_mean[:]
        Hcamera2end = Hcamera2end_refine
        rme = rz.proj_error(Hcamera2end,Hobj2base,pose_list,cam_pose_list,board.GetBoardAllPoints())
        max_rme = np.max(np.abs(rme))
        if not fs is None:
            fs.write("Hcamera2end{0}".format(j),Hcamera2end)
            fs.write("Hobj2base{0}".format(j),Hobj2base)
            fs.write("mean-rme{0}".format(j),np.mean(np.abs(rme)))
            fs.write("max-rme{0}".format(j),max_rme)
        print("rme:",np.mean(np.abs(rme)))
    if not fs is None:
        fs.release()
    return Hcamera2end,Hobj2base,max_rme







