#-*- coding:utf-8 -*-
# @time: 
# @author:张新新
# @email: 1262981714@qq.com
ros_cv2_path = '/opt/ros/kinetic/lib/python2.7/dist-packages'
import sys
if ros_cv2_path in sys.path:
    sys.path.remove(ros_cv2_path)
    import cv2
    sys.path.append(ros_cv2_path)
else:
    import cv2
from handineye import motion
from handineye import rx
from handineye import rz
from utils import depthUtils

from method import dual
from method import tsai
from PIL import Image
import numpy as np
import math

from method import li
import transforms3d
import os
from handineye import rz
from board import apriltagboard





def getPose(root_dir,number):
    poseList =[]
    for i in range(number):
        temp = np.loadtxt(os.path.join(root_dir,str(i)+'.txt'))
        r = transforms3d.euler.euler2mat(temp[1, 0] * math.pi / 180, temp[1, 1] * math.pi / 180,
                                         temp[1, 2] * math.pi / 180, 'sxyz')
        t = np.array([temp[0, :]]).T
        H = np.append(np.append(r, t, 1), np.array([[0, 0, 0, 1]]), 0)
        poseList.append(H)
    return poseList
def getPose2(robot_file):
    temp = np.loadtxt(robot_file)
    poseList = []
    for i in range(temp.shape[0]):
        r = transforms3d.euler.euler2mat(temp[i,3]*math.pi/180,temp[i, 4]*math.pi/180,temp[i,5]*math.pi/180)
        t = np.array([temp[i, :3]]).T
        H = np.append(np.append(r, t, 1), np.array([[0, 0, 0, 1]]), 0)
        poseList.append(H)
    return poseList
def getPose3(robot_file):
    temp = np.loadtxt(robot_file)
    poseList = []
    for i in range(temp.shape[0]):
        r = transforms3d.quaternions.quat2mat(np.array([temp[i,6],temp[i, 3],temp[i,4],temp[i,5]]))
        t = np.array([temp[i, :3]]).T
        H = np.append(np.append(r, t, 1), np.array([[0, 0, 0, 1]]), 0)
        poseList.append(H)
    return poseList


def getImgList(root_dir):
    file_list = os.listdir(root_dir)
    rgb_list = []
    depth_list = []
    for file in file_list:
        if file.endswith("_color.png"):
            rgb_list.append(os.path.join(root_dir,file))
        elif file.endswith("_depth.png"):
            depth_list.append(os.path.join(root_dir,file))
    return rgb_list,depth_list

def main():
    board = apriltagboard.AprilTagBoard("../config/apriltag.yml", "../config/tagId.csv")
    verbose = 0
    root_dir ="../offline_data/handeye2"
    image_list,depth_list = getImgList(root_dir)
    if len(image_list)!= len(depth_list):
        print("numer of img and depth not same")
    image_list.sort()
    depth_list.sort()
    number = len(image_list)
    fs = cv2.FileStorage(root_dir+"/intrinsic.yml", cv2.FILE_STORAGE_READ)
    camera_matrix = fs.getNode("cameraMatrix").mat()
    discoff = fs.getNode("discoff").mat()
    imag_points_list = []
    obj_points_list = []
    depth_points_list = []
    extrinsic_list = []
    for i in range(number):
        image = cv2.imread(image_list[i])
        flag,imagepoints, objpoints = board.getObjImgPointList(image)
        # depth_points, rejectids = depthUtils.get_depth(imagepoints, depth_list[i])
        # imagepoints = np.delete(imagepoints, rejectids, axis=0)
        # depth_points = np.delete(depth_points, rejectids, axis=0)
        # objpoints = np.delete(objpoints, rejectids, axis=0)
        extrinsic = board.extrinsic(objpoints,imagepoints,camera_matrix,discoff)
        extrinsic = board.extrinsic_opt(camera_matrix,discoff,extrinsic,imagepoints,objpoints)
        extrinsic_list.append(extrinsic)
        imag_points_list.append(imagepoints)
        obj_points_list.append(objpoints)
        # depth_points_list.append(depth_points)


    robot_pose = getPose2(os.path.join(root_dir,"pose.txt"))


    while(True):

        A, B = motion.motion_axxb(robot_pose, extrinsic_list)
        Tsai_handeye = tsai.calibration(A, B)
        dual_handeye = dual.calibration(A, B)
        rx_handeye = rx.refine(dual_handeye, robot_pose, extrinsic_list, board.GetBoardAllPoints())
        rx_error = rx.proj_error(rx_handeye, robot_pose, extrinsic_list, board.GetBoardAllPoints())
        q = transforms3d.quaternions.mat2quat(rx_handeye[:3, :3])
        handeye_q = np.array([q[1],q[2],q[3],q[0]])
        handeye_result = np.append(handeye_q,rx_handeye[:3, 3])
        print("camera_matrix:", camera_matrix)
        print("dicoff:", discoff)
        print("rx", q, rx_handeye[:3, 3])

        A, B = motion.motion_axyb(robot_pose, extrinsic_list)
        li_x, li_y = li.calibration(A, B)
        rz_x, rz_y = rz.refine(li_x, li_y, robot_pose, extrinsic_list, board.GetBoardAllPoints())
        q = transforms3d.quaternions.mat2quat(rz_x[:3, :3])
        print("rz", q, rz_x[:3, 3])
        rz_error = rz.proj_error_each_point(rz_x, rz_y, robot_pose, extrinsic_list, board.GetBoardAllPoints())
        x,y = np.where(rz_error.reshape([1,-1])>0.005)
        if y.shape[0]==0:
            break
        x,y = np.where(rz_error.reshape([1,-1])==np.max(rz_error))
        del robot_pose[y[0]]
        del extrinsic_list[y[0]]
    rx_error = rx.proj_error(rx_handeye, robot_pose, extrinsic_list, board.GetBoardAllPoints())
    q = transforms3d.quaternions.mat2quat(rx_handeye[:3, :3])
    print("rme",np.mean(np.abs(rx_error)))
    handeye_q = np.array([q[1], q[2], q[3], q[0]])
    handeye_result = np.append(handeye_q, rx_handeye[:3, 3])
    fs = cv2.FileStorage(root_dir + "/handeye.yml", cv2.FileStorage_WRITE)
    fs.write("cameraMatrix", camera_matrix)
    fs.write("discoff", discoff)
    fs.write("handeye", rx_handeye)
    fs.release()


    return


if __name__ == "__main__":
    main()