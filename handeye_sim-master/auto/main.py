# _*_ coding:utf-8 _*_
# @time: 2020/10/4 上午9:41
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

ros_cv2_path = '/opt/ros/kinetic/lib/python2.7/dist-packages'
import sys
if ros_cv2_path in sys.path:
    sys.path.remove(ros_cv2_path)
    import cv2
    sys.path.append(ros_cv2_path)
else:
    import cv2
import transforms3d
import numpy as np


if __name__ == '__main__':
    board = apriltagboard.AprilTagBoard("../config/apriltag.yml", "../config/tagId.csv")
    cliend = vrep_connect.getVrep_connect()
    camera = Kinect.Camera(cliend)
    camera2 = Kinect_test.Camera(cliend)
    robot = UR5.Robot(cliend)
    guodudian = np.identity(4)
    guodudian[0,3] = -0.2
    guodudian[1,3] = 0
    guodudian[2,3] = 0.5
    robot.move(guodudian)

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

    auto_calibration.set
    Hcamera2end,Hobj2base = auto_calibration.auto_calibration(robot,camera,board,pose,imgsize,intrinsic,dist,max_step=20)





