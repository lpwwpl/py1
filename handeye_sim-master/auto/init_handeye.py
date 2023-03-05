# _*_ coding:utf-8 _*_
# @time: 2020/9/29 上午9:42
# @author: 张新新
# @email: 1262981714@qq.com
from Vrep import vrep_connect
from Vrep import UR5
from Vrep import Kinect
from Vrep import vrep_connect
import transforms3d
from board import apriltagboard
import math
from handineye import motion
from handineye import rx
from method import tsai
from method import li
ros_cv2_path = '/opt/ros/kinetic/lib/python2.7/dist-packages'
import sys
if ros_cv2_path in sys.path:
    sys.path.remove(ros_cv2_path)
    import cv2
    sys.path.append(ros_cv2_path)
else:
    import cv2
import numpy as np
def init_handeye(initpose,camera,robot,board,intrinsic,dist):
    '''

    :param initpose:
    :param camera:
    :param robot:
    :return:
    '''
    cv2.waitKey(1000)
    while(True):
        flag = robot.move(initpose)
        if flag:
            break
        robot.go_home()
        robot.go_guodudian()

    assert flag,"cannot reach init pose"
    rgb_image = camera.get_rgb_image()
    # cv2.imshow("image",rgb_image)
    # cv2.imwrite("../result/temp.png",rgb_image)
    # image2 = cv2.imread('../result/temp.png')
    # cv2.waitKey(0)
    flag,objpoint,imgpoint = board.getObjImgPointList(rgb_image,verbose=0)
    assert flag,"robot init pose cannot see board"
    objpoint_list = []
    imgpoint_list = []
    pose_list = []
    objpoint_list.append(objpoint)
    imgpoint_list.append(imgpoint)
    pose_list.append(initpose)
    ax,ay,az = transforms3d.euler.mat2euler(initpose[:3,:3],'sxyz')
    ax_temp = ax
    objpoint_temp =None
    imgpoint_temp =None
    robot_pose_temp = None
    while(True):
        ax_temp+=math.pi/36
        pose_r = transforms3d.euler.euler2mat(ax_temp,ay,az,'sxyz')
        robot_pose = initpose.copy()
        robot_pose[:3,:3]= pose_r[:,:]
        flag = robot.move(robot_pose)
        if not flag:
            continue
        rgb_image = camera.get_rgb_image()
        flag, objpoint, imgpoint = board.getObjImgPointList(rgb_image)
        if flag:
            objpoint_temp = objpoint.copy()
            imgpoint_temp = imgpoint.copy()
            robot_pose_temp = robot_pose.copy()
        else:
            if not objpoint_temp is None:
                objpoint_list.append(objpoint_temp)
                imgpoint_list.append(imgpoint_temp)
                pose_list.append(robot_pose_temp)
            break
    ax_temp = ax
    objpoint_temp = None
    imgpoint_temp = None
    robot_pose_temp = None
    while (True):
        ax_temp -= math.pi / 18
        pose_r = transforms3d.euler.euler2mat(ax_temp, ay, az, 'sxyz')
        robot_pose = initpose.copy()
        robot_pose[:3, :3] = pose_r[:, :]
        flag = robot.move(robot_pose)
        if not flag:
            continue
        rgb_image = camera.get_rgb_image()
        flag, objpoint, imgpoint = board.getObjImgPointList(rgb_image)

        if flag:
            objpoint_temp = objpoint.copy()
            imgpoint_temp = imgpoint.copy()
            robot_pose_temp = robot_pose.copy()
        else:
            if not objpoint_temp is None:
                objpoint_list.append(objpoint_temp)
                imgpoint_list.append(imgpoint_temp)
                pose_list.append(robot_pose_temp)
            break
    ay_temp = ay
    objpoint_temp = None
    imgpoint_temp = None
    robot_pose_temp = None
    while (True):
        ay_temp += math.pi / 36
        pose_r = transforms3d.euler.euler2mat(ax, ay_temp, az, 'sxyz')
        robot_pose = initpose.copy()
        robot_pose[:3, :3] = pose_r[:, :]
        flag = robot.move(robot_pose)
        if not flag:
            continue
        rgb_image = camera.get_rgb_image()
        flag, objpoint, imgpoint = board.getObjImgPointList(rgb_image)
        if flag:
            objpoint_temp = objpoint.copy()
            imgpoint_temp = imgpoint.copy()
            robot_pose_temp = robot_pose.copy()
        else:
            if not objpoint_temp is None:
                objpoint_list.append(objpoint_temp)
                imgpoint_list.append(imgpoint_temp)
                pose_list.append(robot_pose_temp)
            break
    ay_temp = ay
    objpoint_temp = None
    imgpoint_temp = None
    robot_pose_temp = None
    while (True):
        ay_temp -= math.pi / 36
        pose_r = transforms3d.euler.euler2mat(ax, ay_temp, az, 'sxyz')
        robot_pose = initpose.copy()
        robot_pose[:3, :3] = pose_r[:, :]
        flag = robot.move(robot_pose)
        if not flag:
            continue
        rgb_image = camera.get_rgb_image()
        flag, objpoint, imgpoint = board.getObjImgPointList(rgb_image)
        if flag:
            objpoint_temp = objpoint.copy()
            imgpoint_temp = imgpoint.copy()
            robot_pose_temp = robot_pose.copy()
        else:
            if not objpoint_temp is None:
                objpoint_list.append(objpoint_temp)
                imgpoint_list.append(imgpoint_temp)
                pose_list.append(robot_pose_temp)
            break
    assert len(objpoint_list)>3,"cannot find enough initial data"
    # rme, mtx, dist = board.intrinsic(imgpoint_list,objpoint_list,(640,480))
    # print(rme,mtx,dist)
    camera_pose_list = []

    for i in range(len(objpoint_list)):
        camerapose = board.extrinsic(imgpoint_list[i],objpoint_list[i],intrinsic,dist)
        camera_pose_list.append(camerapose)

    A,B = motion.motion_axxb(pose_list,camera_pose_list)
    Hcamera2end = tsai.calibration(A,B)
    Hcamera2end_refine = rx.refine(Hcamera2end,pose_list,camera_pose_list,board.GetBoardAllPoints())
    q = np.array([])
    t = np.array([])
    for i in range(len(camera_pose_list)):

        Hobj2base = np.dot(pose_list[i],np.dot(Hcamera2end_refine,camera_pose_list[i]))
        q_temp = transforms3d.quaternions.mat2quat(Hobj2base[:3,:3])

        if i==0:
            q0 = q_temp.copy()
        else:
            if np.linalg.norm(q0-q_temp)>np.linalg.norm(q0+q_temp):
                q_temp = -q_temp
        q = np.append(q, q_temp)
        t = np.append(t,Hobj2base[:3,3])
    q = q.reshape([-1,4])
    t = t.reshape([-1,3])
    q_mean = np.mean(q,0)
    t_mean = np.mean(t,0)
    q = q_mean/np.linalg.norm(q)
    Hobj2base_r = transforms3d.quaternions.quat2mat(q)
    Hobj2base = np.identity(4)
    Hobj2base[:3,:3] = Hobj2base_r[:,:]
    Hobj2base[:3,3] = t_mean[:]
    return Hcamera2end_refine,Hobj2base,objpoint_list,imgpoint_list,pose_list,camera_pose_list



if __name__ == '__main__':
    board = apriltagboard.AprilTagBoard("../config/apriltag.yml", "../config/tagId.csv")
    cliend = vrep_connect.getVrep_connect()
    camera = Kinect.Camera(cliend)

    robot = UR5.Robot(cliend)


    fs = cv2.FileStorage("../config/init_sample.yml",cv2.FileStorage_READ)
    init_pose = fs.getNode("initpose").mat()

    q = init_pose[:4].flatten()
    pose_r = transforms3d.quaternions.quat2mat(q)

    pose = np.identity(4)

    pose[:3,:3]= pose_r[:,:]
    pose[0, 3] = init_pose[4]
    pose[1, 3] = init_pose[5]
    pose[2, 3] = init_pose[6]
    fs.release()
    fs2 = cv2.FileStorage("../config/intrinsic_gt.yml",cv2.FileStorage_READ)
    intrinsic = fs2.getNode("intrinsic").mat()
    dist = fs2.getNode("dist").mat()

    init_handeye(pose,camera,robot,board,intrinsic,dist)










