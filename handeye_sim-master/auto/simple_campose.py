# _*_ coding:utf-8 _*_
# @time: 2020/9/29 上午9:41
# @author: 张新新
# @email: 1262981714@qq.com
import sys
ros_cv2_path = '/opt/ros/kinetic/lib/python2.7/dist-packages'
if ros_cv2_path in sys.path:
    sys.path.remove(ros_cv2_path)
    import cv2
    sys.path.append(ros_cv2_path)
else:
    import cv2
import numpy as np

import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import transforms3d
from scipy import optimize as opt
from board import apriltagboard
import math
def generate_simple_pose(config,board,initial_rotation,verbose=0):
    fs = cv2.FileStorage(config, cv2.FILE_STORAGE_READ)
    minZ = fs.getNode("min_Z").real()
    maxZ = fs.getNode("max_Z").real()
    inter_z = fs.getNode("inter_z").real()
    inter_xy = fs.getNode("inter_xy").real()
    optic_angle = fs.getNode("optic_angle").real()
    fs.release()
    z_cout = int((maxZ-minZ)/inter_z)
    board_max_x = board.marker_X*(board.markerSeparation+board.tag_size)
    board_max_y = board.marker_Y*(board.markerSeparation+board.tag_size)
    sample_position = np.array([])
    for i in range(z_cout):
        z = minZ+i * inter_z
        extent = z * math.tan(optic_angle/180.0*math.pi)
        max_x = board_max_x+ extent
        min_x = -extent
        max_y = board_max_y+ extent
        min_y = -extent
        position = np.mgrid[min_x:max_x:inter_xy,min_y:max_y:inter_xy].T.reshape(-1, 2)
        sample_position = np.append(sample_position,np.append(position,np.dot(-z,np.ones([position.shape[0],1])),1))
    sample_position = sample_position.reshape([-1,3])
    sample_pos = np.array([])
    if verbose==1:
        fig = plt.figure()
        ax = fig.gca(projection='3d')
        ax.scatter3D(sample_position[:, 0], sample_position[:, 1], sample_position[:, 2], cmap='Blues')
        ax.scatter3D([0,0,board_max_x,board_max_x],[0,board_max_y,0,board_max_y],[0,0,0,0])
        plt.show()

    # angle = np.mgrid[0:90:angle_inter, 0:90:angle_inter, 0:90:angle_inter].T.reshape(-1, 3) * math.pi / 180.0
    init_base_camepose = getBaseCampose(initial_rotation)
    ax,ay,az = transforms3d.euler.mat2euler(init_base_camepose)
    angle = np.array([[0,0,0],
                      [math.pi/6,0,0],
                      [math.pi/3,0,0],
                      [0,math.pi/6,0],
                      [0,math.pi/3,0],
                      [-math.pi / 6, 0, 0],
                      [-math.pi / 3, 0, 0],
                      [0, -math.pi / 6, 0],
                      [0, -math.pi / 3, 0],
                      ])

    # angle = np.array([[0,0.8,math.pi/2],
    #                   [0,-0.8,math.pi/2],
    #                   [0.8,0,math.pi/2],
    #                   [-0.8,0,math.pi/2],
    #                   [0,0,math.pi/2]])
    candidate_cam_pose = []
    Rlist = []
    for i in range(angle.shape[0]):
        R = transforms3d.euler.euler2mat(angle[i,0],angle[i,1],angle[i,2])
        Rlist.append(np.linalg.inv(np.dot(R,init_base_camepose)))
    sample_cam_pose = []
    for i in range(sample_position.shape[0]):
        for R in Rlist:
            H = np.append(np.append(R,np.transpose([sample_position[i,:]]),1),np.array([[0,0,0,1]]),0)
            sample_cam_pose.append(np.linalg.inv(H))
    return sample_cam_pose

def getBaseCampose(initial_rotation):
    a_=math.pi/2
    b_=math.pi/2
    min_z =2
    for i in range(10):
        a = math.pi/20*i
        for j in range(10):
            b= math.pi/20*j
            rotation = transforms3d.euler.euler2mat(a, b, 0)
            p = np.dot(rotation, initial_rotation)
            dis_z = abs(p[2,2]-1)
            if p[2,2]>0.9:
                min_z=0
                if abs(a)+abs(b)<abs(a_)+abs(b_):
                    a_ = a
                    b_ = b
            else:
                if dis_z<min_z:
                    a_ = a
                    b_ = b
                    min_z=dis_z
    if min_z>0.2:
        def loss(X,initial_rotation):
            a=X[0]
            b=X[1]
            rotation = transforms3d.euler.euler2mat(a,b,0)
            p = np.dot(rotation,initial_rotation)
            return abs(p[2,2]-1)
        init = np.array([0,0])
        solver = opt.minimize(loss,init,initial_rotation)
        X = solver.x
        a_ = X[0]
        b_ = X[1]
    rotation = transforms3d.euler.euler2mat(a_, b_, 0)
    p = np.dot(rotation, initial_rotation)
    return p



def getExpectRobotPose(know_robot_pose,know_cam_Extrinsic,expect_cam_pose,Thand2eye):
    expect_robot_pose = np.array([])
    for i in range(expect_cam_pose.shape[0]):
        q = np.array([])
        t = np.array([])
        R = transforms3d.euler.euler2mat(expect_cam_pose[i,3],expect_cam_pose[i,4],expect_cam_pose[i,5])
        expect_cam_pose_mat = np.append(np.append(R,np.transpose([expect_cam_pose[i,:3]]),1),np.array([[0,0,0,1]]),0)
        for j in range(len(know_robot_pose)):
            temp_robot_pose = np.dot(know_robot_pose[j],np.dot(Thand2eye,np.dot(know_cam_Extrinsic[j],np.dot(np.linalg.inv(expect_cam_pose_mat),np.linalg.inv(Thand2eye)))))
            q = np.append(q, transforms3d.quaternions.mat2quat(temp_robot_pose[:3,:3]))
            t = np.append(t, temp_robot_pose[:3,3])
        q = q.reshape([-1,4])
        t = t.reshape([-1,3])
        mean_q = np.mean(q,0)
        mean_t = np.mean(t,0)
        expect_robot_pose_temp = np.append(mean_t,transforms3d.euler.mat2euler(transforms3d.quaternions.quat2mat(mean_q)))
        expect_robot_pose = np.append(expect_robot_pose,expect_robot_pose_temp)
    return expect_robot_pose.reshape([-1,6])





def writeConfig(config):
    minZ=float(input("please input the simples' min distance to board(m):"))
    maxZ=float(input("please input the simples' max distance to board(m):"))
    inter_z = float(input("please input the simples z interval (m):"))
    inter_xy = float(input("please input the x and y interval(m):"))
    optic_angle = float(input("please input the simples optic angle:"))
    angle_inter = float(input("please input the angle interval(grad):"))
    fs = cv2.FileStorage(config, cv2.FileStorage_WRITE)
    fs.write("min_Z", minZ)
    fs.write("max_Z", maxZ)
    fs.write("inter_z", inter_z)
    fs.write("inter_xy", inter_xy)
    fs.write("optic_angle", optic_angle)
    fs.write("angle_inter", angle_inter)
    fs.release()
    return 0

if __name__ == "__main__":
    # a = math.pi/3
    # b = math.pi/4
    # c = math.pi/6
    # init_rotation = transforms3d.euler.euler2mat(a,b,c,'rxyz')
    # getBaseCampose(init_rotation)
    #writeConfig("../config/auto_sample.yml")
    board = apriltagboard.board("../config/apriltag.yml", "../config/tagId.csv")
    pose = generate_simple_pose("../config/auto_sample.yml",board,1)
    # imgsize = tuple([1920,1080])
    # root_dir = "F:/fbs_data_raw/507"
    # fs = cv2.FileStorage(root_dir + "/intrinsic_gt.yml", cv2.FILE_STORAGE_READ)
    # camera_matrix = fs.getNode("cameraMatrix").mat()
    # select_pose = select_pose_by_view(pose,camera_matrix,imgsize,board)