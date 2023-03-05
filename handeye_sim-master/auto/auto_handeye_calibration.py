#-*- coding:utf-8 -*-
# @time: 
# @author:张新新
# @email: 1262981714@qq.com
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
from handineye import motion
from handineye import rx
from handineye import rz


import math
from method import tsai
from method import dual
from method import li
#import handtoeye
from scipy import optimize as opt
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import random
import threading
from auto import utils

class auto_handeye_calibration(object):
    def __init__(self,board,robot,camera,config):
        '''
        初始化，需要指定标定板，机器臂，相机，以及初始化文件
        :param board: 标定板
        :param robot: 机器臂
        :param camera: 相机
        :param config: 配置文件
        '''
        self.board = board
        self.robot = robot
        self.camera = camera
        fs = cv2.FileStorage(config, cv2.FILE_STORAGE_READ)
        self.minZ = fs.getNode("min_Z").real()
        self.maxZ = fs.getNode("max_Z").real()
        self.inter_z = fs.getNode("inter_z").real()
        self.inter_xy = fs.getNode("inter_xy").real()
        self.optic_angle = fs.getNode("optic_angle").real()
        self.cali_type = int(fs.getNode("cali_type").real())  #calibration type： handineye 0 handtoeye 1
        self.picture_number = int(fs.getNode("picture_number").real())
        if self.cali_type==0:
            from handineye import motion
            from handineye import rx
            from handineye import rz
        else:
            from handtoeye import motion
            from handtoeye import rx
            from handtoeye import rz

        init_pose = fs.getNode("init_robot_pose").mat()

        q = init_pose[:4].flatten()
        pose_r = transforms3d.quaternions.quat2mat(q)

        self.init_robot_pose = np.identity(4)

        self.init_robot_pose[:3, :3] = pose_r[:, :]
        self.init_robot_pose[0, 3] = init_pose[4]
        self.init_robot_pose[1, 3] = init_pose[5]
        self.init_robot_pose[2, 3] = init_pose[6]
        fs.release()

        self.next_step_method = 0
    def set_init_robot_pose(self,pose):
        self.init_robot_pose = pose.copy()

    def init_handeye(self):
        '''
        通过旋转机械臂来初始化手眼标定
        :return:
        '''
        flag = self.robot.move(self.init_robot_pose)
        assert flag, "cannot reach init pose"
        rgb_image = self.camera.get_rgb_image()
        flag, objpoint, imgpoint = self.board.getObjImgPointList(rgb_image, verbose=0)
        assert flag, "robot init pose cannot see board"
        self.objpoint_list = []
        self.imgpoint_list = []
        self.Hend2base = []
        self.Hobj2camera = []
        self.image = []
        self.result = []
        self.objpoint_list.append(objpoint)
        self.imgpoint_list.append(imgpoint)
        self.Hend2base.append(self.init_robot_pose)
        camerapose = self.board.extrinsic(imgpoint, objpoint, self.camera.intrinsic, self.camera.dist)
        self.Hobj2camera.append(camerapose)

        ax, ay, az = transforms3d.euler.mat2euler(self.init_robot_pose[:3, :3], 'sxyz')
        euler = [ax, ay, az]
        for i in [0,1]:
            for j in [-1,1]:
                objpoint_temp = None
                imgpoint_temp = None
                robot_pose_temp = None
                euler_temp = euler.copy()
                while (True):
                    euler_temp[i] += j*math.pi / 36
                    pose_r = transforms3d.euler.euler2mat(euler_temp[0], euler_temp[1], euler_temp[2], 'sxyz')
                    robot_pose = self.init_robot_pose.copy()
                    robot_pose[:3, :3] = pose_r[:, :]
                    flag1 = self.robot.move(robot_pose)
                    rgb_image = self.camera.get_rgb_image()
                    flag, objpoint, imgpoint = self.board.getObjImgPointList(rgb_image)
                    if flag and flag1:
                        objpoint_temp = objpoint.copy()
                        imgpoint_temp = imgpoint.copy()
                        robot_pose_temp = robot_pose.copy()
                        image_temp = rgb_image.copy()
                    else:
                        if not objpoint_temp is None:
                            camerapose = self.board.extrinsic(imgpoint_temp, objpoint_temp, self.camera.intrinsic, self.camera.dist)
                            self.Hobj2camera.append(camerapose)
                            self.objpoint_list.append(objpoint_temp)
                            self.imgpoint_list.append(imgpoint_temp)
                            self.Hend2base.append(robot_pose_temp)
                            self.image.append(image_temp)
                        break
        assert len(self.Hend2base) > 3, "cannot find enough initial data"

    def handeye_cali(self):
        A, B = motion.motion_axxb(self.Hend2base, self.Hobj2camera)
        Hx = tsai.calibration(A, B)
        #Hx = dual.calibration(A, B)
        A, B = motion.motion_axyb(self.Hend2base, self.Hobj2camera)
        # Hx2,Hy2 = li.calibration(A,B)
        Hx = rx.refine(Hx, self.Hend2base, self.Hobj2camera,
                                           self.board.GetBoardAllPoints())
        q = np.array([])
        t = np.array([])
        for i in range(len(self.Hobj2camera)):
            if self.cali_type==0:
                Hy = np.dot(self.Hend2base[i], np.dot(Hx, self.Hobj2camera[i]))
            else:
                Hy = np.dot(np.linalg.inv(self.Hend2base[i]),np.dot(Hx, self.Hobj2camera[i]))
            q_temp = transforms3d.quaternions.mat2quat(Hy[:3, :3])

            if i == 0:
                q0 = q_temp.copy()
            else:
                if np.linalg.norm(q0 - q_temp) > np.linalg.norm(q0 + q_temp):
                    q_temp = -q_temp
            q = np.append(q, q_temp)
            t = np.append(t, Hy[:3, 3])
        q = q.reshape([-1, 4])
        t = t.reshape([-1, 3])
        q_mean = np.mean(q, 0)
        t_mean = np.mean(t, 0)
        q = q_mean / np.linalg.norm(q)
        Hy_r = transforms3d.quaternions.quat2mat(q)
        Hy = np.identity(4)
        Hy[:3, :3] = Hy_r[:, :]
        Hy[:3, 3] = t_mean[:]
        rme = rz.proj_error(Hx,Hy,self.Hend2base,self.Hobj2camera,self.board.GetBoardAllPoints())
        if self.cali_type==0:
            self.result.append({"image_number": len(self.image), "Hcamera2end":Hx,"Hobj2base":Hy,
                                 "mean_rme":np.mean(np.abs(rme)),"max_rme":np.max(np.abs(rme))})
        else:
            self.result.append({"image_number": len(self.image), "Hcamera2base": Hx, "Hobj2end": Hy,
                                "mean_rme": np.mean(np.abs(rme)), "max_rme": np.max(np.abs(rme))})
        self.Hx = Hx
        self.Hy = Hy

    def camera_pose_simple(self,verbose=0):
        def getBaseCampose(initial_rotation):
            a_ = math.pi / 2
            b_ = math.pi / 2
            min_z = 2
            for i in range(10):
                a = math.pi / 20 * i
                for j in range(10):
                    b = math.pi / 20 * j
                    rotation = transforms3d.euler.euler2mat(a, b, 0)
                    p = np.dot(rotation, initial_rotation)
                    dis_z = abs(p[2, 2] - 1)
                    if p[2, 2] > 0.9:
                        min_z = 0
                        if abs(a) + abs(b) < abs(a_) + abs(b_):
                            a_ = a
                            b_ = b
                    else:
                        if dis_z < min_z:
                            a_ = a
                            b_ = b
                            min_z = dis_z
            if min_z > 0.2:
                def loss(X, initial_rotation):
                    a = X[0]
                    b = X[1]
                    rotation = transforms3d.euler.euler2mat(a, b, 0)
                    p = np.dot(rotation, initial_rotation)
                    return abs(p[2, 2] - 1)

                init = np.array([0, 0])
                solver = opt.minimize(loss, init, initial_rotation)
                X = solver.x
                a_ = X[0]
                b_ = X[1]
            rotation = transforms3d.euler.euler2mat(a_, b_, 0)
            p = np.dot(rotation, initial_rotation)
            return p
        z_cout = int((self.maxZ -self.minZ) / self.inter_z)
        sample_position = np.array([])
        for i in range(z_cout):
            z = self.minZ + i * self.inter_z
            extent = z * math.tan(self.optic_angle / 180.0 * math.pi)
            max_x = extent
            min_x = -extent
            max_y = extent
            min_y = -extent
            position = np.mgrid[min_x:max_x:self.inter_xy, min_y:max_y:self.inter_xy].T.reshape(-1, 2)
            sample_position = np.append(sample_position,
                                        np.append(position, np.dot(-z, np.ones([position.shape[0], 1])), 1))
        sample_position = sample_position.reshape([-1, 3])
        if verbose == 1:
            fig = plt.figure()
            ax = fig.gca(projection='3d')
            ax.scatter3D(sample_position[:, 0], sample_position[:, 1], sample_position[:, 2], cmap='Blues')
            #ax.scatter3D([0, 0, board_max_x, board_max_x], [0, board_max_y, 0, board_max_y], [0, 0, 0, 0])
            plt.show()
        init_base_camepose = getBaseCampose(self.Hobj2camera[0][:3,:3])
        ax, ay, az = transforms3d.euler.mat2euler(init_base_camepose)
        angle = np.array([[0, 0, 0],
                          [math.pi / 6, 0, 0],
                          [math.pi / 3, 0, 0],
                          [0, math.pi / 6, 0],
                          [0, math.pi / 3, 0],
                          [-math.pi / 6, 0, 0],
                          [-math.pi / 3, 0, 0],
                          [0, -math.pi / 6, 0],
                          [0, -math.pi / 3, 0],
                          ])
        Rlist = []
        for i in range(angle.shape[0]):
            R = transforms3d.euler.euler2mat(angle[i, 0], angle[i, 1], angle[i, 2])
            Rlist.append(np.dot(R,init_base_camepose))
        sample_cam_pose = []
        for i in range(sample_position.shape[0]):
            for R in Rlist:
                if self.cali_type==0:
                    H = np.append(np.append(np.linalg.inv(R), np.transpose([sample_position[i, :]]), 1), np.array([[0, 0, 0, 1]]), 0)
                    sample_cam_pose.append(np.linalg.inv(H))
                else:
                    H = np.append(np.append(R, np.transpose([sample_position[i, :]]), 1),
                                  np.array([[0, 0, 0, 1]]), 0)
                    sample_cam_pose.append(H)
        return sample_cam_pose

    def select_pose_by_view(self,sample_cam_pose):
        board_max_x = self.board.marker_X * (self.board.markerSeparation + self.board.tag_size)
        board_max_y = self.board.marker_Y * (self.board.markerSeparation + self.board.tag_size)
        mid_x = board_max_x / 2
        mid_y = board_max_y / 2

        mid_points = np.array([[0, mid_x, mid_x, board_max_x],
                               [mid_y, 0, board_max_y, mid_y],
                               [0, 0, 0, 0],
                               [1, 1, 1, 1]])
        select_pose = []
        all = 0
        camera_intrinsic = np.append(self.camera.intrinsic, np.zeros([3, 1]), 1)
        imgsize = self.camera.imgsize
        for i in range(len(sample_cam_pose)):
            camera_pose = sample_cam_pose[i]
            points = np.dot(camera_intrinsic, np.dot(camera_pose, mid_points))
            if (points[2, 0] < 0):
                continue
            points[:, :] = points[:, :] / points[2, :]

            t = 0
            for j in range(4):
                if points[0, j] > 0 and points[0, j] < imgsize[0] and points[1, j] > 0 and points[1, j] < imgsize[1]:
                    t = t + 1
            if t > 3:
                select_pose.append(camera_pose)
            if t == 4:
                all = all + 1
        
        return select_pose

    def score_std(self,campose):
        expect_cam_pose_mat = campose

        q = np.array([])
        t = np.array([])
        for j in range(len(self.Hend2base)):
            if self.cali_type == 0:
                temp_robot_pose = np.dot(self.Hend2base[j], np.dot(self.Hx, np.dot(self.Hobj2camera[j], np.dot(
                    np.linalg.inv(expect_cam_pose_mat), np.linalg.inv(self.Hx)))))
            else:
                temp_robot_pose = np.linalg.multi_dot([self.Hx, campose, np.linalg.inv(self.Hobj2camera[j]),
                                                       np.linalg.inv(self.Hx), self.Hend2base[j]])
            q = np.append(q, transforms3d.quaternions.mat2quat(temp_robot_pose[:3, :3]))
            t = np.append(t, temp_robot_pose[:3, 3])
        q = q.reshape([-1, 4])
        t = t.reshape([-1, 3])
        for i in range(1, q.shape[0]):
            if abs(np.linalg.norm(q[0, :] - q[i, :])) > abs(np.linalg.norm(q[0, :] + q[i, :])):
                q[i, :] = -q[i, :]
        mean_q = np.mean(q, 0)
        mean_t = np.mean(t, 0)
        std_q = np.std(q, axis=0)
        std_t = np.std(t, axis=0)

        expect_robot_pose = np.append(transforms3d.quaternions.quat2mat(mean_q), np.transpose([mean_t]), 1)
        expect_robot_pose = np.append(expect_robot_pose, np.array([[0, 0, 0, 1]]), 0)
        score = np.mean(std_q) + np.mean(std_t)
        return score,expect_robot_pose
    def get_Expect_robot_pose(self,expect_campose):
        expect_cam_pose_mat = expect_campose
        q = np.array([])
        t = np.array([])
        for j in range(len(self.Hend2base)):
            if self.cali_type == 0:
                temp_robot_pose = np.dot(self.Hend2base[j], np.dot(self.Hx, np.dot(self.Hobj2camera[j], np.dot(
                    np.linalg.inv(expect_cam_pose_mat), np.linalg.inv(self.Hx)))))
            else:
                temp_robot_pose = np.linalg.multi_dot([self.Hx, expect_cam_pose_mat, np.linalg.inv(self.Hobj2camera[j]),
                                                       np.linalg.inv(self.Hx), self.Hend2base[j]])
            q = np.append(q, transforms3d.quaternions.mat2quat(temp_robot_pose[:3, :3]))
            t = np.append(t, temp_robot_pose[:3, 3])
        q = q.reshape([-1, 4])
        t = t.reshape([-1, 3])
        for i in range(1, q.shape[0]):
            if abs(np.linalg.norm(q[0, :] - q[i, :])) > abs(np.linalg.norm(q[0, :] + q[i, :])):
                q[i, :] = -q[i, :]
        mean_q = np.mean(q, 0)
        mean_t = np.mean(t, 0)
        expect_robot_pose = np.append(transforms3d.quaternions.quat2mat(mean_q), np.transpose([mean_t]), 1)
        expect_robot_pose = np.append(expect_robot_pose, np.array([[0, 0, 0, 1]]), 0)
        return expect_robot_pose
    def score_no_local(self,expect_robot_pose):
        q0 = transforms3d.quaternions.mat2quat(expect_robot_pose[:3, :3])
        t0 = expect_robot_pose[:3, 3]
        min_score = 0
        for j in range(len(self.Hend2base)):
            q = transforms3d.quaternions.mat2quat(self.Hend2base[j][:3, :3])
            if np.linalg.norm(q - q0) > np.linalg.norm(q + q0):
                q = -q
            t = self.Hend2base[j][:3, 3]
            q_dis = np.linalg.norm(q - q0)
            t_dis = np.linalg.norm(t - t0)
            score = -(math.pow(math.e, -abs(q_dis)) + 1 * math.pow(math.e, -abs(t_dis)))
            if score < min_score:
                min_score = score
        return min_score

    def score_expect_rme(self,campose):
        robot_pose = self.get_Expect_robot_pose(campose)
        rme = rz.proj_error(self.Hx,self.Hy,[robot_pose],[campose],self.board.GetBoardAllPoints())
        return np.max(np.abs(rme)),robot_pose

    def score_main(self,expect_camera_list):

        if self.next_step_method == 1 or self.next_step_method == 3:
            sco_list = []
            for i in range(len(expect_camera_list)):
                score,robot_pose = self.score_std(expect_camera_list[i])
                if not self.robot.moveable(robot_pose):
                    continue
                if self.next_step_method==3:
                    no_local_score = self.score_no_local(robot_pose)
                    score= score*10 + no_local_score
                sco_list.append([i, score, expect_camera_list[i]])
            sco_list.sort(key=lambda x: x[1])
            campose_order_list = []
            for t in sco_list:
                campose_order_list.append(t[2])
            campose_order_list.reverse()
            return campose_order_list
        elif self.next_step_method == 2 or self.next_step_method == 4:
            sco_list = []
            for i in range(len(expect_camera_list)):
                score, robot_pose = self.score_expect_rme(expect_camera_list[i])
                if not self.robot.moveable(robot_pose):
                    continue
                if self.next_step_method == 4:
                    no_local_score = self.score_no_local(robot_pose)
                    score = score*(10**5) +no_local_score
                sco_list.append([i, score, expect_camera_list[i]])
            sco_list.sort(key=lambda x: x[1])
            campose_order_list = []
            for t in sco_list:
                campose_order_list.append(t[2])
            campose_order_list.reverse()
            return campose_order_list
        elif self.next_step_method == 0:
            sco_list = []
            for i in range(len(expect_camera_list)):
                robot_pose = self.get_Expect_robot_pose(expect_camera_list[i])
                no_local_score = self.score_no_local(robot_pose)
                sco_list.append([i, no_local_score, expect_camera_list[i]])
            sco_list.sort(key=lambda x: x[1])
            campose_order_list = []
            for t in sco_list:
                campose_order_list.append(t[2])
            campose_order_list.reverse()
            return campose_order_list
        else:
            random.shuffle(expect_camera_list)
            return expect_camera_list
    def ias_run(self):
        self.init_handeye()
        self.handeye_cali()
        num_p = 3
        x_angle = [0,15,0]
        y_angle = [0,0,15]
        z_angle = [0,0,0]
        d_min = -0.4
        for plane in range(num_p):
            for angle in range(len(x_angle)):
                Hcamera2obj = np.linalg.inv(self.Hobj2camera[0])
                r_x_temp, r_y_temp, r_z_temp = transforms3d.euler.mat2euler(Hcamera2obj[:3,:3])
                r_x_temp = r_x_temp+ x_angle[angle]*math.pi/180
                r_y_temp = r_y_temp+ y_angle[angle]*math.pi/180
                r_z_temp = r_z_temp+ z_angle[angle]*math.pi/180
                for y in range(4):
                    tx_temp,ty_temp,tz_temp = Hcamera2obj[:3, 3]
                    while(True):
                        if y==0:
                            tx_temp = tx_temp+0.05
                        elif y==1:
                            tx_temp = tx_temp-0.05
                        elif y==2:
                            ty_temp = ty_temp+0.05
                        elif y==3:
                            ty_temp = ty_temp-0.05
                        expect_camera2obj_r = transforms3d.euler.euler2mat(r_x_temp,r_y_temp,r_z_temp)
                        expect_camera2obj = np.identity(4)
                        expect_camera2obj[:3,:3] = expect_camera2obj_r
                        expect_camera2obj[0,3] = tx_temp
                        expect_camera2obj[1,3] = ty_temp
                        expect_camera2obj[2,3] = d_min
                        expect_robot_pose = self.get_Expect_robot_pose(np.linalg.inv(expect_camera2obj))
                        flag = self.robot.moveable(expect_robot_pose)
                        if not flag:
                            break
                        flag = self.robot.move(expect_robot_pose)
                        if not flag:
                            break

                        rgb_image = self.camera.get_rgb_image()
                        flag, objpoint, imgpoint = self.board.getObjImgPointList(rgb_image)
                        if not flag:
                            break
                        camerapose = self.board.extrinsic(imgpoint, objpoint, self.camera.intrinsic, self.camera.dist)
                        self.objpoint_list.append(objpoint)
                        self.imgpoint_list.append(imgpoint)
                        self.Hend2base.append(expect_robot_pose)
                        self.Hobj2camera.append(camerapose)
                        self.image.append(rgb_image)
                        self.handeye_cali()
            d_min -= 0.05

    def run(self):
        self.init_handeye()
        self.handeye_cali()
        simple_campose = self.camera_pose_simple()

        simple_campose = self.select_pose_by_view(simple_campose)
        robot_pose_list = []
        while(len(self.image)<self.picture_number):
            cam_list = self.score_main(simple_campose)
            for pose in cam_list:
                robot_pose = self.get_Expect_robot_pose(pose)
                if not self.robot.moveable(robot_pose):
                    continue
                flag = self.robot.move(robot_pose)
                if not flag:
                    continue
                rgb_image = self.camera.get_rgb_image()
                flag, objpoint, imgpoint = self.board.getObjImgPointList(rgb_image)
                if not flag:
                    continue
                robot_pose_list.append(robot_pose)
                camerapose = self.board.extrinsic(imgpoint, objpoint, self.camera.intrinsic, self.camera.dist)
                self.objpoint_list.append(objpoint)
                self.imgpoint_list.append(imgpoint)
                self.Hend2base.append(robot_pose)
                self.Hobj2camera.append(camerapose)
                self.image.append(rgb_image)
                break
            self.handeye_cali()
        utils.json_save(robot_pose_list,"../config/init_robot_pose.json")

    def save_result(self,file):
        from auto import utils
        utils.json_save(self.result,file)

    def set_select_method(self,method_id):
        self.next_step_method=method_id


