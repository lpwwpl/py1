import os
os.environ['QT_API'] = 'pyside'
import pyrealsense2 as rs
import numpy as np
import cv2

import sys
import torch
import torch.nn as nn
import time
import torchvision.models as models
import math
import cv2 as cv
from sklearn.preprocessing import normalize
from scipy.spatial.distance import pdist, squareform


import open3d as o3d
# from PyQt5 import QtCore, QtGui, QtWidgets
# from PyQt5.QtWidgets import *
# from PyQt5.QtGui import *
# from PyQt5.QtCore import *

from PySide2 import QtCore, QtGui, QtWidgets
from PySide2.QtWidgets import *
from PySide2.QtGui import *
from PySide2.QtCore import *

from traits.api import HasTraits, Instance, on_trait_change
from traitsui.api import View, Item,Group
from mayavi.core.ui.api import MayaviScene, MlabSceneModel, \
        SceneEditor

from sklearn.decomposition import PCA
from scipy.spatial.transform import Rotation as R
import scipy.cluster.hierarchy as sch
import pdb
import threading
import kdata_lx


# from Robot import *
# from Part import *
# from FreeCAD import *
# import FreeCAD as App

pca = PCA()
currentSpeed = "300"
currentAccuracy = "300"
timestamp = None


def fcMoveHome():
    App.activeDocument().Robot.Axis1 = 0
    App.activeDocument().Robot.Axis2 = -30
    App.activeDocument().Robot.Axis3 = 30
    App.activeDocument().Robot.Axis4 = 0
    App.activeDocument().Robot.Axis5 = 0
    App.activeDocument().Robot.Axis6 = 0
    App.ActiveDocument.recompute()
    fcInsertRobotTcp()


def fcDraw(param):
    w = Waypoint(App.activeDocument().Robot.Tcp.multiply(App.activeDocument().Robot.Tool), "PTP", "Pt")
    if param[0] != "":
        w.Pos.Base.x = w.Pos.Base.x + float(param[0])
    if param[1] != "":
        w.Pos.Base.y = w.Pos.Base.y + float(param[1])
    if param[2] != "":
        w.Pos.Base.z = w.Pos.Base.z + float(param[2])

    w.Velocity = currentSpeed
    w.Tool = 1

    t = App.activeDocument().Trajectory.Trajectory
    t.insertWaypoints(w)


def fcLMove(param):
    # qq = euler_to_quaternion_fc(param[3], param[4], param[5])
    # euler = quaternion_to_euler_fc(qq[0], qq[1], qq[2], qq[3])

    # pl = App.Placement(App.Vector(param[0], param[1], param[2]), App.Rotation(euler[0], euler[1], euler[2]),
    #                   App.Vector(0, 0, 0))

    pl = App.Placement(App.Vector(param[0], param[1], param[2]), App.Rotation(param[3], param[4], param[5]),
                       App.Vector(0, 0, 0))

    App.activeDocument().Robot.Tcp = pl.multiply(App.activeDocument().Robot.Tool.inverse())

    pOffset1 = App.Placement(App.Vector(0, 0, 0), App.Rotation(App.Vector(180, 0, 0), 0.00))
    print(pOffset1)

    pOffset2 = App.Placement(App.Vector(0, 0, 0), App.Rotation(App.Vector(180, 0, 0), 0.00))
    print(pOffset2)

    # pl = App.activeDocument().Robot.Tcp.multiply(pOffset1)

    App.activeDocument().Robot.Tcp = App.activeDocument().Robot.Base.inverse().multiply(pl).multiply(
        App.activeDocument().Robot.Tool.inverse())
    fcInsertRobotTcp()


def fcJMove(param):
    App.activeDocument().Robot.Axis1 = param[0]
    App.activeDocument().Robot.Axis2 = param[1]
    App.activeDocument().Robot.Axis3 = param[2]
    App.activeDocument().Robot.Axis4 = param[3]
    App.activeDocument().Robot.Axis5 = param[4]
    App.activeDocument().Robot.Axis6 = param[5]
    fcInsertRobotTcp()


def fcSetTool(param, toolName):
    print(App.activeDocument().Robot.Tool)
    App.activeDocument().Robot.Tool = App.Placement(App.Vector(param[0], param[1], param[2]),
                                                    App.Rotation(App.Vector(param[3], param[4], param[5]), 0.00))
    App.activeDocument().Robot.ToolShape = App.activeDocument().getObject(toolName)
    App.ActiveDocument.recompute()


def fcSetSpeed(param):
    global currentSpeed
    print(currentSpeed)
    currentSpeed = param


def fcSetAccuracy(param):
    global currentAccuracy
    currentAccuracy = param


def fcInsertRobotTcp():
    App.activeDocument().recompute()
    w = Waypoint(App.activeDocument().Robot.Tcp.multiply(App.activeDocument().Robot.Tool), "LIN", "Pt")
    w.Velocity = currentSpeed
    w.Tool = 1
    t = App.activeDocument().Trajectory.Trajectory
    t.insertWaypoints(w)


# def Fun_Tool2Base_Shooting():
#     Tool2Base_Shooting = [0.10767868646835144*1000, -0.3787937230921638*1000, 0.15660759*1000, 0.009104541870731944,
#                           3.130781700067779, 0.05941563583525477]
#     return Tool2Base_Shooting


def Fun_Eyeinhand_Shooting(flag):
    if flag == False:
        Tool2Base_Shooting = [0.15802303*1000, -0.32541972*1000, 0.1903955*1000, 0.00931757, 3.13076056, 0.05933887]
    else:
        Tool2Base_Shooting = [-0.26879975 , -0.32541972 , 0.1903955 , 0.009104541870731944,
                              3.130781700067779,
                              0.05941563583525477]
    return Tool2Base_Shooting



def rm2rv(R):
    theta = np.arccos((R[0][0] + R[1][1] + R[2][2] - 1) / 2)
    K = (1 / (2 * np.sin(theta))) * np.asarray([R[2][1] - R[1][2], R[0][2] - R[2][0], R[1][0] - R[0][1]])
    r = theta * K
    r1 = [R[0][3], R[1][3], R[2][3], r[0], r[1], r[2]]
    return r1



def rv2rm(rv):
    rx = rv[3]
    ry = rv[4]
    rz = rv[5]
    np.seterr(invalid='ignore')
    theta = np.linalg.norm([rx, ry, rz])
    kx = rx / theta
    ky = ry / theta
    kz = rz / theta

    c = np.cos(theta)
    s = np.sin(theta)
    v = 1 - c

    R = np.zeros((4, 4))
    R[0][0] = kx * kx * v + c
    R[0][1] = kx * ky * v - kz * s
    R[0][2] = kx * kz * v + ky * s
    R[0][3] = rv[0]

    R[1][0] = ky * kx * v + kz * s
    R[1][1] = ky * ky * v + c
    R[1][2] = ky * kz * v - kx * s
    R[1][3] = rv[1]

    R[2][0] = kz * kx * v - ky * s
    R[2][1] = kz * ky * v + kx * s
    R[2][2] = kz * kz * v + c
    R[2][3] = rv[2]
    R[3][3] = 1

    return R



def rm2rpy(R):
    # sy = np.sqrt(R[0][0] * R[0][0] + R[1][0] * R[1][0])
    sy = np.sqrt(R[2][1] * R[2][1] + R[2][2] * R[2][2])
    singular = sy < 1e-6

    if not singular:
        rotatex = np.arctan2(R[2][1], R[2][2])
        rotatey = np.arctan2(-R[2][0], sy)
        rotatez = np.arctan2(R[1][0], R[0][0])
    else:
        rotatex = np.arctan2(-R[1][2], R[1][1])
        rotatey = np.arctan2(-R[2][0], sy)
        rotatez = 0

    return np.asarray([R[0][3], R[1][3], R[2][3], rotatex, rotatey, rotatez])



def rpy2rm(rpy):
    # Rx = np.zeros((3, 3), dtype=rpy.dtype)
    # Ry = np.zeros((3, 3), dtype=rpy.dtype)
    # Rz = np.zeros((3, 3), dtype=rpy.dtype)

    R0 = np.zeros((4, 4))

    x = rpy[0]
    y = rpy[1]
    z = rpy[2]
    thetaX = rpy[3]
    thetaY = rpy[4]
    thetaZ = rpy[5]

    cx = np.cos(thetaX)
    sx = np.sin(thetaX)

    cy = np.cos(thetaY)
    sy = np.sin(thetaY)

    cz = np.cos(thetaZ)
    sz = np.sin(thetaZ)

    R0[0][0] = cz * cy
    R0[0][1] = cz * sy * sx - sz * cx
    R0[0][2] = cz * sy * cx + sz * sx
    R0[0][3] = x
    R0[1][0] = sz * cy
    R0[1][1] = sz * sy * sx + cz * cx
    R0[1][2] = sz * sy * cx - cz * sx
    R0[1][3] = y
    R0[2][0] = -sy
    R0[2][1] = cy * sx
    R0[2][2] = cy * cx
    R0[2][3] = z
    R0[3][3] = 1
    return R0


def rv2rpy(rv):
    R = rv2rm(rv)
    rpy = rm2rpy(R)
    return rpy


def rpy2rv(rpy):
    R = rpy2rm(rpy)
    rv = rm2rv(R)
    return rv


def rpy2qt(rpy):
    rx = rpy[3]
    ry = rpy[4]
    rz = rpy[5]
    x = np.cos(ry / 2) * np.cos(rz / 2) * np.sin(rx / 2) - np.sin(ry / 2) * np.sin(rz / 2) * np.cos(rx / 2)
    y = np.sin(ry / 2) * np.cos(rz / 2) * np.cos(rx / 2) + np.cos(ry / 2) * np.sin(rz / 2) * np.sin(rx / 2)
    z = np.cos(ry / 2) * np.sin(rz / 2) * np.cos(rx / 2) - np.sin(ry / 2) * np.cos(rz / 2) * np.sin(rx / 2)
    w = np.cos(ry / 2) * np.cos(rz / 2) * np.cos(rx / 2) + np.sin(ry / 2) * np.sin(rz / 2) * np.sin(rx / 2)
    return [x, y, z, w]


def text_read():
    # exec_path = executable_path()
    # camFileName = os.path.abspath(os.path.join(exec_path, 'picslx/Cam2Tool.txt'))
    # with open(camFileName, "r") as f:
    #    Cam2BaseTxt1 = f.read().replace('[', '').replace(']', '')
    # Cam2BaseTxt2 = Cam2BaseTxt1.splitlines()
    # Cam2BaseTxt = [row.split() for row in Cam2BaseTxt2 if row.strip()]
    # col = len(Cam2BaseTxt[0])
    # row = len(Cam2BaseTxt)
    # Cam2Base = list(range(6))
    # for j in range(col):
    #    Cam2Base[j] = float(Cam2BaseTxt[0][j])
    # Cam2Base_rm = rpy2rm(Cam2Base)
    Cam2Base_rm = [[0.99958932, -0.02117886, 0.01930385, -0.06328652], [0.0221976, 0.99828422, -0.0541838, -0.16513022],
                   [-0.01812318, 0.05459005, 0.99834437, -0.11598357], [0, 0, 0, 1]]
    return Cam2Base_rm


def Fun_Tool2Base_Suction_Interim(Object2Cam):
    # Object2Cam = client_srv()
    Object2Cam_rm = rpy2rm(Object2Cam)
    Cam2Tool_rm = text_read()
    Tool2Base = get_current_tcp()
    Tool2Base_rm = rv2rm(Tool2Base)
    Object2Base = Tool2Base_rm.dot(Cam2Tool_rm).dot(Object2Cam_rm)
    Tool2Base_Suction_rm = Object2Base
    Tool2Base_Suction = rm2rv(Tool2Base_Suction_rm)
    Move_rm = np.array([[1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 1, -0.05 ], [0, 0, 0, 1]])
    Tool2Base_Interim1_rm = Tool2Base_Suction_rm.dot(Move_rm)
    Tool2Base_Interim1 = rm2rv(Tool2Base_Interim1_rm)
    Tool2Base_Interim2 = Tool2Base_Suction + np.array([0, 0, 0.15 , 0, 0, 0])
    return Tool2Base_Suction, Tool2Base_Interim1, Tool2Base_Interim2



def Fun_Eyeinhand_Release(flag):
    x = 0
    y = 0
    z = 0
    if flag == False:
        Tool2Base_Release = [(-0.16 - 0.12 * x)*1000, (-0.56 + 0.08 * y)*1000, (0.075 + 0.05 * z)*1000, 0.00909552, 3.13077267,
                             0.05922008]
    else:
        Tool2Base_Release = [(0.17 + 0.12 * x) , (-0.56 + 0.08 * y) , (0.075 + 0.05 * z) , 0.00909552,
                             3.13077267,
                             0.05922008]
    Tool2Base_Release_Interim1 = Tool2Base_Release + np.array([0, 0, 0.01 , 0, 0, 0])
    Tool2Base_Release_Interim2 = Tool2Base_Release + np.array([0, 0, 0.12 , 0, 0, 0])
    return Tool2Base_Release, Tool2Base_Release_Interim1, Tool2Base_Release_Interim2


def get_current_tcp():
    value = [1, 1, 1, 1, 1, 1]
    tcp = App.getDocument("ur5e_lx").getObject("Robot").Tcp
    p = tcp.Base
    a = tcp.Rotation.Axis
    value = [p[0], p[1], p[2], a[0], a[1], a[2]]
    return value


class KCore(QtCore.QThread):
    pic_signal = QtCore.Signal()

    def __init__(self, parent=None):
        super(KCore, self).__init__(parent)
        self.working = True
        #self.cam = CamHandler()
        self.exec_path = executable_path()
        intriFileName = os.path.abspath(os.path.join(self.exec_path, "picslx/test-camera-intrinsics.txt"))
        self.colorCamIntrinsics = np.loadtxt(intriFileName, dtype=float)

    def __del__(self):
        self.working = False
        #self.cam.stop_cam()

    def client_srv(self):
        #self.cam.getFrame()
        exec_path = executable_path()
        colorFileName = os.path.abspath(os.path.join(exec_path, 'picslx/frame-20210303_175313.color.png'))
        deptFileName = os.path.abspath(os.path.join(exec_path, "picslx/frame-20210303_175313.depth.png"))

        color_img = cv.cvtColor(cv.imread(colorFileName, flags=1), cv.COLOR_BGR2RGB)
        depth_img = cv.imread(deptFileName, flags=-1)

        inferFileName = os.path.abspath(os.path.join(self.exec_path, "picslx/frame-{}.infer.png".format(timestamp)))
        # inference(inferFileName, self.cam.color_img, self.cam.depth_img)
        inference(inferFileName, color_img, depth_img)
        inferImg = cv2.imread(inferFileName, flags=-1)

        # centroids_cam, rotations, confidence_clusters = self.postProcess(False, self.cam.color_img, self.cam.depth_img,
        #                                                                  inferImg, self.colorCamIntrinsics)
        centroids_cam, rotations, confidence_clusters = self.postProcess(False, color_img, depth_img,
                                                                         inferImg, self.colorCamIntrinsics)
        Object2Cam = []

        centroids_cam_array = np.asarray(centroids_cam)
        dim = centroids_cam_array.ndim

        if (len(confidence_clusters) > 0):
            if (dim == 1):
                EulerXYZ = rotations[0]
                eulerAngle = rotationMatrixToEulerAngles(EulerXYZ)
                Object2Cam = [centroids_cam[0] , centroids_cam[1] , centroids_cam[2] , eulerAngle[0],
                              eulerAngle[1],
                              eulerAngle[2]]
            else:
                Transitions = centroids_cam[0]
                EulerXYZ = rotations[0]
                eulerAngle = rotationMatrixToEulerAngles(EulerXYZ)
                Object2Cam = [Transitions[0] , Transitions[1] , Transitions[2] , eulerAngle[0],
                              eulerAngle[1],
                              eulerAngle[2]]

        return Object2Cam

    def postProcess(self, ShowImages=False, colorImg=None, depthImg=None, inferImg=None, cameraIntrinsics=None):
        iheight = 480
        iwidth = 640
        depthImg = depthImg.astype('double') / 10000.0;

        threthold_lowerbound = 0.5
        affordance_threthold = inferImg.max().astype('double') / 255.0 * 0.64
        if affordance_threthold < threthold_lowerbound:
            affordance_threthold = threthold_lowerbound
        mask_th = 255 * affordance_threthold
        mask_p = (inferImg >= mask_th)

        x = np.arange(1, iwidth + 1)
        y = np.arange(1, iheight + 1)
        [pixX, pixY] = np.meshgrid(x, y)
        pixZ = depthImg.astype(np.double)

        pixX = pixX.T
        pixX_clusters = pixX[mask_p.T]

        pixY = pixY.T
        pixY_clusters = pixY[mask_p.T]
        pixZ = pixZ.T
        pixZ_clusters = pixZ[mask_p.T]
        pixels_clusters = np.asarray([pixX_clusters, pixY_clusters])
        if mask_p.sum() == 0:
            cluster_idx = []
            cluster_count = 0
        else:
            Y = pdist(pixels_clusters.T, 'euclidean')
            Z = sch.linkage(Y, 'single', True)
            # inconsis = inconsistent(Z)
            cluster_idx = sch.fcluster(Z, t=0.6, criterion='inconsistent')  # 'Cutoff'
            cluster_count = max(cluster_idx)

        camX_clusters = (pixX_clusters - cameraIntrinsics[0, 2]) * pixZ_clusters / cameraIntrinsics[0, 0]
        camY_clusters = (pixY_clusters - cameraIntrinsics[1, 2]) * pixZ_clusters / cameraIntrinsics[1, 1]
        camZ_clusters = pixZ_clusters
        camPoints_clusters = np.asarray([camX_clusters, camY_clusters, camZ_clusters])

        camPointsColor_clusters = []
        for i in range(0, 3):
            temp = colorImg[:, :, i]
            temp = temp[mask_p]
            temp = temp.reshape(-1)
            if len(camPointsColor_clusters) == 0:
                camPointsColor_clusters = temp
            else:
                camPointsColor_clusters = np.row_stack((camPointsColor_clusters, temp))


        camX = (pixX.reshape(-1) - cameraIntrinsics[0, 2]) * pixZ.reshape(-1) / cameraIntrinsics[0, 0]
        camY = (pixY.reshape(-1) - cameraIntrinsics[1, 2]) * pixZ.reshape(-1) / cameraIntrinsics[1, 1]
        camZ = pixZ.reshape(-1)
        camPoints = np.asarray([camX, camY, camZ])


        # camPointsColor = colorImg.reshape((-1, 3), order='F')

        camPointsColor = []
        for i in range(0, 3):
            temp = colorImg[:, :, i]
            temp = temp.reshape(-1)
            if len(camPointsColor) == 0:
                camPointsColor = temp
            else:
                camPointsColor = np.row_stack((camPointsColor, temp))

        del_idx = []
        for i in range(1, cluster_count + 1):
            idx_filter = (cluster_idx == i)
            tmp = np.asarray([camX_clusters[idx_filter], camY_clusters[idx_filter], camZ_clusters[idx_filter]])
            tmp = tmp.T
            tmp_ = np.where(idx_filter)[0]
            cluster_filter_temp = cluster_filter(tmp, tmp_)
            # lpw = np.where(idx_filter)
            # tmp1 = camPoints_clusters_T[lpw]
            # tmp2 = np.where(cluster_idx == i)
            # cluster_filter_temp = cluster_filter(tmp1, tmp2)
            del_idx = np.concatenate((del_idx, cluster_filter_temp))

        if len(del_idx):
            del_idx = del_idx.astype('int')
        if len(del_idx) > 0:
            pixX_clusters[del_idx] = False
            pixY_clusters[del_idx] = False
            pixZ_clusters[del_idx] = False
            pixels_clusters[0][del_idx] = False
            pixels_clusters[1][del_idx] = False
            camX_clusters[del_idx] = False
            camY_clusters[del_idx] = False
            camZ_clusters[del_idx] = False
            camPoints_clusters[0][del_idx] = False
            camPoints_clusters[1][del_idx] = False
            camPoints_clusters[2][del_idx] = False
            cluster_idx[del_idx] = False

        clusters = np.zeros([iheight, iwidth], 'int')
        for i in range(1, cluster_count + 1):
            temp1 = pixels_clusters[0][cluster_idx == i]
            temp2 = pixels_clusters[1][cluster_idx == i]
            temp = (temp2 - 1) * iwidth + temp1
            clusters.flat[temp] = i * 255.0 / cluster_count

        print('Number of clusters: ', cluster_count)
        for i in range(1, cluster_count + 1):
            print('Number of points in cluster' + str(i) + ":" + str(sum(cluster_idx == i)))

        # infer_clusters = inferImg.T[mask_p.T]
        infer_clusters = inferImg[mask_p]
        confidence_clusters = []
        for i in range(1, cluster_count + 1):
            idx_cluster = (cluster_idx == i)
            x = infer_clusters[idx_cluster]
            if len(infer_clusters[idx_cluster]) > 0:
                temp = infer_clusters[cluster_idx == i]
                confidence_clusters.append(max(temp))
        confidence_idx = np.argsort(confidence_clusters)  # [0 2 1]
        confidence_clusters = np.sort(confidence_clusters)[::-1]  # descend

        centroids_cam = []
        for i in range(1, cluster_count + 1):
            idx_filter = (cluster_idx[::] == i)
            cent_tmp1 = camPoints_clusters[0][idx_filter].mean()
            cent_tmp2 = camPoints_clusters[1][idx_filter].mean()
            cent_tmp3 = camPoints_clusters[2][idx_filter].mean()
            if len(centroids_cam) == 0:
                centroids_cam = [cent_tmp1, cent_tmp2, cent_tmp3]
            else:
                centroids_cam = np.row_stack((centroids_cam, [cent_tmp1, cent_tmp2, cent_tmp3]))
        k = []
        # if centroids_cam != []:
        # row,col= centroids_cam.shape
        #
        # if centroids_cam != []:
        #
        #     dim = centroids_cam.ndim
        #
        #     if dim > 1:
        #         for i in range(centroids_cam.shape[0]):
        #             mask_c1 = centroids_cam[i, 2] > 0.44
        #             mask_c2 = centroids_cam[i, 2] <= 0
        #             mask_c = mask_c1 | mask_c2
        #             if mask_c:
        #                 k.append(i)
        #     else:
        #         mask_c1 = centroids_cam[2] > 0.44
        #         mask_c2 = centroids_cam[2] <= 0
        #         mask_c = mask_c1 | mask_c2
        #         if mask_c:
        #             k.append(i)
        #
        # k = list(set(k))

        rotations = []  # 3*3*3

        for i in range(1, cluster_count + 1):
            idx_filter = (cluster_idx[::] == i)
            cent_tmp1 = camPoints_clusters[0][idx_filter]
            cent_tmp2 = camPoints_clusters[1][idx_filter]
            cent_tmp3 = camPoints_clusters[2][idx_filter]
            temp = np.asarray([cent_tmp1, cent_tmp2, cent_tmp3])
            rotation = pca.fit(temp).transform(temp)
            rotation = recalRotation(rotation)
            rotations.append(rotation)


        # axis: red for x, green for y, blue for z
        axisPoints = []
        axisPointsColor = []
        for i in range(0, cluster_count):
            [axisPoints_tmp, axisPointsColor_tmp] = calAxisPoints(centroids_cam[i], rotations[i], 0.06, 50)

            if len(axisPointsColor) == 0:
                axisPoints = axisPoints_tmp
            else:
                axisPoints = np.row_stack((axisPointsColor, axisPoints_tmp))
            if len(axisPointsColor) == 0:
                axisPointsColor = axisPointsColor_tmp
            else:
                axisPointsColor = np.row_stack((axisPointsColor, axisPointsColor_tmp))


        for i in range(centroids_cam.shape[0]):
            imageAxisPix = calImageAxis(centroids_cam[i], rotations[i], 0.06, cameraIntrinsics)
            # cv2.circle(colorImg,(int(imageAxisPix[0,0]), int(imageAxisPix[0,1])), 5, (255,0,0))
            cv2.line(colorImg,(int(imageAxisPix[0,0]), int(imageAxisPix[0,1])),(int(imageAxisPix[1,0]), int(imageAxisPix[1,1])),(255, 0, 0))
            cv2.line(colorImg,(int(imageAxisPix[0,0]), int(imageAxisPix[0,1])),(int(imageAxisPix[2,0]), int(imageAxisPix[2,1])),( 0, 255,  0))
            cv2.line(colorImg,(int(imageAxisPix[0,0]), int(imageAxisPix[0,1])),(int(imageAxisPix[3,0]), int(imageAxisPix[3,1])),(0, 0,255))


        # kdata_lx.sem.tryAcquire()
        kdata_lx.colorImg = colorImg
        kdata_lx.depthImg = depthImg
        kdata_lx.inferImg = inferImg
        kdata_lx.clusters = clusters
        kdata_lx.pc1 = camPoints
        kdata_lx.pc2 = camPoints_clusters
        kdata_lx.axisPoints = axisPoints
        kdata_lx.axisPointsColor = axisPointsColor
        kdata_lx.camPointsColor = camPointsColor
        kdata_lx.camPointsColor_clusters = camPointsColor_clusters
        # kdata_lx.sem.release()
        self.pic_signal.emit()
        # rotations = np.array(rotations)
        # if len(k) > 0:
        #     for i in range(0, len(k)):
        #         dim = rotations.ndim
        #         print(k)
        #         if dim ==3:
        #             print(rotations)
        #             rotations[k[i],:, :] = [0]
        #             centroids_cam[k[i], :] = [0]
        #         else:
        #             rotations[k[i],:] = [0]
        #             centroids_cam[k[i]] = [0]
        # cluster_count = centroids_cam.shape(0)
        # [confidence_clusters, confidence_idx] = sort(confidence_clusters, 'descend');

        return centroids_cam, rotations, confidence_clusters

    def Fun_Suction_Grip(self):
        pass

    def Fun_Suction_Release(self):
        pass

    def move_to_tcp(self, target_tcp, tool_acc, tool_vel):
        fcLMove(target_tcp)
        tool_pos_tolerance = [0.01, 0.01, 0.01, 0.05, 0.05, 0.05]

        actual_pos = get_current_tcp()  # target_tcp
        t1 = time.time()
        while not (all([np.abs(actual_pos[j] - target_tcp[j]) < tool_pos_tolerance[j] for j in range(3)])
                   and all(
                    [np.abs(actual_pos[j + 3] - target_tcp[j + 3]) < tool_pos_tolerance[j + 3] for j in range(3)])):
            # [410.72627118678747, -57.835160228166224, 403.55144691507746, -0.2824616264256288, 0.8656988465117846, -0.4132565023628232]
            actual_pos = get_current_tcp()
            t2 = time.time()
            if (t2 - t1) > 3:
                return
            time.sleep(0.01)

    def suction_process(self):
        target_tcp_shooting = Fun_Eyeinhand_Shooting(False)
        # self.move_to_tcp(target_tcp_shooting, 1.5, 1)
        Object2Cam = self.client_srv()
        print(Object2Cam)
        return
        if Object2Cam:
            [target_tcp_suction, target_tcp_interim1,
             target_tcp_interim2] = Fun_Tool2Base_Suction_Interim(Object2Cam)
            self.move_to_tcp(target_tcp_interim1, 1.5, 1)
            self.move_to_tcp(target_tcp_suction, 0.5, 0.5)
            self.Fun_Suction_Grip()
            self.move_to_tcp(target_tcp_interim2, 0.5, 1)
            [target_tcp_release, target_tcp_interim_1, target_tcp_interim_2] = Fun_Eyeinhand_Release(False)
            self.move_to_tcp(target_tcp_interim_2, 1.5, 1)
            self.move_to_tcp(target_tcp_release, 0.5, 0.5)
            self.Fun_Suction_Release()
            time.sleep(1)
            self.move_to_tcp(target_tcp_interim_1, 0.5, 0.5)


    def run(self):
        while self.working == True:
            self.suction_process()
            time.sleep(0.5)



class RevNet(nn.Module):
    def __init__(self, baseNet='resnet50', pretrained=False):
        super(RevNet, self).__init__()

        if baseNet == 'resnet101':
            net_imported = models.resnet101(pretrained=pretrained)
        elif baseNet == 'resnet34':
            net_imported = models.resnet34(pretrained=pretrained)
        else:
            net_imported = models.resnet50(pretrained=pretrained)

        if baseNet == 'resnet34':
            out_size = 512 / 2
        else:
            out_size = 2048 / 2

        self.resTower1 = nn.Sequential(*list(net_imported.children())[:-3])
        # self.resTower2 = nn.Sequential(*list(net_imported.children())[:-3])
        self.conv_e1 = nn.Conv2d(int(out_size), int(out_size / 4), kernel_size=1, stride=1, bias=False)  # 2048,512
        self.conv_e2 = nn.Conv2d(int(out_size / 4), int(out_size / 16), kernel_size=1, stride=1, bias=False)  # 512,128
        self.conv_e3 = nn.Conv2d(int(out_size / 16), 3, kernel_size=1, stride=1, bias=False)  # 128,3
        self.upsample1 = nn.UpsamplingBilinear2d(scale_factor=2)

        n = self.conv_e1.kernel_size[0] * self.conv_e1.kernel_size[1] * self.conv_e1.out_channels
        self.conv_e1.weight.data.normal_(0, math.sqrt(2. / n))
        n = self.conv_e2.kernel_size[0] * self.conv_e2.kernel_size[1] * self.conv_e2.out_channels
        self.conv_e2.weight.data.normal_(0, math.sqrt(2. / n))
        n = self.conv_e3.kernel_size[0] * self.conv_e3.kernel_size[1] * self.conv_e3.out_channels
        self.conv_e3.weight.data.normal_(0, math.sqrt(2. / n))

    def forward(self, x, phase):
        if phase == 1:
            with torch.no_grad():
                x = self.resTower1(x)
        else:
            x = self.resTower1(x)
        x = self.conv_e1(x)
        x = self.conv_e2(x)
        x = self.conv_e3(x)
        x = self.upsample1(x)

        return x


def recalRotation(rotation):
    z = rotation[:, 2]
    z = normalize([z], norm='l2')
    z_cam = np.asarray([0, 0, 1])

    # if (z_cam * z.T < 0):
    if float(np.dot(z_cam, z.T)) < 0:
        z = z * (-1)

    crossProd = np.cross(z_cam, z)
    if (np.linalg.norm(crossProd, ord=2) == 0):
        return np.eye(3)

    rotAxis = normalize(crossProd, norm='l2')
    rotAngle = math.acos(np.dot(z_cam, z.T))

    # M = makehgtform('axisrotate',rotAxis,rotAngle)
    # x = np.arange(1,3)
    # y = np.arange(1,3)
    # rotation=M(x,y)
    r1 = R.from_rotvec(rotAxis)
    r2 = R.from_euler('xyz', [rotAngle, rotAngle, rotAngle])
    rotation = (r1 * r2).as_matrix()
    return rotation[0]


def isRotationMatrix(R):
    Rt = np.transpose(R)
    shouldBeIdentity = np.dot(Rt, R)
    I = np.identity(3, dtype=R.dtype)
    n = np.linalg.norm(I - shouldBeIdentity)
    return n < 1e-6


def rotationMatrixToEulerAngles(R):
    sy = math.sqrt(R[0, 0] * R[0, 0] + R[1, 0] * R[1, 0])
    singular = sy < 1e-6
    if not singular:
        x = math.atan2(R[2, 1], R[2, 2])
        y = math.atan2(-R[2, 0], sy)
        z = math.atan2(R[1, 0], R[0, 0])
    else:
        x = math.atan2(-R[1, 2], R[1, 1])
        y = math.atan2(-R[2, 0], sy)
        z = 0
    return np.array([x, y, z])


def cluster_filter(cluster, idx):
    # [value, ~] = sort(cluster, 1, 'descend');
    value = cluster[:]
    value = np.sort(value, axis=0)[::-1]  # descend
    sz = cluster.shape
    mid_point = value[round(sz[0] / 2), :]
    temp = np.subtract(cluster, mid_point)
    dist = 0
    for i in range(0, sz[1]):
        # xx = temp[:,i]
        dist = dist + (np.square(temp[:, i]))

    dist = np.sqrt(dist)
    index = np.argsort(dist)
    value = np.sort(dist)
    delta_dist = []

    if len(value) == 1:
        print(value)
        del_index = index(1)
    else:

        for i in range(0, len(value) - 1):
            delta_dist.append(value[i + 1] - value[i])

        delete_point_found = False
        for i in range(0, len(delta_dist)):
            if delta_dist[i] > 1e-2:
                delete_point_found = True
                break
        if delete_point_found:
            del_index = index[i + 1:]
            del_idx = idx[del_index]
            return del_idx
        else:
            del_index = []
            return []


def calAxisPoints(centroid, rotation, length, point_num):
    x = rotation[0].T
    y = rotation[1].T
    z = rotation[2].T
    dis = length / point_num

    xAxisPoints = []
    yAxisPoints = []
    zAxisPoints = []
    for i in range(0, point_num):
        xAxisPoints.append(centroid + x * (i * dis))
        yAxisPoints.append(centroid + y * (i * dis))
        zAxisPoints.append(centroid + z * (i * dis))
    # xAxisPoints = [(centroid+x*(i*dis)) for i in range(1, point_num+1)]
    # yAxisPoints = [(centroid+y*(i*dis)) for i in range(1, point_num+1)]
    # zAxisPoints = [(centroid+z*(i*dis)) for i in range(1, point_num+1)]
    axisPoints = np.vstack([xAxisPoints, yAxisPoints, zAxisPoints])
    axisPointsColor = np.vstack((np.tile([255, 0, 0], (point_num, 1)), np.tile([0, 255, 0], (point_num, 1)),
                                 np.tile([0, 0, 255], (point_num, 1))))
    return [axisPoints, axisPointsColor]


def calImageAxis(centroid, rotation, length, cameraIntrinsics):
    o = centroid.reshape(3, 1)
    xyz = rotation * length + np.tile(centroid, (3, 1)).T

    oxyz = np.hstack((o, xyz))
    imageAxisPix = np.dot(cameraIntrinsics, oxyz)
    imageAxisPix = imageAxisPix / np.tile(imageAxisPix[2], (3, 1))
    return imageAxisPix[0:2].T


def executable_path():
    return os.path.dirname(sys.argv[0])


def image_wrapper(colorImg, depthImg):
    mean = 0.456  # np.array([0.485, 0.456, 0.406])
    std = 0.225  # np.array([0.229, 0.224, 0.225])

    # change image data type
    grayImg = cv2.cvtColor(colorImg, cv2.COLOR_RGB2GRAY).astype('float32')
    depthImg = depthImg.astype('float32')

    # pre-process gray image
    grayImg = (grayImg / 255.0 - mean) / std
    grayImg = grayImg[:, :, np.newaxis]

    # pre-process depth image
    depthImg = depthImg / 10000
    depthImg = np.clip(depthImg, 0.0, 2.0)  # Depth range limit
    depthImg = (depthImg - mean) / std
    depthImg = depthImg[:, :, np.newaxis]

    # form data
    data = np.concatenate((grayImg, depthImg, depthImg), axis=2)

    # reshape dimensions
    data = torch.tensor(data, dtype=torch.float32)
    data = data.permute(2, 0, 1)  # dimensions [3,480,640]
    data = data.unsqueeze(dim=0)

    return data


def inference(inferImgSavePath, colorImg, deptImg):
    data = image_wrapper(colorImg, deptImg)

    with torch.no_grad():
        # move data to device
        data = data.to(torch_device)

        # forward inference
        t1 = time.time()
        out = net(data, phase=2)  # out size [1,3,60,80]
        t2 = time.time()

        # calculate inference image
        out = softmax(out)
        out = torch.nn.functional.interpolate(out, scale_factor=2, mode='bilinear', align_corners=True)
        inferImg = out[0, 1, :, :]  # size [120,160]
        # inferImg = (inferImg * 255).clamp_(0, 255).round().numpy().astype('uint8')
        # .cuda()
        # inferImg = (inferImg * 255).clamp_(0, 255).round().cuda().data.cpu().numpy().astype('uint8')
        inferImg = (inferImg * 255).clamp_(0, 255).round().numpy().astype('uint8')
        inferImg = cv2.resize(inferImg, (0, 0), fx=4, fy=4, interpolation=cv2.INTER_LINEAR)  # size [480,640]

    # print inference time usage
    print('Inference service time usage (in secs): %.3f' % (t2 - t1))

    cv2.imwrite(inferImgSavePath, inferImg)

################################################################################
#The actual visualization
class Visualization(HasTraits):
    pointcloud = None
    colors = None
    axisPoints = None
    axisPointsColors = None

    scene = Instance(MlabSceneModel, ())

    def __init__(self, **traits):
        HasTraits.__init__(self, **traits)



    #静态监听age属性的变化
    # def _age_changed(self,old,new):
    #     print("%s.age change: from %s to %s"%(self,old,new))
    # 'scene.activated',
    @on_trait_change([pointcloud,colors,axisPoints,axisPointsColors])
    def update_plot(self):
        # This function is called when the view is opened. We don't
        # populate the scene when the view is not yet open, as some
        # VTK features require a GLContext.

        # We can do normal mlab calls on the embedded scene.
        # self.scene.mlab.test_points3d()
        self.showData1()


    # the layout of the dialog screated
    # , Group("_", "pointcloud", "colors", "axisPoints", "axisPointsColors")
    view = View(Item('scene', editor=SceneEditor(scene_class=MayaviScene),
                     height=250, width=300, show_label=False),
                resizable=True # We need this to resize with the parent widget
                )

    def showData1(self):
        if(self.pointcloud is None):
            return
        if(self.colors is None):
            return
        if (self.axisPoints is None):
            return
        if (self.axisPointsColors is None):
            return
        self.scene.mlab.figure = self.scene.mlab.gcf()
        # pcd = o3d.io.read_point_cloud(file_path1)
        # colors = np.asarray(pcd.colors) * 255
        # pointcloud = np.asarray(pcd.points)
        x = self.pointcloud[0,:]  # x position of point
        xmin = np.amin(x, axis=0)
        xmax = np.amax(x, axis=0)
        y = self.pointcloud[1,:]  # y position of point
        ymin = np.amin(y, axis=0)
        ymax = np.amax(y, axis=0)
        z = self.pointcloud[2,:]  # z position of point
        zmin = np.amin(z, axis=0)
        zmax = np.amax(z, axis=0)
        self.scene.mlab.points3d(x, y, z,
                             self.colors[0,:],  # Values used for Color
                             mode="point",
                             # 灰度图的伪彩映射
                             colormap='spectral',  # 'bone', 'copper', 'gnuplot'
                             # color=(0, 1, 0),   # Used a fixed (r,g,b) instead
                             figure=self.scene.mlab.figure,
                             )
        # 绘制原点
        self.scene.mlab.points3d(0, 0, 0, color=(1, 1, 1), mode="sphere", scale_factor=0.2)
        # 绘制坐标
        axes = np.array(
            [[20.0, 0.0, 0.0, 0.0], [0.0, 20.0, 0.0, 0.0], [0.0, 0.0, 20.0, 0.0]],
            dtype=np.float64,
        )
        # x轴
        self.scene.mlab.plot3d(
            [0, axes[0, 0]],
            [0, axes[0, 1]],
            [0, axes[0, 2]],
            color=(1, 0, 0),
            tube_radius=None,
            figure=self.scene.mlab.figure,
        )
        # y轴
        self.scene.mlab.plot3d(
            [0, axes[1, 0]],
            [0, axes[1, 1]],
            [0, axes[1, 2]],
            color=(0, 1, 0),
            tube_radius=None,
            figure=self.scene.mlab.figure,
        )
        # z轴
        self.scene.mlab.plot3d(
            [0, axes[2, 0]],
            [0, axes[2, 1]],
            [0, axes[2, 2]],
            color=(0, 0, 1),
            tube_radius=None,
            figure=self.scene.mlab.figure,
        )


        ax = self.axisPoints[:,0]
        ay = self.axisPoints[:,1]
        az = self.axisPoints[:,2]
        self.scene.mlab.points3d(ax, ay, az,
                             self.axisPointsColors[:,0],  # Values used for Color
                             mode="point",
                             # 灰度图的伪彩映射
                             colormap='spectral',  # 'bone', 'copper', 'gnuplot'
                             # color=(0, 1, 0),   # Used a fixed (r,g,b) instead
                             figure=self.scene.mlab.figure,
                             )



################################################################################
# # The QWidget containing the visualization, this is pure PyQt4 code.
class MayaviQWidget(QtWidgets.QWidget):
    def __init__(self, parent=None):
        QtWidgets.QWidget.__init__(self, parent)
        layout = QtWidgets.QVBoxLayout(self)
        layout.setContentsMargins(0,0,0,0)
        layout.setSpacing(0)
        self.visualization = Visualization()

        # If you want to debug, beware that you need to remove the Qt
        # input hook.
        #QtCore.pyqtRemoveInputHook()
        #import pdb ; pdb.set_trace()
        #QtCore.pyqtRestoreInputHook()

        # The edit_traits call will generate the widget to embed.
        self.ui = self.visualization.edit_traits(parent=self,
                                                 kind='subpanel').control
        layout.addWidget(self.ui)
        self.ui.setParent(self)

    def updateData(self,pointClound,colors,axisPoint,axisPointColors):

        self.visualization.pointcloud = pointClound
        self.visualization.colors = colors
        self.visualization.axisPoints = axisPoint
        self.visualization.axisPointsColors = axisPointColors
        # self.visualization.trait_set([pointClound,colors,axisPoint,axisPointColors])
        self.visualization.showData1()

class CamHandler:
    def __init__(self):
        self.pipeline = rs.pipeline()
        self.config = rs.config()

        self.config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
        self.config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
        self.pf = self.pipeline.start(self.config)
        self.color_frame = ''
        self.depth_frame = ''
        self.color_img = ''
        self.depth_img = ''
        self.exec_path = executable_path()

    def getColoredImg(self):
        self.color_frame = self.frame.get_color_frame()
        self.color_img = np.asanyarray(self.color_frame.get_data())

    def getDepthImg(self):
        self.depth_frame = self.frame.get_depth_frame()
        self.depth_img = np.asanyarray(self.depth_frame.get_data())

    def getFrame(self):
        self.frame = self.pipeline.wait_for_frames()
        self.getColoredImg()
        self.getDepthImg()
        # Apply colormap on depth image (image must be converted to 8-bit per pixel first)
        depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(self.depth_img, alpha=0.03), cv2.COLORMAP_JET)
        depth_colormap_dim = depth_colormap.shape
        color_colormap_dim = self.color_img.shape

        # If depth and color resolutions are different, resize color image to match depth image for display
        if depth_colormap_dim != color_colormap_dim:
            resized_color_image = cv2.resize(self.color_img, dsize=(depth_colormap_dim[1], depth_colormap_dim[0]),
                                             interpolation=cv2.INTER_AREA)
            images = np.hstack((resized_color_image, depth_colormap))
        else:
            images = np.hstack((self.color_img, depth_colormap))

        ct = time.time()
        local_time = time.localtime(ct)
        timestamp = time.strftime("%Y%m%d_%H%M%S", local_time)
        colorFileName = os.path.abspath(os.path.join(self.exec_path, "AIQA/picslx/frame-{}.color.png".format(timestamp)))
        depthFileName = os.path.abspath(os.path.join(self.exec_path, "AIQA/picslx/frame-{}.depth.png".format(timestamp)))

    def getIntrinsicArray(self):
        profile = self.pipeline.get_active_profile()
        # print('profile',profile)
        color_profile = rs.video_stream_profile(profile.get_stream(rs.stream.color))
        color_intrinsics = color_profile.get_intrinsics()
        return [color_intrinsics.fx, 0, color_intrinsics.ppx, 0, color_intrinsics.fy, color_intrinsics.ppy, 0, 0, 1]

    def stop_cam(self):
        self.pipeline.stop()




class Kawasaki(QMainWindow):

    def __init__(self, parent=None):
        super(Kawasaki, self).__init__(parent)
        self.initUI()
        self.core = KCore()
        self.core.pic_signal.connect(self.showImage)
        # self.core.start()

    def capture(self): # real signature unknown; restored from __doc__
        self.core.suction_process()

    def reset(self):
        pass

    def initUI(self):
        self.takePicAct = QAction(
            "Capture", self,
            statusTip="take a picture",
            triggered=self.capture)

        self.resetAct = QAction(
            "Capture", self,
            statusTip="reset",
            triggered=self.reset)

        self.standardToolbar = QToolBar("Standard")
        self.standardToolbar.setMovable(False)
        self.standardToolbar.setContextMenuPolicy(Qt.PreventContextMenu)
        self.standardToolbar.setMaximumHeight(26)
        self.standardToolbar.setObjectName("StandardToolBar")

        self.standardToolbar.addAction(self.takePicAct)
        self.standardToolbar.addAction(self.resetAct)
        self.addToolBar(self.standardToolbar)


        self.widget = QWidget()
        self.grid = QtWidgets.QGridLayout()
        self.widget.setLayout(self.grid)
        self.setCentralWidget(self.widget)



        self.label1 = QtWidgets.QLabel()
        self.label2 = QtWidgets.QLabel()
        self.label3 = QtWidgets.QLabel()
        self.label4 = QtWidgets.QLabel()


        self.label1.resize(640, 480)
        self.label2.resize(640, 480)
        self.label3.resize(640, 480)
        self.label4.resize(640, 480)

        self.mayavi_widget_pc1 = MayaviQWidget(self.widget)
        self.mayavi_widget_pc2 = MayaviQWidget(self.widget)


        self.grid.setContentsMargins(0, 0, 0, 0)
        self.grid.addWidget(self.label1, 0, 0)
        self.grid.addWidget(self.label2, 0, 1)
        self.grid.addWidget(self.label3, 0, 2)
        self.grid.addWidget(self.label4, 1, 0)
        self.grid.addWidget(self.mayavi_widget_pc1, 1, 1)
        self.grid.addWidget(self.mayavi_widget_pc2, 1, 2)

        tmp = cv2.cvtColor(kdata_lx.depthImg.astype(np.uint8), cv2.COLOR_BGR2RGB).astype('uint8')
        qimg = QtGui.QImage(tmp, 640, 480, QtGui.QImage.Format_RGB888)
        img = QtGui.QPixmap.fromImage(qimg)
        self.label1.setPixmap(img)
        self.label2.setPixmap(img)
        self.label3.setPixmap(img)
        self.label4.setPixmap(img)

        self.resize(1920, 960)


    def closeEvent(self, event):
        self.core.working = False
        self.core.wait()

    def showImage(self):
        # kdata_lx.sem.tryAcquire()
        tmp1 = cv2.cvtColor(kdata_lx.colorImg, cv2.COLOR_BGR2RGB).astype('uint8')
        qImg1 = QtGui.QImage(tmp1, 640, 480, QtGui.QImage.Format_RGB888)
        tmp2 = cv2.cvtColor(kdata_lx.depthImg.astype(np.uint8), cv2.COLOR_BGR2RGB).astype('uint8')
        qImg2 = QtGui.QImage(tmp2, 640, 480, QtGui.QImage.Format_RGB888)
        tmp3 = cv2.cvtColor(kdata_lx.inferImg, cv2.COLOR_BGR2RGB).astype('uint8')
        qImg3 = QtGui.QImage(tmp3, 640, 480, QtGui.QImage.Format_RGB888)
        tmp4 = cv2.cvtColor(kdata_lx.clusters.astype(np.uint8), cv2.COLOR_BGR2RGB).astype('uint8')
        qImg4 = QtGui.QImage(tmp4, 640, 480, QtGui.QImage.Format_RGB888)


        img1 = QtGui.QPixmap.fromImage(qImg1)
        self.label1.setPixmap(img1)

        img2 = QtGui.QPixmap.fromImage(qImg2)
        self.label2.setPixmap(img2)

        img3 = QtGui.QPixmap.fromImage(qImg3)
        self.label3.setPixmap(img3)

        img4 = QtGui.QPixmap.fromImage(qImg4)
        self.label4.setPixmap(img4)

        self.mayavi_widget_pc1.updateData(kdata_lx.pc1,kdata_lx.camPointsColor,kdata_lx.axisPoints,kdata_lx.axisPointsColor)
        self.mayavi_widget_pc2.updateData(kdata_lx.pc2, kdata_lx.camPointsColor_clusters,kdata_lx.axisPoints,kdata_lx.axisPointsColor)
        # self.plot_container_pc1.draw_graph(kdata_lx.pc1[0], kdata_lx.pc1[1], kdata_lx.pc1[2], kdata_lx.camPointsColor,kdata_lx.axisPointsColor)
        # self.plot_container_pc2.draw_graph(kdata_lx.pc2[0], kdata_lx.pc2[1], kdata_lx.pc2[2], kdata_lx.camPointsColor_clusters,kdata_lx.axisPointsColor)
        # kdata_lx.sem.release()



if __name__ == "__main__":
    torch_device = torch.device("cuda:0" if torch.cuda.is_available() else "cpu")
    # load model
    net = RevNet(baseNet='resnet34', pretrained=False)
    softmax = nn.Softmax(dim=1)

    # load model parameters
    # load model parameters
    modelPath = os.path.join(executable_path(),
                             'models/snapshot-model_param_on_epoch_29_1T_gdd.pth')
    states = torch.load(modelPath, map_location=torch.device(torch_device))
    net.load_state_dict({k.replace('module.', ''): v for k, v in states['model'].items()})
    net = net.to(torch_device)
    softmax = softmax.to(torch_device)
    net.eval()
    app = QApplication.instance()
    if app is None:
        app = QApplication(sys.argv)
    kawasaki = Kawasaki()
    kawasaki.show()

    sys.exit(app.exec_())