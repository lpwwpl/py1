import torch
import torch.nn as nn
import torchvision.models as models
import cv2 as cv
from sklearn.preprocessing import normalize
from scipy.spatial.distance import pdist, squareform
from Kawasaki_Control import Kawasaki_cntl_diag
from UtilSet import *
from ThreeDSurfaceGraphWindow import ThreeDSurfaceGraphWindow
from matplotlib import cm
from matplotlib.pyplot import figure
from matplotlib.ticker import LinearLocator, FormatStrFormatter
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas

from sklearn.decomposition import PCA
from scipy.spatial.transform import Rotation as R
import scipy.cluster.hierarchy as sch
from environment_pick_place import *
import pdb
import threading
import kdata_lx
from PyQt5 import QtCore, QtGui, QtWidgets
from PyQt5.QtWidgets import *
import urx
from PyQt5.QtGui import *
from PyQt5.QtCore import *
# from Robot import *
# from Part import *
# from FreeCAD import *
# import FreeCAD as App

pca = PCA()
currentSpeed = "300"
currentAccuracy = "300"
timestamp = None

# 机器人拍摄位姿计算(指定拍摄位姿)
# def Fun_Tool2Base_Shooting():
#     Tool2Base_Shooting = [0.10767868646835144*1000, -0.3787937230921638*1000, 0.15660759*1000, 0.009104541870731944,
#                           3.130781700067779, 0.05941563583525477]
#     return Tool2Base_Shooting











class KCore(QtCore.QThread):
    pic_signal = QtCore.pyqtSignal()

    def __init__(self, parent=None):
        super(KCore, self).__init__(parent)
        self.working = True
        #self.cam = CamHandler()
        self.parent = parent
        self.exec_path = executable_path()
        intriFileName = os.path.abspath(os.path.join(self.exec_path, "picslx/test-camera-intrinsics.txt"))
        self.colorCamIntrinsics = np.loadtxt(intriFileName, dtype=float)
        # print(self.colorCamIntrinsics)
        t_max = 10.0,
        steps_per_seg = 2,
        height = 0.32

        self.ee = Suction

        # planner_cls = planners.PickPlacePlanner if self.ee == Suction else planners.PushPlanner
        # self._planner = planner_cls(steps_per_seg, t_max, height)

        # 0.05, 0, 0.05
        self.extrinsic = None
        self.pose_of_capture =None

    def __del__(self):
        self.working = False
        #self.cam.stop_cam()

    def text_read(self):
        # Cam2Base_rm = [[0.99958932, -0.02117886, 0.01930385, -0.06328652],
        #                [0.0221976, 0.99828422, -0.0541838, -0.16513022],
        #                [-0.018123108, 0.05459005, 0.99834437, -0.11598357], [0, 0, 0, 1]]
        if kdata_lx.isVirt:
            # Cam2Base_rm = [[9.99988512e-01, - 2.01915004e-03,  4.34717410e-03,  3.66447867e-02],
            #                  [-2.05280673e-03, - 9.99967848e-01,  7.75170904e-03,  9.82474297e-04],
            #                 [4.33138246e-03, - 7.76054390e-03, - 9.99960506e-01,  3.62461623e-02],

            # Cam2Base_rm = [[ 9.99988281e-01, -2.01676429e-03,  4.40121636e-03,  3.33351252e-02],
            #                  [-2.05082873e-03, -9.99967873e-01,  7.74903915e-03,  9.83412530e-04],
            #                  [ 4.38544698e-03, -7.75797448e-03, -9.99960290e-01,  7.62382200e-02],
            #                  [ 0.00000000e+00,  0.00000000e+00,  0.00000000e+00, 1.00000000e+00]]
            # Cam2Base_rm = [[1, 0, 0, 0.05],
            #                [0, -1, 0,0],
            #                [0, 0, -1, 0.05], [0, 0, 0, 1]]
            Cam2Base_rm = [[9.99994199e-01,  9.47682884e-04,  3.27178992e-03,  5.14732905e-02],
                            [9.29586803e-04 ,- 9.99984288e-01 , 5.52803946e-03,  2.11069855e-03],
                            [3.27697734e-03 ,- 5.52496598e-03 ,- 9.99979368e-01, 3.58851006e-02],
                            [0.00000000e+00,  0.00000000e+00,  0.00000000e+00,  1.00000000e+00]]


        if kdata_lx.isReal:
            # Cam2Base_rm = [[0.99958932, -0.02117886, 0.01930385, -0.06328652],
            #                [0.0221976, 0.99828422, -0.0541838, -0.16513022],
            #                [-0.01812318, 0.05459005, 0.99834437, -0.11598357], [0, 0, 0, 1]]
            Cam2Base_rm = [[   0.048402,     0.94935,     0.31047 ,     0.0939],
                             [   0.032417 ,    0.30918,    -0.95045 ,    0.18593],
                             [    -0.9983,    0.056068,    -0.01581 ,   0.033142],
                             [          0 ,          0 ,          0  ,         1]]
        return Cam2Base_rm

    def setEnv(self,env):
        self.env = env

    def setReal(self,real):
        self.real = real

    def start_env(self):
        self.env.start_manual_control()

    # 机器人拍摄位姿计算(指定拍摄位姿)
    def Fun_Eyeinhand_Shooting(self,flag):
        if flag == False:
            # qua =[0,0,0,1]
            # a=quaternion_to_euler_fc(qua)
            # print(a)
            Tool2Base_Shooting = [0.000, -0.000, 0.36010000000000003 ,0, 0, 0]
            # Tool2Base_Shooting = [0.4657*1000, -0.1556*1000, 0.36010000000000003*1000 , -92.4419938142771/180*3.1415926, 0.2653949997374077/180*3.1415926,
            #                       -178.95237249832894/180*3.1415926]
        else:
            Tool2Base_Shooting = [-0.26879975 , -0.32541972 , 0.1903955 , 0.009104541870731944,
                                  3.130781700067779,
                                  0.05941563583525477]
        return Tool2Base_Shooting


    def Fun_Tool2Base_Suction_Interim(self,Object2Cam):
        # Object2Cam = client_srv()
        Object2Cam_rm = rpy2rm(Object2Cam)
        Cam2Tool_rm = self.text_read()
        # Tool2Base = self.get_current_tcp()
        Tool2Base = self.pose_of_capture
        print('Tool2Base:{}'.format(Tool2Base))
        # base2world=np.array([[1, 0, 0,-0.5], [0, 1, 0, 0], [0, 0, 1,0 ], [0, 0, 0, 1]])
        Tool2Base_rm = rv2rm(Tool2Base)
        Object2Base = Tool2Base_rm.dot(Cam2Tool_rm).dot(Object2Cam_rm)
        Tool2Base_Suction_rm = Object2Base
        Tool2Base_Suction = rm2rv(Tool2Base_Suction_rm)  # 抓取位姿
        Move_rm = np.array([[1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 1, -0.05 ], [0, 0, 0, 1]])
        Tool2Base_Interim1_rm = Tool2Base_Suction_rm.dot(Move_rm)
        Tool2Base_Interim1 = rm2rv(Tool2Base_Interim1_rm)  # 过渡位姿
        Tool2Base_Interim2 = Tool2Base_Interim1
        return Tool2Base_Suction, Tool2Base_Interim1, Tool2Base_Interim2

    # 机器人放置位姿计算
    def     Fun_Eyeinhand_Release(self,flag):
        x = 0
        y = 0
        z = 0
        if flag == False:
            Tool2Base_Release = [(-0.10 - 0.12 * x) , (-0.36 + 0.08 * y) , (0.075 + 0.05 * z) ,
                                 0.00909552, 3.13077267,
                                 0.05922008]
        else:
            Tool2Base_Release = [(0.17 + 0.12 * x) , (-0.56 + 0.08 * y) , (0.075 + 0.05 * z) ,
                                 0.00909552,
                                 3.13077267,
                                 0.05922008]
        Tool2Base_Release_Interim1 = Tool2Base_Release + np.array([0, 0, 0.01 , 0, 0, 0])  # 过渡位姿
        Tool2Base_Release_Interim2 = Tool2Base_Release + np.array([0, 0, 0.01 , 0, 0, 0])  # 过渡位姿
        return Tool2Base_Release, Tool2Base_Release_Interim1, Tool2Base_Release_Interim2

    def get_current_tcp(self):
        if kdata_lx.isVirt and self.env:
            return self.env.get_current_tcp()
        if kdata_lx.isReal and self.real:
            return self.real.get_current_tcp()

    def virt_capture(self):
        if kdata_lx.isVirt and self.env:
            ret,pose = self.env.capture()
            self.pose_of_capture = pose
            self.extrinsic = self.env.extrinsic


            # filepath = "picsyy/lpw.color.png"
            # deptfilepath = "picsyy/lpw.depth.png"
            # cv.imwrite(filepath, kdata_lx.colorImg)
            # depth_mat = np.zeros((480, 640), np.float32)
            #
            # for y in range(480):
            #     for x in range(640):
            #         depth_short = kdata_lx.depthImg[y, x] * 10000
            #         depth_mat[y, x] = depth_short
            # cv.imwrite(deptfilepath, depth_mat)


            return ret
        return False

    def real_capture(self):
        # if kdata_lx.isReal and self.real:
        return self.real.capture()
        # return False

    def client_srv(self, prefix=None):

        #self.cam.getFrame()
        exec_path = executable_path()
        if prefix:
            colorFileName = os.path.abspath(os.path.join(exec_path, "picslx/{}.color.png".format(prefix)))
            deptFileName = os.path.abspath(os.path.join(exec_path, "picslx/{}.depth.png".format(prefix)))
            inferFileName = os.path.abspath(os.path.join(self.exec_path, "picslx/{}.infer.png".format(prefix)))
            color_img = cv.cvtColor(cv.imread(colorFileName, flags=1), cv.COLOR_BGR2RGB)
            depth_img = cv.imread(deptFileName, flags=-1)
        else:

            ###virtual image or realImage(camHandler)
            value = None
            if kdata_lx.isReal:
                value = self.real_capture()
            if not value:
                print("virt_capture}")
                value = self.virt_capture()
            if not value:
                return
            color_img = kdata_lx.colorImg
            depth_img = kdata_lx.depthImg

            depth_img = depth_img*10000
            # print(depth_img)
            # color_img=
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
                # EulerXYZ = rotations[0]
                # eulerAngle = rotationMatrixToEulerAngles(EulerXYZ)
                eulerAngle=[0,0,0]
                Object2Cam = [centroids_cam[0] , centroids_cam[1] , centroids_cam[2] , eulerAngle[0],
                              eulerAngle[1],
                              eulerAngle[2]]
            else:
                Transitions = centroids_cam[0]
                # EulerXYZ = rotations[0]
                eulerAngle = [0, 0, 0]
                # eulerAngle = rotationMatrixToEulerAngles(EulerXYZ)
                Object2Cam = [Transitions[0] , Transitions[1] , Transitions[2] , eulerAngle[0],
                              eulerAngle[1],
                              eulerAngle[2]]

        return Object2Cam

    def postProcess(self, ShowImages=False, colorImg=None, depthImg=None, inferImg=None, cameraIntrinsics=None):
        iheight = 480
        iwidth = 640
        depthImg = depthImg.astype('double') / 10000.0;

        threthold_lowerbound = 0.3
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
            if len(camPointsColor_clusters) == 0:
                camPointsColor_clusters = temp
            else:
                camPointsColor_clusters = np.concatenate((camPointsColor_clusters, temp))


        camX = (pixX - cameraIntrinsics[0, 2]) * pixZ / cameraIntrinsics[0, 0]
        camY = (pixY - cameraIntrinsics[1, 2]) * pixZ / cameraIntrinsics[1, 1]
        camZ = pixZ
        camPoints = [camX, camY, camZ]

        # camPointsColor = colorImg.reshape((-1, 3), order='F')

        camPointsColor = []
        for i in range(0, 3):
            temp = colorImg[:, :, i]
            if len(camPointsColor) == 0:
                camPointsColor = temp
            else:
                camPointsColor = np.concatenate((camPointsColor, temp))

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
            # rotation = pca.fit(temp).transform(temp)
            # rotation = recalRotation(rotation)

            rotation = [[-0.5082,0.28273 ,0.81351],[0.71813 ,  0.66054 ,  0.21905], [-0.47543 ,  0.69552 , -0.53872]]
            rotation = np.asarray(rotation)
            rotations.append(rotation)


        # axis: red for x, green for y, blue for z
        axisPoints = []
        axisPointsColor = []
        for i in range(0, cluster_count):
            [axisPoints_tmp, axisPointsColor_tmp] = calAxisPoints(centroids_cam[i], rotations[i], 0.06, 50)
            axisPoints.append(axisPoints_tmp)
            axisPointsColor.append(axisPointsColor_tmp)

        axisPoints = np.asarray(axisPoints)
        axisPointsColor=np.asarray(axisPointsColor)

        centroids_cam = np.asarray(centroids_cam)
        dim = centroids_cam.ndim
        try:
            if (dim == 1 and len(centroids_cam) > 0):
                imageAxisPix = calImageAxis(centroids_cam, rotations[0], 0.06, cameraIntrinsics)
                # cv2.circle(colorImg,(int(imageAxisPix[0,0]), int(imageAxisPix[0,1])), 5, (255,0,0))
                cv2.line(colorImg, (int(imageAxisPix[0, 0]), int(imageAxisPix[0, 1])),
                         (int(imageAxisPix[1, 0]), int(imageAxisPix[1, 1])), (255, 0, 0))
                cv2.line(colorImg, (int(imageAxisPix[0, 0]), int(imageAxisPix[0, 1])),
                         (int(imageAxisPix[2, 0]), int(imageAxisPix[2, 1])), (0, 255, 0))
                cv2.line(colorImg, (int(imageAxisPix[0, 0]), int(imageAxisPix[0, 1])),
                         (int(imageAxisPix[3, 0]), int(imageAxisPix[3, 1])), (0, 0, 255))
            else:
                for i in range(centroids_cam.shape[0]):
                    imageAxisPix = calImageAxis(centroids_cam[i], rotations[i], 0.06, cameraIntrinsics)
                    # cv2.circle(colorImg,(int(imageAxisPix[0,0]), int(imageAxisPix[0,1])), 5, (255,0,0))
                    cv2.line(colorImg, (int(imageAxisPix[0, 0]), int(imageAxisPix[0, 1])),
                             (int(imageAxisPix[1, 0]), int(imageAxisPix[1, 1])), (255, 0, 0))
                    cv2.line(colorImg, (int(imageAxisPix[0, 0]), int(imageAxisPix[0, 1])),
                             (int(imageAxisPix[2, 0]), int(imageAxisPix[2, 1])), (0, 255, 0))
                    cv2.line(colorImg, (int(imageAxisPix[0, 0]), int(imageAxisPix[0, 1])),
                             (int(imageAxisPix[3, 0]), int(imageAxisPix[3, 1])), (0, 0, 255))
        except Exception as e:
            print(e)
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
        if kdata_lx.isVirt:
            self.env.gripper(0)
        if kdata_lx.isReal:
            self.real.gripper(0)

    def Fun_Suction_Release(self):
        if kdata_lx.isVirt:
            self.env.gripper(1)
        if kdata_lx.isReal:
            self.real.gripper(1)

    def move_to_tcp(self, target_tcp, tool_acc, tool_vel,active=False):
        if kdata_lx.isReal:
            print(target_tcp)
            self.real.movep(target_tcp)
        if kdata_lx.isVirt:
            self.env.movep_6dof(target_tcp, tool_acc, tool_vel,active)

    def real_pick_place(self,action):
        pick_pose = action['pose0']
        place_pose = action['pose1']
        self.real.movep(pick_pose)
        self.Fun_Suction_Grip()
        self.real.movep(place_pose)

    def suction_process(self):
        target_tcp_shooting = self.Fun_Eyeinhand_Shooting(False)  # 机器人拍摄位姿计算
        ## self.move_to_tcp(target_tcp_shooting, 1.5, 1)  # 移动至机器人拍摄位姿
        ##print("Fun_Eyeinhand_Shooting:{}".format(target_tcp_shooting))
        ##print("current:{}".format(self.env.get_current_tcp()))
        ##self.env.getobjs()
        # return
        Object2Cam = None
        try:
            Object2Cam = self.client_srv()
            print(Object2Cam)
        except Exception as e:
            print(e)
            raise(e)
        if Object2Cam:
            print("obj2cam:{}".format(Object2Cam))
            # temp=Object2Cam[0]
            # Object2Cam[0]=Object2Cam[1]
            # Object2Cam[1]=temp
            [target_tcp_suction, target_tcp_interim1,
             target_tcp_interim2] = self.Fun_Tool2Base_Suction_Interim(Object2Cam)  # 机器人抓取位姿和过渡位姿计算
            print("target_tcp_suction:{}".format(target_tcp_suction))
            # self.move_to_tcp(target_tcp_interim1, 1.5, 1)  # 移动至机器人抓取过渡位姿
            # self.move_to_tcp(target_tcp_suction, 0.5, 0.5,True)  # 移动至机器人抓取位姿
            # self.Fun_Suction_Grip()
            # self.move_to_tcp(target_tcp_interim2, 0.5, 1)  # 移动至机器人过渡位姿
            # [target_tcp_release, target_tcp_interim_1, target_tcp_interim_2] = self.Fun_Eyeinhand_Release(False)  # 机器人放置位姿计算
            # self.move_to_tcp(target_tcp_interim_2, 1.5, 1)  # 移动至机器人释放过渡位姿
            # self.move_to_tcp(target_tcp_release, 0.5, 0.5)  # 移动至机器人释放位姿
            # print("target_tcp_release:{}".format(target_tcp_release))
            # self.Fun_Suction_Release()
            # self.move_to_tcp(target_tcp_interim_1, 0.5, 0.5)  # 移动至机器人释放位姿

    # 机器人吸盘抓取过程
    def run(self):
        while self.working == True:
            self.suction_process()
            time.sleep(0.5)
            self.working =False



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
        data = data.to(kdata_lx.torch_device)

        # forward inference
        t1 = time.time()
        out = kdata_lx.net(data, phase=2)  # out size [1,3,60,80]
        t2 = time.time()

        # calculate inference image
        out = kdata_lx.softmax(out)
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



class Kawasaki_lx(QWidget):

    def __init__(self, parent=None):
        QtWidgets.QWidget.__init__(self, parent)
        self.initTorch()
        self.core = KCore(self)
        self.core.pic_signal.connect(self.showImage)
        self.initUI()
        self.initCaptureList()
        self.extrinsic = None
        # self.extrinsic =  [[0.99958932, -0.02117886, 0.01930385, -0.06328652], [0.0221976, 0.99828422, -0.0541838, -0.16513022],
        #            [-0.01812318, 0.05459005, 0.99834437, -0.11598357], [0, 0, 0, 1]]
        # self.core.start()

    def initTorch(self):
        kdata_lx.torch_device = torch.device("cuda:0" if torch.cuda.is_available() else "cpu")
        # load model
        kdata_lx.net = RevNet(baseNet='resnet34', pretrained=False)
        kdata_lx.softmax = nn.Softmax(dim=1)

        # load model parameters
        # load model parameters
        modelPath = os.path.join(executable_path(),
                                 'models/snapshot-model_param_on_epoch_29_1T_gdd.pth')
        states = torch.load(modelPath, map_location=torch.device(kdata_lx.torch_device))
        kdata_lx.net.load_state_dict({k.replace('module.', ''): v for k, v in states['model'].items()})
        kdata_lx.net = kdata_lx.net.to(kdata_lx.torch_device)
        kdata_lx.softmax = kdata_lx.softmax.to(kdata_lx.torch_device)
        kdata_lx.net.eval()

    def manual_capture(self): # real signature unknown; restored from __doc__
        self.real_capture()
        # self.core.suction_process()

    def viewItem(self):
        if self.captureList.currentText() == 'None':
            kdata_lx.depthImg = kdata_lx.zero
            kdata_lx.colorImg = kdata_lx.zero
            kdata_lx.inferImg = kdata_lx.zero
            self.showImage()
            return
        try:
            self.core.client_srv(self.captureList.currentText())
        except Exception as e:
            print(e)


    def initCaptureList(self):
        files_path ="./picslx"
        strList = ['None']
        for fl in os.listdir(files_path):
            fileInfo = QFileInfo(fl)
            suffix = fileInfo.suffix()
            if suffix == 'png':
                strList.append(fileInfo.baseName())
        strList = set(strList)
        self.captureList.blockSignals(True)
        self.captureList.addItems(strList)
        self.captureList.blockSignals(False)
        try:
            self.captureList.setCurrentText('None')
        except Exception as e:
            print(e)

    def auto(self):
        self.core.working = True
        self.core.start()

    def stop(self):
        self.core.working = False
        self.core.wait()

    def initUI(self):

        self.captureList = QComboBox()
        self.captureList.currentIndexChanged.connect(self.viewItem)
        self.btnCapture = QPushButton("Capture")
        self.btnAuto = QPushButton("Auto")
        self.btnStop = QPushButton("Stop")

        self.btnRealControl = QPushButton("Real")
        self.btnVirtControl = QPushButton("Virt")
        self.grid = QtWidgets.QGridLayout()
        self.setLayout(self.grid)

        self.controlDialog = Kawasaki_cntl_diag()
        self.env = Environment_Pick_Place()
        self.env.set_task(self.core)
        # self.virt()
        # self.control = Kawasaki_Control()
        self.label1 = QtWidgets.QLabel()
        self.label2 = QtWidgets.QLabel()
        self.label3 = QtWidgets.QLabel()
        self.label4 = QtWidgets.QLabel()


        self.label1.resize(640, 480)
        self.label2.resize(640, 480)
        self.label3.resize(640, 480)
        self.label4.resize(640, 480)


        self.plot_container_pc1 = ThreeDSurfaceGraphWindow()
        self.plot_container_pc2 = ThreeDSurfaceGraphWindow()

        self.grid.setContentsMargins(0, 0, 0, 0)

        # self.grid.addWidget(self.control, 0, 0,1,2)

        gBox = QGridLayout()
        widgetOpt = QWidget()
        widgetOpt.setLayout(gBox)
        self.btnCapture.setFixedSize(160,50)
        self.btnCapture.clicked.connect(self.manual_capture)
        self.btnAuto.setFixedSize(160, 50)
        self.btnStop.setFixedSize(100, 50)

        self.btnRealControl.setFixedSize(160, 50)
        self.btnVirtControl.setFixedSize(160, 50)
        self.btnVirtControl.clicked.connect(self.virt)
        self.btnRealControl.clicked.connect(self.onReal)
        self.btnAuto.clicked.connect(self.auto)
        self.btnStop.clicked.connect(self.stop)
        gBox.addWidget(self.btnCapture,0,0,1,1)
        gBox.addWidget(self.btnAuto,0,1,1,1)
        gBox.addWidget(self.btnRealControl, 0, 2, 1, 1)
        gBox.addWidget(self.btnVirtControl, 0, 3, 1, 1)
        gBox.addWidget(self.captureList,0,4,1,4)
        self.grid.addWidget(widgetOpt, 0, 1, 1, 2)
        self.grid.addWidget(self.label1, 1, 0)
        self.grid.addWidget(self.label2, 1, 1)
        self.grid.addWidget(self.label3, 1, 2)
        self.grid.addWidget(self.label4, 2, 0)
        self.grid.addWidget(self.plot_container_pc1, 2, 1)
        self.grid.addWidget(self.plot_container_pc2, 2, 2)


        tmp = cv2.cvtColor(kdata_lx.depthImg.astype(np.uint8), cv2.COLOR_BGR2RGB).astype('uint8')
        qimg = QtGui.QImage(tmp, 640, 480, QtGui.QImage.Format_RGB888)
        img = QtGui.QPixmap.fromImage(qimg)
        self.label1.setPixmap(img)
        self.label2.setPixmap(img)
        self.label3.setPixmap(img)
        self.label4.setPixmap(img)

        self.core.setEnv(self.env)
        self.core.setReal(self.controlDialog.cntl)

    def virt(self):
        self.core.start_env()
        kdata_lx.isVirt = True

    def virt_capture(self):
        if kdata_lx.isVirt and self.env:
            ret = self.env.capture()
            filepath = "picsyy/lpw.color.png"
            deptfilepath = "picsyy/lpw.depth.png"
            cv.imwrite(filepath,kdata_lx.colorImg)
            depth_mat = np.zeros((480,640),np.float32)


            for y in range(480):
                for x in range(640):
                    depth_short = kdata_lx.depthImg[y,x]*10000
                    depth_mat[y,x]=depth_short
            cv.imwrite(deptfilepath,depth_mat)
            self.extrinsic = self.env.extrinsic
            return ret
        return False


    def real_capture(self):
        if kdata_lx.isReal and self.core.real:
            return self.core.real_capture()
        return False

    def onReal(self):
        # self.env.stop_manual_control()
        # del self.env
        self.controlDialog.show()
        kdata_lx.isReal =  True

    def movej(self,tcp,acc,vel):
        if kdata_lx.isVirt:
            self.env.moveJ(tcp,acc,vel)
        if kdata_lx.isReal:
            self.controlDialog.moveJ(tcp,acc,vel)

    def get_current_tcp(self):
        if kdata_lx.isVirt:
            return self.env.get_current_tcp()
        else:
            return self.controlDialog.cntl.get_current_tcp()

    def closeEvent(self, event):
        self.core.working = False
        self.core.wait()
        # self.control.close()
        self.controlDialog.close()
        event.accept()

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

        if kdata_lx.isVirt:
            return
        self.plot_container_pc1.draw_graph(kdata_lx.pc1[0], kdata_lx.pc1[1], kdata_lx.pc1[2], kdata_lx.camPointsColor,kdata_lx.axisPointsColor)
        self.plot_container_pc2.draw_graph(kdata_lx.pc2[0], kdata_lx.pc2[1], kdata_lx.pc2[2], kdata_lx.camPointsColor_clusters,kdata_lx.axisPointsColor)
        # kdata_lx.sem.release()



if __name__ == "__main__":
    kdata_lx.torch_device = torch.device("cuda:0" if torch.cuda.is_available() else "cpu")
    # load model
    kdata_lx.net = RevNet(baseNet='resnet34', pretrained=False)
    kdata_lx.softmax = nn.Softmax(dim=1)

    # load model parameters
    # load model parameters
    modelPath = os.path.join(executable_path(),
                             'models/snapshot-model_param_on_epoch_29_1T_gdd.pth')
    states = torch.load(modelPath, map_location=torch.device(kdata_lx.torch_device))
    kdata_lx.net.load_state_dict({k.replace('module.', ''): v for k, v in states['model'].items()})
    kdata_lx.net = kdata_lx.net.to(kdata_lx.torch_device)
    kdata_lx.softmax = kdata_lx.softmax.to(kdata_lx.torch_device)
    kdata_lx.net.eval()
    app = QApplication(sys.argv)
    kawasaki = Kawasaki_lx()
    kawasaki.show()

    sys.exit(app.exec())