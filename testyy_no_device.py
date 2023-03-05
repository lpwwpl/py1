import os
# os.environ['QT_API'] = 'pyside'
# os.environ["FORCE_CPU"] = 'True'
import cv2
import time
import cv2 as cv
from UtilSet import *
from Kawasaki_Control import Kawasaki_cntl_diag
from ThreeDSurfaceGraphWindow import ThreeDSurfaceGraphWindow
# import open3d as o3d
from PyQt5 import QtCore, QtGui, QtWidgets
from PyQt5.QtWidgets import *
from PyQt5.QtCore import *
import kdata_lx
from environment import Environment
import planners
from grippers import Suction
timestamp = None

class KCore(QtCore.QThread):
    pic_signal = pyqtSignal()

    def __init__(self, parent=None):
        super(KCore, self).__init__(parent)
        self.working = True
        self.parent = parent
        #self.cam = CamHandler()
        self.exec_path = executable_path()
        # intriFileName = os.path.abspath(os.path.join(self.exec_path, "picyy/test-camera-intrinsics.txt"))
        # self.colorCamIntrinsics = np.loadtxt(intriFileName, dtype=float)

        t_max = 10.0,
        steps_per_seg = 2,
        height = 0.32

        self.ee = Suction

        planner_cls = planners.PickPlacePlanner if self.ee == Suction else planners.PushPlanner
        self._planner = planner_cls(steps_per_seg, t_max, height)

        # 0.05, 0, 0.05
        self.extrinsic = None

    def __del__(self):
        self.working = False
        #self.cam.stop_cam()

    # 边缘提取
    def edge_demo(self,image):
        blurred = cv.GaussianBlur(image, (5, 5), 0)  # 高斯降噪，适度
        gray = cv.cvtColor(blurred, cv.COLOR_BGR2GRAY)
        # 求梯度
        xgrd = cv.Sobel(gray, cv.CV_16SC1, 1, 0)
        ygrd = cv.Sobel(gray, cv.CV_16SC1, 0, 1)

        # egde_output = cv.Canny(xgrd, ygrd, 50, 150)  # 50低阈值，150高阈值
        egde_output = cv.Canny(gray, 50, 150)  # 都可使用
        # cv.imshow('canny_edge', egde_output)
        return egde_output

    def setEnv(self,env):
        self.env = env

    def setReal(self,real):
        self.real = real

    def start_env(self):
        self.env.start_manual_control()

    def get_current_tcp(self):
        if kdata_lx.isVirt and self.env:
            return self.env.get_current_tcp()
        if kdata_lx.isReal and self.real:
            return self.real.get_current_tcp()

    def getDistance(self,p1, p2):
        return np.linalg.norm(p1 - p2)

    def measure_object(self,image, depthImg):
        # dst = cv.cvtColor(image,cv.COLOR_BGR2GRAY)
        # ret,binary = cv.threshold(dst,0,255,cv.THRESH_BINARY_INV|cv.THRESH_OTSU)
        # print('threshold valus: %s'%ret)
        # cv.imshow('binary image',binary)

        binary = self.edge_demo(image)
        dst = cv.cvtColor(binary, cv.COLOR_GRAY2BGR)
        contours, heriachy = cv.findContours(binary, cv.RETR_EXTERNAL,
                                             cv.CHAIN_APPROX_SIMPLE)  # 注意 我的版本只有两个返回参数，具体看函数说明是返回几个参数

        camPoints_clusters=[]
        cluster_count = len(contours)
        print(cluster_count)
        for i, contour in enumerate(contours):
            # area = cv.contourArea(contour)
            # rect = cv.boundingRect(contour)  # 得到轮廓的外接矩形

            minRect = cv.minAreaRect(contour)
            rectCnt = np.int64(cv.boxPoints(minRect))
            x, y, w, h = cv.boundingRect(contour)

            mm = cv.moments(contour)  # 得到几何矩
            type(mm)

            area = cv.contourArea(rectCnt)
            # print('contour area:%s' % area)

            # cam = CamHandler()
            # cam.getFrame()
            image_rgb = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
            # image_depth = cv2.applyColorMap(cv2.convertScaleAbs(depthImg, alpha=0.6), cv2.COLORMAP_JET)
            depth = depthImg.astype('double') / 10000
            # depth = depthImg.astype('double') / 10000
            # np.savetxt('some_array', depthImg)
            # cameraIntrinsics = cam.getIntrinsicArray()
            intriFileName = os.path.abspath(os.path.join(self.exec_path, "picsyy/test-camera-intrinsics.txt"))
            cameraIntrinsics = np.loadtxt(intriFileName, dtype=float)

            # depthImg = double(depthImg) / 10000.0; % in meters
            # 面积筛选
            if area >= 5:
                # cv.polylines(dst, pts=[rectCnt], isClosed=True, color=(0, 0, 255), thickness=1)

                first_max_dis = 0
                second_max_dis = 0

                first_max_point = np.array([0, 0])
                second_max_point = np.array([0, 0])

                rect_x = 0
                rect_y = 0

                for p1 in rectCnt:
                    min_dis = 100000
                    rect_x = rect_x + p1[0]
                    rect_y = rect_y + p1[1]
                    for p2 in contour:
                        len_ = self.getDistance(p1, p2)
                        if len_ < min_dis:
                            min_dis = len_

                    if min_dis > first_max_dis:
                        second_max_dis = first_max_dis
                        second_max_point = first_max_point

                        first_max_dis = min_dis
                        first_max_point = p1
                    elif min_dis > second_max_dis:
                        second_max_dis = min_dis
                        second_max_point = p1

                # 小头中点
                small_rect_center_x = (first_max_point[0] + second_max_point[0]) / 2.0
                small_rect_center_y = (first_max_point[1] + second_max_point[1]) / 2.0
                # 算角度
                radian = math.atan2((rect_y / 4 - small_rect_center_y), (rect_x / 4 - small_rect_center_x))
                angle = 180 - radian * 180 / 3.1415926
                print('%d rectangle 角度:%s' % (i, round(angle, 2)))
                # cv.circle(dst, (np.int(small_rect_center_x), np.int(small_rect_center_y)), 5, (0, 255, 255), -1)
                # cv.circle(dst, (np.int(rect_x/4), np.int(rect_y/4)), 5, (0, 255, 255), -1)

                # 多边形拟合
                approxCurve = cv.approxPolyDP(contour, 4, True)
                # print(approxCurve.shape)
                cv.drawContours(dst, contours, i, (255, 0, 0), 1)
                pixX_clusters = int(rect_x / 4)
                pixY_clusters = int(rect_y / 4)
                cv.arrowedLine(dst, (int(rect_x / 4), int(rect_y / 4)),
                               (int(small_rect_center_x), int(small_rect_center_y)), (0, 0, 255), 1, 8, 0, 0.5)

                pixZ_clusters = depth[int(rect_y / 4), int(rect_x / 4)]

                camX_clusters = (pixX_clusters - cameraIntrinsics[0, 2]) * pixZ_clusters / cameraIntrinsics[0, 0]
                camY_clusters = (pixY_clusters - cameraIntrinsics[1, 2]) * pixZ_clusters / cameraIntrinsics[1, 1]
                camZ_clusters = pixZ_clusters
                camPoints_clusters.append((camX_clusters, camY_clusters, camZ_clusters, 0, 0, 0))
                # print(camX_clusters, camY_clusters, camZ_clusters)
                cv.putText(dst, str(round(angle, 2)), (int(rect_x / 4), int(rect_y / 4)), cv.FONT_HERSHEY_DUPLEX, 0.5,
                           (0, 255, 0), 1)
        camPoints_clusters1 = np.array(camPoints_clusters)

        cv.imwrite("picsyy/results/res1.jpg", dst)
        kdata_lx.inferImg = dst

        # depthImg = depthImg.astype('double') / 10000.0
        # threthold_lowerbound = 0.5
        # affordance_threthold = depthImg.max().astype('double') / 255.0 * 0.64
        # if affordance_threthold < threthold_lowerbound:
        #     affordance_threthold = threthold_lowerbound
        # mask_th = 255 * affordance_threthold
        # mask_p = (depthImg >= mask_th)
        #
        # centroids_cam=[]
        # camPointsColor = []
        # rotations = []
        # for i in range(0, 3):
        #     temp = image[:, :, i]
        #     if len(camPointsColor) == 0:
        #         camPointsColor = temp
        #     else:
        #         camPointsColor = np.concatenate((camPointsColor, temp))
        # camPointsColor_clusters = []
        # for i in range(0, 3):
        #     temp = image[:, :, i]
        #     temp = temp[mask_p]
        #     if len(camPointsColor_clusters) == 0:
        #         camPointsColor_clusters = temp
        #     else:
        #         camPointsColor_clusters = np.concatenate((camPointsColor_clusters, temp))
        #
        # axisPoints = []
        # axisPointsColor = []
        # for i in range(0, cluster_count):
        #     [axisPoints_tmp, axisPointsColor_tmp] = calAxisPoints(centroids_cam[i], rotations[i], 0.06, 50)
        #     axisPoints.append(axisPoints_tmp)
        #     axisPointsColor.append(axisPointsColor_tmp)
        #
        # axisPoints = np.asarray(axisPoints)
        # axisPointsColor=np.asarray(axisPointsColor)
        # kdata_lx.axisPoints = axisPoints
        # kdata_lx.axisPointsColor = axisPointsColor
        # kdata_lx.camPointsColor = camPointsColor
        # kdata_lx.camPointsColor_clusters = camPointsColor_clusters

        return camPoints_clusters1

    def Fun_Eyeinhand_Shooting(self,flag):
        if flag == False:
            Tool2Base_Shooting = [0.000001+0.5, -0.00001, 0.35823156, 0, 0, 0]
        else:
            Tool2Base_Shooting = [-0.26879975 , -0.32541972 , 0.1903955 , 0.009104541870731944,
                                  3.130781700067779,
                                  0.05941563583525477]
        return Tool2Base_Shooting

    def text_read(self):
        # exec_path = executable_path()
        # camFileName = os.path.abspath(os.path.join(exec_path, 'picsyy/Cam2Tool.txt'))
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
        if kdata_lx.isVirt:
            Cam2Base_rm = [[0, 0, 0, 0.05],
                           [0, -1, 0,0],
                           [0, 0, -1, 0.05], [0, 0, 0, 1]]
        if kdata_lx.isReal:
            Cam2Base_rm = [[0.99958932, -0.02117886, 0.01930385, -0.06328652],
                           [0.0221976, 0.99828422, -0.0541838, -0.16513022],
                           [-0.01812318, 0.05459005, 0.99834437, -0.11598357], [0, 0, 0, 1]]
        return Cam2Base_rm



    def Fun_Tool2Base_Suction_Interim(self,Object2Cam):
        # Object2Cam = client_srv()
        Object2Cam_rm = rpy2rm(Object2Cam)
        Cam2Tool_rm = self.text_read()
        Tool2Base = self.get_current_tcp()
        Tool2Base_rm = rv2rm(Tool2Base)
        # Object2Base = Object2Cam_rm.dot(Cam2Tool_rm).dot(Tool2Base_rm)
        Object2Base = Tool2Base_rm.dot(Cam2Tool_rm).dot(Object2Cam_rm)
        Tool2Base_Suction_rm = Object2Base
        Tool2Base_Suction = rm2rv(Tool2Base_Suction_rm)

        Move_rm = np.array([[1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 1, -0.05 ], [0, 0, 0, 1]])
        Tool2Base_Interim1_rm = Tool2Base_Suction_rm.dot(Move_rm)
        Tool2Base_Interim1 = rm2rv(Tool2Base_Interim1_rm)
        Tool2Base_Interim2 = Tool2Base_Interim1
        return Tool2Base_Suction , Tool2Base_Interim1, Tool2Base_Interim2

    def Fun_Eyeinhand_Release(self,flag):
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
        Tool2Base_Release_Interim1 = Tool2Base_Release + np.array([0, 0, 0.01 , 0, 0, 0])
        Tool2Base_Release_Interim2 = Tool2Base_Release + np.array([0, 0, 0.01 , 0, 0, 0])
        return Tool2Base_Release , Tool2Base_Release_Interim1, Tool2Base_Release_Interim2

    def virt_capture(self):
        if kdata_lx.isVirt and self.env:
            ret = self.env.capture()
            filepath = "picsyy/lpw.color.png"
            deptfilepath = "picsyy/lpw.depth.png"
            cv.imwrite(filepath,kdata_lx.colorImg)
            # cv::Mat
            # depth_mat(frame_height, frame_width, CV_16UC1);
            # for (size_t y = 0; y < frame_height; y++)
            #     for (size_t x = 0; x < frame_width; x++) {
            #         unsigned short depth_short = (unsigned short)(depth_values[y * frame_width + x] * 10000); // 从摄像机读过来的深度数据的单位都是m，此处将单位转为10 ^ -4 m
            #     // depth_short = (depth_short >> 13 | depth_short << 3);
            #     depth_mat.at < unsigned short > (y, x) = depth_short;
            #     }
            # std::vector < int > compression_params;
            # compression_params.push_back(cv::IMWRITE_PNG_COMPRESSION);
            # compression_params.push_back(9);
            # cv::imwrite(depth_file, depth_mat, compression_params);

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
        if kdata_lx.isReal and self.real:
            return self.real.capture()
        return False

    def cv_obj_detect(self,prefix=None):

        # filepath_depth = "images/1.depth.PNG"
        # depthImg = cv.imread(filepath_depth)

        # cv.namedWindow("input image", cv.WINDOW_AUTOSIZE)
        # cv.imshow("input image", img)

        # cam = CamHandler()
        # cam.getFrame()
        # depthImg = cam.depth_img
        # img = cam.color_img

        # filepath = "images/lpw.color.png"
        # cv.imwrite(filepath, img)
        if prefix:
            filepath_depth = "picsyy/{}.depth.png".format(prefix)
            filepath = "picsyy/{}.color.png".format(prefix)
            img = cv.imread(filepath)  # blue green red
            depthImg = cv.imread(filepath_depth, flags=-1)

        else:
            value = self.real_capture()
            if not value:
                value = self.virt_capture()
            if not value:
                return np.asarray([])
            img = kdata_lx.colorImg
            depthImg = kdata_lx.depthImg
            depthImg = depthImg * 10000

        cameraIntrinsics = np.array([(6.12692566e+02, 0.00000000e+00, 3.23764923e+02),
                                     (0.00000000e+00, 6.12443115e+02, 2.33248459e+02),
                                     (0.00000000e+00, 0.00000000e+00, 1.00000000e+00)])
        # depthImg = cv.imwrite(filepath_depth, depthImg)
        # image_rgb = cv2.cvtColor(cam.color_img, cv2.COLOR_BGR2RGB)
        # image_depth = cv2.applyColorMap(cv2.convertScaleAbs(cam.depth_img, alpha=0.6), cv2.COLORMAP_JET)
        # depth = cv2.split(image_depth)[0] / 10000
        # cameraIntrinsics = cam.getIntrinsicArray()

        Object2Cam = self.measure_object(img, depthImg)

        image_rgb = cv.cvtColor(img, cv.COLOR_BGR2RGB)
        for i in range(Object2Cam.shape[0]):
            imageAxisPix = self.calImageAxis(Object2Cam[i], 0.06, cameraIntrinsics)
            cv2.circle(image_rgb, (int(imageAxisPix[0, 0]), int(imageAxisPix[0, 1])), 5, (255, 255, 255))
            # cv2.line(image_bgr, (int(imageAxisPix[0, 0]), int(imageAxisPix[0, 1])),
            #          (int(imageAxisPix[1, 0]), int(imageAxisPix[1, 1])), (255, 0, 0))
            # cv2.line(image_bgr, (int(imageAxisPix[0, 0]), int(imageAxisPix[0, 1])),
            #          (int(imageAxisPix[2, 0]), int(imageAxisPix[2, 1])), (0, 255, 0))
            # cv2.line(image_bgr, (int(imageAxisPix[0, 0]), int(imageAxisPix[0, 1])),
            #          (int(imageAxisPix[3, 0]), int(imageAxisPix[3, 1])), (0, 0, 255))
        # colorImg_uint8.write('D:/lpw.jpg')
        kdata_lx.colorImg = image_rgb
        kdata_lx.depthImg = depthImg


        # kdata_lx.sem.release()
        self.pic_signal.emit()

        return Object2Cam
        # cv.waitKey(0)
        # cv.destroyAllWindows()

    # Object2Cam
    def calImageAxis(self, Object2Cam, length, cameraIntrinsics):
        centroid = np.asarray([Object2Cam[0],Object2Cam[1],Object2Cam[2]])
        rotation = np.asarray([Object2Cam[3],Object2Cam[4],Object2Cam[5]])
        o = centroid.reshape(3, 1)
        xyz = rotation * length + np.tile(centroid, (3, 1)).T

        oxyz = np.hstack((o, xyz))
        imageAxisPix = np.dot(cameraIntrinsics, oxyz)
        imageAxisPix = imageAxisPix / np.tile(imageAxisPix[2], (3, 1))
        return imageAxisPix[0:2].T


    # def act(self, obs, info):
    #     """Get oracle action from planner."""
    #     if not self._actions:
    #         # Query the base oracle for pick and place poses.
    #         act = self._base_oracle.act(obs, info)
    #         if act is None:
    #             return
    #         self._actions = self._planner(self._env.get_ee_pose(), act['pose0'],
    #                                       act['pose1'])
    #     act = self._actions.pop(0)
    #     act['acts_left'] = len(self._actions)
    #     return act
    def client_srv(self,prefix=None):
        #self.cam.getFrame()
        # exec_path = executable_path()
        # colorFileName = os.path.abspath(os.path.join(exec_path, 'picyy/frame-20210419_144150.color.png'))
        # deptFileName = os.path.abspath(os.path.join(exec_path, "picyy/frame-20210419_144150.depth.png"))
        #
        # color_img = cv.cvtColor(cv.imread(colorFileName, flags=1), cv.COLOR_RGB2BGR)
        # depth_img = cv.imread(deptFileName, flags=-1)
        #
        # cameraIntrinsics = np.array([(6.12692566e+02, 0.00000000e+00, 3.23764923e+02),
        #                              (0.00000000e+00, 6.12443115e+02, 2.33248459e+02),
        #                              (0.00000000e+00, 0.00000000e+00, 1.00000000e+00)])

        # _, Object2Cam = self.image_detection(cameraIntrinsics,color_img,depth_img)

        Object2Cam = self.cv_obj_detect(prefix)

        dim = Object2Cam.ndim
        if (len(Object2Cam) > 0):
            if (dim == 1):
                pass
            else:
                Object2Cam = Object2Cam[0]

        return Object2Cam


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
            self.real.movep(target_tcp)
        if kdata_lx.isVirt:
            self.env.movep_6dof(target_tcp, tool_acc, tool_vel,active)

    def real_pick_place(self,action):
        pick_pose = action['pose0']
        place_pose = action['pose1']
        self.real.movep(pick_pose)
        self.Fun_Suction_Grip()
        self.real.movep(place_pose)

    def pick_place(self,action):
        if kdata_lx.isReal:
            self.real_pick_place(action)
        if kdata_lx.isVirt:
            pose0=action['pose0']
            pose1=action['pose1']
            qua0 = rpy2qt(pose0)

            qua1 = rpy2qt(pose1)
            action['pose0'] = [pose0[:3],qua0]
            action['pose1'] = [pose1[:3],qua1]
            self.env.pick_place_ex(action)

    def suction_process(self):
        target_tcp_shooting = self.Fun_Eyeinhand_Shooting(False)
        self.move_to_tcp(target_tcp_shooting, 1.5, 1)
        Object2Cam = self.client_srv()

        self.env.getobjs()
        if len(Object2Cam) > 0:
            print("obj2cam:{}".format(Object2Cam))
            [target_tcp_suction, target_tcp_interim1,
             target_tcp_interim2] = self.Fun_Tool2Base_Suction_Interim(Object2Cam)
            # pick_pose = self.Fun_Tool2Base_Suction_Interim(Object2Cam)
            # place_pose = self.Fun_Eyeinhand_Release(False)
            ## action = {'pose0': pick_pose, 'pose1': place_pose}
            ## self.pick_place(action)
            print("suction pos:\n{}\n{}\n{}".format(target_tcp_interim1,target_tcp_suction,target_tcp_interim2))
            # self.move_to_tcp(target_tcp_interim1, 1.5, 1)
            self.move_to_tcp(target_tcp_suction, 0.5, 0.5,True)
            self.Fun_Suction_Grip()
            print("Fun_Suction_Grip")
            self.move_to_tcp(target_tcp_interim2, 0.5, 1)
            [target_tcp_release, target_tcp_interim_1, target_tcp_interim_2] = self.Fun_Eyeinhand_Release(False)
            print("release pos:\n{}\n{}\n{}".format(target_tcp_interim1, target_tcp_release,target_tcp_interim2))
            self.move_to_tcp(target_tcp_interim_1, 1.5, 1)
            self.move_to_tcp(target_tcp_release, 0.5, 0.5)
            self.Fun_Suction_Release()
            print("Fun_Suction_Release")
            time.sleep(0.5)
            self.move_to_tcp(target_tcp_interim_2, 0.5, 0.5)


    def run(self):
        while self.working == True:
            self.suction_process()
            time.sleep(0.5)
            self.working = False




class Kawasaki_yy(QWidget):

    def __init__(self, parent=None):
        QtWidgets.QWidget.__init__(self, parent)
        self.core = KCore()
        # self.core.InitYolo(**vars(self.opt))
        self.core.pic_signal.connect(self.showImage)
        self.initUI()
        self.initCaptureList()

        # self.core.start()


    def manual_capture(self): # real signature unknown; restored from __doc__
        pass



    def auto(self):
        self.core.working = True
        self.core.start()

    def stop(self):
        self.core.working = False
        self.core.wait()

    def viewItem(self):
        if self.captureList.currentText() == 'None':
            kdata_lx.depthImg = kdata_lx.zero
            kdata_lx.colorImg = kdata_lx.zero
            kdata_lx.inferImg = kdata_lx.zero
            self.showImage()
            return
        self.core.client_srv(self.captureList.currentText())

    def initCaptureList(self):
        files_path ="./picsyy"
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
            # if len(strList)>=1:
            #     self.core.client_srv(self.captureList.currentText())
        except Exception as e:
            print(e)

    # def eventFilter(self, a0: 'QObject', a1: 'QEvent') -> bool:
    #     return QWidget.eventFilter(a0,a1)

    def initUI(self):
        self.captureList = QComboBox()
        self.captureList.currentIndexChanged.connect(self.viewItem)
        # self.captureList.installEventFilter(self)
        self.captureList.setMinimumWidth(640)
        self.btnCapture = QPushButton("Capture")
        self.btnAuto = QPushButton("Auto")
        self.btnStop = QPushButton("Stop")
        self.btnRealControl = QPushButton("Real")
        self.btnVirtControl = QPushButton("Virt")

        self.grid = QtWidgets.QGridLayout()
        self.setLayout(self.grid)

        # self.control = Kawasaki_Control()
        self.controlDialog = Kawasaki_cntl_diag()
        # self.env = Environment()

        self.env  = Environment()
        self.env.set_task(self.core)
        self.label1 = QtWidgets.QLabel()
        self.label2 = QtWidgets.QLabel()
        self.label3 = QtWidgets.QLabel()
        self.label1.resize(640, 480)
        self.label2.resize(640, 480)
        self.label3.resize(640, 480)
        self.plot_container_pc1 = ThreeDSurfaceGraphWindow()
        self.plot_container_pc2 = ThreeDSurfaceGraphWindow()


        self.grid.setContentsMargins(0, 0, 0, 0)
        gBox = QGridLayout()
        widgetOpt = QWidget()
        widgetOpt.setLayout(gBox)
        self.btnCapture.setFixedSize(130, 50)
        self.btnCapture.clicked.connect(self.manual_capture)
        self.btnAuto.setFixedSize(100, 50)
        self.btnStop.setFixedSize(100,50)
        self.btnAuto.clicked.connect(self.auto)
        self.btnStop.clicked.connect(self.stop)
        self.btnRealControl.setFixedSize(100, 50)
        self.btnVirtControl.setFixedSize(100, 50)
        self.btnVirtControl.clicked.connect(self.virt)
        self.btnRealControl.clicked.connect(self.real)
        gBox.addWidget(self.btnCapture, 0, 0, 1, 1)
        gBox.addWidget(self.btnAuto, 0, 1, 1, 1)
        gBox.addWidget(self.btnStop, 0, 2, 1, 1)
        gBox.addWidget(self.btnRealControl, 0, 3, 1, 1)
        gBox.addWidget(self.btnVirtControl, 0, 4, 1, 1)
        gBox.addWidget(self.captureList, 0, 5, 1, 4)
        self.grid.addWidget(widgetOpt, 0, 1, 1, 2)
        self.grid.addWidget(self.label1, 1, 0)
        self.grid.addWidget(self.label2, 1, 1)
        self.grid.addWidget(self.label3, 1, 2)
        self.grid.addWidget(self.plot_container_pc1, 2, 0)
        self.grid.addWidget(self.plot_container_pc2, 2, 1)

        tmp = cv2.cvtColor(kdata_lx.depthImg.astype(np.uint8), cv2.COLOR_BGR2RGB).astype('uint8')
        qimg = QtGui.QImage(tmp, 640, 480, QtGui.QImage.Format_RGB888)
        img = QtGui.QPixmap.fromImage(qimg)
        self.label1.setPixmap(img)
        self.label2.setPixmap(img)
        self.label3.setPixmap(img)
        self.setSizePolicy(QSizePolicy.Fixed, QSizePolicy.Fixed)

        self.core.setEnv(self.env)
        self.core.setReal(self.controlDialog.cntl)

    def virt(self):
        # self.control.hide()
        self.core.start_env()
        kdata_lx.isVirt = True

    def real(self):
        self.controlDialog.show()
        kdata_lx.isReal = True

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
        tmp3 = cv2.cvtColor(kdata_lx.inferImg.astype(np.uint8), cv2.COLOR_BGR2RGB).astype('uint8')
        qImg3 = QtGui.QImage(tmp3, 640, 480, QtGui.QImage.Format_RGB888)

        img1 = QtGui.QPixmap.fromImage(qImg1)
        self.label1.setPixmap(img1)

        img2 = QtGui.QPixmap.fromImage(qImg2)
        self.label2.setPixmap(img2)

        img3 = QtGui.QPixmap.fromImage(qImg3)
        self.label3.setPixmap(img3)

        # self.plot_container_pc1.draw_graph(kdata_lx.pc1[0], kdata_lx.pc1[1], kdata_lx.pc1[2], kdata_lx.camPointsColor,kdata_lx.axisPointsColor)
        # self.plot_container_pc2.draw_graph(kdata_lx.pc2[0], kdata_lx.pc2[1], kdata_lx.pc2[2], kdata_lx.camPointsColor_clusters,kdata_lx.axisPointsColor)
        # kdata_lx.sem.release()



if __name__ == "__main__":

    app = QApplication.instance()
    if app is None:
        app = QApplication(sys.argv)

    kawasaki = Kawasaki_yy()
    kawasaki.show()

    sys.exit(app.exec_())