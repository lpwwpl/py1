import pyrealsense2 as rs
import numpy as np
import cv2
import os
import sys
import torch
import torch.nn as nn
import time
import torchvision.models as models
import math
from pathlib import Path
import cv2 as cv
import urx
from sklearn.preprocessing import normalize
from scipy.spatial.distance import pdist, squareform

from matplotlib import cm
from matplotlib.pyplot import figure
from matplotlib.ticker import LinearLocator, FormatStrFormatter
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas

from sklearn.decomposition import PCA
from scipy.spatial.transform import Rotation as R
import scipy.cluster.hierarchy as sch
import pdb
import threading
import kdata_lx
from PyQt5 import QtCore, QtGui, QtWidgets
from PyQt5.QtWidgets import *
from PyQt5.QtGui import *
from PyQt5.QtCore import *
# from Robot import *
# from Part import *
# from FreeCAD import *
# import FreeCAD as App
from abb import *
from UtilSet import *
from utils.general import  increment_path
from environment_stereo import Environment_Stereo
from Kawasaki_Control import Kawasaki_cntl_diag
from Kawasaki_Control import Kawasaki_cntl_virt_diag_stereo
from grippers import Suction
class KCore(QtCore.QThread):
    pic_signal = pyqtSignal()

    def __init__(self, parent=None):
        super(KCore, self).__init__(parent)
        self.working = True
        self.parent = parent
        #self.cam = CamHandler()
        self.exec_path = executable_path()
        self.ee = Suction

    def __del__(self):
        self.working = False
        # self.cam.stop_cam()

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

class Kawasaki_cali_collect(QWidget):

    pic_signal = QtCore.pyqtSignal()
    calib_grid_step=0.05
    # workspace_limits = [[-0.1, 0.06], [-0.08, 0.04], [0.28, 0.34]]
    workspace_limits = [[-0.08, 0.03], [-0.03, 0.03], [0.30, 0.48]]
    # workspace_limits = [[0.26418, 0.31366], [-0.08751, 0.13863], [0.45002, 0.48573]]
    # tool_orientation = [1.226,-2.890,0.00]
    tool_orientation = [0,0,0]

    def __init__(self, parent=None):
        QtWidgets.QWidget.__init__(self, parent)
        self.initUI()
        self.save_dir= None

        self.gen_temp_dir()
        self.datafile='data_cali.txt'
        self.tcpfile='data_robotxyzrpy.txt'
        self.env = Environment_Stereo()
        self.core = KCore()
        self.env.set_task(self.core)

        self.controlDialog = Kawasaki_cntl_diag()
        self.control_Virt_dialog = Kawasaki_cntl_virt_diag_stereo()

        self.control_Virt_dialog.setEnv(self.env)
        self.core.setEnv(self.env)
        self.capture_num = 0

        self.tool_orientation = np.array(self.tool_orientation)
        self.workspace_limits = np.array(self.workspace_limits)
        self.auto_thread = None

    def gen_temp_dir(self,save_dir='runs/calibration_collect/temp',mkdir=True):
        path = Path(save_dir)  # os-agnostic
        dir = path if path.suffix == '' else path.parent  # directory
        if not dir.exists() and mkdir:
            dir.mkdir(parents=True, exist_ok=True)  # make directory
        self.save_dir = path
        return path

    def capture(self):
        # p = "{}\{}".format(os.path.abspath(self.save_dir),
        #                               "lpw.dat")

        # with open (p,'r') as f:
        #     lines = f.readlines()
        #     line = lines[0]
        #     self.capture_num = int(line)
        if kdata_lx.isReal and self.controlDialog:
            self.controlDialog.cntl.capture()

            record = "{}\{}".format(os.path.abspath(self.save_dir),self.datafile)
            record_tcp_path="{}\{}".format(os.path.abspath(self.save_dir),self.tcpfile)
            pose = self.controlDialog.cntl.pose_of_capture
            record_tcp=""
            if pose is not None:
                record_tcp = "{},{},{},{},{},{}".format(pose[0], pose[1], pose[2], pose[3]/180*3.1415926,pose[4]/180*3.1415926, pose[5]/180*3.1415926)
                poseFileName = "{}\{}".format(os.path.abspath(self.save_dir),
                                              "movingcam_robot_pose_{:0>2d}.dat".format(self.capture_num))

                with open(poseFileName, 'w+') as f:
                    f.write('f 2\n')
                    f.write("r {} {} {}\n".format(pose[3], pose[4], pose[5]))
                    f.write("t {} {} {}\n".format(pose[0] / 1000, pose[1] / 1000, pose[2] / 1000))

            ct = time.time()
            local_time = time.localtime(ct)
            # timestamp = time.strftime("%Y%m%d_%H%M%S", local_time)



            colorFileName = "{}\{}".format(os.path.abspath(self.save_dir), "Color_{:0>2d}.png".format(self.capture_num))
            depthFileName = "{}\{}".format(os.path.abspath(self.save_dir), "Depth_{:0>2d}.png".format(self.capture_num))

            cv.imwrite(colorFileName,  kdata_lx.colorImg)
            # cv.imwrite(depthFileName,  kdata_lx.depthImg)
            # with open(p,'w') as f:
            #     f.write(self.capture_num+1)
            #     f.close()
            depth_mat = np.zeros((480, 640), np.uint16)
            for y in range(480):
                for x in range(640):
                    depth_short = kdata_lx.depthImg[y, x] * 10000
                    depth_mat[y, x] = depth_short
            cv.imwrite(depthFileName, depth_mat)
            with open(record,'a+') as f:
                f.write(colorFileName+'\n')
            with open(record_tcp_path,'a+') as f:
                f.write(record_tcp+'\n')
            self.capture_num = self.capture_num + 1
        if kdata_lx.isVirt:
            # self.control_Virt_dialog.cntl.capture()
            # record = "{}\{}".format(os.path.abspath(self.save_dir), self.datafile)
            # record_tcp_path = "{}\{}".format(os.path.abspath(self.save_dir), self.tcpfile)
            # pose = self.control_Virt_dialog.cntl.pose_of_capture
            # record_tcp = "{},{},{},{},{},{}".format(pose[0], pose[1], pose[2], pose[3],pose[4], pose[5])
            # ct = time.time()
            # local_time = time.localtime(ct)
            # timestamp = time.strftime("%Y%m%d_%H%M%S", local_time)

            l_colorFileName = "{}\{}".format(os.path.abspath(self.save_dir), "Color_l_{:0>2d}.png".format(self.capture_num))
            # depthFileName = "{}\{}".format(os.path.abspath(self.save_dir), "Depth_{:0>2d}.png".format(self.capture_num))
            l_colorFileName = increment_path(l_colorFileName)
            # depthFileName = increment_path(depthFileName)
            l_color_img = cv.cvtColor(kdata_lx.color_l_streaming, cv2.COLOR_RGB2BGR)
            cv.imwrite(l_colorFileName.as_posix(),  l_color_img)

            r_colorFileName = "{}\{}".format(os.path.abspath(self.save_dir), "Color_r_{:0>2d}.png".format(self.capture_num))
            # depthFileName = "{}\{}".format(os.path.abspath(self.save_dir), "Depth_{:0>2d}.png".format(self.capture_num))
            r_colorFileName = increment_path(r_colorFileName)
            # depthFileName = increment_path(depthFileName)
            r_color_img = cv.cvtColor(kdata_lx.color_r_streaming, cv2.COLOR_RGB2BGR)
            cv.imwrite(r_colorFileName.as_posix(),  r_color_img)
            # cv.imwrite(depthFileName.as_posix(),  kdata_lx.depthImg)
            # depth_mat = np.zeros((480, 640), np.uint16)
            #
            # for y in range(480):
            #     for x in range(640):
            #         depth_short = kdata_lx.depthImg[y, x] * 10000
            #         depth_mat[y, x] = depth_short
            # cv.imwrite(depthFileName.as_posix(),depth_mat)
            # with open(record, 'a+') as f:
            #     f.write(colorFileName.as_posix() + '\n')
            # with open(record_tcp_path, 'a+') as f:
            #     f.write(record_tcp + '\n')

            self.capture_num = self.capture_num + 1

    def Auto(self):
        self.gen_save_dir()
        if not self.auto_thread or not self.auto_thread.is_alive():
            # self.auto_thread = threading.Thread(target=self.auto_thread_real_func)
            self.auto_thread = threading.Thread(target=self.auto_thread_func)
            self.auto_thread.start()

    def auto_thread_real_func(self):
        gridspace_x = np.linspace(self.workspace_limits[0][0], self.workspace_limits[0][1], int(
            1 + (self.workspace_limits[0][1] - self.workspace_limits[0][0]) / self.calib_grid_step))
        gridspace_y = np.linspace(self.workspace_limits[1][0], self.workspace_limits[1][1], int(
            1 + (self.workspace_limits[1][1] - self.workspace_limits[1][0]) / self.calib_grid_step))
        gridspace_z = np.linspace(self.workspace_limits[2][0], self.workspace_limits[2][1], int(
            1 + (self.workspace_limits[2][1] - self.workspace_limits[2][0]) / self.calib_grid_step))
        calib_grid_x, calib_grid_y, calib_grid_z = np.meshgrid(gridspace_x, gridspace_y, gridspace_z)

        num_calib_grid_pts = calib_grid_x.shape[0] * calib_grid_x.shape[1] * calib_grid_x.shape[2]
        calib_grid_x.shape = (num_calib_grid_pts, 1)
        calib_grid_y.shape = (num_calib_grid_pts, 1)
        calib_grid_z.shape = (num_calib_grid_pts, 1)
        calib_grid_pts = np.concatenate((calib_grid_x, calib_grid_y, calib_grid_z), axis=1)

        for calib_pt_idx in range(num_calib_grid_pts):
            tool_position = calib_grid_pts[calib_pt_idx, :]
            print('Calibration point: ', calib_pt_idx, '/', num_calib_grid_pts)
            tcp = [tool_position[0], tool_position[1], tool_position[2], self.tool_orientation[0],
                   self.tool_orientation[1], self.tool_orientation[2]]
            print(tcp)
            self.controlDialog.cntl.moveTo(tcp)
            # self.env.movep_6dof(tcp)
            self.capture()
            print("calib_pt_idx")


    def auto_thread_func(self):

        gridspace_x = np.linspace(self.workspace_limits[0][0], self.workspace_limits[0][1], int(
            1 + (self.workspace_limits[0][1] - self.workspace_limits[0][0]) / self.calib_grid_step))
        gridspace_y = np.linspace(self.workspace_limits[1][0], self.workspace_limits[1][1], int(
            1 + (self.workspace_limits[1][1] - self.workspace_limits[1][0]) / self.calib_grid_step))
        gridspace_z = np.linspace(self.workspace_limits[2][0], self.workspace_limits[2][1], int(
            1 + (self.workspace_limits[2][1] - self.workspace_limits[2][0]) / self.calib_grid_step))
        calib_grid_x, calib_grid_y, calib_grid_z = np.meshgrid(gridspace_x, gridspace_y, gridspace_z)

        num_calib_grid_pts = calib_grid_x.shape[0] * calib_grid_x.shape[1] * calib_grid_x.shape[2]
        calib_grid_x.shape = (num_calib_grid_pts, 1)
        calib_grid_y.shape = (num_calib_grid_pts, 1)
        calib_grid_z.shape = (num_calib_grid_pts, 1)
        calib_grid_pts = np.concatenate((calib_grid_x, calib_grid_y, calib_grid_z), axis=1)

        for calib_pt_idx in range(num_calib_grid_pts):
            tool_position = calib_grid_pts[calib_pt_idx, :]
            print('Calibration point: ', calib_pt_idx, '/', num_calib_grid_pts)
            tcp = [tool_position[0],tool_position[1],tool_position[2],self.tool_orientation[0],self.tool_orientation[1],self.tool_orientation[2]]
            print(tcp)
            # self.controlDialog.cntl.moveTo(tcp)
            self.env.movep_6dof(tcp)
            self.capture()
            print("calib_pt_idx")
            # time.sleep(1)
            # Wait for a coherent pair of frames: depth and color
            # camera_color_img, camera_depth_img,_ = self.env.render_camera(self.env.oracle_cams[0])


    def virt(self):
        self.core.start_env()
        self.control_Virt_dialog.show()
        kdata_lx.isVirt = True

    def real(self):
        self.control_Virt_dialog.close()
        self.controlDialog.show()
        kdata_lx.isReal = True

    def gen_save_dir(self, save_dir='runs/calibration_collect/exp'):
        self.save_dir = increment_path(save_dir, exist_ok=save_dir != 'runs/calibration_collect/exp', mkdir=True)  # increment save_dir


    def initUI(self):
        self.grid = QtWidgets.QGridLayout()
        self.setLayout(self.grid)

        self.btnCapture = QPushButton("Capture")
        self.btnAuto = QPushButton("Auto")
        self.btnReal = QPushButton("Real")
        self.btnVirt = QPushButton("Virt")


        self.grid.addWidget(self.btnCapture, 0, 1, 1, 1)
        self.grid.addWidget(self.btnAuto, 0, 2, 1, 1)
        self.grid.addWidget(self.btnReal,0,3,1,1)
        self.grid.addWidget(self.btnVirt, 0, 4, 1, 1)



        self.btnCapture.clicked.connect(self.capture)
        self.btnAuto.clicked.connect(self.Auto)
        self.btnVirt.clicked.connect(self.virt)
        self.btnReal.clicked.connect(self.real)
        self.setSizePolicy(QSizePolicy.Fixed,QSizePolicy.Fixed)

    def closeEvent(self, event):
        self.controlDialog.close()
        self.control_Virt_dialog.close()
        self.close()


if __name__ == "__main__":
    app = QApplication(sys.argv)
    kawasaki = Kawasaki_cali_collect()
    kawasaki.show()

    sys.exit(app.exec())