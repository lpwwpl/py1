import cv2
from pathlib import Path
import cv2 as cv
import urx
from UtilSet import *
import threading
import kdata_lx
from PyQt5 import QtCore, QtGui, QtWidgets
from PyQt5.QtWidgets import *
from PyQt5.QtGui import *
from PyQt5.QtCore import *
from abb import *
from utils.general import  increment_path
from CamHandler import CamHandler
from signleton import *
# def Singleton(cls):
#     _instance = {}
#
#     def _singleton(*args, **kargs):
#         if cls not in _instance:
#             _instance[cls] = cls(*args, **kargs)
#         return _instance[cls]
#
#     return _singleton
#
# @Singleton
class Kawasaki_Control(QWidget):
    # pic_signal = QtCore.pyqtSignal()
    def __init__(self, parent=None):
        QtWidgets.QWidget.__init__(self, parent)
        self._stopev = False
        self.initUI()
        self.timer = QTimer()
        self.timer.setInterval(200)
        self.timer.timeout.connect(self.updatexyzrpy)
        self.timer.start()

        self.rtimer = QTimer()
        self.rtimer.setInterval(200)
        self.rtimer.timeout.connect(self.updater_info)
        self.rtimer.start()
        self.running = "Not connected"

        self.r = None
        self.cam = None
        self.robot_type = kdata_lx.ROBOT_TYPE.NoType
        if self.robot_type == kdata_lx.ROBOT_TYPE.UR:
            self.open_ur()
        elif self.robot_type == kdata_lx.ROBOT_TYPE.ABB:
            self.openAbb()
        self.save_dir = None

        self.pose_of_capture = None
        self.pose = None
        # self.pic_signal.connect(self.showImage)
        # self.gen_temp_dir()
        # self.datafile = 'datapath.txt'
        # self.tcpfile = 'pose.txt'


    def updater_info(self):
        if self.r and self.robot_type == kdata_lx.ROBOT_TYPE.ABB:
            pass
        elif self.r and self.robot_type == kdata_lx.ROBOT_TYPE.UR:
            self.running = str(self.r.is_running())

        if self.stateLineEdit.text() != self.running:
            self.stateLineEdit.setText(self.running)

    def disconnect_ur(self):

        if self.r:
            self.r.close()

        self.r = None

    def disconnect_abb(self):
        if self.r:
            self.r.close()
        # self.r.close()
        self.r = None

    def show_error(self, msg, level=1):
        print("showing error: ", msg, level)
        # self.statusBar.show()
        # self.statusBar.setStyleSheet("QStatusBar { background-color : red; color : black; }")
        # self.statusBar.showMessage(str(msg))
        # QTimer.singleShot(1500, self.statusBar.hide)

    def open_ur(self):
        if self.r:
            try:
                self.disconnect_ur()
            except:
                print("Error while disconnecting")
        uri = self.addrComboBox.currentText()
        try:
            self.r = urx.Robot(uri)
        except Exception as ex:
            self.show_error(ex)

    def get_current_tcp(self):
        if self.r and self.robot_type == kdata_lx.ROBOT_TYPE.ABB:
            cartesian = self.r.get_cartesian()
            xyz = cartesian[0]
            rpy = quaternion_to_euler_fc(cartesian[1])
            return [xyz[0],xyz[1],xyz[2],rpy[0],rpy[1],rpy[2]]
        elif self.r and self.robot_type == kdata_lx.ROBOT_TYPE.UR:
            pose = self.r.getl()
            pose = [round(i, 4) for i in pose]
            return pose
        # return

    def gripper(self,openess):
        pass

    def abb_move_to(self,cartesian,acc=None,vel=None):
        self.r.set_cartesian(cartesian)

    def movep(self,tcp,acc=None,vel=None):
        if self.r and self.robot_type == kdata_lx.ROBOT_TYPE.ABB:
            qua = rpy2qt([0, 0, 0, tcp[3], tcp[4], tcp[5]])
            cartesian = [[tcp[0], tcp[1], tcp[2]],
                         qua]
            print(cartesian)
            self.thread = threading.Thread(target=self.abb_move_to, args=(cartesian,acc,vel))
            self.thread.start()
        elif self.r and self.robot_type == kdata_lx.ROBOT_TYPE.UR:
            self.thread = threading.Thread(target=self.ur_move_to, args=(tcp,acc,vel))
            self.thread.start()

    def moveTo(self,pose):
        qua = rpy2qt([0, 0, 0, pose[3] / 180 * 3.1415926, pose[4] / 180 * 3.1415926,
                      pose[5] / 180 * 3.1415926])
        cartesian = [[pose[0], pose[1], pose[2]],
                     qua]
        if self.r and self.robot_type == kdata_lx.ROBOT_TYPE.ABB:
            # self.r.set_cartesian(cartesian)
            self.thread = threading.Thread(target=self.abb_move_to, args=(cartesian,))
            self.thread.start()
        elif self.r and self.robot_type == kdata_lx.ROBOT_TYPE.UR:
            pose = [pose[0], pose[1], pose[2],
                    pose[3] / 180 * 3.1415926,
                    pose[4] / 180 * 3.1415926, pose[5] / 180 * 3.1415926]
            self.thread = threading.Thread(target=self.ur_move_to, args=(pose,))
            self.thread.start()
            self.changeUpdatePose()

    def moveTo(self):
        qua = rpy2qt([0, 0, 0, self.rr_spin.value()/180*3.1415926, self.pp_spin.value()/180*3.1415926, self.yy_spin.value()/180*3.1415926])
        cartesian = [[self.x_spin.value(), self.y_spin.value(), self.z_spin.value()],
                     qua]
        if self.r and self.robot_type == kdata_lx.ROBOT_TYPE.ABB:
            # self.r.set_cartesian(cartesian)
            self.thread = threading.Thread(target=self.abb_move_to, args=(cartesian,))
            self.thread.start()
        elif self.r and self.robot_type == kdata_lx.ROBOT_TYPE.UR:
            pose = [self.x_spin.value(), self.y_spin.value(), self.z_spin.value(), self.rr_spin.value()/180*3.1415926,
                    self.pp_spin.value()/180*3.1415926, self.yy_spin.value()/180*3.1415926]
            self.thread = threading.Thread(target=self.ur_move_to, args=(pose,))
            self.thread.start()
            self.changeUpdatePose()

    def ur_move_to(self, pose,acc=None,vel=None):
        self.r.movep(pose,acc,vel)

    # def gen_temp_dir(self, save_dir='runs/calibration_collect/temp', mkdir=True):
    #     path = Path(save_dir)  # os-agnostic
    #     dir = path if path.suffix == '' else path.parent  # directory
    #     if not dir.exists() and mkdir:
    #         dir.mkdir(parents=True, exist_ok=True)  # make directory
    #     self.save_dir = path
    #     return path

    def capture(self):
        if self.cam:
            self.cam.getFrame()
            src_color = self.cam.color_img

            color_img = cv.cvtColor(src_color, cv2.COLOR_BGR2BGRA)
            self.pose_of_capture = self.get_current_tcp()
            depth_img = self.cam.depth_img
            # print(depth_img)
            kdata_lx.colorImg = color_img
            kdata_lx.depthImg = depth_img*kdata_lx.depth_scale

            filepath = "picslx/lpw.color.png"
            deptfilepath = "picslx/lpw.depth.png"
            cv.imwrite(filepath, color_img)
            cv.imwrite(deptfilepath, kdata_lx.depthImg*10000)

            return True
        return False



    def gen_save_dir(self, save_dir='runs/calibration_collect/exp'):
        self.save_dir = increment_path(save_dir, exist_ok=save_dir != 'runs/calibration_collect/exp',
                                       mkdir=True)  # increment save_dir

    def initUI(self):
        self.x_spin = QDoubleSpinBox()
        self.x_spin.setRange(-1000, 1000)
        self.x_spin.setReadOnly(False)
        self.x_spin.setDecimals(2)
        self.x_spin.setSingleStep(0.01)
        self.y_spin = QDoubleSpinBox()
        self.y_spin.setReadOnly(False)
        self.y_spin.setRange(-1000, 1000)
        self.y_spin.setSingleStep(0.01)
        self.y_spin.setDecimals(2)
        self.z_spin = QDoubleSpinBox()
        self.z_spin.setRange(-1000, 1000)
        self.z_spin.setReadOnly(False)
        self.z_spin.setSingleStep(0.01)
        self.z_spin.setDecimals(2)

        self.rr_spin = QDoubleSpinBox()
        self.pp_spin = QDoubleSpinBox()
        self.yy_spin = QDoubleSpinBox()
        self.rr_spin.setDecimals(2)
        self.pp_spin.setDecimals(2)
        self.yy_spin.setDecimals(2)

        self.rr_spin.setRange(-360, 360)
        self.rr_spin.setSingleStep(0.01)
        self.pp_spin.setRange(-360, 360)
        self.pp_spin.setSingleStep(0.01)
        self.yy_spin.setRange(-360, 360)
        self.yy_spin.setSingleStep(0.01)

        # self.addToolBar(self.standardToolbar)
        # self.widget = QWidget()
        self.grid = QtWidgets.QGridLayout()
        # self.widget.setLayout(self.grid)
        # self.setCentralWidget(self.widget)
        self.setLayout(self.grid)

        self.label_tcp = QtWidgets.QLabel()
        self.label_readPose =  QtWidgets.QLabel()
        self.label1 = QtWidgets.QLabel()
        self.label2 = QtWidgets.QLabel()

        self.label1.resize(640, 480)
        self.label2.resize(640, 480)
        self.label_tcp.resize(1280,50)
        self.label_readPose.resize(1280,50)
        self.btnMoveTo = QPushButton("MoveTo")
        # self.btnCapture = QPushButton("Capture")
        # self.btnAuto = QPushButton("Auto")
        self.btnReadPose = QPushButton("ReadPose")
        self.btnWritePose = QPushButton("WritePose")
        self.btnOpenCam = QPushButton("OpenCam")
        self.btncloseCam = QPushButton("CloseCam")
        self.btnMoveTo.clicked.connect(self.moveTo)
        # self.btnCapture.clicked.connect(self.capture)
        # self.btnAuto.clicked.connect(self.Auto)
        self.btnOpenCam.clicked.connect(self.openCam)
        self.btnReadPose.clicked.connect(self.readPose)
        self.btnWritePose.clicked.connect(self.writePose)
        self.btncloseCam.clicked.connect(self.closeCam)
        self.grid.setContentsMargins(0, 0, 0, 0)
        self.grid.setSpacing(10)
        self.grid.addWidget(self.label1, 1, 0, 1, 3)
        self.grid.addWidget(self.label2, 1, 3, 1, 3)
        self.grid.addWidget(self.label_tcp,2,0,1,6)
        self.grid.addWidget(self.x_spin, 3, 0, 1, 1)
        self.grid.addWidget(self.y_spin, 3, 1, 1, 1)
        self.grid.addWidget(self.z_spin, 3, 2, 1, 1)
        self.grid.addWidget(self.rr_spin, 3, 3, 1, 1)
        self.grid.addWidget(self.pp_spin, 3, 4, 1, 1)
        self.grid.addWidget(self.yy_spin, 3, 5, 1, 1)
        self.grid.addWidget(self.label_readPose,4,0,1,6)
        # self.grid.addWidget(self.btnAuto, 3, 1, 1, 1)

        self.grid.addWidget(self.btnWritePose,5,0,1,1)
        self.grid.addWidget(self.btnReadPose,5,1,1,1)
        self.grid.addWidget(self.btnOpenCam, 5, 2, 1, 1)
        self.grid.addWidget(self.btncloseCam, 5, 3, 1, 1)
        self.grid.addWidget(self.btnMoveTo, 5, 4, 1, 1)

        self.btnstop = QPushButton("Stop")
        self.btnstop.clicked.connect(self.stop)
        self.grid.addWidget(self.btnstop, 5, 5, 1, 1)
        # self.grid.addWidget(self.btnCapture, 3, 5, 1, 1)

        self.settings = QSettings("vision", "vision")

        self.addrComboBox = QComboBox()
        sizePolicy = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.Expanding, QtWidgets.QSizePolicy.Fixed)
        self.addrComboBox.setSizePolicy(sizePolicy)
        self.addrComboBox.setEditable(True)
        self.addrComboBox.setModelColumn(0)
        self.addrComboBox.setObjectName("addrComboBox")
        self.grid.addWidget(self.addrComboBox, 0, 0, 1, 1)

        self.tComboBox = QComboBox()
        self.tComboBox.setSizePolicy(sizePolicy)
        self.tComboBox.setEditable(True)
        self.tComboBox.setModelColumn(0)
        self.tComboBox.setObjectName("addrComboBox")
        self.grid.addWidget(self.tComboBox, 0, 1, 1, 1)

        self.connectButton = QtWidgets.QPushButton("connect")
        self.connectButton.setObjectName("connectButton")
        self.grid.addWidget(self.connectButton, 0, 2, 1, 1)
        self.disconnectButton = QtWidgets.QPushButton("disconnect")
        self.disconnectButton.setObjectName("disconnectButton")
        self.grid.addWidget(self.disconnectButton, 0, 3, 1, 1)

        self.btnNoUpdatePose = QPushButton("CopyTo")
        self.btnNoUpdatePose.clicked.connect(self.changeUpdatePose)
        self.grid.addWidget(self.btnNoUpdatePose, 0, 4, 1, 1)

        self.connectButton.clicked.connect(self.connect)
        self.disconnectButton.clicked.connect(self.disconnect)

        self.tComboBox.insertItem(-1, "UR")
        self.tComboBox.insertItem(-1, "ABB")

        self._address_list = self.settings.value("address_list", ["10.13.0.206", "192.168.0.50"])
        for addr in self._address_list:
            self.addrComboBox.insertItem(-1, addr)

        self.stateLineEdit = QtWidgets.QLineEdit()
        self.stateLineEdit.setReadOnly(True)
        self.stateLineEdit.setObjectName("stateLineEdit")
        self.grid.addWidget(self.stateLineEdit, 0, 5, 1, 1)

        # self.statusBar = QtWidgets.QStatusBar()
        # self.statusBar.setObjectName("statusBar")
        # self.setStatusBar(self.statusBar)

        tmp = cv2.cvtColor(kdata_lx.depth_streaming.astype(np.uint8), cv2.COLOR_BGR2RGB).astype('uint8')
        qimg = QtGui.QImage(tmp, 640, 480, QtGui.QImage.Format_RGB888)
        img = QtGui.QPixmap.fromImage(qimg)
        self.label1.setPixmap(img)
        self.label2.setPixmap(img)

        self.pose_cur = 0
        self.lines = None

        with open('pose.txt', 'r') as f:
            self.lines = f.readlines(self.pose_cur)

            print(self.lines)
        # self.resize(1280, 640)
        self.setFixedSize(1280, 640)

        self.stopCam = True

        self.thread_cam = None


    def stop(self):
        if self.r and self.robot_type == kdata_lx.ROBOT_TYPE.ABB:
            self.r.close()

    def writePose(self):
        with open('pose.txt', 'a+') as f:
            pose = "{},{},{},{},{},{}".format( self.pose[0], self.pose[1], self.pose[2], self.pose[3], self.pose[4], self.pose[5])
            f.write(pose + '\n')

    def readPose(self):
        if self.lines:
            line = self.lines[self.pose_cur]
            xyzrpy = line.split(' ')

            pose = [float(xyzrpy[0]),float(xyzrpy[1]),float(xyzrpy[2]),float(xyzrpy[3]),float(xyzrpy[4]),float(xyzrpy[5])]
            xyz = [round(float(xyzrpy[0])*1000,2),round(float(xyzrpy[1])*1000,2),round(float(xyzrpy[2])*1000,2)]
            rpy = [float(xyzrpy[3]),float(xyzrpy[4]),float(xyzrpy[5])]

            self.x_spin.setValue(xyz[0])
            self.y_spin.setValue(xyz[1])
            self.z_spin.setValue(xyz[2])
            self.rr_spin.setValue(round(rpy[0],2))
            self.pp_spin.setValue(round(rpy[1],2))
            self.yy_spin.setValue(round(rpy[2],2))

            qt = rpy2qt(pose)

            rpy = [round(rpy[0],3),round(rpy[1],3),round(rpy[2],3)]
            qt = [round(qt[0],7),round(qt[1],7),round(qt[2],7),round(qt[3],7)]
            self.label_readPose.setText("current:{},{}".format(xyz,qt))

        self.pose_cur = self.pose_cur + 1
        if  self.pose_cur >= len(self.lines):
            self.pose_cur = 0

    def changeUpdatePose(self):
        if self.r and self.robot_type == kdata_lx.ROBOT_TYPE.ABB:
            xyz = [0, 0, 0]
            rpy = [0, 0, 0]
            cartesian = [[0, 0, 0], [0, 0, 0, 0]]
            cartesian = self.r.get_cartesian()
            xyz = cartesian[0]
            rpy = quaternion_to_euler_fc(cartesian[1])
            self.x_spin.setValue(xyz[0])
            self.y_spin.setValue(xyz[1])
            self.z_spin.setValue(xyz[2])
            self.rr_spin.setValue(rpy[0])
            self.pp_spin.setValue(rpy[1])
            self.yy_spin.setValue(rpy[2])
        elif self.r and self.robot_type == kdata_lx.ROBOT_TYPE.UR:
            pose = self.r.getl()
            pose = [round(i, 4) for i in pose]
            self.x_spin.setValue(pose[0])
            self.y_spin.setValue(pose[1])
            self.z_spin.setValue(pose[2])
            self.rr_spin.setValue(pose[3])
            self.pp_spin.setValue(pose[4])
            self.yy_spin.setValue(pose[5])

    def _save_address_list(self):
        uri = self.addrComboBox.currentText()
        if uri == self._address_list[0]:
            return
        if uri in self._address_list:
            self._address_list.remove(uri)
        self._address_list.insert(0, uri)
        self._address_list = self._address_list[:int(self.settings.value("address_list_max_count", 10))]
        self.settings.setValue("address_list", self._address_list)

    def _updater(self):
        while not self._stopev:
            time.sleep(0.5)
            self._update_robot_state()

    def _update_robot_state(self):
        pass
        # pose_str = ""
        # joints_str = ""
        # bits = 0
        # running = "Not connected"
        # if self.r:
        #
        # self.update_state.emit(running, pose_str, joints_str, bits)

    def connect(self):
        if self.tComboBox.currentText() == 'UR':
            self.open_ur()
            self.robot_type = kdata_lx.ROBOT_TYPE.UR
        elif self.tComboBox.currentText() == 'ABB':
            self.openAbb()
            self.robot_type = kdata_lx.ROBOT_TYPE.ABB
        self._save_address_list()
        print("Connected to ", self.r)

    def disconnect(self):
        if self.tComboBox.currentText() == 'UR':
            self.disconnect_ur()
            self.running = "Not connected"
        elif self.tComboBox.currentText() == 'ABB':
            self.disconnect_abb()
            self.running = "Not connected"

        self._stopev = True
        print("Disconnected")

    def openAbb(self):
        try:
            self.r = Robot()
        except Exception as e:
            print(e)

    def closeCam(self):
        if self.thread_cam:
            self.stopCam = True
            self.thread_cam.join()
            self.cam.stop_cam()
        kdata_lx.color_streaming = np.zeros((480, 640, 3), np.uint8)
        kdata_lx.depth_streaming = np.zeros((480, 640, 3), np.uint8)
        self.showImage()

    def openCam(self):
        if not self.thread_cam or not self.thread_cam.is_alive():

            try:
                self.cam = CamHandler()
                kdata_lx.depth_scale = self.cam.depth_scale
                print(self.cam.getIntrinsicArray())
                self.stopCam = False
            except Exception as e:
                print(e)
                return
            self.thread_cam = Thread(target=self.updateCam)
            self.stopCam = False
            self.thread_cam.start()

    def updateCam(self):
        while (True):
            if self.stopCam:
                break

            self.cam.getFrame()
            color_img = self.cam.color_img
            depth_img = self.cam.depth_img
            kdata_lx.color_streaming = color_img
            kdata_lx.depth_streaming = depth_img
            self.showImage()

            time.sleep(0.1)

    def updatexyzrpy(self):
        if self.r and self.robot_type == kdata_lx.ROBOT_TYPE.ABB:
            xyz = [0, 0, 0]
            rpy = [0, 0, 0]
            cartesian = [[0, 0, 0], [0, 0, 0, 0]]
            cartesian = self.r.get_cartesian()
            xyz = cartesian[0]
            rpy = quaternion_to_euler_fc(cartesian[1])
            # joints = self.r.get_joints()
            # print(joints)
            self.pose = [xyz[0],xyz[1],xyz[2],rpy[0]/180*3.1415926,rpy[1]/180*3.1415926,rpy[2]/180*3.1415926]
            self.label_tcp.setText("TCP:{} {}".format(xyz,rpy))
        elif self.r and self.robot_type == kdata_lx.ROBOT_TYPE.UR:
            pose = self.r.getl()
            pose = [round(i, 4) for i in pose]
            self.label_tcp.setText("TCP:{}").format(pose)
            # pose_str = str(pose)

    def showImage(self):
        # kdata_lx.sem.tryAcquire()
        tmp1 = cv2.cvtColor(kdata_lx.color_streaming, cv2.COLOR_BGR2RGB).astype('uint8')
        qImg1 = QtGui.QImage(tmp1, 640, 480, QtGui.QImage.Format_RGB888)
        tmp2 = cv2.cvtColor(kdata_lx.depth_streaming.astype(np.uint8), cv2.COLOR_BGR2RGB).astype('uint8')
        qImg2 = QtGui.QImage(tmp2, 640, 480, QtGui.QImage.Format_RGB888)

        img1 = QtGui.QPixmap.fromImage(qImg1)
        self.label1.setPixmap(img1)

        img2 = QtGui.QPixmap.fromImage(qImg2)
        self.label2.setPixmap(img2)

    def closeEvent(self, event):
        if self.thread:
            self._stopev = True
            self.thread.join()
        if self.thread_cam:
            self.stopCam = True
            self.thread_cam.join()
        self.close()




class Kawasaki_Control_Virt(QWidget):
    # pic_signal = QtCore.pyqtSignal()
    def __init__(self, parent=None):
        QtWidgets.QWidget.__init__(self, parent)
        self._stopev = False
        self.initUI()
        self.env = None

        self.timer = QTimer()
        self.timer.setInterval(200)
        self.timer.timeout.connect(self.updatexyzrpy)
        self.timer.start()

        self.pose_of_capture = None

    def setEnv(self,env):
        self.env = env

    def get_current_tcp(self):
        return self.env.get_current_tcp()


    def gripper(self,openess):
        pass

    def movep(self,tcp,acc=None,vel=None):
        self.env.movep_6dof(tcp, acc, vel)

    def moveTo(self):
        qua = rpy2qt([0, 0, 0, self.rr_spin.value(), self.pp_spin.value(), self.yy_spin.value()])
        pose = [self.x_spin.value(), self.y_spin.value(), self.z_spin.value(), self.rr_spin.value(),
                self.pp_spin.value(), self.yy_spin.value()]
        self.thread = threading.Thread(target=self.movep, args=(pose,))
        self.thread.start()
        self.changeUpdatePose()#########################

    def capture(self):

        ret = self.env.capture()
        self.pose_of_capture = self.env.get_current_tcp()

        # self.extrinsic = self.env.extrinsic
        return ret

    def gen_save_dir(self, save_dir='runs/calibration_collect/exp'):
        self.save_dir = increment_path(save_dir, exist_ok=save_dir != 'runs/calibration_collect/exp',
                                       mkdir=True)  # increment save_dir

    def initUI(self):

        self.x_spin = QDoubleSpinBox()
        # self.x_spin.setRange(-5, 5)
        self.x_spin.setReadOnly(False)
        self.x_spin.setDecimals(2)
        self.x_spin.setSingleStep(0.01)
        self.x_spin.setRange(-1000, 1000)
        self.y_spin = QDoubleSpinBox()
        self.y_spin.setReadOnly(False)
        self.y_spin.setRange(-1000, 1000)
        self.y_spin.setSingleStep(0.01)
        self.y_spin.setDecimals(2)
        self.z_spin = QDoubleSpinBox()
        self.z_spin.setRange(-1000, 1000)
        self.z_spin.setReadOnly(False)
        self.z_spin.setSingleStep(0.01)
        self.z_spin.setDecimals(2)

        self.rr_spin = QDoubleSpinBox()
        self.pp_spin = QDoubleSpinBox()
        self.yy_spin = QDoubleSpinBox()
        self.rr_spin.setRange(-360, 360)
        self.rr_spin.setSingleStep(0.01)
        self.pp_spin.setRange(-360, 360)
        self.pp_spin.setSingleStep(0.01)
        self.yy_spin.setRange(-360, 360)
        self.yy_spin.setSingleStep(0.01)

        # self.addToolBar(self.standardToolbar)
        # self.widget = QWidget()
        self.grid = QtWidgets.QGridLayout()
        # self.widget.setLayout(self.grid)
        # self.setCentralWidget(self.widget)
        self.setLayout(self.grid)

        self.label_tcp = QtWidgets.QLabel()
        self.label1 = QtWidgets.QLabel()
        self.label2 = QtWidgets.QLabel()

        self.label1.resize(640, 480)
        self.label2.resize(640, 480)
        self.label_tcp.resize(1280,50)
        self.btnMoveTo = QPushButton("MoveTo")

        # self.btnCapture = QPushButton("Capture")
        # self.btnAuto = QPushButton("Auto")
        self.btnMoveTo.clicked.connect(self.moveTo)

        # self.btnCapture.clicked.connect(self.capture)
        # self.btnAuto.clicked.connect(self.Auto)
        self.grid.setContentsMargins(0, 0, 0, 0)
        self.grid.setSpacing(10)
        self.grid.addWidget(self.label1, 0, 0, 1, 3)
        self.grid.addWidget(self.label2, 0, 3, 1, 3)
        self.grid.addWidget(self.label_tcp,1,0,1,6)
        self.grid.addWidget(self.x_spin, 2, 0, 1, 1)
        self.grid.addWidget(self.y_spin, 2, 1, 1, 1)
        self.grid.addWidget(self.z_spin, 2, 2, 1, 1)
        self.grid.addWidget(self.rr_spin, 2, 3, 1, 1)
        self.grid.addWidget(self.pp_spin, 2, 4, 1, 1)
        self.grid.addWidget(self.yy_spin, 2, 5, 1, 1)


        self.settings = QSettings("vision", "vision")

        self.grid.addWidget(self.btnMoveTo, 3, 2, 1, 1)
        # self.grid.addWidget(self.btnstop,3,3,1,1)
        self.btnNoUpdatePose = QPushButton("CopyTo")
        self.btnNoUpdatePose.clicked.connect(self.changeUpdatePose)
        self.grid.addWidget(self.btnNoUpdatePose, 3, 3, 1, 1)

        # self.statusBar = QtWidgets.QStatusBar()
        # self.statusBar.setObjectName("statusBar")
        # self.setStatusBar(self.statusBar)

        tmp = cv2.cvtColor(kdata_lx.depth_streaming.astype(np.uint8), cv2.COLOR_BGR2RGB).astype('uint8')
        qimg = QtGui.QImage(tmp, 640, 480, QtGui.QImage.Format_RGB888)
        img = QtGui.QPixmap.fromImage(qimg)
        self.label1.setPixmap(img)
        self.label2.setPixmap(img)

        # self.resize(1280, 640)
        self.setFixedSize(1280, 640)



        self.thread_cam = Thread(target=self.updateCam)
        self.stopCam = False
        self.thread_cam.start()



    def changeUpdatePose(self):
        if self.env and self.env.isReset:
            pose = self.env.get_current_tcp()
            self.x_spin.setValue(pose[0])
            self.y_spin.setValue(pose[1])
            self.z_spin.setValue(pose[2])
            self.rr_spin.setValue(pose[3])
            self.pp_spin.setValue(pose[4])
            self.yy_spin.setValue(pose[5])




    def updateCam(self):
        while (True):
            if self.stopCam:
                break

            # self.cam.getFrame()
            # color_img = self.cam.color_img
            # depth_img = self.cam.depth_img
            # kdata_lx.color_streaming = color_img
            # kdata_lx.depth_streaming = depth_img
            self.showImage()

            time.sleep(0.1)

    def updatexyzrpy(self):
        if self.env and self.env.isReset and self.env.isConnect():
            pose = self.env.get_current_tcp()
            # pose = [round(i, 4) for i in pose]
            self.label_tcp.setText("TCP:{}".format(pose))


    def showImage(self):
        # kdata_lx.sem.tryAcquire()
        tmp1 = cv2.cvtColor(kdata_lx.color_streaming, cv2.COLOR_BGR2RGB).astype('uint8')
        qImg1 = QtGui.QImage(tmp1, 640, 480, QtGui.QImage.Format_RGB888)
        tmp2 = cv2.cvtColor(kdata_lx.depth_streaming.astype(np.uint8), cv2.COLOR_BGR2RGB).astype('uint8')
        qImg2 = QtGui.QImage(tmp2, 640, 480, QtGui.QImage.Format_RGB888)

        img1 = QtGui.QPixmap.fromImage(qImg1)
        self.label1.setPixmap(img1)

        img2 = QtGui.QPixmap.fromImage(qImg2)
        self.label2.setPixmap(img2)

    def closeEvent(self, event):
        if self.thread:
            self._stopev = True
            self.thread.join()
        if self.thread_cam:
            self.stopCam = True
            self.thread_cam.join()
        self.close()



class Kawasaki_Control_Virt_Stereo(QWidget):
    # pic_signal = QtCore.pyqtSignal()
    def __init__(self, parent=None):
        QtWidgets.QWidget.__init__(self, parent)
        self._stopev = False
        self.initUI()
        self.env = None

        self.timer = QTimer()
        self.timer.setInterval(200)
        self.timer.timeout.connect(self.updatexyzrpy)
        self.timer.start()

        self.pose_of_capture = None

    def setEnv(self,env):
        self.env = env

    def get_current_tcp(self):
        return self.env.get_current_tcp()


    def gripper(self,openess):
        pass

    def movep(self,tcp,acc=None,vel=None):
        self.env.movep_6dof(tcp, acc, vel)

    def moveTo(self):
        qua = rpy2qt([0, 0, 0, self.rr_spin.value(), self.pp_spin.value(), self.yy_spin.value()])
        pose = [self.x_spin.value(), self.y_spin.value(), self.z_spin.value(), self.rr_spin.value(),
                self.pp_spin.value(), self.yy_spin.value()]
        self.thread = threading.Thread(target=self.movep, args=(pose,))
        self.thread.start()
        self.changeUpdatePose()#########################

    def capture(self):

        ret = self.env.capture()
        self.pose_of_capture = self.env.get_current_tcp()

        # self.extrinsic = self.env.extrinsic
        return ret

    def gen_save_dir(self, save_dir='runs/calibration_collect/exp'):
        self.save_dir = increment_path(save_dir, exist_ok=save_dir != 'runs/calibration_collect/exp',
                                       mkdir=True)  # increment save_dir

    def initUI(self):

        self.x_spin = QDoubleSpinBox()
        # self.x_spin.setRange(-5, 5)
        self.x_spin.setReadOnly(False)
        self.x_spin.setDecimals(2)
        self.x_spin.setSingleStep(0.01)
        self.x_spin.setRange(-1000, 1000)
        self.y_spin = QDoubleSpinBox()
        self.y_spin.setReadOnly(False)
        self.y_spin.setRange(-1000, 1000)
        self.y_spin.setSingleStep(0.01)
        self.y_spin.setDecimals(2)
        self.z_spin = QDoubleSpinBox()
        self.z_spin.setRange(-1000, 1000)
        self.z_spin.setReadOnly(False)
        self.z_spin.setSingleStep(0.01)
        self.z_spin.setDecimals(2)

        self.rr_spin = QDoubleSpinBox()
        self.pp_spin = QDoubleSpinBox()
        self.yy_spin = QDoubleSpinBox()
        self.rr_spin.setRange(-360, 360)
        self.rr_spin.setSingleStep(0.01)
        self.pp_spin.setRange(-360, 360)
        self.pp_spin.setSingleStep(0.01)
        self.yy_spin.setRange(-360, 360)
        self.yy_spin.setSingleStep(0.01)

        # self.addToolBar(self.standardToolbar)
        # self.widget = QWidget()
        self.grid = QtWidgets.QGridLayout()
        # self.widget.setLayout(self.grid)
        # self.setCentralWidget(self.widget)
        self.setLayout(self.grid)

        self.label_tcp = QtWidgets.QLabel()
        self.label1 = QtWidgets.QLabel()
        self.label2 = QtWidgets.QLabel()

        self.label1.resize(640, 480)
        self.label2.resize(640, 480)
        self.label_tcp.resize(1280,50)
        self.btnMoveTo = QPushButton("MoveTo")

        # self.btnCapture = QPushButton("Capture")
        # self.btnAuto = QPushButton("Auto")
        self.btnMoveTo.clicked.connect(self.moveTo)

        # self.btnCapture.clicked.connect(self.capture)
        # self.btnAuto.clicked.connect(self.Auto)
        self.grid.setContentsMargins(0, 0, 0, 0)
        self.grid.setSpacing(10)
        self.grid.addWidget(self.label1, 0, 0, 1, 3)
        self.grid.addWidget(self.label2, 0, 3, 1, 3)
        self.grid.addWidget(self.label_tcp,1,0,1,6)
        self.grid.addWidget(self.x_spin, 2, 0, 1, 1)
        self.grid.addWidget(self.y_spin, 2, 1, 1, 1)
        self.grid.addWidget(self.z_spin, 2, 2, 1, 1)
        self.grid.addWidget(self.rr_spin, 2, 3, 1, 1)
        self.grid.addWidget(self.pp_spin, 2, 4, 1, 1)
        self.grid.addWidget(self.yy_spin, 2, 5, 1, 1)


        self.settings = QSettings("vision", "vision")

        self.grid.addWidget(self.btnMoveTo, 3, 2, 1, 1)
        # self.grid.addWidget(self.btnstop,3,3,1,1)
        self.btnNoUpdatePose = QPushButton("CopyTo")
        self.btnNoUpdatePose.clicked.connect(self.changeUpdatePose)
        self.grid.addWidget(self.btnNoUpdatePose, 3, 3, 1, 1)

        # self.statusBar = QtWidgets.QStatusBar()
        # self.statusBar.setObjectName("statusBar")
        # self.setStatusBar(self.statusBar)

        tmp = cv2.cvtColor(kdata_lx.depth_streaming.astype(np.uint8), cv2.COLOR_BGR2RGB).astype('uint8')
        qimg = QtGui.QImage(tmp, 640, 480, QtGui.QImage.Format_RGB888)
        img = QtGui.QPixmap.fromImage(qimg)
        self.label1.setPixmap(img)
        self.label2.setPixmap(img)

        # self.resize(1280, 640)
        self.setFixedSize(1280, 640)



        self.thread_cam = Thread(target=self.updateCam)
        self.stopCam = False
        self.thread_cam.start()



    def changeUpdatePose(self):
        if self.env and self.env.isReset:
            pose = self.env.get_current_tcp()
            self.x_spin.setValue(pose[0])
            self.y_spin.setValue(pose[1])
            self.z_spin.setValue(pose[2])
            self.rr_spin.setValue(pose[3])
            self.pp_spin.setValue(pose[4])
            self.yy_spin.setValue(pose[5])




    def updateCam(self):
        while (True):
            if self.stopCam:
                break

            # self.cam.getFrame()
            # color_img = self.cam.color_img
            # depth_img = self.cam.depth_img
            # kdata_lx.color_streaming = color_img
            # kdata_lx.depth_streaming = depth_img
            self.showImage()

            time.sleep(0.1)

    def updatexyzrpy(self):
        if self.env and self.env.isReset and self.env.isConnect():
            pose = self.env.get_current_tcp()
            # pose = [round(i, 4) for i in pose]
            self.label_tcp.setText("TCP:{}".format(pose))


    def showImage(self):
        # kdata_lx.sem.tryAcquire()
        tmp1 = cv2.cvtColor(kdata_lx.color_l_streaming, cv2.COLOR_BGR2RGB).astype('uint8')
        qImg1 = QtGui.QImage(tmp1, 640, 480, QtGui.QImage.Format_RGB888)
        tmp2 = cv2.cvtColor(kdata_lx.color_r_streaming.astype(np.uint8), cv2.COLOR_BGR2RGB).astype('uint8')
        qImg2 = QtGui.QImage(tmp2, 640, 480, QtGui.QImage.Format_RGB888)

        img1 = QtGui.QPixmap.fromImage(qImg1)
        self.label1.setPixmap(img1)

        img2 = QtGui.QPixmap.fromImage(qImg2)
        self.label2.setPixmap(img2)

    def closeEvent(self, event):
        if self.thread:
            self._stopev = True
            self.thread.join()
        if self.thread_cam:
            self.stopCam = True
            self.thread_cam.join()
        self.close()

@Singleton
class Kawasaki_cntl_virt_diag(QDialog):
    def __init__(self, parent=None):
        QtWidgets.QDialog.__init__(self, parent, QtCore.Qt.Window |
                                   QtCore.Qt.WindowCloseButtonHint)

        self.resize(1280, 640)
        mainLayout = QtWidgets.QVBoxLayout()
        mainLayout.setContentsMargins(0, 0, 0, 0)
        mainLayout.setSpacing(0)
        self.setLayout(mainLayout)

        self.cntl = Kawasaki_Control_Virt()
        mainLayout.addWidget(self.cntl)

    def setEnv(self,env):
        self.cntl.setEnv(env)

    def closeEvent(self, e):
        kdata_lx.isVirt = False
        e.accept()


@Singleton
class Kawasaki_cntl_virt_diag_stereo(QDialog):
    def __init__(self, parent=None):
        QtWidgets.QDialog.__init__(self, parent, QtCore.Qt.Window |
                                   QtCore.Qt.WindowCloseButtonHint)

        self.resize(1280, 640)
        mainLayout = QtWidgets.QVBoxLayout()
        mainLayout.setContentsMargins(0, 0, 0, 0)
        mainLayout.setSpacing(0)
        self.setLayout(mainLayout)

        self.cntl = Kawasaki_Control_Virt_Stereo()
        mainLayout.addWidget(self.cntl)

    def setEnv(self,env):
        self.cntl.setEnv(env)

    def closeEvent(self, e):
        kdata_lx.isVirt = False
        e.accept()

@Singleton
class Kawasaki_cntl_diag(QDialog):
    def __init__(self, parent=None):
        QtWidgets.QDialog.__init__(self, parent, QtCore.Qt.Window |
                                   QtCore.Qt.WindowCloseButtonHint)

        self.resize(1280, 640)
        mainLayout = QtWidgets.QVBoxLayout()
        mainLayout.setContentsMargins(0, 0, 0, 0)
        mainLayout.setSpacing(0)
        self.setLayout(mainLayout)

        self.cntl = Kawasaki_Control()
        mainLayout.addWidget(self.cntl)

    def closeEvent(self, e):
        kdata_lx.isReal = False
        e.accept()

# qt = [0,1,0,0]
# a = quaternion_to_euler_fc(0,1,0,0)
# print(a)

# str1="{:0>2d}".format(3)
# str2="{:0>2d}".format(999)
# print(str1)
# print(str2)
# capture_num = 1
# with open('pose.txt') as f:
#     lines = f.readlines()
#     for i in range(len(lines)):
#         xyzrpy = lines[i].split(' ')
#         print(xyzrpy)
#         save_dir = 'runs/calibration_collect/temp'
#         save_dir = increment_path(save_dir, exist_ok=save_dir != 'runs/calibration_collect/exp', mkdir=True)  # increment save_dir
#
#         poseFileName = "{}\{}".format(os.path.abspath(save_dir),"movingcam_robot_pose_{:0>2d}.dat".format(capture_num))
#         pose = [float(xyzrpy[0]),float(xyzrpy[1]),float(xyzrpy[2]),float(xyzrpy[3]),float(xyzrpy[4]),float(xyzrpy[5])]
#         with open(poseFileName,'a+') as f:
#             f.write('f 2\n')
#             f.write("r {} {} {}\n".format(pose[5]*180/3.1415926,pose[4]*180/3.1415926, pose[3]*180/3.1415926))
#             f.write("t {} {} {}\n".format(pose[0], pose[1], pose[2]))
#
#             f.close()
#         capture_num = capture_num + 1
# pose = [0.09389998528127, 0.185933027965967, 0.0331420678795657,  36.4031555878774 ,296.938793223939, 192.227279160989]
# value = rpy2rm(pose)
# print(value)