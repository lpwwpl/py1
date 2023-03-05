from PyQt5 import QtCore, QtGui, QtWidgets
from PyQt5.QtWidgets import *
from PyQt5.QtGui import *
from PyQt5.QtCore import *
from PyQt5 import QtCore, QtGui, QtWidgets
import sys
import os
from Extensions.StackSwitcher import StackSwitcher
from Extensions import StyleSheet

from testyolo_no_device import Kawasaki_yolo
from testyy_no_device import Kawasaki_yy
from testlx_no_device_matploit import Kawasaki_lx
from testyolo_no_device import *
from testcali_collect import Kawasaki_cali_collect
from testcali_calibration import Kawasaki_calibration

class PVision(QMainWindow):
    def __init__(self, parent=None):
        super(PVision, self).__init__(parent)
        # self.setWindowFlags(Qt.FramelessWindowHint)
        self.setWindowIcon(
            QtGui.QIcon(os.path.join("Resources", "images", "Icon")))
        self.setWindowTitle("Vision")

        self.widget = QWidget(self)
        mainLayout = QtWidgets.QVBoxLayout(self.widget)
        mainLayout.setSpacing(0)
        mainLayout.setContentsMargins(0, 0, 0, 0)  # .setContentsMargins(0,0,0,0)
        hbox = QtWidgets.QHBoxLayout()

        self.pagesStack = QtWidgets.QStackedWidget()
        self.projectSwitcher = StackSwitcher(self.pagesStack)
        self.projectSwitcher.setStyleSheet(StyleSheet.mainMenuStyle)



        hbox.setSpacing(5)
        hbox.addWidget(self.projectSwitcher)
        mainLayout.addLayout(hbox)
        mainLayout.addWidget(self.pagesStack)
        hbox.addStretch(1)



        self.widget.setLayout(mainLayout)
        self.setCentralWidget(self.widget)
        self.resize(1920, 960)


        opt = parse_opt()

        self.cali_collect = Kawasaki_cali_collect(self.pagesStack)
        self.widget_collect = QWidget()
        hbox_collect = QHBoxLayout()
        hbox_collect.addWidget(self.cali_collect)
        self.widget_collect.setLayout(hbox_collect)

        self.cali_calibration = Kawasaki_calibration()
        self.widget_calib = QWidget()
        hbox_calib = QHBoxLayout()
        hbox_calib.addWidget(self.cali_calibration)
        self.widget_calib.setLayout(hbox_calib)

        self.yolo = Kawasaki_yolo(opt)
        self.widget_yolo = QWidget()
        hbox_yolo = QHBoxLayout()
        hbox_yolo.addWidget(self.yolo)
        self.widget_yolo.setLayout(hbox_yolo)

        self.lx = Kawasaki_lx()
        self.widget_lx = QWidget()
        hbox_lx = QHBoxLayout()
        hbox_lx.addWidget(self.lx)
        self.widget_lx.setLayout(hbox_lx)


        self.widget_yy = QWidget(self.pagesStack)
        self.yy = Kawasaki_yy(self.widget_yy)
        hbox_yy = QHBoxLayout()
        hbox_yy.addWidget(self.yy)
        self.widget_yy.setLayout(hbox_yy)
        # self.addPage(self.projectWindowStack, "Calibration", QtGui.QIcon(
        #     os.path.join("Resources", "images", "hire-me")))

        self.addPage(self.widget_collect, "caliCol", QtGui.QIcon(
            os.path.join("Resources", "images", "hire-me")))
        self.addPage(self.widget_calib, "calibration", QtGui.QIcon(
            os.path.join("Resources", "images", "hire-me")))
        self.addPage(self.widget_lx, "PickPlace", QtGui.QIcon(
            os.path.join("Resources", "images", "hire-me")))
        self.addPage(self.widget_yy, "WorkPiece", QtGui.QIcon(
            os.path.join("Resources", "images", "hire-me")))
        self.addPage(self.widget_yolo, "Yolov5", QtGui.QIcon(
            os.path.join("Resources", "images", "hire-me")))
        self.projectSwitcher.setDefault()

        # self.setLayout(mainLayout)



    def closeEvent(self, event):
        self.yy.close()
        self.cali_collect.close()
        self.cali_calibration.close()
        self.lx.close()
        self.yolo.close()
        app.closeAllWindows()
        event.accept()


    def addPage(self, pageWidget, name, iconPath):
        self.projectSwitcher.addButton(name=name, icon=iconPath)
        self.pagesStack.addWidget(pageWidget)

app = QtWidgets.QApplication(sys.argv)

splash = QtWidgets.QSplashScreen(
    QtGui.QPixmap(os.path.join("Resources", "images", "splash")))
splash.show()

main = PVision()

splash.finish(main)
main.show()
sys.exit(app.exec_())