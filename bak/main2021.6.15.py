from kcore import *
import os
import kdata
import json
import sys
from PyQt5.QtGui import *
from PyQt5.QtWidgets import *
import xlrd
from kafka_plc import *

def executable_path():
    return os.path.dirname(sys.argv[0])

def image():
    return os.path.abspath(os.path.join(executable_path(),'images'))

def icon(name,sufix="svg"):
    path = os.path.join(image(),  '{}.{}'.format(name,sufix))
    return QIcon(path)

# def i18n(s):
#     lang, encode = locale.getdefaultlocale()
#     return pyqode_i18n.tr(s, lang=lang)

class WidgetSpacer(QWidget):
    def __init__(self, parent, wmax=None):
        super(WidgetSpacer, self).__init__(parent)
        self.setSizePolicy(QSizePolicy.Expanding,
                           QSizePolicy.Expanding)
        if wmax:
            self.setMaximumWidth(wmax)

def robot_In_condition(x):
    robot = int(x['robot'])
    return (robot >= 1001) and (robot <= 1256)

def robot_out_condition(x):
    robot = int(x['robot'])
    return (robot > 0) and (robot <= 256)

class RobotInPLCOutRWidget(QDockWidget):
    def __init__(self, parent):
        super(RobotInPLCOutRWidget, self).__init__(('PLC->Robot'), parent)
        self.timer = QTimer()
        self.timer.timeout.connect(self.timeout)
        self.timer.setInterval(2)
        widget = QWidget(self)
        horizontalHeader = ["State","PLC", "Robot","Bits", "Value", "tri","Desc"]

        self.table = QTableWidget() # 也可以QTableWidget(5,2)
        self.table.setColumnCount(7) # 5列
        self.table.setColumnHidden(5,True)
        self.table.setSortingEnabled(True)
        self.table.setHorizontalHeaderLabels(horizontalHeader)
        self.table.setEditTriggers(QTableWidget.NoEditTriggers)
        self.table.setSelectionBehavior(QTableWidget.SelectRows)
        self.table.setSelectionMode(QTableWidget.SingleSelection)
        self.table.setAlternatingRowColors(True)
        self.table.horizontalHeader().setSectionResizeMode(QHeaderView.ResizeToContents)
        self.table.horizontalHeader().setSectionResizeMode(6,QHeaderView.Stretch)
        self.table.cellChanged.connect(self.cellChanged)
        self.bSortAsc = Qt.DescendingOrder
        self.table.horizontalHeader().sectionClicked.connect(self.sortbyclounm)
        mainLayout = QHBoxLayout()
        mainLayout.addWidget(self.table)
        widget.setLayout(mainLayout)
        self.setWidget(widget)
        self.resize(1024,512)
        matches = [x for x in kdata.dictlist if robot_In_condition(x)]
        self.table.setRowCount(len(matches))
        self.table.blockSignals(True)
        for i in range(len(matches)):
            item = matches[i]
            chkBoxItem = QTableWidgetItem()
            chkBoxItem.setFlags(Qt.ItemIsUserCheckable | Qt.ItemIsEnabled)
            chkBoxItem.setCheckState(Qt.Unchecked)
            chkBoxItem.setTextAlignment(Qt.AlignHCenter | Qt.AlignVCenter)
            self.table.setItem(i, 0, chkBoxItem)

            if(item['plc'] != ""):
                newItem1 = QTableWidgetItem(str(item['plc']))
                newItem1.setTextAlignment(Qt.AlignHCenter | Qt.AlignVCenter)
                self.table.setItem(i, 1, newItem1)

            newItem1 = QTableWidgetItem(str(int(item['robot'])))
            newItem1.setTextAlignment(Qt.AlignHCenter | Qt.AlignVCenter)
            self.table.setItem(i, 2, newItem1)

            if len(str(item['bits']))>0:
                newItem1 = QTableWidgetItem(str(int(item['bits'])))
                newItem1.setTextAlignment(Qt.AlignHCenter | Qt.AlignVCenter)
                self.table.setItem(i, 3, newItem1)
            else:
                newItem1 = QTableWidgetItem("")
                newItem1.setTextAlignment(Qt.AlignHCenter | Qt.AlignVCenter)
                self.table.setItem(i, 3, newItem1)

            newItem1 = QTableWidgetItem(str(item['value']))
            newItem1.setTextAlignment(Qt.AlignHCenter | Qt.AlignVCenter)
            self.table.setItem(i, 4, newItem1)

            newItem1 = QTableWidgetItem("0")
            newItem1.setTextAlignment(Qt.AlignHCenter | Qt.AlignVCenter)
            self.table.setItem(i, 5, newItem1)

            newItem1 = QTableWidgetItem(item['desc'])
            newItem1.setTextAlignment(Qt.AlignHCenter | Qt.AlignVCenter)
            self.table.setItem(i,6, newItem1)
        self.table.blockSignals(False)
        self.table.sortItems(1)
        self.timer.start()

    def timeout(self):
        rows = self.table.rowCount()
        if(len(kdata.swait_dict)<=0):
            return
        for index in range(rows):

            strPlc = self.table.item(index,1).text()
            intPlc = int(strPlc)
            if (intPlc in (20001, 20002, 20003, 20004)):
                self.table.item(index, 4).setText(str(kdata.plc_20001_to_20004[intPlc]))
                self.table.item(index, 0).setCheckState(Qt.Checked)
                self.table.item(index, 5).setText("1")
            if(intPlc in kdata.out_plc_change):
                if kdata.out_plc[intPlc-20001] == 1:
                    self.table.item(index, 0).setCheckState(Qt.Checked)
                    self.table.item(index, 0).setBackground(QColor(255, 0, 0))
                    self.table.item(index, 5).setText("1")
                else:
                    self.table.item(index, 0).setCheckState(Qt.Unchecked)
                    self.table.item(index, 0).setBackground(self.table.item(index, 1).background())
                    self.table.item(index, 5).setText("0")

            ############################################
            strRobot = self.table.item(index, 2).text()
            intRobot = int(strRobot)
            if (intRobot in kdata.swait_dict.keys() ):
                self.table.blockSignals(True)
                if(kdata.input[intRobot-1001] != kdata.swait_dict[intRobot]):
                    self.table.item(index, 0).setBackground(QColor(128, 255, 155))
                    self.table.item(index, 5).setText("2")
                else:
                    self.table.item(index, 0).setBackground(self.table.item(index, 1).background())
                    self.table.item(index, 5).setText("0")
                    kdata.swait_dict.pop(intRobot)
                self.table.blockSignals(False)
            ############################################
        kdata.out_plc_change.clear()
        self.table.sortItems(5, self.bSortAsc)

    def sortbyclounm(self, col):
        if (col != 0):
            return
        self.bSortAsc = Qt.DescendingOrder if (self.bSortAsc==Qt.AscendingOrder) else (Qt.AscendingOrder)
        print(self.bSortAsc)
        self.table.sortItems(5, self.bSortAsc)

    def cellChanged(self,row,col):
        plc_num = self.table.item(row, 1)
        robot_num = self.table.item(row, 2)
        if(plc_num == None or robot_num == None):
            return
        plc_num = plc_num.text()
        robot_num = robot_num.text()
        plc_num = int(plc_num)
        robot_num = int(robot_num)

        plc_num_idx = plc_num
        robot_num_idx = robot_num
        if (plc_num >= 20001):
            plc_num_idx = plc_num - 20001
        if (robot_num >= 1001):
            robot_num_idx = robot_num - 1001
        if self.table.item(row,col).checkState() == Qt.Checked:
            if(plc_num_idx > 0):
                kdata.out_plc[plc_num_idx] = 1
            kdata.input[robot_num_idx] = 1
        else:
            if (plc_num_idx > 0):
                kdata.out_plc[plc_num_idx] = 0
            kdata.input[robot_num_idx] = 0
        self.table.blockSignals(True)
        if(self.table.item(row, 0).checkState() == Qt.Checked):
            self.table.item(row, 0).setBackground(QColor(255, 0, 0))
            self.table.item(row, 5).setText("1")
        else:
            self.table.item(row, 0).setBackground(self.table.item(row, 1).background())
            self.table.item(row, 5).setText("0")
        self.table.sortItems(5, self.bSortAsc)
        self.table.blockSignals(False)


    # def slotUpdateSignals(self):
    #     rows = self.table.rowCount()
    #     for index in range(rows):
    #         strPlc = self.table.item(index,1).text()
    #         intPlc = int(strPlc)
    #         if(intPlc in (20001,20002,20003,20004)):
    #             self.table.item(index, 4).setText(str(kdata.plc_20001_to_20004[intPlc]))
    #             self.table.item(index, 0).setCheckState(Qt.Checked)
    #             self.table.item(index, 5).setText("1")
    #         if(intPlc in kdata.out_plc_change):
    #             if kdata.out_plc[intPlc-20001] == 1:
    #                 self.table.item(index, 0).setCheckState(Qt.Checked)
    #                 self.table.item(index, 0).setBackground(QColor(255, 0, 0))
    #                 self.table.item(index, 5).setText("1")
    #             else:
    #                 self.table.item(index, 0).setCheckState(Qt.Unchecked)
    #                 self.table.item(index, 0).setBackground(self.table.item(index, 1).background())
    #                 self.table.item(index, 5).setText("0")
    #
    #     self.table.sortItems(5, self.bSortAsc)
    #     kdata.out_plc_change.clear()

class RobotOutPLCInWidget(QDockWidget):

    def __init__(self, parent):
        super(RobotOutPLCInWidget, self).__init__(('Robot->PLC'), parent)
        self.timer = QTimer()
        self.timer.timeout.connect(self.timeout)
        self.timer.setInterval(2)
        widget = QWidget(self)
        horizontalHeader = ["State","Robot", "PLC", "Tri","Desc"]
        self.table = QTableWidget() # 也可以QTableWidget(5,2)
        self.table.setSortingEnabled(True)
        self.table.setColumnCount(5) # 5列
        self.table.setColumnHidden(3, True)
        self.matches = [x for x in kdata.dictlist if robot_out_condition(x)]
        self.table.setRowCount(len(self.matches)) # 3行
        self.table.setAlternatingRowColors(True)
        self.table.setShowGrid(True)
        self.table.setHorizontalHeaderLabels(horizontalHeader)
        self.table.setEditTriggers(QTableWidget.NoEditTriggers)
        self.table.setSelectionBehavior(QTableWidget.SelectRows)
        self.table.setSelectionMode(QTableWidget.SingleSelection)
        self.table.horizontalHeader().setSectionResizeMode(QHeaderView.ResizeToContents)
        self.table.horizontalHeader().setSectionResizeMode(4,QHeaderView.Stretch)
        self.table.cellChanged.connect(self.cellChanged)
        self.table.horizontalHeader().sectionClicked.connect(self.sortbyclounm)
        self.bSortAsc = Qt.DescendingOrder
        mainLayout = QHBoxLayout()
        mainLayout.addWidget(self.table)
        widget.setLayout(mainLayout)
        self.setWidget(widget)
        self.resize(1024,512)

        self.table.blockSignals(True)
        for i in range(len(self.matches)):
            item = self.matches[i]
            chkBoxItem = QTableWidgetItem()
            chkBoxItem.setTextAlignment(Qt.AlignHCenter | Qt.AlignVCenter)
            chkBoxItem.setFlags(Qt.ItemIsUserCheckable | Qt.ItemIsEnabled)
            chkBoxItem.setCheckState(Qt.Unchecked)
            self.table.setItem(i, 0, chkBoxItem)

            newItem1 = QTableWidgetItem(str(int(item['robot'])))
            newItem1.setTextAlignment(Qt.AlignHCenter | Qt.AlignVCenter)
            self.table.setItem(i, 1, newItem1)

            if(item['plc'] != ""):
                newItem1 = QTableWidgetItem(str(item['plc']))
                newItem1.setTextAlignment(Qt.AlignHCenter | Qt.AlignVCenter)
                self.table.setItem(i, 2, newItem1)

            newItem1 = QTableWidgetItem("0")
            newItem1.setTextAlignment(Qt.AlignHCenter | Qt.AlignVCenter)
            self.table.setItem(i, 3, newItem1)

            newItem1 = QTableWidgetItem(item['desc'])
            newItem1.setTextAlignment(Qt.AlignHCenter | Qt.AlignVCenter)
            self.table.setItem(i, 4, newItem1)
        self.table.blockSignals(False)
        self.timer.start()

    def sortbyclounm(self, col):
        if (col != 0):
            return
        self.bSortAsc = Qt.DescendingOrder if (self.bSortAsc==Qt.AscendingOrder) else (Qt.AscendingOrder)
        print(self.bSortAsc)
        self.table.sortItems(3, self.bSortAsc)

    def cellChanged(self,row,col):
        plc_num = self.table.item(row, 2)
        robot_num = self.table.item(row, 1)
        if (plc_num == None or robot_num == None):
            return
        plc_num = plc_num.text()
        robot_num = robot_num.text()
        plc_num = int(plc_num)
        robot_num = int(robot_num)
        if plc_num >= 10001:
            plc_num = plc_num - 10001
        if self.table.item(row,col).checkState() == Qt.Checked:
            kdata.output[robot_num - 1] = 1
            if plc_num > 0:
                kdata.input_plc[plc_num] = 1
        else:
            kdata.output[robot_num - 1] = 0
            if plc_num > 0:
                kdata.input_plc[plc_num] = 0
        self.table.blockSignals(True)
        if self.table.item(row, 0).checkState() == Qt.Checked:
            self.table.item(row, 0).setBackground(QColor(255, 0, 0))
            self.table.item(row, 3).setText("1")
        else:
            self.table.item(row, 0).setBackground(self.table.item(row, 1).background())
            self.table.item(row, 3).setText("0")
        self.table.sortItems(3, self.bSortAsc)
        self.table.blockSignals(False)

    def timeout(self):
        rows = self.table.rowCount()
        for index in range(rows):
            try:
                strRobot = self.table.item(index, 1).text()
                intRobot = int(strRobot)
                if (intRobot in kdata.out_robot_change):
                    if kdata.output[intRobot - 1] == 1:
                        self.table.item(index, 0).setCheckState(Qt.Checked)
                        self.table.item(index, 0).setBackground(QColor(255, 0, 0))
                        self.table.item(index, 3).setText("1")
                    else:
                        self.table.item(index, 0).setCheckState(Qt.UnChecked)
                        self.table.item(index, 0).setBackground(self.table.item(index, 1).background())
                        self.table.item(index, 3).setText("0")
            except:
                pass
        self.table.sortItems(3, self.bSortAsc)
        kdata.out_robot_change.clear()

    # def slotUpdateSignals(self):
    #     rows = self.table.rowCount()
    #     for index in range(rows):
    #         try:
    #             strRobot = self.table.item(index,1).text()
    #             intRobot = int(strRobot)
    #
    #             if(intRobot in kdata.out_robot_change):
    #                 if kdata.output[intRobot-1] == 1:
    #                     self.table.item(index, 0).setCheckState(Qt.Checked)
    #                     self.table.item(index, 0).setBackground(QColor(255, 0, 0))
    #                     self.table.item(index, 3).setText("1")
    #                 else:
    #                     self.table.item(index, 0).setCheckState(Qt.UnChecked)
    #                     self.table.item(index, 0).setBackground(self.table.item(index, 1).background())
    #                     self.table.item(index, 3).setText("0")
    #         except:
    #             pass
    #     self.table.sortItems(3, self.bSortAsc)
    #     kdata.out_robot_change.clear()

class Kawasaki(QMainWindow):
    def __init__(self, parent=None):
        super(Kawasaki, self).__init__(parent)
        bar = QToolBar('Toolbar',self)
        bar.setIconSize(QSize(48, 48))
        bar.addWidget(WidgetSpacer(self))
        self.runAction = bar.addAction(icon("run","png"), ("Run"), self.progRun)
        self.stopAction = bar.addAction(icon("stop", "svg"), ("Stop"), self.stopRun)
        bar.addAction(icon("about","png"), ("Help"), self.showhelp)
        self.addToolBar(bar)


        self.robotInPLCOutRWidget = RobotInPLCOutRWidget(self)
        self.addDockWidget(Qt.LeftDockWidgetArea,self.robotInPLCOutRWidget)

        self.robotOutPLCInWidget = RobotOutPLCInWidget(self)
        self.addDockWidget(Qt.RightDockWidgetArea,self.robotOutPLCInWidget)

        # self.tabber = QTabWidget(self)
        # self.stack = QStackedWidget(self)
        # self.stack.addWidget(self.tabber)
        # self.setCentralWidget(self.stack)
        self.resize(1024, 1024)
        self.kcore = Core()
        ##robot
        # self.kcore.signalUpdateRobot.connect(self.robotOutPLCInWidget.slotUpdateSignals)#slotUpdateTheOut
        # self.kcore.signalUpdatePLC.connect(self.robotInPLCOutRWidget.slotUpdateSignals) #slotUpdateTheOut

        ##plc
        self.kafka = Kafaka()
        self.kafka.signalupdatePLCSig.connect(self.kcore.slotUpdatePLCSigs)

    def showhelp(self):
        dlg = QDialog(self)
        dlg.setWindowTitle('Help')
        l = QVBoxLayout(dlg)
        l.setContentsMargins(6, 6, 6, 6)
        tabWidget = QTabWidget(dlg)
        buttonBar = QDialogButtonBox(dlg)
        buttonBar.addButton(QDialogButtonBox.Close)
        buttonBar.accepted.connect(dlg.close)
        buttonBar.rejected.connect(dlg.close)
        l.addWidget(tabWidget)
        l.addWidget(buttonBar)
        tv = QTextBrowser(tabWidget)
        image = QLabel(tabWidget)
        tabWidget.addTab(image, ('About'))
        tabWidget.addTab(tv, ('Help'))
        dlg.exec_()

    def progRun(self):
        if(self.kcore.isRunning()):
            self.kcore.working = False
            self.kcore.terminate()
        self.kcore.working = True
        self.kcore.start()
        if(self.kafka.isRunning()):
            self.kafka.working = False
            self.kafka.wait()
        self.kafka.working = True
        self.kafka.consumer.resume()
        self.kafka.start()

    def stopRun(self):
        if(self.kafka.isRunning()):
            self.kafka.working = False
            self.kafka.wait()
        if (self.kcore.isRunning()):
            self.kcore.working = False

            self.kcore.terminate()

def takeSecond(elem):
    # print(elem['robot'])
    return int(elem['robot'])

def readsignal():
    path = os.path.abspath(os.path.join(executable_path(),"plc.xls"))
    data = xlrd.open_workbook(path)
    table = data.sheets()[0]
    nor = table.nrows
    nol = table.ncols
    kdata.dictlist = [dict() for x in range(nor-1)]

    for i in range(1, nor):
        elem = {}
        for j in range(nol):
            title = table.cell_value(0, j)
            value = table.cell_value(i, j)
            elem[title] = value
        kdata.dictlist[i-1] = elem
    kdata.dictlist.sort(key=takeSecond)


if __name__ == '__main__':
    readsignal()
    app = QApplication(sys.argv)
    kawasaki = Kawasaki()
    kawasaki.show()
    sys.exit(app.exec())


