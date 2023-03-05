from kcore import *
import os
import sys
from PyQt5.QtGui import *
from PyQt5.QtWidgets import *
import xlrd
from kafka_plc import *
from UtilSet import *

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

def robot_eq_condition(x,num):
    robot = int(x['robot'])
    return (robot == num)

def plc_eq_condition(x,num):
    plc = int(x['plc'])
    return (plc == num)

def robot_out_condition(x):
    robot = int(x['robot'])
    return (robot > 0) and (robot <= 256)

class LabelWidget(QDockWidget):
    def __init__(self, parent):
        super(LabelWidget, self).__init__(('Tips'), parent)
        self.label = QTextEdit(self)
        self.label.setStyleSheet('border-width: 1px;border-style: solid;border-color: rgb(255, 170, 0);')
        self.setWidget(self.label)

        self.label.setAlignment(Qt.AlignTop)

class RobotInPLCOutRWidget(QDockWidget):
    def __init__(self, parent, core):
        super(RobotInPLCOutRWidget, self).__init__(('PLC->Robot'), parent)
        # self.timer = QTimer()
        # self.timer.timeout.connect(self.timeout)
        # self.timer.setInterval(5000)
        self.kcore = core
        widget = QWidget(self)
        horizontalHeader = ["State","PLC", "Robot","Bits", "Value", "Desc"]

        self.editWidget = None
        self.curItem = None
        self.table = QTableWidget() # 也可以QTableWidget(5,2)
        self.table.setColumnCount(6) #
        self.table.setSortingEnabled(True)
        self.table.setHorizontalHeaderLabels(horizontalHeader)
        self.table.setEditTriggers(QTableWidget.NoEditTriggers)
        self.table.setSelectionBehavior(QTableWidget.SelectRows)
        self.table.setSelectionMode(QTableWidget.SingleSelection)
        self.table.setAlternatingRowColors(True)
        self.table.horizontalHeader().setSectionResizeMode(QHeaderView.ResizeToContents)
        self.table.horizontalHeader().setSectionResizeMode(5,QHeaderView.Stretch)
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
            #newItem1.setFlags(Qt.ItemIsEnabled | Qt.ItemIsEditable)
            newItem1.setTextAlignment(Qt.AlignHCenter | Qt.AlignVCenter)
            self.table.setItem(i, 4, newItem1)

            newItem1 = QTableWidgetItem(item['desc'])
            newItem1.setTextAlignment(Qt.AlignHCenter | Qt.AlignVCenter)
            self.table.setItem(i,5, newItem1)

        self.table.doubleClicked.connect(self.doubleClick)
        self.table.blockSignals(False)
        self.table.sortItems(1)

    def doubleClick(self, index):
        qindex = (QModelIndex)(index)
        column = qindex.column()
        if column == 4 and self.table.item(qindex.row(),1).text() in {"20001","20002","20003","20004"}:
            item = self.table.item(qindex.row(),qindex.column())
            self.table.openPersistentEditor(item)
            self.table.editItem(item)
            self.curItem = item
            self.editWidget = self.table.indexWidget(index)
            self.editWidget.setFocus(True)
            self.editWidget.editingFinished.connect(self.closeEditing)
            return
        if column == 4 and self.table.item(qindex.row(),2).text() in {"1033","1169"}:
            item = self.table.item(qindex.row(),qindex.column())
            self.table.openPersistentEditor(item)
            self.table.editItem(item)
            self.curItem = item
            self.editWidget = self.table.indexWidget(index)
            self.editWidget.setFocus(True)
            self.editWidget.editingFinished.connect(self.closeEditing)
            return

    def closeEditing(self):
        if self.curItem:
            try:
                text = self.table.item(self.curItem.row(), 4).text()
                if text != "" :
                    startBit = int(self.table.item(self.curItem.row(), 2).text())
                    bits = int(self.table.item(self.curItem.row(), 3).text())
                    value = int(self.table.item(self.curItem.row(), 4).text())
                    self.kcore.bits_assign(value,startBit,bits)
            except Exception as e:
                QMessageBox.warning(self, "Exception", "str 2 int exception", QMessageBox.Yes | QMessageBox.No)
                self.table.item(self.curItem.row(), 4).setText("")
            self.editWidget.close()
            self.table.closePersistentEditor(self.curItem)
            self.curItem = None


    def timeout(self):
        rows = self.table.rowCount()
        kdata.str = kdata.str +"PlC_Outputs:\n"
        self.table.blockSignals(True)
        for index in range(rows):
            strPlc = self.table.item(index,1).text()
            intPlc = int(strPlc)
            num = intPlc
            strRobot = self.table.item(index,2).text()
            intRobot = int(strRobot)
            if (intPlc in (20001, 20002, 20003, 20004) and kdata.plc_20001_to_20004.get(intPlc) != None):
                self.table.item(index, 4).setText(str(kdata.plc_20001_to_20004[intPlc]))
                self.table.item(index, 0).setCheckState(Qt.Checked)
                self.table.item(index, 5).setBackground(QColor(128, 255, 155))
                matches = [x for x in kdata.dictlist if plc_eq_condition(x, intPlc)]
                if len(matches) == 1:
                    wait_str = ("{}:{}\n").format(num, matches[0]['desc'])
                    kdata.str = kdata.str + wait_str
            try:
                if(intRobot in kdata.in_robot_change):
                    bits_num = int(self.table.item(index, 3).text())
                    newValue = self.kcore.bits(intRobot, bits_num)
                    oldValue = 0
                    oldValue_str = self.table.item(index,4).text()
                    if(oldValue_str ==''):
                        oldValue = 0
                    else:
                        oldValue = int(oldValue_str)
                    if newValue!=oldValue:
                        self.table.item(index, 4).setText(str(self.kcore.bits(intRobot, bits_num)))
                    print(kdata.in_robot_change)
                    if kdata.in_robot_change[intRobot] == 0:
                        self.table.item(index, 5).setBackground(QColor(255, 255, 0))   #黄
                        self.table.item(index, 0).setCheckState(Qt.Checked)
                        item = self.table.item(index,0)
                        self.table.scrollToItem(item)
                    else:
                        self.table.item(index, 0).setCheckState(Qt.Unchecked)
                        self.table.item(index, 5).setBackground(QColor(128, 255, 155))  #绿色
            except Exception as e:
                print(e)
            if(intPlc in kdata.out_plc_change):
                if kdata.out_plc[intPlc-20001] == 1:
                    self.table.item(index, 5).setBackground(QColor(128, 255, 155))
                    self.table.item(index, 0).setCheckState(Qt.Checked)
                else:
                    self.table.item(index, 0).setCheckState(Qt.Unchecked)
                    self.table.item(index, 5).setBackground(QColor(255, 0, 0))
                    num = -1 * num
                matches = [x for x in kdata.dictlist if plc_eq_condition(x, intPlc)]
                if len(matches) == 1:
                    wait_str = ("{}:{}\n").format(num, matches[0]['desc'])
                    kdata.str = kdata.str + wait_str

        kdata.str = kdata.str + "RobotWaits for:\n"
        for index in range(rows):
            ############################################
            strRobot = self.table.item(index, 2).text()
            intRobot = int(strRobot)
            if (intRobot in kdata.swait_dict.keys() ):
                num = intRobot
                if kdata.swait_dict[intRobot] != None and kdata.swait_dict[intRobot] <= 0:
                    num = -1 * num
                matches = [x for x in kdata.dictlist if robot_eq_condition(x, intRobot)]
                if len(matches) == 1:
                    wait_str = ("{}:{}\n").format(num, matches[0]['desc'])
                    kdata.str = kdata.str + wait_str

                if(kdata.input[intRobot-1001] != kdata.swait_dict[intRobot]):
                    #self.table.item(index, 0).setBackground(QColor(128, 255, 155))
                    self.table.item(index, 5).setBackground(QColor(255, 255, 0))
                    item = self.table.item(index, 0)
                    self.table.scrollToItem(item)
                else:
                    kdata.swait_dict.pop(intRobot)

        self.table.blockSignals(False)
            ############################################
        # kdata.out_plc_change.clear()


    def resetTable(self):
        rows = self.table.rowCount()
        self.table.blockSignals(True)
        for index in range(rows):
            self.table.item(index, 5).setBackground(self.table.item(index, 1).background())
        self.table.blockSignals(False)

    def sortbyclounm(self, col):
        return
        if col != 0:
            return
        self.bSortAsc = Qt.DescendingOrder if (self.bSortAsc==Qt.AscendingOrder) else (Qt.AscendingOrder)
        # self.table.sortItems(5, self.bSortAsc)

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
                kdata.out_plc_change[plc_num_idx]=1
            kdata.input[robot_num_idx] = 1
        else:
            if (plc_num_idx > 0):
                kdata.out_plc[plc_num_idx] = 0
                kdata.out_plc_change[plc_num_idx] = 0
            kdata.input[robot_num_idx] = 0
        self.table.blockSignals(True)
        if(self.table.item(row, 0).checkState() == Qt.Checked):
            self.table.item(row, 5).setBackground(QColor(128, 255, 155))
        else:
            self.table.item(row, 5).setBackground(self.table.item(row, 1).background())
        self.table.blockSignals(False)


class RobotOutPLCInWidget(QDockWidget):

    def __init__(self, parent,core):
        super(RobotOutPLCInWidget, self).__init__(('Robot->PLC'), parent)
        widget = QWidget(self)
        self.curItem = None
        self.kcore = core
        horizontalHeader = ["State","Robot", "PLC","Bits", "Value", "Desc"]
        self.table = QTableWidget() # 也可以QTableWidget(5,2)
        self.table.setSortingEnabled(True)
        self.table.setColumnCount(6) # 5列
        self.matches = [x for x in kdata.dictlist if robot_out_condition(x)]
        self.table.setRowCount(len(self.matches)) # 3行
        self.table.setAlternatingRowColors(True)
        self.table.setShowGrid(True)
        self.table.setHorizontalHeaderLabels(horizontalHeader)
        self.table.setEditTriggers(QTableWidget.NoEditTriggers)
        self.table.setSelectionBehavior(QTableWidget.SelectRows)
        self.table.setSelectionMode(QTableWidget.SingleSelection)
        self.table.horizontalHeader().setSectionResizeMode(QHeaderView.ResizeToContents)
        self.table.horizontalHeader().setSectionResizeMode(5,QHeaderView.Stretch)
        self.table.cellChanged.connect(self.cellChanged)
        self.table.horizontalHeader().sectionClicked.connect(self.sortbyclounm)
        self.bSortAsc = Qt.DescendingOrder
        mainLayout = QHBoxLayout()
        mainLayout.addWidget(self.table)
        widget.setLayout(mainLayout)
        self.setWidget(widget)
        self.resize(1024,24)

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

                if len(str(item['bits'])) > 0:
                    newItem1 = QTableWidgetItem(str(int(item['bits'])))
                    newItem1.setTextAlignment(Qt.AlignHCenter | Qt.AlignVCenter)
                    self.table.setItem(i, 3, newItem1)
                else:
                    newItem1 = QTableWidgetItem("")
                    newItem1.setTextAlignment(Qt.AlignHCenter | Qt.AlignVCenter)
                    self.table.setItem(i, 3, newItem1)

                newItem1 = QTableWidgetItem(str(item['value']))
                # newItem1.setFlags(Qt.ItemIsEnabled | Qt.ItemIsEditable)
                newItem1.setTextAlignment(Qt.AlignHCenter | Qt.AlignVCenter)
                self.table.setItem(i, 4, newItem1)

                newItem1 = QTableWidgetItem(item['desc'])
                newItem1.setTextAlignment(Qt.AlignHCenter | Qt.AlignVCenter)
                self.table.setItem(i, 5, newItem1)

        self.table.blockSignals(False)
        self.table.doubleClicked.connect(self.doubleClick)

    def doubleClick(self, index):
        qindex = (QModelIndex)(index)
        column = qindex.column()
        if column == 4 and self.table.item(qindex.row(), 1).text() in {"41"}:
            item = self.table.item(qindex.row(), qindex.column())
            self.table.openPersistentEditor(item)
            self.table.editItem(item)
            self.curItem = item
            self.editWidget = self.table.indexWidget(index)
            self.editWidget.setFocus()
            self.editWidget.editingFinished.connect(self.closeEditing)

    def closeEditing(self):
        if self.curItem:
            try:
                text = self.table.item(self.curItem.row(), 4).text()
                if text != "":
                    startBit = int(self.table.item(self.curItem.row(), 1).text())
                    bits = int(self.table.item(self.curItem.row(), 3).text())
                    value = int(self.table.item(self.curItem.row(), 4).text())
                    self.kcore.bits_assign(value, startBit, bits)
            except Exception as e:
                QMessageBox.warning(self, "Exception", "str 2 int exception", QMessageBox.Yes | QMessageBox.No)
                self.table.item(self.curItem.row(), 4).setText("")
            self.editWidget.close()
            self.table.closePersistentEditor(self.curItem)
            self.curItem = None


    def sortbyclounm(self, col):
        return
        if (col != 0):
            return
        self.bSortAsc = Qt.DescendingOrder if (self.bSortAsc==Qt.AscendingOrder) else (Qt.AscendingOrder)
        # self.table.sortItems(3, self.bSortAsc)

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
            kdata.out_robot_change[robot_num - 1] = 1
            if plc_num > 0:
                kdata.input_plc[plc_num] = 1
        else:
            kdata.output[robot_num - 1] = 0
            kdata.out_robot_change[robot_num - 1] = 0
            if plc_num > 0:
                kdata.input_plc[plc_num] = 0
        self.table.blockSignals(True)
        if self.table.item(row, 0).checkState() == Qt.Checked:
            self.table.item(row, 5).setBackground(QColor(128, 255, 155))
        else:
            #self.table.item(row, 0).setBackground(self.table.item(row, 1).background())
            self.table.item(row, 5).setBackground(QColor(255, 225, 255))

        # self.table.sortItems(3, self.bSortAsc)
        self.table.blockSignals(False)

    def timeout(self):
        rows = self.table.rowCount()
        self.table.blockSignals(True)
        for index in range(rows):
            try:
                strRobot = self.table.item(index, 1).text()
                intRobot = int(strRobot)
                if intRobot == 41:
                    bits_num = int(self.table.item(index, 3).text())
                    self.table.item(index, 4).setText(str(self.kcore.bits(intRobot, bits_num)))
                if (intRobot in kdata.out_robot_change):
                    if kdata.output[intRobot - 1] == 1:
                        self.table.item(index, 0).setCheckState(Qt.Checked)
                        self.table.item(index, 5).setBackground(QColor(128, 255, 155))
                    else:
                        self.table.item(index, 0).setCheckState(Qt.Unchecked)
                        self.table.item(index, 5).setBackground(QColor(255, 225 ,255))

            except Exception as e:
                print(e)
                pass
        self.table.blockSignals(False)
        kdata.sem.acquire()
        kdata.str = kdata.str + "RobotOutputs:\n"
        for idx in kdata.out_robot_change.keys():
            num = idx
            if kdata.output[num - 1] == 0:
                num = -1 * num
            matches = [x for x in kdata.dictlist if robot_eq_condition(x, idx)]
            if len(matches) == 1:
                wait_str = ("{}:{}\n").format(num, matches[0]['desc'])
                kdata.str = kdata.str + wait_str

        #kdata.out_robot_change.clear()
        kdata.sem.release()

    def resetTable(self):
        rows = self.table.rowCount()
        self.table.blockSignals(True)
        for index in range(rows):
            self.table.item(index, 5).setBackground(self.table.item(index, 1).background())
        self.table.blockSignals(False)

class Kawasaki(QMainWindow):
    def __init__(self, parent=None):
        super(Kawasaki, self).__init__(parent)
        self.timer = QTimer()
        self.timer.timeout.connect(self.timeout)
        self.timer.setInterval(2000)

        bar = QToolBar('Toolbar',self)
        bar.setIconSize(QSize(48, 48))
        bar.addWidget(WidgetSpacer(self))

        self.runAction = bar.addAction(icon("run","png"), ("Run"), self.progRun)
        self.stopAction = bar.addAction(icon("stop", "svg"), ("Stop"), self.stopRun)
        bar.addAction(icon("about","png"), ("Help"), self.showhelp)
        self.addToolBar(bar)

        self.kcore = Core()

        self.label = LabelWidget(self)
        self.label.setMinimumWidth(300)
        self.label.setMaximumWidth(400)
        self.addDockWidget(Qt.LeftDockWidgetArea, self.label)
        self.robotInPLCOutRWidget = RobotInPLCOutRWidget(self,self.kcore)
        self.addDockWidget(Qt.RightDockWidgetArea,self.robotInPLCOutRWidget)
        self.robotInPLCOutRWidget.setMinimumWidth(900)
        self.robotInPLCOutRWidget.setMinimumHeight(400)
        self.robotOutPLCInWidget = RobotOutPLCInWidget(self,self.kcore)
        self.addDockWidget(Qt.RightDockWidgetArea,self.robotOutPLCInWidget)
        self.robotOutPLCInWidget.setMinimumWidth(900)

        # self.tabber = QTabWidget(self)
        # self.stack = QStackedWidget(self)
        # self.stack.addWidget(self.tabber)
        # self.setCentralWidget(self.stack)

        self.resize(1640, 400)


        ##plc
        #self.kafka = Kafaka()
        #self.kafka.signalupdatePLCSig.connect(self.kcore.slotUpdatePLCSigs)
        self.timer.start()

    def timeout(self):
        self.robotOutPLCInWidget.timeout()
        self.robotInPLCOutRWidget.timeout()
        self.label.label.setText(kdata.str)
        QTimer.singleShot(1000, self.resetTable)

    def resetTable(self):
        #self.robotOutPLCInWidget.resetTable()
        #self.robotInPLCOutRWidget.resetTable()
        kdata.str = ""
        # self.label.label.setText(kdata.str)

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
    # pose_file = open('abb.txt', 'r')
    # try:
    #     index = 0
    #     while True:
    #         text_line = pose_file.readline()
    #         if text_line:
    #             worlds = text_line.split(',')
    #             robotXYZABC = []
    #             for c in worlds:
    #                 robotXYZABC.append(float(c))
    #             rpy=quaternion_to_euler_fc([robotXYZABC[3],robotXYZABC[4],robotXYZABC[5],robotXYZABC[6]])
    #             pose = [robotXYZABC[0],robotXYZABC[1],robotXYZABC[2],rpy[0],rpy[1],rpy[2]]
    #             print(pose)
    #             index = index + 1
    #         else:
    #             break
    # finally:
    #     pose_file.close()


    # pose_file = open('abb_rpy.txt', 'r')
    # try:
    #     index = 0
    #     while True:
    #         text_line = pose_file.readline()
    #         if text_line:
    #             worlds = text_line.split(',')
    #             robotXYZABC = []
    #             for c in worlds:
    #                 robotXYZABC.append(float(c))
    #             rpy=[robotXYZABC[3]/180*3.1415926,robotXYZABC[4]/180*3.1415926,robotXYZABC[5]/180*3.1415926]
    #             pose = [robotXYZABC[0],robotXYZABC[1],robotXYZABC[2],rpy[0],rpy[1],rpy[2]]
    #             print(pose)
    #             index = index + 1
    #         else:
    #             break
    # finally:
    #     pose_file.close()


    readsignal()
    app = QApplication(sys.argv)
    kawasaki = Kawasaki()
    kawasaki.show()
    sys.exit(app.exec())


