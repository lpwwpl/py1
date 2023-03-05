# This is a sample Python script.

# Press Shift+F10 to execute it or replace it with your code.
# Press Double Shift to search everywhere for classes, files, tool windows, actions, and settings.

import os
import goto
import numpy
from bitarray import bitarray
import sys
from PyQt5.QtCore import *
from PyQt5.QtGui import *
from PyQt5.QtWidgets import *
from kafka import KafkaConsumer
from dominate.tags import label
# import pyqode_i18n
import xlrd
# import markdown
dictlist = []
posnum = -1
jonum = 1
tooltet_k = [0.000000, 0.000000 ,371.000000, 0.000000 ,0.000000 ,0.000000]
TOOL = []
BASE = []
_pounce = numpy.zeros([3,6],dtype=float)
#read file
_pounce[1]=[-59.998997, -30.001200, -29.997200, 0.000000, -89.997704, 0.000000]
_pounce[2]=[90.000000, -30.001200, -29.998898, 0.000000, -90.000000, 0.000000]
_guodu = numpy.zeros([3,6],dtype=float)
_guodu[1] = [-101.199799, 31.869701, -54.043400, -78.818802, -90.773804, -53.225201]
_guodu[2] = [76.614998, 51.946201, -23.856701, -77.010498, -93.227501, -80.227699]
_tx1 = numpy.zeros([3,6],dtype=float)
_tx1[1] = [-98.765701, 55.930801, -34.390499, -81.223701, -89.925102, -48.895000]
_tx1[2] = [77.517502, 69.446297, -1.614500, -78.132103, -93.880898, -79.864502]
tx2 = numpy.zeros([63,6],dtype=float)
tx6 = numpy.zeros([3,6],dtype=float)
tx8 = numpy.zeros([3,6],dtype=float)
tx9 = numpy.zeros([3,6],dtype=float)
tx2[1] = [-1559.426880, 864.351990, 9.396500, 89.985100, 90.049210, 40.723793]
tx2[2] = [2047.815918, -590.146729, -65.465302, -90.012207, 90.152504, 28.519400]
tx2[3] = [-1559.401123, 858.414917, 23.059700, 89.984001, 90.049507, 40.723194]
tx2[4] = [2047.838257, -581.708130, -41.065399, -90.012405, 90.152710, 28.520899]
tx2[5] = [-1559.416992, 860.965210, 18.353800, 89.985397, 90.049210 ,40.722698]
tx2[6] = [2047.833496, -579.441711, -53.171101, -90.012711, 90.152710, 28.520998]
tx2[7] = [-1559.417236, 868.637817, 13.708600, 89.985199, 90.049309, 40.723595]
tx2[8] = [2047.824097, -584.691223, -51.510101, -90.012611, 90.152809, 28.520597]
tx2[9] = [-1564.628174, 862.964172, 51.431000, 89.985397, 90.049309, 40.722000]
tx2[10] = [2024.630737, -585.614624, -64.617401, -90.011810, 90.167206, 28.552799]
tx2[11] = [-1576.210815, 865.015686, 29.637501, 89.985100, 90.049110, 40.723793]
tx2[12] = [2043.386108, -591.193176, -65.460197, -90.011909, 90.152504, 28.519604]
tx2[13] = [-1559.427734, 862.671692, 23.051100, 89.985703, 90.049110, 40.723400]
tx2[14] = [2047.818237, -586.064270, -65.466301, -90.012009, 90.152306, 28.519400]
tx2[15] = [-1548.231567, 856.033203, 29.652800, 89.985603, 90.049507, 40.722595]
tx2[16] = [2047.821533, -591.346985, -58.218899, -90.011909 ,90.152504 ,28.519604]
tx2[17] = [-1575.945068, 875.215210, 23.049500, 89.985603, 90.049110, 40.723095]
tx2[18] = [2047.811890, -578.803711, -50.698399, -90.012108 ,90.152405 ,28.519098]
tx2[19] = [-1565.008667, 855.206299, 10.547700, 89.984497, 90.049408, 40.723400]
tx2[20] = [2047.811157, -583.685425, -49.708302, -90.014206, 90.152809, 28.520399]
tx2[21] = [-1559.439453, 865.690308, 8.395000, 89.984802, 90.049110, 40.725098]
tx2[22] = [2047.814087, -583.883789, -55.089802, -90.011909, 90.152504, 28.519201]
tx2[23] = [-1561.735718, 850.767822, 31.128201, 89.984703, 90.049408, 40.722996]
tx2[24] = [2048.257813, -589.813782, -56.311401, -90.012207, 90.152405, 28.519501]
tx2[25] = [-1559.421631, 866.044006, 15.260300, 89.985397, 90.049110, 40.722897]
tx2[26] = [2047.821777, -582.632690, -60.100601, -90.012009, 90.152504, 28.519602]
tx2[27] = [-1560.270508, 858.476318, 24.045900, 89.984001, 90.049011, 40.723301]
tx2[28] = [2047.805542, -585.439026, -58.554100, -90.012108, 90.152504, 28.518702]
tx2[29] = [-1559.440430, 868.468811, 23.070299, 89.985802, 90.049408, 40.723896]
tx2[30] = [2047.818848, -585.143005, -64.202904, -90.012108, 90.152504, 28.519501]
tx2[31] = [-1559.418945, 846.310425, 23.083900, 89.984703, 90.049309, 40.723797]
tx2[32] = [2047.820557, -583.883484, -65.468399, -90.011909, 90.152504, 28.519302]
tx2[33] = [-1559.425659, 856.415588,19.673401, 89.985100, 90.049011, 40.723701]
tx2[34] = [2044.709717, -588.558899, -65.464500, -90.012009, 90.152405, 28.519201]
tx2[35] = [-1559.422241, 861.276611, 23.050301, 89.985497, 90.049011, 40.722996]
tx2[36] = [2047.818970, -580.781372, -65.460503, -90.012009, 90.152504, 28.519503]
tx2[37] = [-1559.399902, 862.644897, 22.988701, 89.987099, 90.048805, 40.719101]
tx2[38] = [2047.814331, -592.978516 ,-57.500099, -90.012009, 90.152306, 28.518997]
tx2[39] = [-1555.590820, 873.957520, 20.723900, 89.986298, 90.049408, 40.722195]
tx2[40] = [2030.133667, -580.743225, -41.832401, -90.013306, 90.152710, 28.522499]
tx2[41] = [-1551.033447, 857.846375, 22.902000, 89.985001, 90.049507, 40.723495]
tx2[42] = [2031.996948, -583.889587, -57.667599, -90.013008, 90.153107, 33.837502]
tx2[43] = [-1559.415649, 858.982788, 23.039301, 89.985397, 90.049309, 40.722401]
tx2[44] = [2047.812622, -586.656189, -63.561298, -90.012108, 90.152306, 28.518896]
tx2[45] = [-1559.411377, 873.862122, 23.027901, 89.985199, 90.049408, 40.722095]
tx2[46] = [2047.820679, -592.636719, -58.839600, -90.012108, 90.247505, 28.519598]
tx2[47] = [-1550.565918, 862.624878,13.088300, 89.986099, 90.049210, 40.723499]
tx2[48] = [2047.818970, -586.385498, -65.462997, -90.012009, 90.152405 ,28.519400]
tx2[49] = [-1559.432251, 873.493530, 22.130301, 89.986099, 90.049507, 40.723400]
tx2[50] = [2047.822388, -588.507080, -65.459198, -90.011909, 90.152405, 28.519503]
tx2[51] = [-1559.402588, 858.454224, 23.061001, 89.984398, 90.049507, 40.723095]
tx2[52] = [2047.819214, -586.081116, -64.177696, -90.012009, 90.152405 ,28.519602]
tx2[53] = [-1559.423096, 866.348572, 23.051901, 89.985397, 90.049110, 40.723305]
tx2[54] = [2047.815918, -573.621826, -65.442497, -90.013611, 90.153107, 28.519400]
tx2[55] = [-1559.422852, 866.382874, 23.063101, 89.984901, 90.049110, 40.723701]
tx2[56] = [2032.098877, -581.904724, -64.925903, -90.013008, 90.162605, 28.540501]
tx2[57] = [-1559.923950, 859.863770, 23.981100, 89.986198, 90.050110, 40.761700]
tx2[58] = [2043.585327, -589.369629, -65.482498, -90.012207, 90.152405, 28.518402]
tx2[59] = [-1559.427734, 862.671692, 23.051100, 89.985703, 90.049110, 40.723400]
tx2[60] = [2047.820557, -581.517029, -50.005100, -90.012009, 90.152405, 28.519701]
tx2[61] = [-1559.416504, 854.525513, 23.038900, 89.985603, 90.049210, 40.722298]
tx2[62] = [2047.817749, -583.892029, -63.148800, -90.012207, 90.152504, 28.519602]
tx6[1] = [-1271.158447, 884.720398, 511.108795, 89.942703, 108.356010, 40.111801]
tx6[2] = [1746.301147, -599.541626, 429.343994, -90.024109, 109.197510, 36.575005]
tx8[1] = [-1267.918945, 883.513428, 359.025391, 89.957298, 108.350906, 40.156506]
tx8[2] = [1746.316772, -599.565979, 284.974487, -90.023911, 109.197006, 36.577595]
tx9[1] = [-1270.223633, 474.742889, 331.236603, 89.957703, 89.928703, 40.173801]
tx9[2] = [1734.526489, -299.983307, 256.315094, -90.009209, 90.546608, 36.584499]

_rx1 = numpy.zeros([3,6],dtype=float)
_rx1[1] = [-94.457001, 58.110199, -43.170502, -85.560204, -89.208900, 204.404099]
_rx1[2] = [79.176498, 67.103401, -14.254000, -79.232002, -91.087898, 188.565201]

input = 256*bitarray('0',endian ='big')
output = 256*bitarray('0',endian ='big')

out_start = 1
in_start = 1001

jobnum_check=1028
picknum=-1
palletnum=-1
mlhnum=-1

in_plc_start = 10001
out_plc_start = 20001

input_plc = 256*bitarray('0',endian ='big')
out_plc = 256*bitarray('0',endian ='big')
class Core(QThread):
    # signalUpdateTheOut = pyqtSignal(int)
    # signalUpdateThePLCOut = pyqtSignal(int)
    signalUpdateRobot = pyqtSignal()
    signalUpdatePLC = pyqtSignal()
    def __init__(self, parent=None):
        super(Core, self).__init__(parent)
        self.working = True

    def __del__(self):
        # 线程状态改变与线程终止
        self.working = False

    def run(self):
        print()
        while self.working == True:
            self.pickc1()
            break

    def sig(self,num):
        print()
        global input
        global output
        global in_start
        global out_start
        if(num>=in_start):
            return input[num - in_start] == 1
        elif num>=out_start:
            return output[num - out_start] == 1

    def bits(self, bit_start, bit_nums):
        global input
        global output
        global in_start
        global out_start
        start_bit = 0
        end_bit = 0
        if(bit_start>=in_start):
            start_bit = bit_start - in_start
            end_bit = start_bit + bit_nums  - 1
            return input[start_bit: end_bit]
        else:
            start_bit = bit_start - out_start
            end_bit = start_bit + bit_nums - 1
            return output[start_bit:end_bit]

    def bits_assign(self, value, bit_start, bit_nums):
        global input
        global output
        global in_start
        global out_start

        value_str = bin(value).replace('0b', '')
        value = value_str.zfill(bit_nums)

        start_bit = 0
        end_bit = 0
        if(bit_start>=in_start):
            start_bit = bit_start-in_start
            end_bit = start_bit + bit_nums - 1
            input[start_bit : end_bit] = bitarray(value)
        else:
            start_bit = bit_start - out_start
            end_bit = start_bit + bit_nums - 1
            output[start_bit: end_bit] = bitarray(value)

    def rposnum(self):
        global posnum
        global jonum
        label .x558
        posnum = 2
        if posnum <= 0 or posnum > 66:
            self.twait(0.5)
            goto .x558
        else:
            print(posnum)
        if posnum % 2 == 1:
            jonum = 1
        else:
            jonum = 2

    # freecad jmove
    def jmove(self, param, jonum):
        print()
        # Gui.applcation.....
        # print(param[jonum])

    def pause(self):
        print()

    def continue_(self):
        print()

    def find_by_plc_num(self,x,num):
        plc = int(x['plc'])
        return (plc == num)

    def find_by_robot_num(self,x,num):
        plc = int(x['robot'])
        return (plc == num)

    def signal_plc(self, updateUI=False, *array):
        global out_plc
        global input
        global dictlist
        for number in array:
            num = int(number)
            num_abs = abs(num)
            matches = [x for x in dictlist if self.find_by_plc_num(x,num_abs)]
            if num < 0:
                out_plc[num_abs - 1] = 0
                if(len(matches)>0):
                    dict = matches[0]
                    input[int(dict['robot'])] = 0
            else:
                out_plc[num_abs - 1] = 1
                if(len(matches)>0):
                    dict = matches[0]
                    input[int(dict['robot'])] = 1
            # if (updateUI):
            #     self.signalUpdateThePLCOut.emit(num_abs)
        if (updateUI):
            self.signalUpdatePLC.emit()
            self.signalUpdateRobot.emit()

    def signal(self, updateUI=False, *array):
        global output
        global input_plc
        global dictlist
        for number in array:
            num = int(number)
            num_abs = abs(num)
            matches = [x for x in dictlist if self.find_by_robot_num(x, num_abs)]
            if num < 0:
                output[num_abs - 1] = 0
                if (len(matches) > 0):
                    dict = matches[0]
                    idx = int(dict['plc'])
                    if (idx >= 10001):
                        idx = idx - 10001
                    else:
                        continue
                    input_plc[idx] = 0
            else:
                output[num_abs - 1] = 1
                if (len(matches) > 0):
                    dict = matches[0]
                    idx = int(dict['plc'])
                    if(idx>=10001):
                        idx = idx-10001
                    else:
                        continue
                    input_plc[idx] = 1
            # if (updateUI):
                # self.timer.start(1000)
                # self.signalUpdateTheOut.emit(num_abs)
        if (updateUI):
            self.signalUpdateRobot.emit()
            self.signalUpdatePLC.emit()

    # freecad lmove
    def lmove(self, param, posnum):
        print()
        # print(param[posnum])

    # freecad draw
    def draw(self, param):
        print()  # array

    # freecad break_
    def break_(self):
        # 等
        print()

    def swait(self,*array):
        global input
        global in_start
        flag = False
        while True:
            if(flag):
                break
            flag = True
            self.sleep(2)
            for number in array:
                num = int(number)
                num_abs = abs(num)
                num_abs = num_abs-in_start
                if (num < 0 and input[num_abs] == 0) or (num > 0 and input[num_abs] == 1):
                    flag = flag & True
                else:
                    flag = flag & False

    def twait(self, num):
        counter = 0
        while True:
            if counter>=num:
                break
            self.sleep(1)
            counter = counter + 1

    # freecad lappro
    def lappro(self, param, posnum, num):
        print()

    # freecad drive
    def drive(self, *array):
        print()

    # freecad tdraw
    def tdraw(self):
        print()

    # freecad here
    def here(self):
        print()

    def rpicknum(self):
        label .x632
        # picknum = self.bits(1185,8)
        picknum = 1
        if(picknum < 0) or (picknum > 48):
            self.twait(0.5)
            goto .x632
        else:
            print(picknum)

    def weight(self):
        print()

    def pickc4f(self):
        print()

    def pickc4q(self):
        global TOOL
        global tooltet_k
        global picknum
        self.rpicknum()
        # BASE NULL
        TOOL = tooltet_k
        self.weight()
        # ACCURACY 100 ALWAYS
        # SPEED 100 ALWAYS
        if picknum == 0:
            # LAPPRO dg[posnum*10+1],650
            self.signal(1,-2,-255,236)
            # SPEED 20
            # ACCURACY 1 FINE
        else:
            print()

    def pickc3(self):
        print()

    def pickc2(self):
        global jobnum
        global TOOL
        global BASE
        # BASE NULL
        TOOL = tooltet_k
        # ACCURACY 100 ALWAYS
        # SPEED 100 ALWAYS
        # ACCURACY 10
        # self.jappro fqq[posnum],222
        # ACCURACY 1 FINE
        # lmove() fqq[posnum];放气位置点
        # twait
        # swait 1197
        # accuracy 10
        # jappro fqq[posnum],222
        # signal(1,-2,-222,-221)
        # swait(1001,-1002)
        # accuracy 1
        # jmove qmlfront1[jonum]
        # signal(237)
        # lmove mldzx[posnum]
        # speed 50
        # accuracy 1 fine
        # lmove qml[posnum]
        # signal(-1,2)
        # twait(0.2)
        # swait(-1001,1002,1003)
        # lmove mldzx[posnum]
        # accuracy 50
        # jmove qmlfront4[posnum]
        label .replay1
        if self.sig(1003):
            self.signal(-232)
        else:
            self.signal(232)
            self.pause()
            goto .replay1
        # accuracy 100 always
        # speed 70 always
        self.jmove(_pounce,jobnum) #pounce[jobnum]
        self.signal(-214,-237)


    def tool(self,value,str):
        print()

    def accuracy(self,num):
        print()

    def speed(self,num):
        print()

    def lappro(self):
        print()

    def jappro(self):
        print()

    def shift(self,param,x,y,z):
        print()

    def point(self):
        print()

    #最上面为第一层序号1 - 16
    #中间为第二层序号17 - 32
    #使用粗定位值为最下面一层
    #如果最下面一层未识别使用最上面一层数据
    def pickf(self):
        # BASE NULL
        # tool()
        self.speed(100)
        self.accuracy(100)
        label .x160
        self.rpalletnum()   #读取码垛号
        if palletnum ==0:
            self.twait(0.5)
            goto .x160
        if palletnum > 32 and palletnum < 49:
            #位姿定义
            palletnum2 = palletnum - 32
            self.point() #aa[palletnum] = SHIFT(aa[palletnum2] BY 0,0,420)
            self.point() #a = aa[palletnum]
            self.point() #POINT pallet[palletnum] = a+cam
            self.point() #POINT/X aguodu = pallet[palletnum]
            self.point() #POINT/OAT aguodu = pallet[palletnum]
            up_limit = 350
            down_limit = -5
            #码垛
            # self.lmove(aguodu)
            # accuracy 1
            # JAPPRO pallet[palletnum], up_limit
            self.signal(237)
            self.signal (- 256)
            # SPEED 100
            # ACCURACY 1
            # LMOVE pallet[palletnum]
            # SPEED 40
            # LAPPRO pallet[palletnum], down_limit
            self.break_()
            self.signal(1, -2)
            self.twait(           0.1)
            self.SWAIT(        1001, -1002)
            self.signal(206)
            #码垛计数
            # SPEED           20
            # LMOVE           pallet[palletnum]
            # ACCURACY             1
            # LAPPRO            pallet[palletnum], up_limit
            self.signal(- 237, -214, 255)
            self.break_()
            # ACCURACY            20
            self.signal( 209)
            #码垛完成
            self.twait(0.1)
        if palletnum>16 and palletnum < 33:
            print()
            # palletnum3 = palletnum - 16
            # POINT      aa[palletnum] = SHIFT(aa[palletnum3]
            # BY           0, 0, 210)
            # POINT            a = aa[palletnum]
            # POINT            pallet[palletnum] = a + cam
            # POINT / X            aguodu = aa[palletnum]
            # POINT / OAT            aguodu = aa[palletnum]
            # up_limit = 350
            # down_limit = -5
        if palletnum > 0 and palletnum<17:
            print()

    def rpalletnum(self):
        label .x732
        palletnum = self.bits(1033,8)
        if palletnum < 0 or palletnum > 48:
            self.twait(0.5)
            goto .x732
        else:
            print(palletnum)

    def rmlhnum(self):
        label .x628
        mlhnum = self.bits(1161,8)
        if mlhnum <= 0 or mlhnum > 5:
            self.twait(0.5)
            goto .x628
        else:
            print(mlhnum)

    def pickc3(self):
        # BASE NULL
        self.tool(tooltet_k, "tooltet_k")
        self.accuracy(100)
        self.speed(100)
        self.signal(32)
        self.rpalletnum()
        if palletnum == 0:
            self.rmlhnum()
            self.accuracy(10)
            # lappro dg[]
            self.speed(20)
            # accuracy 1 fine
            # lmove dg
            self.break_()
            self.signal(1, -2, 206)
            self.twait(0.3)
            self.swait(1001, -1002)
            self.accuracy(1)
            # lappro
            self.break_()
            self.signal(206)
        else:
            # lmove
            # jmove
            self.pickf()
            # jmove
            # self.lmove
            self.signal(-32)

    def pickc1(self):
        global TOOL
        global BASE
        global _pounce
        global jonum
        global posnum
        TOOL = tooltet_k #tool()
        # ACCURACY 100 ALWAYS
        # SPEED 100 ALWAYS
        self.jmove(_pounce, jonum)
        self.signal(True, "-1", "2", "-3", "4")
        self.jmove(_guodu, jonum)
        self.jmove(_tx1, jonum)
        # ACCURACY 1 FINE
        self.lmove(tx2, posnum)
        param=["","",100]
        self.draw(param)
        # ACCURACY 1 FINE
        # SPEED 15
        self.lmove(tx6, jonum)
        self.break_()
        self.signal(True, "195")
        self.twait(0.3)
        # self.swait(1008,1016)


        # if self.sig(254) == True and self.sig(1006) == False:
        #     self.bits_assign(18,41,8)
        #     pos = self.bits(41,8)
        #     print(pos)
        # else:
        #     pos = self.bits(1,8)
        #     print(pos)

        # ACCURACY 1 FINE
        self.lmove(tx8, jonum)
        self.signal(True, "225")
        self.jmove(tx9, jonum)

        self.signal(True, "1", "-2", "214")
        # ACCURACY 50
        self.jmove(_rx1, jonum)
        # ACCURACY 1 FINE
        # lappro(rx2,posnum,5)
        self.signal(True, "-231")

        self.drive("6", "-370", "60")
        self.signal(True, "-225")
        self.drive("6", "-10", "60")
        # pause()
        if jonum == 1:
            param = ["",-40,"","","-50","60"]
            self.draw(param)
            # ACCURACY 1 FINE
            # DRAW, -32,, , , , 30
            self.break_()
        else:
            # DRAW, 40,, , 45,, 60 MM / s
            # ACCURACY 1 FINE
            # DRAW, 30,, , , , 30
            self.break_()
        self.signal(True, "3", "-4")
        self.twait(0.5)
        self.signal(True, "221")
        # ACCURACY 1 FINE
        self.drive("6", "25", "50")
        self.here()
        self.break_()

        # ACCURACY 1 ALWAYS
        TOOL = tooltet_k
        self.tdraw()
        self.signal(True, "222")
        if jonum == 1:
            # ACCURACY 1 FINE
            # self.draw()
            # self.draw()
            self.signal(True, "238")
            self.break_()
        else:
            # ACCURACY 1 FINE
            # self.draw("", "", 65, "", "", "", 100)
            # self.draw("198", "", "")
            self.signal(True, "238")
            self.break_()
        self.signal(True, "-3", "4")
        # ACCURACY 1 FINE
        self.tdraw()
        self.break_()
        self.signal(True, "-195", "-225", "-238", "-229")
        pos = self.bits(1, 8)
        print(pos)

def executable_path():
    if hasattr(sys, 'frozen'):
        return os.path.dirname(sys.executable)
    else:
        return os.path.dirname(sys.argv[0])

def image():
    return os.path.abspath(os.path.join(executable_path(),
                                         'images'))
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

kcore = Core()

def takeSecond(elem):
    # print(elem['robot'])
    return int(elem['robot'])

def readsignal():
    global dictlist

    path = os.path.abspath(os.path.join(executable_path(),"plc.xls"))
    data = xlrd.open_workbook(path)
    table = data.sheets()[0]
    nor = table.nrows
    nol = table.ncols
    dictlist = [dict() for x in range(nor-1)]

    for i in range(1, nor):
        elem = {}
        for j in range(nol):
            title = table.cell_value(0, j)
            value = table.cell_value(i, j)
            elem[title] = value
        dictlist[i-1] = elem
    dictlist.sort(key=takeSecond)


readsignal()

def robot_In_condition(x):
    robot = int(x['robot'])
    return (robot > 1001) and (robot <= 1256)

class RobotInPLCOutRWidget(QDockWidget):
    def __init__(self, parent):
        super(RobotInPLCOutRWidget, self).__init__(('PLC->Robot'), parent)
        widget = QWidget(self)
        horizontalHeader = ["State","PLC", "Robot","Desc"]

        self.table = QTableWidget() # 也可以QTableWidget(5,2)
        self.table.setColumnCount(4) # 5列
        self.table.setRowCount(256) # 3行
        self.table.setHorizontalHeaderLabels(horizontalHeader)
        self.table.setEditTriggers(QTableWidget.NoEditTriggers)
        self.table.setSelectionBehavior(QTableWidget.SelectRows)
        self.table.setSelectionMode(QTableWidget.SingleSelection)
        self.table.setAlternatingRowColors(True)
        self.table.horizontalHeader().setSectionResizeMode(QHeaderView.ResizeToContents)
        self.table.horizontalHeader().setSectionResizeMode(3,QHeaderView.Stretch)
        self.table.cellChanged.connect(self.cellChanged)
        mainLayout = QHBoxLayout()
        mainLayout.addWidget(self.table)
        widget.setLayout(mainLayout)
        self.setWidget(widget)
        self.resize(1024,512)
        matches = [x for x in dictlist if robot_In_condition(x)]

        for i in range(len(matches)):
            item = matches[i]
            chkBoxItem = QTableWidgetItem()
            chkBoxItem.setFlags(Qt.ItemIsUserCheckable | Qt.ItemIsEnabled)
            chkBoxItem.setCheckState(Qt.Unchecked)
            self.table.setItem(i, 0, chkBoxItem)

            if(item['plc'] != ""):
                newItem1 = QTableWidgetItem(str(item['plc']))
                newItem1.setTextAlignment(Qt.AlignHCenter | Qt.AlignVCenter)
                self.table.setItem(i, 1, newItem1)
            # else:
            #     newItem1 = QTableWidgetItem('unknown')
            #     newItem1.setTextAlignment(Qt.AlignHCenter | Qt.AlignVCenter)
            #     self.table.setItem(i, 1, newItem1)

            newItem1 = QTableWidgetItem(str(int(item['robot'])))
            newItem1.setTextAlignment(Qt.AlignHCenter | Qt.AlignVCenter)
            self.table.setItem(i, 2, newItem1)

            newItem1 = QTableWidgetItem(item['desc'])
            newItem1.setTextAlignment(Qt.AlignHCenter | Qt.AlignVCenter)
            self.table.setItem(i,3, newItem1)

    def cellChanged(self,row,col):
        global out_plc
        global input
        global dictlist
        plc_num = self.table.item(row, 1)
        robot_num = self.table.item(row, 2)
        if(plc_num == None or robot_num == None):
            return
        plc_num = plc_num.text()
        robot_num = robot_num.text()
        plc_num = int(plc_num)
        robot_num = int(robot_num)
        if (plc_num >= 20001):
            plc_num = plc_num - 20001
        if (robot_num >= 1001):
            robot_num = robot_num - 1001
        if self.table.item(row,col).checkState() == Qt.Checked:
            if(plc_num > 0):
                out_plc[plc_num - 1] = 1
            input[robot_num] = 1
        else:
            if (plc_num > 0):
                out_plc[plc_num - 1] = 0
            input[robot_num] = 0

def robot_out_condition(x):
    robot = int(x['robot'])
    return (robot > 0) and (robot <= 256)

class RobotOutPLCInWidget(QDockWidget):
    def __init__(self, parent):
        super(RobotOutPLCInWidget, self).__init__(('Robot->PLC'), parent)
        global dictlist
        widget = QWidget(self)
        horizontalHeader = ["State","Robot", "PLC", "Desc"]
        matches = [x for x in dictlist if robot_out_condition(x)]
        print(matches)
        self.table = QTableWidget() # 也可以QTableWidget(5,2)
        self.table.setColumnCount(4) # 5列
        self.table.setRowCount(len(matches)) # 3行
        self.table.setAlternatingRowColors(True)
        self.table.setShowGrid(True)
        self.table.setHorizontalHeaderLabels(horizontalHeader)
        self.table.setEditTriggers(QTableWidget.NoEditTriggers)
        self.table.setSelectionBehavior(QTableWidget.SelectRows)
        self.table.setSelectionMode(QTableWidget.SingleSelection)
        self.table.horizontalHeader().setSectionResizeMode(QHeaderView.ResizeToContents)
        self.table.horizontalHeader().setSectionResizeMode(3,QHeaderView.Stretch)
        self.table.cellChanged.connect(self.cellChanged)
        mainLayout = QHBoxLayout()
        mainLayout.addWidget(self.table)
        widget.setLayout(mainLayout)
        self.setWidget(widget)
        self.resize(1024,512)

        for i in range(len(matches)):
            item = matches[i]
            chkBoxItem = QTableWidgetItem()
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
            # else:
            #     newItem1 = QTableWidgetItem('unknown')
            #     newItem1.setTextAlignment(Qt.AlignHCenter | Qt.AlignVCenter)
            #     self.table.setItem(i, 2, newItem1)

            newItem1 = QTableWidgetItem(item['desc'])
            newItem1.setTextAlignment(Qt.AlignHCenter | Qt.AlignVCenter)
            self.table.setItem(i, 3, newItem1)

    def cellChanged(self,row,col):
        global output
        global input_plc
        global dictlist
        plc_num = self.table.item(row, 2)
        robot_num = self.table.item(row, 1)
        if (plc_num == None or robot_num == None):
            return
        plc_num = plc_num.text()
        robot_num = robot_num.text()
        plc_num = int(plc_num)
        robot_num = int(robot_num)
        if (plc_num >= 10001):
            plc_num = plc_num - 10001
        else:
            print(plc_num)
        if self.table.item(row,col).checkState() == Qt.Checked:
            output[robot_num - 1] = 1
            if(plc_num>0):
                input_plc[plc_num] = 1
        else:
            output[robot_num - 1] = 0
            if (plc_num > 0):
                input_plc[plc_num] = 0

class RobotSignalWidget(QDockWidget):
    def __init__(self, parent):
        super(RobotSignalWidget, self).__init__(('RobotSignalWidget'), parent)
        self.setWindowTitle(("RobotSignalWidget"))
        widget = QWidget(self)
        vlayout = QVBoxLayout()

        global input
        global output
        self.inpageNum = 0
        self.outpageNum = 0
        self.ONEPAGE = 32
        self.gridBox_input = QGridLayout()
        positions = [(i, j) for i in range(1) for j in range(32)]
        self.groupBox_input = QGroupBox("Input")
        self.inLabel1 = QLabel()
        self.inLabel2 = QLabel()
        self.inLabel3 = QLabel()
        self.inLabel4 = QLabel()
        self.cbx_in = [0 for i in range(0,32)]
        for x in range(0,32):
            self.cbx_in[x] = QCheckBox()

        for position, i in zip(positions , range(32)):
            self.gridBox_input.addWidget(self.cbx_in[i], *position)
            self.cbx_in[i].stateChanged.connect(self.change_input_signal)

        self.in_upbtn = QPushButton()
        self.in_upbtn.setFixedSize(QSize(48, 48))
        self.in_upbtn.setIcon(icon("icon_arrow_l"))
        self.in_downbtn = QPushButton()
        self.in_downbtn.setFixedSize(QSize(48, 48))
        self.in_downbtn.setIcon(icon("icon_arrow_r"))
        self.updateInLabel()
        self.gridBox_input.addWidget(self.inLabel1, 1, 0, 1, 1)
        self.gridBox_input.addWidget(self.inLabel2, 1, 7, 1, 1)
        self.gridBox_input.addWidget(self.inLabel3, 1, 15, 1, 1)
        self.gridBox_input.addWidget(self.inLabel4, 1, 31, 1, 1)
        self.gridBox_input.addWidget(self.in_upbtn, 0, 32)
        self.gridBox_input.addWidget(self.in_downbtn, 1, 32, 1, 1)
        self.in_upbtn.clicked.connect(self.in_upbtn_clicked)
        self.in_downbtn.clicked.connect(self.in_downbtn_clicked)
        self.groupBox_input.setLayout(self.gridBox_input)

        self.gridBox_output = QGridLayout()
        positions = [(i, j) for i in range(1) for j in range(32)]
        self.groupBox_output = QGroupBox("Output")
        self.outLabel1 = QLabel()
        self.outLabel2 = QLabel()
        self.outLabel3 = QLabel()
        self.outLabel4 = QLabel()
        self.cbx_out = [0 for i in range(0,32)]

        for x in range(0,32):
            self.cbx_out[x] = QCheckBox()

        for position, i in zip(positions, range(32)):
            self.gridBox_output.addWidget(self.cbx_out[i], *position)
            self.cbx_out[i].stateChanged.connect(self.change_output_signal)

        self.out_upbtn = QPushButton()
        self.out_upbtn.setFixedSize(QSize(48, 48))
        self.out_upbtn.setIcon(icon("icon_arrow_l"))
        self.out_downbtn = QPushButton()
        self.out_downbtn.setFixedSize(QSize(48, 48))
        self.out_downbtn.setIcon(icon("icon_arrow_r"))
        self.updateOutLabel()
        self.gridBox_output.addWidget(self.outLabel1, 1, 0, 1, 1)
        self.gridBox_output.addWidget(self.outLabel2, 1, 7, 1, 1)
        self.gridBox_output.addWidget(self.outLabel3, 1, 15, 1, 1)
        self.gridBox_output.addWidget(self.outLabel4, 1, 31, 1, 1)
        self.gridBox_output.addWidget(self.out_upbtn, 0, 32)
        self.gridBox_output.addWidget(self.out_downbtn, 1, 32, 1, 1)
        self.out_upbtn.clicked.connect(self.out_upbtn_clicked)
        self.out_downbtn.clicked.connect(self.out_downbtn_clicked)
        self.groupBox_output.setLayout(self.gridBox_output)

        for i in range(0, 32):
            self.cbx_in[i].setCheckState(Qt.Unchecked)
        for i in range(0, 32):
            self.cbx_out[i].setCheckState(Qt.Unchecked)

        vlayout.addWidget(self.groupBox_input)
        vlayout.addWidget(self.groupBox_output)
        vlayout.addStretch()
        widget.setLayout(vlayout)
        self.setWidget(widget)
        self.timer = QTimer()
        self.timer.timeout.connect(self.timeout)
        # self.timer.start(1000)

    def timeout(self):
        self.updateOutLabel()
        self.updateInLabel()
        self.timer.stop()

    def change_input_signal(self):
        global input
        pButton = self.sender()  # 取信号发射对象
        offset = self.inpageNum*self.ONEPAGE
        value = offset + self.cbx_in.index(pButton)
        bitValue = 1
        if(pButton.checkState() == Qt.Unchecked):
            # value = -1 * value
            bitValue = 0
        input[value] = bitValue

    def change_output_signal(self):
        global kcore
        pButton = self.sender()  # 取信号发射对象
        offset = self.outpageNum * self.ONEPAGE
        value = offset + self.cbx_out.index(pButton) + 1
        if(pButton.checkState() == Qt.Unchecked):
            value = -1 * value
        kcore.signal(False,value)

    def updateInLabel(self):
        global input
        global in_start
        label1_num = self.inpageNum*self.ONEPAGE + in_start
        self.inLabel1.setText(str(label1_num))
        self.inLabel2.setText(str(label1_num+7))
        self.inLabel3.setText(str(label1_num+15))
        self.inLabel4.setText(str(label1_num+31))
        for i in range(label1_num,label1_num+32):
            check_num = i - label1_num
            siganl_num = i - in_start
            self.cbx_in[check_num].blockSignals(True)
            if input[siganl_num] == 1:
                self.cbx_in[check_num].setCheckState(Qt.Checked)
            else:
                self.cbx_in[check_num].setCheckState(Qt.Unchecked)
            self.cbx_in[check_num].blockSignals(False)

    def slotUpdateSignals(self):
        self.timeout()

    def slotUpdateTheIn(self,signal_num):
        print()

    def slotUpdateTheOut(self, signal_num):
        global out_start
        global output
        label1_num = self.outpageNum * self.ONEPAGE + out_start
        start = label1_num
        end = label1_num+31
        # signal_num = signal_num-out_start
        if signal_num >= start and signal_num <= end:
            check_num = signal_num - start
            signal_numIndex = signal_num - 1
            if output[signal_numIndex] == 1:
                self.cbx_out[check_num].setCheckState(Qt.Checked)
            else:
                self.cbx_out[check_num].setCheckState(Qt.Unchecked)

    def updateOutLabel(self):
        global output
        global out_start
        label1_num = self.outpageNum*self.ONEPAGE+out_start
        self.outLabel1.setText(str(label1_num+0))
        self.outLabel2.setText(str(label1_num+7))
        self.outLabel3.setText(str(label1_num+15))
        self.outLabel4.setText(str(label1_num+31))
        for i in range(label1_num,label1_num+32):
            check_num = i - label1_num   #0,31
            signal_num = i - out_start           #0,31
            self.cbx_out[check_num].blockSignals(True)
            if output[signal_num] == 1:
                self.cbx_out[check_num].setCheckState(Qt.Checked)
            else:
                self.cbx_out[check_num].setCheckState(Qt.Unchecked)
            self.cbx_out[check_num].blockSignals(False)

    def in_upbtn_clicked(self):
        if(self.inpageNum==0):
            return
        for i in range(0,32):
            self.cbx_in[i].blockSignals(True)
            self.cbx_in[i].setCheckState(Qt.Unchecked)
            self.cbx_in[i].blockSignals(False)
        self.inpageNum = self.inpageNum - 1
        self.updateInLabel()


    def in_downbtn_clicked(self):
        if (self.inpageNum >= 7):
            return
        for i in range(0,32):
            self.cbx_in[i].blockSignals(True)
            self.cbx_in[i].setCheckState(Qt.Unchecked)
            self.cbx_in[i].blockSignals(False)
        self.inpageNum = self.inpageNum + 1
        self.updateInLabel()

    def out_upbtn_clicked(self):
        if(self.outpageNum==0):
            return
        for i in range(0,32):
            self.cbx_out[i].blockSignals(True)
            self.cbx_out[i].setCheckState(Qt.Unchecked)
            self.cbx_out[i].blockSignals(False)
        self.outpageNum = self.outpageNum - 1
        self.updateOutLabel()

    def out_downbtn_clicked(self):
        if (self.outpageNum >= 7):
            return
        for i in range(0,32):
            self.cbx_out[i].blockSignals(True)
            self.cbx_out[i].setCheckState(Qt.Unchecked)
            self.cbx_out[i].blockSignals(False)
        self.outpageNum = self.outpageNum + 1
        self.updateOutLabel()

class PLCSignalWidget(QDockWidget):
    def __init__(self, parent):
        super(PLCSignalWidget, self).__init__(('PLCSignalWidget'), parent)
        self.setWindowTitle(("PLCSignalWidget"))
        widget = QWidget(self)
        vlayout = QVBoxLayout()

        global input
        global output
        self.inpageNum = 0
        self.outpageNum = 0
        self.ONEPAGE = 32
        self.gridBox_input = QGridLayout()
        positions = [(i, j) for i in range(1) for j in range(32)]
        self.groupBox_input = QGroupBox("Input")
        self.inLabel1 = QLabel()
        self.inLabel2 = QLabel()
        self.inLabel3 = QLabel()
        self.inLabel4 = QLabel()
        self.cbx_in = [0 for i in range(0, 32)]
        for x in range(0, 32):
            self.cbx_in[x] = QCheckBox()

        for position, i in zip(positions, range(32)):
            self.gridBox_input.addWidget(self.cbx_in[i], *position)
            self.cbx_in[i].stateChanged.connect(self.change_input_signal)

        self.in_upbtn = QPushButton()
        self.in_upbtn.setFixedSize(QSize(48,48))
        self.in_upbtn.setIcon(icon("icon_arrow_l"))
        self.in_downbtn = QPushButton()
        self.in_downbtn.setFixedSize(QSize(48, 48))
        self.in_downbtn.setIcon(icon("icon_arrow_r"))
        self.updateInLabel()
        self.gridBox_input.addWidget(self.inLabel1, 1, 0, 1, 1)
        self.gridBox_input.addWidget(self.inLabel2, 1, 7, 1, 1)
        self.gridBox_input.addWidget(self.inLabel3, 1, 15, 1, 1)
        self.gridBox_input.addWidget(self.inLabel4, 1, 31, 1, 1)
        self.gridBox_input.addWidget(self.in_upbtn, 0, 32)
        self.gridBox_input.addWidget(self.in_downbtn, 1, 32, 1, 1)
        self.in_upbtn.clicked.connect(self.in_upbtn_clicked)
        self.in_downbtn.clicked.connect(self.in_downbtn_clicked)
        self.groupBox_input.setLayout(self.gridBox_input)

        self.gridBox_output = QGridLayout()
        positions = [(i, j) for i in range(1) for j in range(32)]
        self.groupBox_output = QGroupBox("Output")
        self.outLabel1 = QLabel()
        self.outLabel2 = QLabel()
        self.outLabel3 = QLabel()
        self.outLabel4 = QLabel()
        self.cbx_out = [0 for i in range(0, 32)]

        for x in range(0, 32):
            self.cbx_out[x] = QCheckBox()

        for position, i in zip(positions, range(32)):
            self.gridBox_output.addWidget(self.cbx_out[i], *position)
            self.cbx_out[i].stateChanged.connect(self.change_output_signal)

        self.out_upbtn = QPushButton()
        self.out_upbtn.setFixedSize(QSize(48, 48))
        self.out_upbtn.setIcon(icon("icon_arrow_l"))
        self.out_downbtn = QPushButton()
        self.out_downbtn.setFixedSize(QSize(48, 48))
        self.out_downbtn.setIcon(icon("icon_arrow_r"))
        self.updateOutLabel()
        self.gridBox_output.addWidget(self.outLabel1, 1, 0, 1, 1)
        self.gridBox_output.addWidget(self.outLabel2, 1, 7, 1, 1)
        self.gridBox_output.addWidget(self.outLabel3, 1, 15, 1, 1)
        self.gridBox_output.addWidget(self.outLabel4, 1, 31, 1, 1)
        self.gridBox_output.addWidget(self.out_upbtn, 0, 32)
        self.gridBox_output.addWidget(self.out_downbtn, 1, 32, 1, 1)
        self.out_upbtn.clicked.connect(self.out_upbtn_clicked)
        self.out_downbtn.clicked.connect(self.out_downbtn_clicked)
        self.groupBox_output.setLayout(self.gridBox_output)

        for i in range(0, 32):
            self.cbx_in[i].setCheckState(Qt.Unchecked)
        for i in range(0, 32):
            self.cbx_out[i].setCheckState(Qt.Unchecked)

        vlayout.addWidget(self.groupBox_input)
        vlayout.addWidget(self.groupBox_output)
        vlayout.addStretch()
        widget.setLayout(vlayout)
        self.setWidget(widget)
        self.timer = QTimer()
        self.timer.timeout.connect(self.timeout)
        # self.timer.start(1000)

    def timeout(self):
        self.updateInLabel()
        self.updateOutLabel()
        self.timer.stop()

    def change_input_signal(self):
        global input_plc
        pButton = self.sender()  # 取信号发射对象
        offset = self.inpageNum * self.ONEPAGE
        value = offset + self.cbx_in.index(pButton)
        bitValue = 1
        if (pButton.checkState() == Qt.Unchecked):
            # value = -1 * value
            bitValue = 0
        input_plc[value] = bitValue

    def change_output_signal(self):
        global kcore
        pButton = self.sender()  # 取信号发射对象
        offset = self.outpageNum * self.ONEPAGE
        value = offset + self.cbx_out.index(pButton) + 1
        if (pButton.checkState() == Qt.Unchecked):
            value = -1 * value
        kcore.signal_plc(False, value)

    def updateInLabel(self):
        global input_plc
        global in_plc_start
        label1_num = self.inpageNum * self.ONEPAGE + in_plc_start
        self.inLabel1.setText(str(label1_num))
        self.inLabel2.setText(str(label1_num + 7))
        self.inLabel3.setText(str(label1_num + 15))
        self.inLabel4.setText(str(label1_num + 31))
        for i in range(label1_num, label1_num + 32):
            check_num = i - label1_num
            siganl_num = i - in_plc_start
            self.cbx_in[check_num].blockSignals(True)
            if input_plc[siganl_num] == 1:
                self.cbx_in[check_num].setCheckState(Qt.Checked)
            else:
                self.cbx_in[check_num].setCheckState(Qt.Unchecked)
            self.cbx_in[check_num].blockSignals(False)

    def slotUpdateTheIn(self,signal_num):
        print()

    def slotUpdateSignals(self):
        self.timeout()

    def slotUpdateTheOut(self, signal_num):
        global out_plc_start
        global out_plc
        label1_num = self.outpageNum * self.ONEPAGE + out_plc_start
        start = label1_num
        end = label1_num + 31
        # signal_num = signal_num-out_start
        if signal_num >= start and signal_num <= end:
            check_num = signal_num - start
            signal_numIndex = signal_num - 1
            if out_plc[signal_numIndex] == 1:
                self.cbx_out[check_num].setCheckState(Qt.Checked)
            else:
                self.cbx_out[check_num].setCheckState(Qt.Unchecked)

    def updateOutLabel(self):
        global out_plc
        global out_plc_start
        label1_num = self.outpageNum * self.ONEPAGE + out_plc_start
        self.outLabel1.setText(str(label1_num + 0))
        self.outLabel2.setText(str(label1_num + 7))
        self.outLabel3.setText(str(label1_num + 15))
        self.outLabel4.setText(str(label1_num + 31))
        for i in range(label1_num, label1_num + 32):
            check_num = i - label1_num  # 0,31
            signal_num = i - out_plc_start  # 0,31
            self.cbx_out[check_num].blockSignals(True)
            if out_plc[signal_num] == 1:
                self.cbx_out[check_num].setCheckState(Qt.Checked)
            else:
                self.cbx_out[check_num].setCheckState(Qt.Unchecked)
            self.cbx_out[check_num].blockSignals(False)

    def in_upbtn_clicked(self):
        if (self.inpageNum == 0):
            return
        for i in range(0, 32):
            self.cbx_in[i].blockSignals(True)
            self.cbx_in[i].setCheckState(Qt.Unchecked)
            self.cbx_in[i].blockSignals(False)
        self.inpageNum = self.inpageNum - 1
        self.updateInLabel()

    def in_downbtn_clicked(self):
        if (self.inpageNum >= 7):
            return
        for i in range(0, 32):
            self.cbx_in[i].blockSignals(True)
            self.cbx_in[i].setCheckState(Qt.Unchecked)
            self.cbx_in[i].blockSignals(False)
        self.inpageNum = self.inpageNum + 1
        self.updateInLabel()

    def out_upbtn_clicked(self):
        if (self.outpageNum == 0):
            return
        for i in range(0, 32):
            self.cbx_out[i].blockSignals(True)
            self.cbx_out[i].setCheckState(Qt.Unchecked)
            self.cbx_out[i].blockSignals(False)
        self.outpageNum = self.outpageNum - 1
        self.updateOutLabel()

    def out_downbtn_clicked(self):
        if (self.outpageNum >= 7):
            return
        for i in range(0, 32):
            self.cbx_out[i].blockSignals(True)
            self.cbx_out[i].setCheckState(Qt.Unchecked)
            self.cbx_out[i].blockSignals(False)
        self.outpageNum = self.outpageNum + 1
        self.updateOutLabel()
        
class Kawasaki(QMainWindow):
    def __init__(self, parent=None):
        super(Kawasaki, self).__init__(parent)
        bar = QToolBar('Toolbar',self)
        bar.setIconSize(QSize(48, 48))
        # bar.addAction(icon("document-new"), ("New"), self.fileNew)
        # bar.addAction(icon("document-open"), ("Open"), self.fileOpen)
        # bar.addAction(icon("document-save"), ("Save"), self.fileSave)
        bar.addWidget(WidgetSpacer(self))
        self.runAction = bar.addAction(icon("run","png"), ("Run"), self.progRun)
        bar.addAction(icon("about","png"), ("Help"), self.showhelp)
        self.addToolBar(bar)

        self.robotsignalWidget = RobotSignalWidget(self)
        self.addDockWidget(Qt.BottomDockWidgetArea, self.robotsignalWidget)

        self.plcsignalWidget = PLCSignalWidget(self)
        self.addDockWidget(Qt.BottomDockWidgetArea,self.plcsignalWidget)

        self.robotInPLCOutRWidget = RobotInPLCOutRWidget(self)
        self.addDockWidget(Qt.TopDockWidgetArea,self.robotInPLCOutRWidget)

        self.robotOutPLCInWidget = RobotOutPLCInWidget(self)
        self.addDockWidget(Qt.TopDockWidgetArea,self.robotOutPLCInWidget)

        # self.tabber = QTabWidget(self)
        # self.stack = QStackedWidget(self)
        # self.stack.addWidget(self.tabber)
        # self.setCentralWidget(self.stack)
        self.resize(1024, 1024)

        kcore.signalUpdateRobot.connect(self.robotsignalWidget.slotUpdateSignals)#slotUpdateTheOut
        kcore.signalUpdatePLC.connect(self.plcsignalWidget.slotUpdateSignals) #slotUpdateTheOut

        # self.kcore = Core()
        # kcore.signalUpdateTheOut.connect(self.robotsignalWidget.slotUpdateSignals)#slotUpdateTheOut
        # kcore.signalUpdateThePLCOut.connect(self.plcsignalWidget.slotUpdateSignals) #slotUpdateTheOut
        # self.plcsignalWidget.signalUpdateTheOut.connect(self.robotsignalWidget.slotUpdateTheIn)
        # self.robotsignalWidget.signalUpdateTheOut.connect(self.plcsignalWidget.slotUpdateTheIn)
        #kcore.start()

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
        # with open(self._cssfile()) as f:
        #     tv.document().setDefaultStyleSheet(f.read())
        # try:
        #     with open(self._mdhelp()) as f:
        #         tv.document().setHtml(markdown.markdown(f.read()))
        # except:
        #     try:
        #         with open(self._htmlhelp()) as f:
        #             tv.document().setHtml(f.read())
        #     except:
        #         tv.setHtml("No help")
        dlg.exec_()

    def progRun(self):
        if(kcore.isRunning()):
            kcore.working = False
            kcore.wait()
        kcore.start()


# Press the green button in the gutter to run the script.
if __name__ == '__main__':
    # consumer = KafkaConsumer('sx_tp_0608',bootstrap_servers=['113.31.111.188:9092'])
    # for msg in consumer:
    #     print(msg)
    app = QApplication(sys.argv)
    kawasaki = Kawasaki()
    kawasaki.show()
    sys.exit(app.exec())


