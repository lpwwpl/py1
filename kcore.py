from PyQt5.QtCore import *
from bitarray import bitarray
import bitarray.util
from goto import *
import json


import kdata

class Core(QThread):

    def __init__(self, parent=None):
        super(Core, self).__init__(parent)
        self.working = True

    def __del__(self):
        self.working = False

    def run(self):
        while self.working == True:
            self.main() #main
            break

    def slotUpdatePLCSigs(self):
        msg = kdata.jqr_msg.decode('utf-8')
        data = json.loads(msg)
        for (k,v) in data.items():
            k = int(k)
            matches = [x for x in kdata.dictlist if self.find_by_plc_num(x, k)]
            if(k>=10001 and k <= 10046):
                k = k - 10001
                kdata.input_plc[k] = v
            if(k>=20005 and k<= 20039):
                kdata.out_plc_change[k] = 1
                k = k - 20001
                kdata.out_plc[k] = v
                if (len(matches) > 0):
                    dict = matches[0]
                    idx = int(dict['robot'])
                    if (idx >= 1001):
                        idx = idx - 1001
                    else:
                        continue
                    kdata.input[idx] = v
            elif(k in (20001,20002,20003,20004)):
                kdata.plc_20001_to_20004[k]=v


    def sig(self,*array):
        flag = True
        for number in array:
            num = int(number)
            num_abs = abs(num)
            if(num_abs>=kdata.in_start):
                if num>0:
                    flag = flag & (kdata.input[num - kdata.in_start] == 1)
                else:
                    flag = flag & (kdata.input[num - kdata.in_start] == 0)
            elif num_abs>=kdata.out_start:
                if num > 0:
                    flag = flag & (kdata.output[num - kdata.out_start] == 1)
                else:
                    flag = flag & (kdata.output[num - kdata.out_start] == 0)
        return flag

    def bits(self, bit_start, bit_nums):
        start_bit = 0
        end_bit = 0
        if(bit_start >= kdata.in_start):
            start_bit = bit_start - kdata.in_start
            end_bit = start_bit + bit_nums
            value = int(kdata.input[start_bit: end_bit].to01(),2)
            return value
        else:
            start_bit = bit_start - kdata.out_start
            end_bit = start_bit + bit_nums
            value = int(kdata.output[start_bit:end_bit].to01(),2)
            return value

    def bits_assign(self, value, bit_start, bit_nums):
        value_str = bin(value).replace('0b', '')
        value = value_str.zfill(bit_nums)

        start_bit = 0
        end_bit = 0
        if(bit_start >= kdata.in_start):
            start_bit = bit_start - kdata.in_start
            end_bit = start_bit + bit_nums
            kdata.input[start_bit : end_bit] = bitarray.bitarray(value,endian ='big')
        else:
            start_bit = bit_start - kdata.out_start
            end_bit = start_bit + bit_nums
            kdata.output[start_bit: end_bit] = bitarray.bitarray(value,endian ='big')
            kdata.out_robot_change[start_bit] = 1

    # freecad jmove
    def jmove(self, param, str, JointType=True):
        pass

    def pause(self):
        pass

    def find_by_plc_num(self,x,num):
        plc = int(x['plc'])
        return (plc == num)

    def find_by_robot_num(self,x,num):
        robot = int(x['robot'])
        return (robot == num)

    def signal_plc(self,  *array):
        for number in array:
            num = int(number)
            num_abs = abs(num)
            matches = [x for x in kdata.dictlist if self.find_by_plc_num(x,num_abs)]
            if (len(matches) > 0):
                dict = matches[0]
                plc_idx = num_abs
                robot_idx = int(dict['robot'])
                if (plc_idx >= 20001):
                    plc_idx = plc_idx - 20001
                else:
                    continue
                if (robot_idx >= 1001):
                    robot_idx = robot_idx - 1001
                else:
                    continue
                if num < 0:
                    kdata.out_plc[plc_idx] = 0
                    kdata.input[robot_idx] = 0
                else:
                    kdata.out_plc[plc_idx] = 1
                    kdata.input[robot_idx] = 1

        self.signalUpdatePLC.emit()

    def signal(self,  *array):
        kdata.sem.acquire()
        for number in array:
            num = int(number)
            num_abs = abs(num)
            matches = [x for x in kdata.dictlist if self.find_by_robot_num(x, num_abs)]
            if (len(matches) > 0):
                dict = matches[0]
                idx = int(dict['plc'])
                if (idx >= 10001):
                    idx = idx - 10001
                else:
                    pass
            else:
                continue
            kdata.out_robot_change[num_abs] = 1
            if num < 0:
                kdata.output[num_abs - 1] = 0
                kdata.input_plc[idx] = 0
            else:
                kdata.output[num_abs - 1] = 1
                kdata.input_plc[idx] = 1
        kdata.sem.release()


    # freecad lmove
    def lmove(self, param, str):
        pass

    # freecad draw
    def draw(self, str):
        pass

    def type(self,str):
        pass

    def decode(self,val1,val2,val3):
        pass

    def val(self,val1):
        pass

    # freecad break_
    def break_(self):
        pass

    def swait(self,*array):
        flag = False
        while True:
            if(flag):
                break
            flag = True
            self.sleep(2)
            for number in array:
                num = int(number)
                num_abs = abs(num)
                num_abs_idx = num_abs - kdata.in_start
                if num < 0 and kdata.input[num_abs_idx] != 0:
                    kdata.swait_dict[num_abs] = 0
                elif num > 0 and kdata.input[num_abs_idx] != 1:
                    kdata.swait_dict[num_abs] = 1
                if (num < 0 and kdata.input[num_abs_idx] == 0) or (num > 0 and kdata.input[num_abs_idx] == 1):
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
    def lappro(self, param, str,num):
        pass

    # freecad drive
    def drive(self, str):
        pass

    # freecad tdraw
    def tdraw(self, str):
        pass

    # freecad here
    def here(self):
        pass

    def weight(self,str):
        pass

    def ldepart(self,value):
        pass

    def base(self,param,str):
        pass

    def tool(self,param,str):
        pass

    def accuracy(self,str):
        pass

    def speed(self,str):
        pass

    def jappro(self,param,str,value):
        pass

    def pulse(self,val1,val2):
        pass

    def shift(self,param,x,y,z):
        value = 6 * [0]
        value[0] = param[0] + x
        value[1] = param[1] + y
        value[2] = param[2] + z
        value[3] = param[3]
        value[4] = param[4]
        value[5] = param[5]
        return value

    def home(self, param):
        pass

    def pick_originl_au1(self):
        kdata.pick_original[48] = self.shift(kdata.original_new, 0, 0, 0)
        kdata.pick_original[47] = self.shift(kdata.original_new, -265, 0, 0)
        kdata.pick_original[46] = self.shift(kdata.original_new, -265 * 2, 0, 0)
        kdata.pick_original[45] = self.shift(kdata.original_new, -265 * 3, 0, 0)
        kdata.pick_original[44] = self.shift(kdata.original_new, 0, 265, 0)
        kdata.pick_original[43] = self.shift(kdata.original_new, -265, 265, 0)
        kdata.pick_original[42] = self.shift(kdata.original_new, -265 * 2, 265, 0)
        kdata.pick_original[41] = self.shift(kdata.original_new, -265 * 3, 265, 0)
        kdata.pick_original[40] = self.shift(kdata.original_new, 0, 265 * 2, 0)
        kdata.pick_original[39] = self.shift(kdata.original_new, -265, 265 * 2, 0)
        kdata.pick_original[38] = self.shift(kdata.original_new, -265 * 2, 265 * 2, 0)
        kdata.pick_original[37] = self.shift(kdata.original_new, -265 * 3, 265 * 2, 0)
        kdata.pick_original[36] = self.shift(kdata.original_new, 0, 265 * 3, 0)
        kdata.pick_original[35] = self.shift(kdata.original_new, -265, 265 * 3, 0)
        kdata.pick_original[34] = self.shift(kdata.original_new, -265 * 2, 265 * 3, 0)
        kdata.pick_original[33] = self.shift(kdata.original_new, -265 * 3, 265 * 3, 0)

    def pick_originl_au2(self):
        kdata.pick_original[48]=self.shift(kdata.original_new,0,0,0)
        kdata.pick_original[47]=self.shift(kdata.original_new,-285,0,0)
        kdata.pick_original[46]=self.shift(kdata.original_new,-285 * 2,0,0)
        kdata.pick_original[45]=self.shift(kdata.original_new,-285 * 3,0,0)
        kdata.pick_original[44]=self.shift(kdata.original_new,0,265,0)
        kdata.pick_original[43]=self.shift(kdata.original_new,-285,265,0)
        kdata.pick_original[42]=self.shift(kdata.original_new,-285 * 2,265,0)
        kdata.pick_original[41]=self.shift(kdata.original_new,-285 * 3,265,0)
        kdata.pick_original[40]=self.shift(kdata.original_new,0,265 * 2,0)
        kdata.pick_original[39]=self.shift(kdata.original_new,-285,265 * 2,0)
        kdata.pick_original[38]=self.shift(kdata.original_new,-285 * 2,265 * 2,0)
        kdata.pick_original[37]=self.shift(kdata.original_new,-285 * 3,265 * 2,0)
        kdata.pick_original[36]=self.shift(kdata.original_new,0,265 * 3,0)
        kdata.pick_original[35]=self.shift(kdata.original_new,-285,265 * 3,0)
        kdata.pick_original[34]=self.shift(kdata.original_new,-285 * 2,265 * 3,0)
        kdata.pick_original[33]=self.shift(kdata.original_new,-285 * 3,265 * 3,0)

    # def pick_originl_au(self):
    #     kdata.original_new=kdata.aa[48]
    #     print("Pallet_type",kdata.pallet_type)
    #     numbers={
    #         1:
    #             self.pick_originl_au1,
    #         2:
    #             self.pick_originl_au2
    #     }
    #     method=numbers.get(kdata.pallet_type)
    #     method()

    def stop(self):
        pass

    def pickdg_tech(self):
        pass

    def other(self):
        pass

    def distance(self,val1,val2):
        pass

    def other(self):
        pass

    #self.str_format("pick_original[{}]","picknum")
    def str_format(self,param1,param2):
        ret = param1.format(param2)
        print(ret)
        return ret

    @allow_goto
    def cam_rlink_gv(self):
        kdata.recv_buf[1] = ""
        self.stscus_tcp()
        self.com_init()
        self.open_socket()
        if kdata.sock_id < 0:
            pass
        kdata.text_id = 0
        kdata.tout = 60
        kdata.ret = 0
        self.communicate5()
        self.twait(0.6)
        self.communicate3()
        self.twait(3)
        self.communicate4()
        self.twait(1.5)
        self.communicate6()
        self.twait(1)
        self.close_socket()

    @allow_goto
    def close_socket(self):
        # self.tcp_close(kdata.ret,kdata.sock_id)
        if kdata.ret < 0:
            print("断开连接错误 返回值：(", kdata.ret, " )", "错误代码：", )
            # self.tcp_close(kdata.ret1,kdata.sock_id)
            if kdata.ret1 < 0:
                print("强制断开连接错误 返回值：", kdata.sock_id)
        else:
            print("断开连接成功 返回值：", kdata.sock_id)

    @allow_goto
    def com_init(self):
        kdata.port = 8899
        kdata.ip[1] = 192
        kdata.ip[2] = 168
        kdata.ip[3] = 11
        kdata.ip[4] = 12
        kdata.max_length = 255
        kdata.tout_open = 20
        kdata.tout = 30
        kdata.tout_rec = 10
        kdata.sret = 0
        kdata.ret = 0
        kdata.text_id = 0
        num = 0
        kdata.connect_count = 5

    @allow_goto
    def communicate1(self):
        kdata.chufa_buf[1] = "000025"
        kdata.chufa_buf[2] = "#"
        kdata.chufa_buf[3] = "0111"
        kdata.chufa_buf[4] = "@"
        kdata.chufa_buf[5] = "0009"
        kdata.chufa_buf[6] = "&"
        kdata.chufa_buf[7] = "0004"
        kdata.chufa_buf[8] = " ,"
        kdata.chufa_buf[9] = "0"
        kdata.chufa_buf[10] = ","
        kdata.chufa_buf[11] = "0"
        kdata.chufa_buf[12] = ";"
        kdata.chufa_buf[13] = "0000"
        kdata.chufa_buf[14] = "#"
        kdata.buf_n = 14
        label.sent_rt
        # self.tcp_send(kdata.sret,kdata.sock_id,kdata.chufa_buf[1],kdata.buf_n,kdata.tout)

    @allow_goto
    def communicate2(self):
        kdata.chufa_buf[1] = "000025"
        kdata.chufa_buf[2] = "#"
        kdata.chufa_buf[3] = "0111"
        kdata.chufa_buf[4] = "@"
        kdata.chufa_buf[5] = "0009"
        kdata.chufa_buf[6] = "&"
        kdata.chufa_buf[7] = "0004"
        kdata.chufa_buf[8] = " ,"
        kdata.chufa_buf[9] = "1"
        kdata.chufa_buf[10] = ","
        kdata.chufa_buf[11] = "0"
        kdata.chufa_buf[12] = ";"
        kdata.chufa_buf[13] = "0000"
        kdata.chufa_buf[14] = "#"
        kdata.buf_n = 14
        label.sent_rt
        # self.tcp_send(kdata.sret,kdata.sock_id,kdata.chufa_buf[1],kdata.buf_n,kdata.tout)

    @allow_goto
    def communicate3(self):
        kdata.chufa_buf[1] = "000021"
        kdata.chufa_buf[2] = "#"
        kdata.chufa_buf[3] = "0118"
        kdata.chufa_buf[4] = "@"
        kdata.chufa_buf[5] = "0005"
        kdata.chufa_buf[6] = "&"
        kdata.chufa_buf[7] = "0000"
        kdata.chufa_buf[8] = " ;"
        kdata.chufa_buf[9] = "0000"
        kdata.chufa_buf[10] = "#"
        kdata.buf_n = 10
        label.sent_rt
        # self.tcp_send(kdata.sret,kdata.sock_id,kdata.chufa_buf[1],kdata.buf_n,kdata.tout)

    @allow_goto
    def communicate4(self):
        kdata.chufa_buf[1] = "000021"
        kdata.chufa_buf[2] = "#"
        kdata.chufa_buf[3] = "0119"
        kdata.chufa_buf[4] = "@"
        kdata.chufa_buf[5] = "0005"
        kdata.chufa_buf[6] = "&"
        kdata.chufa_buf[7] = "0000"
        kdata.chufa_buf[8] = " ;"
        kdata.chufa_buf[9] = "0000"
        kdata.chufa_buf[10] = "#"
        kdata.buf_n = 10
        label.sent_rt
        # self.tcp_send(kdata.sret,kdata.sock_id,kdata.chufa_buf[1],kdata.buf_n,kdata.tout)

    @allow_goto
    def communicate5(self):
        kdata.chufa_buf[1] = "000021"
        kdata.chufa_buf[2] = "#"
        kdata.chufa_buf[3] = "0103"
        kdata.chufa_buf[4] = "@"
        kdata.chufa_buf[5] = "0005"
        kdata.chufa_buf[6] = "&"
        kdata.chufa_buf[7] = "0000"
        kdata.chufa_buf[8] = " ;"
        kdata.chufa_buf[9] = "0000"
        kdata.chufa_buf[10] = "#"
        kdata.buf_n = 10
        label.sent_rt
        # self.tcp_send(kdata.sret,kdata.sock_id,kdata.chufa_buf[1],kdata.buf_n,kdata.tout)

    @allow_goto
    def communicate6(self):
        kdata.chufa_buf[1] = "000021"
        kdata.chufa_buf[2] = "#"
        kdata.chufa_buf[3] = "0101"
        kdata.chufa_buf[4] = "@"
        kdata.chufa_buf[5] = "0005"
        kdata.chufa_buf[6] = "&"
        kdata.chufa_buf[7] = "0000"
        kdata.chufa_buf[8] = " ;"
        kdata.chufa_buf[9] = "0000"
        kdata.chufa_buf[10] = "#"
        kdata.buf_n = 10
        label.sent_rt
        # self.tcp_send(kdata.sret,kdata.sock_id,kdata.chufa_buf[1],kdata.buf_n,kdata.tout)

    @allow_goto
    def initialize(self):
        kdata.jobnum_check = 1028
        self.home(1)
        self.twait(0.3)
        self.signal(1, -2, -3, 25, -27, -26, -199, -195, -225, -221, -229, -231, -223, -224, -226, -228, -232)
        self.tool(None, "NULL")
        self.base(None, "NULL")
        self.signal(-210, -212, -213, -211, -215, -205, -206, -253, -254, -255, -256, -236, -214, -237, -209)
        self.bits_assign(0, 190, 10)
        self.bits_assign(0, 200, 3)
        self.bits_assign(0, 41, 8)
        kdata.picknum = -1
        kdata.palletnum = -1
        kdata.rmlhnum = -1
        self.speed50200()
        return

    @allow_goto
    def jianju(self):
        kdata.d[1] = self.distance(kdata.aa[48], kdata.aa[47])
        kdata.d[2] = self.distance(kdata.aa[47], kdata.aa[46])
        kdata.d[3] = self.distance(kdata.aa[46], kdata.aa[45])
        kdata.d[4] = self.distance(kdata.aa[44], kdata.aa[43])
        kdata.d[5] = self.distance(kdata.aa[43], kdata.aa[42])
        kdata.d[6] = self.distance(kdata.aa[42], kdata.aa[41])
        kdata.d[7] = self.distance(kdata.aa[40], kdata.aa[39])
        kdata.d[8] = self.distance(kdata.aa[39], kdata.aa[38])
        kdata.d[9] = self.distance(kdata.aa[38], kdata.aa[37])
        kdata.d[10] = self.distance(kdata.aa[36], kdata.aa[35])
        kdata.d[11] = self.distance(kdata.aa[35], kdata.aa[34])
        kdata.d[12] = self.distance(kdata.aa[34], kdata.aa[33])
        kdata.d[13] = self.distance(kdata.aa[48], kdata.aa[44])
        kdata.d[14] = self.distance(kdata.aa[47], kdata.aa[43])
        kdata.d[15] = self.distance(kdata.aa[46], kdata.aa[42])
        kdata.d[16] = self.distance(kdata.aa[45], kdata.aa[41])
        kdata.d[17] = self.distance(kdata.aa[44], kdata.aa[40])
        kdata.d[18] = self.distance(kdata.aa[43], kdata.aa[39])
        kdata.d[19] = self.distance(kdata.aa[42], kdata.aa[38])
        kdata.d[20] = self.distance(kdata.aa[41], kdata.aa[37])
        kdata.d[21] = self.distance(kdata.aa[40], kdata.aa[36])
        kdata.d[22] = self.distance(kdata.aa[39], kdata.aa[35])
        kdata.d[23] = self.distance(kdata.aa[38], kdata.aa[34])
        kdata.d[24] = self.distance(kdata.aa[37], kdata.aa[33])
        self.twait(0.3)
        kdata.konglun_285_cou = 0
        kdata.pallet_type = 0
        for kdata.i in range(24):
            if kdata.d[kdata.i] > 278:
                kdata.konglun_285_cou = kdata.konglun_285_cou + 1
        if kdata.konglun_285_cou >= 8:
            print("该托盘为新垛盘")
            kdata.pallet_type = 2
            self.jianju_2()
        if kdata.konglun_285_cou <= 4:
            print("该托盘为旧垛盘")
            kdata.pallet_type = 1
            self.jianju_1()
        if kdata.konglun_285_cou > 4 and kdata.konglun_285_cou < 8:
            print("托盘未捅直，检查垛盘")
            kdata.pallet_type = 3
            kdata.konglun_ng_coun = 24
        return

    @allow_goto
    def jianju_type_pos(self):
        kdata.d[1] = self.distance(kdata.aa[48], kdata.aa[47])
        kdata.d[2] = self.distance(kdata.aa[47], kdata.aa[46])
        kdata.d[3] = self.distance(kdata.aa[48], kdata.aa[46])
        print("d[1]", kdata.d[1])
        print("d[2]", kdata.d[2])
        print("d[3]", kdata.d[3])
        kdata.pallet_type = 0
        if kdata.d[1] > 275 and kdata.d[2] < 295:
            if kdata.d[2] > 275 and kdata.d[2] < 295:
                if kdata.d[3] > 555 and kdata.d[3] < 580:
                    print("该托盘为新垛盘")
                    kdata.pallet_type = 2
        if kdata.d[1] > 255 and kdata.d[2] < 275:
            if kdata.d[2] > 255 and kdata.d[2] < 275:
                if kdata.d[3] > 520 and kdata.d[3] < 545:
                    print("该托盘为旧垛盘")
                    kdata.pallet_type = 1
        return

    @allow_goto
    def main(self):
        self.signal(-25)
        label .start
        self.home(1)
        self.initialize()
        kdata.jobnum=self.bits(1041,8)
        kdata.in_robot_change[1041] = 0
        print("jobnum",kdata.jobnum)
        numbers = {
            2: self.work2,
            3: self.work3,
            4: self.work4,
            5: self.work5,
            6: self.work6
        }
        method=numbers.get(kdata.jobnum, self.other)
        if(method):
            method()
        self.bits_assign(0, 1041, 8)
        goto .start
        self.home(1)
        self.stop()

    @allow_goto
    def open_socket(self):
        er_count = 0
        label.connect
        # self.tcp_connect(kdata.sock_id,kdata.port,kdata.ip[1],kdata.tout_open)
        if kdata.sock_id < 0:
            if er_count >= 5:
                print("cannot connect with PC")
            else:
                er_count = er_count + 1
                print("TCP_CONNECT error id=", kdata.sock_id, "error count=", er_count)
                goto.connect

    @allow_goto
    def pick_originl(self):
        self.base(None, "NULL")
        self.tool(None, "NULL")
        self.twait(3)
        self.lmove(kdata.original, "original")
        kdata.pick_original[48] = self.shift(kdata.original, 0, 0, 0)
        kdata.pick_original[47] = self.shift(kdata.original, -285, 0, 0)
        kdata.pick_original[46] = self.shift(kdata.original, -285 * 2, 0, 0)
        kdata.pick_original[45] = self.shift(kdata.original, -285 * 3, 0, 0)
        kdata.pick_original[44] = self.shift(kdata.original, 0, 265, 0)
        kdata.pick_original[43] = self.shift(kdata.original, -285, 265, 0)
        kdata.pick_original[42] = self.shift(kdata.original, -285 * 2, 265, 0)
        kdata.pick_original[41] = self.shift(kdata.original, -285 * 3, 265, 0)
        kdata.pick_original[40] = self.shift(kdata.original, 0, 265 * 2, 0)
        kdata.pick_original[39] = self.shift(kdata.original, -285, 265 * 2, 0)
        kdata.pick_original[38] = self.shift(kdata.original, -285 * 2, 265 * 2, 0)
        kdata.pick_original[37] = self.shift(kdata.original, -285 * 3, 265 * 2, 0)
        kdata.pick_original[36] = self.shift(kdata.original, 0, 265 * 3, 0)
        kdata.pick_original[35] = self.shift(kdata.original, -285, 265 * 3, 0)
        kdata.pick_original[34] = self.shift(kdata.original, -285 * 2, 265 * 3, 0)
        kdata.pick_original[33] = self.shift(kdata.original, -285 * 3, 265 * 3, 0)

    @allow_goto
    def pick_originl_au(self):
        kdata.original_new = kdata.aa[48]
        numbers = {
            1: self.pick_originl_au1(),
            2: self.pick_originl_au2(),
        }

        method = numbers.get(kdata.pallet_type)  #, self.other
        if method:
            method()

    @allow_goto
    def picka1(self):
        self.base(None, "NULL")
        self.tool(None, "NULL")
        if self.sig(1207) == 1:
            self.signal(6)
            self.signal(207)
            kdata.socket_connect = 0
            kdata.pallet_type = 0
            self.picka1_type_pos()
            self.pick_originl_au()
            label.x201
            self.qingling()
            kdata.vision_want_cou = 16
            label.x202
            kdata.vision_ng_count = 0
            self.signal(-1, 2)
            self.swait(-1001, 1002)
            self.base(None, "NULL")
            self.tool(None, "NULL")
            kdata.j = kdata.vision_want[1]
            kdata.aguodu[0] = kdata.pick_original[kdata.j][0]
            self.lmove(kdata.aguodu, "aguodu")
            self.break_()
            for kdata.s in range(kdata.vision_want_cou):
                kdata.vision_ok = -1
                kdata.picknum = kdata.vision_want[kdata.s]
                print("picknum", kdata.picknum)
                self.lmove(kdata.pick_original[kdata.picknum], self.str_format("pick_original[{}]", kdata.picknum))
                self.break_()
                self.twait(0.8)
                self.socket1()
                if kdata.vision_ok == 1:
                    kdata.pos[kdata.picknum, 1] = kdata.p - kdata.pos_recv[2]
                    kdata.pos[kdata.picknum, 2] = kdata.q - kdata.pos_recv[3]
                    print("偏移值", kdata.pos[kdata.picknum, 1], kdata.pos[kdata.picknum, 2])
                    kdata.aa[kdata.picknum] = self.shift(kdata.pick_original[kdata.picknum],
                                                         -kdata.pos[kdata.picknum, 1], -kdata.pos[kdata.picknum, 2], 0)
                else:
                    self.twait(0.8)
                    self.socket2()
                    if kdata.vision_ok == 1:
                        kdata.pos[kdata.picknum, 1] = kdata.p - kdata.pos_recv[2]
                        kdata.pos[kdata.picknum, 2] = kdata.q - kdata.pos_recv[3]
                        print("偏移值", kdata.pos[kdata.picknum, 1], kdata.pos[kdata.picknum, 2])
                        kdata.aa[kdata.picknum] = self.shift(kdata.pick_original[kdata.picknum],
                                                             -kdata.pos[kdata.picknum, 1], -kdata.pos[kdata.picknum, 2],
                                                             0)
                    else:
                        kdata.vision_ng_count = kdata.vision_ng_count + 1
                        kdata.vision_ng[kdata.vision_ng_count] = kdata.picknum
                        if kdata.vision_ng_count > 3:
                            self.printng()
                            print("检测失败数量大于3个，检测是否为相机未连接上")
                            self.pause()
                            goto.x201
            kdata.g = kdata.vision_want[kdata.vision_want_cou]
            kdata.aguodu[0] = kdata.pick_original[kdata.g][0]
            self.twait(0.2)
            self.lmove(kdata.aguodu, "aguodu")
            self.home(1)
            if kdata.vision_ng_count > 0:
                self.printng()
                self.pause()
                goto.x202
            else:
                print("检测数量16")
            self.jianju()
            if kdata.konglun_ng_coun > 0:
                self.pause()
                goto.x201
            else:
                print("间距ok")
            self.signal(-6)
            self.signal(-207)

    @allow_goto
    def picka1_type_pos(self):
        self.base(None, "NULL")
        self.tool(None, "NULL")
        label.x201
        self.qingling_type_p()
        kdata.vision_want_cou = 3
        label.x202
        kdata.pick_original[48] = self.shift(kdata.original_new, 0, 0, 0)
        kdata.pick_original[47] = self.shift(kdata.original_new, -265, 0, 0)
        kdata.pick_original[46] = self.shift(kdata.original_new, -265 * 2, 0, 0)
        kdata.vision_265 = 0
        kdata.vision_ng_count = 0
        self.signal(-1, 2)
        self.swait(-1001, 1002)
        kdata.j = kdata.vision_want[1]
        kdata.aguodu[0] = kdata.pick_original[kdata.j][0]
        self.lmove(kdata.aguodu, "aguodu")
        self.break_()
        for kdata.s in range(kdata.vision_want_cou):
            kdata.vision_ok = -1
            kdata.picknum = kdata.vision_want[kdata.s]
            print("picknum", kdata.picknum)
            label.vision2
            self.lmove(kdata.pick_original[kdata.picknum], self.str_format("pick_original[{}]", kdata.picknum))
            self.break_()
            self.twait(0.8)
            self.socket1()
            if kdata.vision_ok == 1:
                kdata.pos[kdata.picknum, 1] = kdata.p - kdata.pos_recv[2]
                kdata.pos[kdata.picknum, 2] = kdata.q - kdata.pos_recv[3]
                print("偏移值", kdata.pos[kdata.picknum, 1], kdata.pos[kdata.picknum, 2])
                kdata.aa[kdata.picknum] = self.shift(kdata.pick_original[kdata.picknum], -kdata.pos[kdata.picknum, 1],
                                                     -kdata.pos[kdata.picknum, 2], 0)
            else:
                self.twait(0.8)
                self.socket2()
                if kdata.vision_ok == 1:
                    kdata.pos[kdata.picknum, 1] = kdata.p - kdata.pos_recv[2]
                    kdata.pos[kdata.picknum, 2] = kdata.q - kdata.pos_recv[3]
                    print("偏移值", kdata.pos[kdata.picknum, 1], kdata.pos[kdata.picknum, 2])
                    kdata.aa[kdata.picknum] = self.shift(kdata.pick_original[kdata.picknum],
                                                         -kdata.pos[kdata.picknum, 1], -kdata.pos[kdata.picknum, 2], 0)
                else:
                    if kdata.vision_265 == 0 and kdata.picknum < 48 and kdata.picknum > 45:
                        kdata.pick_original[47] = self.shift(kdata.original_new, -285, 0, 0)
                        kdata.pick_original[46] = self.shift(kdata.original_new, -285 * 2, 0, 0)
                        kdata.vision_265 = 1
                        goto.vision2
                    else:
                        kdata.vision_ng_count = kdata.vision_ng_count + 1
                        kdata.vision_ng[kdata.vision_ng_count] = kdata.picknum
        kdata.g = kdata.vision_want[kdata.vision_want_cou]
        kdata.aguodu[0] = kdata.pick_original[kdata.g][0]
        self.twait(0.2)
        self.lmove(kdata.aguodu, "aguodu")
        self.home(1)
        if kdata.vision_ng_count > 0:
            self.printng()
            self.pause()
            goto.x202
        else:
            print("检测数量3")
        self.jianju_type_pos()
        if kdata.pallet_type == 0:
            print("垛盘48,47,46号轮未捅直")
            self.pause()
            goto.x201

    @allow_goto
    def picka2(self):
        self.base(None, "NULL")
        self.tool(None, "NULL")
        self.speed("100 ALWAYS")
        self.accuracy("100 ALWAYS")
        self.signal(1, -2)
        self.swait(1001, -1002)
        if kdata.picknum > 32 and kdata.picknum < 49:
            print("偏移值", kdata.pos[kdata.picknum, 1], kdata.pos[kdata.picknum, 2])
            kdata.aguodu[0] = kdata.aa[kdata.picknum][0]
            kdata.aguodu[3] = kdata.aa[kdata.picknum][3]
            kdata.aguodu[4] = kdata.aa[kdata.picknum][4]
            kdata.aguodu[5] = kdata.aa[kdata.picknum][5]
            kdata.a = kdata.aa[kdata.picknum]
            kdata.up_limit = 350
            kdata.down_limit = -1
            self.signal(-255, 236)
            self.accuracy("1")
            self.jappro(kdata.a + kdata.cam, "a + cam", kdata.up_limit)
            self.lmove(kdata.a + kdata.cam, "a + cam")
            self.speed("40")
            self.lappro(kdata.a + kdata.cam, "a + cam", kdata.down_limit)
            self.break_()
            self.signal(-1, 2)
            self.swait(-1001, 1002)
            self.twait(0.5)
            self.accuracy("50")
            self.lappro(kdata.a + kdata.cam, "a + cam", kdata.up_limit)
            self.signal(256)
            self.signal(205)
            self.home(1)
            return
        if kdata.picknum > 16 and kdata.picknum < 33:
            kdata.picknum5 = kdata.picknum + 16
            print("偏移值", kdata.pos[kdata.picknum5, 1], kdata.pos[kdata.picknum5, 2])
            kdata.aa[kdata.picknum] = self.shift(kdata.aa[kdata.picknum5], 0, 0, -210)
            kdata.aguodu[0] = kdata.aa[kdata.picknum][0]
            kdata.aguodu[3] = kdata.aa[kdata.picknum][3]
            kdata.aguodu[4] = kdata.aa[kdata.picknum][4]
            kdata.aguodu[5] = kdata.aa[kdata.picknum][5]
            kdata.a = kdata.aa[kdata.picknum]
            kdata.up_limit = 350
            kdata.down_limit = -2
            self.signal(-255, 236)
            self.accuracy("1")
            self.jappro(kdata.a + kdata.cam, "a + cam", kdata.up_limit)
            self.lmove(kdata.a + kdata.cam, "a + cam")
            self.speed("40")
            self.lappro(kdata.a + kdata.cam, "a + cam", kdata.down_limit)
            self.break_()
            self.signal(-1, 2)
            self.swait(-1001, 1002)
            self.twait(0.5)
            self.accuracy("50")
            self.lappro(kdata.a + kdata.cam, "a + cam", kdata.up_limit)
            self.signal(256)
            self.signal(205)
            self.home(1)
            return
        if kdata.picknum > 0 and kdata.picknum < 17:
            kdata.picknum6 = kdata.picknum + 32
            print("偏移值", kdata.pos[kdata.picknum6, 1], kdata.pos[kdata.picknum6, 2])
            kdata.aa[kdata.picknum] = self.shift(kdata.aa[kdata.picknum6], 0, 0, -420)
            kdata.pick_original[kdata.picknum] = self.shift(kdata.aa[kdata.picknum6], 0, 0, -420)
            kdata.up_limit = 350
            kdata.down_limit = -2
            self.vision_one()
            kdata.a = kdata.aa[kdata.picknum]
            kdata.pick[kdata.picknum] = kdata.a + kdata.cam
            kdata.aguodu[0] = kdata.pick[kdata.picknum][0]
            kdata.aguodu[3] = kdata.pick[kdata.picknum][3]
            kdata.aguodu[4] = kdata.pick[kdata.picknum][4]
            kdata.aguodu[5] = kdata.pick[kdata.picknum][5]
            self.break_()
            self.accuracy("1")
            self.lappro(kdata.pick[kdata.picknum], self.str_format("pick[{}]", kdata.picknum), kdata.up_limit)
            self.signal(1, -2)
            self.twait(0.5)
            self.swait(1001, -1002)
            self.signal(-255, 236)
            self.lmove(kdata.pick[kdata.picknum], self.str_format("pick[{}]", kdata.picknum))
            self.speed("40")
            self.lappro(kdata.pick[kdata.picknum], self.str_format("pick[{}]", kdata.picknum), kdata.down_limit)
            self.break_()
            self.signal(-1, 2)
            self.swait(-1001, 1002)
            self.twait(0.2)
            self.accuracy("50")
            self.lappro(kdata.pick[kdata.picknum], self.str_format("pick[{}]", kdata.picknum), kdata.up_limit)
            self.signal(256)
            self.signal(205)
            self.home(1)
            return

    @allow_goto
    def pickb(self):
        self.base(None, "NULL")
        self.tool(kdata.tooltet_m, "tooltet_m")
        self.lmove(kdata._pounce[kdata.jonum], self.str_format("#pounce[{}]", kdata.jonum))
        self.accuracy("10")
        self.lappro(kdata.dg[kdata.posnum * 10 + 1], self.str_format("dg[{}]", kdata.posnum * 10 + 1), 500)
        self.speed("60")
        self.accuracy("1")
        self.signal(-214)
        self.lmove(kdata.dg[kdata.posnum * 10 + 1], self.str_format("dg[{}]", kdata.posnum * 10 + 1))
        self.break_()
        self.signal(-256)
        self.signal(1, -2)
        self.twait(0.2)
        self.swait(1001, -1002)
        self.accuracy("30")
        self.lappro(kdata.dg[kdata.posnum * 10 + 1], self.str_format("dg[{}]", kdata.posnum * 10 + 1), 500)
        self.signal(255)
        self.lmove(kdata._pounce[kdata.jonum], self.str_format("#pounce[{}]", kdata.jonum))
        self.break_()
        self.home(1)
        kdata.posnum1 = kdata.posnum
        return

    @allow_goto
    def pickc1(self):
        self.base(None, "NULL")
        self.tool(kdata.tooltet_k, "tooltet_k")
        self.accuracy("100 ALWAYS")
        self.speed("100 ALWAYS")
        self.jmove(kdata._pounce[kdata.jonum], self.str_format("#pounce[{}]", kdata.jonum))
        self.swait(1200)
        self.signal(-1, 2, -3, 4)
        self.jmove(kdata._guodu[kdata.jonum], self.str_format("#guodu[{}]", kdata.jonum))
        self.swait(-1001, 1002, 1004, -1005)
        self.jmove(kdata._tx1[kdata.jonum], self.str_format("#tx1[{}]", kdata.jonum))
        self.accuracy("1 FINE")
        self.lmove(kdata.tx2[kdata.posnum], self.str_format("tx2[{}]", kdata.posnum))
        self.draw(",,100")
        self.accuracy("1 FINE")
        self.speed("15")
        self.lmove(kdata.tx6[kdata.jonum], self.str_format("tx6[{}]", kdata.jonum))
        self.break_()
        self.signal(195)
        self.swait(1220)
        self.twait(0.3)
        self.accuracy("1 FINE")
        self.lmove(kdata.tx8[kdata.jonum], self.str_format("tx8[{}]", kdata.jonum))
        self.signal(225)
        self.jmove(kdata.tx9[kdata.jonum], self.str_format("tx9[{}]", kdata.jonum))
        self.signal(1, -2, 214)
        self.accuracy("50")
        self.jmove(kdata._rx1[kdata.jonum], self.str_format("#rx1[{}]", kdata.jonum))
        self.swait(1001, -1002)
        self.accuracy("1 FINE")
        self.lappro(kdata._rx2[kdata.posnum], self.str_format("#rx2[{}]", kdata.posnum), 5)
        self.signal(-231)
        self.drive("6,-370,60")
        self.signal(-225)
        self.drive("6,-10,60")
        if kdata.jonum == 1:
            self.draw(",-40,,,-50,,60 MM/S")
            self.accuracy("1 FINE")
            self.draw(",-32,,,,,30")
            self.break_()
        else:
            self.draw(",40,,,45,,60 MM/S")
            self.accuracy("1 FINE")
            self.draw(",30,,,,,30")
            self.break_()
        self.signal(3, -4)
        self.swait(1005, -1004)
        self.twait(0.5)
        self.signal(221)
        self.swait(1215, 1222)
        self.accuracy("1 FINE")
        self.drive("6,25,50")
        self.here
        self.break_()
        self.accuracy("1 ALWAYS")
        self.tool(kdata.tooltet_t, "tooltet_t")
        self.tdraw("20,,,-45,-10,,100 MM/S")
        self.signal(222)
        if kdata.jonum == 1:
            self.accuracy("1 FINE")
            self.draw(",,60,,,,100 MM/S")
            self.draw("-198,,10,,,,100 MM/S")
            self.signal(238)
            self.break_()
        else:
            self.accuracy("1 FINE")
            self.draw(",,65,,,,100 MM/S")
            self.draw("198,,10,,,,100 MM/S")
            self.signal(238)
            self.break_()
        self.signal(-3, 4)
        self.accuracy("1 FINE")
        self.tdraw(",,-200")
        self.break_()
        self.swait(-1005, 1004)
        self.signal(-195, -225, -238, -229)

    @allow_goto
    def pickc2(self):
        self.base(None, "NULL")
        self.tool(kdata.tooltet_k, "tooltet_k")
        self.accuracy("100 ALWAYS")
        self.speed("100 ALWAYS")
        self.accuracy("10")
        self.jappro(kdata.fqq[kdata.posnum], self.str_format("fqq[{}]", kdata.posnum), 222)
        self.accuracy("1 FINE")
        self.lmove(kdata.fqq[kdata.posnum], self.str_format("fqq[{}]", kdata.posnum))
        self.twait(1.5)
        self.swait(1197)
        self.accuracy("10")
        self.jappro(kdata.fqq[kdata.posnum], self.str_format("fqq[{}]", kdata.posnum), 222)
        self.signal(1, -2, -222, -221)
        self.swait(1001, -1002)
        self.accuracy("1")
        self.jmove(kdata.qmlfront1[kdata.jonum], self.str_format("qmlfront1[{}]", kdata.jonum))
        self.signal(237)
        self.lmove(kdata.mldzx[kdata.posnum], self.str_format("mldzx[{}]", kdata.posnum))
        self.speed("50")
        self.accuracy("1 FINE")
        self.lmove(kdata.qml[kdata.posnum], self.str_format("qml[{}]", kdata.posnum))
        self.signal(-1, 2)
        self.twait(0.2)
        self.swait(-1001, 1002, 1003)
        self.lmove(kdata.mldzx[kdata.posnum], self.str_format("mldzx[{}]", kdata.posnum))
        self.accuracy("50")
        self.jmove(kdata.qmlfront4[kdata.posnum], self.str_format("qmlfront4[{}]", kdata.posnum))
        label.replay1
        if self.sig(1003):
            self.signal(-232)
            self.type("工字轮抓取_OK")
        else:
            self.type("未检测到工字轮,抓取_NG")
            self.type("工字轮可能掉落,请检查!!!")
            self.signal(232)
            self.pause()
            goto.replay1
        self.accuracy("100 ALWAYS")
        self.speed("70 ALWAYS")
        self.jmove(kdata._pounce[kdata.jonum], self.str_format("#pounce[{}]", kdata.jonum))
        self.signal(-214, -237)
        label.replay2
        if self.sig(1003):
            self.type("工字轮抓取_OK")
        else:
            self.type("未检测到工字轮,抓取_NG")
            self.type("工字轮可能掉落,请检查!!!")
            self.pause()
            goto.replay2

    @allow_goto
    def pickc3(self):
        self.base(None, "NULL")
        self.tool(kdata.tooltet_m, "tooltet_m")
        self.accuracy("100 ALWAYS")
        self.speed("100 ALWAYS")
        self.signal(32)
        self.rpalletnum()
        if kdata.palletnum == 0:
            self.rmlhnum()
            self.accuracy("10")
            self.lappro(kdata.dg[kdata.posnum * 10 + kdata.mlhnum],
                        self.str_format("dg[{}]", kdata.posnum * 10 + kdata.mlhnum), 650)
            self.speed("20")
            self.accuracy("1 FINE")
            self.lmove(kdata.dg[kdata.posnum * 10 + kdata.mlhnum],
                       self.str_format("dg[{}]", kdata.posnum * 10 + kdata.mlhnum))
            self.break_()
            self.signal(1, -2, 206)
            self.twait(0.3)
            self.swait(1001, -1002)
            self.accuracy("1")
            self.lappro(kdata.dg[kdata.posnum * 10 + kdata.mlhnum],
                        self.str_format("dg[{}]", kdata.posnum * 10 + kdata.mlhnum), 750)
            self.break_()
            self.signal(206)
            self.signal(-32)
        else:
            self.lmove(kdata._pounce[kdata.jonum], self.str_format("#pounce[{}]", kdata.jonum))
            self.jmove(kdata._buffer, "#buffer")
            self.pickf()
            self.jmove(kdata._buffer, "#buffer")
            self.lmove(kdata._pounce[kdata.jonum], self.str_format("#pounce[{}]", kdata.jonum))
            self.signal(-32)

    @allow_goto
    def pickc4(self):
        self.base(None, "NULL")
        self.tool(kdata.tooltet_k, "tooltet_k")
        self.accuracy("100 ALWAYS")
        self.speed("100 ALWAYS")
        self.accuracy("5")
        self.lappro(kdata.bs[kdata.posnum], self.str_format("bs[{}]", kdata.posnum), 650)
        self.signal(1, -2)
        self.swait(1001, -1002)
        self.speed("20")
        self.accuracy("1 FINE")
        self.lmove(kdata.bs[kdata.posnum], self.str_format("bs[{}]", kdata.posnum))
        self.break_()
        label.replay1
        if self.sig(1003):
            self.type("工字轮抓取_OK")
        else:
            self.type("未检测到工字轮,抓取_NG")
            self.type("工字轮可能掉落,请检查!!!")
            self.pause()
            goto.replay1
        self.signal(-1, 2)
        self.swait(-1001, 1002)
        self.speed("20")
        self.lappro(kdata.bs[kdata.posnum], self.str_format("bs[{}]", kdata.posnum), 100)
        self.accuracy("1")
        self.lappro(kdata.bs[kdata.posnum], self.str_format("bs[{}]", kdata.posnum), 650)
        self.accuracy("1")
        self.lmove(kdata._klguodu[kdata.jonum], self.str_format("#klguodu[{}]", kdata.jonum))
        self.base(None, "NULL")
        self.tool(kdata.tooltet_m, "tooltet_m")
        self.accuracy("1 FINE")
        self.jmove(kdata._fklfront1[kdata.jonum], self.str_format("#fklfront1[{}]", kdata.jonum))
        self.lmove(kdata.fklfront2[kdata.posnum], self.str_format("fklfront2[{}]", kdata.posnum))
        self.speed("7")
        self.accuracy("1 FINE")
        self.lmove(kdata.fkl[kdata.posnum], self.str_format("fkl[{}]", kdata.posnum))
        self.twait(0.5)
        self.signal(1, -2)
        self.swait(1001, -1002)
        self.accuracy("1")
        self.lmove(kdata.fklfront1[kdata.jonum], self.str_format("fklfront1[{}]", kdata.jonum))
        self.tool(kdata.tooltet_k, "tooltet_k")
        self.accuracy("1")
        self.lappro(kdata.dqg[kdata.posnum], self.str_format("dqg[{}]", kdata.posnum), 371)
        self.accuracy("1 FINE")
        self.lmove(kdata.dqg[kdata.posnum], self.str_format("dqg[{}]", kdata.posnum))
        self.twait(1.5)
        self.accuracy("1")
        self.lappro(kdata.dqg[kdata.posnum], self.str_format("dqg[{}]", kdata.posnum), 371)
        self.signal(199)
        self.lmove(kdata.ldrxfront[kdata.jonum], self.str_format("ldrxfront[{}]", kdata.jonum))
        self.break_()
        self.speed("50 ALWAYS")
        self.accuracy("1")
        self.lmove(kdata.ldtx1[kdata.jonum], self.str_format("ldtx1[{}]", kdata.jonum))
        self.accuracy("1 FINE")
        self.lmove(kdata.ldtx2[kdata.jonum], self.str_format("ldtx2[{}]", kdata.jonum))
        self.accuracy("1")
        self.lmove(kdata.ldtx3[kdata.jonum], self.str_format("ldtx3[{}]", kdata.jonum))
        self.accuracy("1 FINE")
        self.lmove(kdata.ldtx4[kdata.jonum], self.str_format("ldtx4[{}]", kdata.jonum))
        self.break_()
        self.signal(226)
        self.swait(1218)
        self.accuracy("1 FINE")
        self.lmove(kdata.ldtx5[kdata.jonum], self.str_format("ldtx5[{}]", kdata.jonum))
        self.accuracy("1")
        self.lmove(kdata.ldtx8[kdata.jonum], self.str_format("ldtx8[{}]", kdata.jonum))
        self.accuracy("5")
        self.lmove(kdata.ldrx1[kdata.jonum], self.str_format("ldrx1[{}]", kdata.jonum))
        self.accuracy("1 FINE")
        self.lmove(kdata.ldrx2[kdata.posnum], self.str_format("ldrx2[{}]", kdata.posnum))
        self.break_()
        self.drive("6,120,25")
        self.signal(227)
        self.swait(1226)
        self.signal(-227)
        self.speed("25")
        if kdata.jonum == 1:
            self.draw(",-92,,,120,,30 MM/S")
        else:
            self.draw(",90,,,-120,,30 MM/S")
        self.signal(227)
        self.swait(1226)
        self.signal(-227)
        self.speed("25")
        if kdata.jonum == 1:
            self.draw(",48,,,170,,10 MM/S")
        else:
            self.draw(",-65,,,-150,,10 MM/S")
        self.speed("2")
        if kdata.jonum == 1:
            self.accuracy("1 FINE")
            self.draw(",-25,,,,,2")
        else:
            self.accuracy("1 FINE")
            self.draw(",30,,,,,2")
        self.break_()
        self.signal(3, -4)
        self.swait(-1004, 1005)
        self.twait(0.5)
        self.signal(228)
        self.swait(1221)
        self.drive("6,-45,20")
        self.break_()
        self.here
        self.tool(kdata.tooltet_t, "tooltet_t")
        self.twait(0.5)
        self.accuracy("1")
        self.speed("75 MM/S")
        self.lmove(kdata.ldrxdj1[kdata.jonum], self.str_format("ldrxdj1[{}]", kdata.jonum))
        self.accuracy("1 FINE")
        self.lmove(kdata.ldrxdj2[kdata.jonum], self.str_format("ldrxdj2[{}]", kdata.jonum))
        self.accuracy("1")
        self.speed("100 MM/S")
        self.lmove(kdata.ldrxdj3[kdata.jonum], self.str_format("ldrxdj3[{}]", kdata.jonum))
        self.accuracy("1")
        self.lmove(kdata.ldrxdj4[kdata.jonum], self.str_format("ldrxdj4[{}]", kdata.jonum))
        self.accuracy("1 FINE")
        self.speed("50")
        self.tool(kdata.tooltet_k, "tooltet_k")
        #"sjy+sj2y[jobnum]"
        self.lmove(kdata.sjy + kdata.sjy2[kdata.jobnum], "sjy" + self.str_format("sjy2[{}]",kdata.jobnum))
        self.twait(0.5)
        self.signal(223)
        self.swait(1219)
        self.twait(0.5)
        self.signal(-3, 4)
        self.swait(1004, -1005)
        self.accuracy("1 FINE")
        self.tdraw(",,-400")
        self.break_()
        self.pulse(224, 8)
        self.speed("100 ALWAYS")
        self.base(None, "NULL")
        self.tool(None, "NULL")
        self.lmove(kdata._guodu[kdata.jonum], self.str_format("#guodu[{}]", kdata.jonum))
        self.jmove(kdata._pounce[kdata.jonum], self.str_format("#pounce[{}]", kdata.jonum))
        self.home(1)
        self.break_()
        self.twait(2)
        self.signal(-199, -226, -228, -223, -227)
        kdata.posnum = 0
        kdata.picknum = -1
        return
        kdata.picknum = -1

    @allow_goto
    def pickc4f(self):
        self.accuracy("100 ALWAYS")
        self.speed("100 ALWAYS")
        label.replay1
        if self.sig(1003):
            self.type("工字轮抓取_OK")
        else:
            self.type("未检测到工字轮,抓取_NG")
            self.type("工字轮可能掉落,请检查!!!")
            self.pause()
        self.accuracy("1")
        self.signal(214)
        self.lmove(kdata._pounce[kdata.jonum], self.str_format("#pounce[{}]", kdata.jonum))
        self.base(None, "NULL")
        self.tool(kdata.tooltet_m, "tooltet_m")
        self.accuracy("1")
        self.jmove(kdata._fklfront1[kdata.jonum], self.str_format("#fklfront1[{}]", kdata.jonum))
        self.jappro(kdata.fkl[kdata.posnum], self.str_format("fkl[{}]", kdata.posnum), 400)
        self.signal(-256)
        self.lmove(kdata.fklfront2[kdata.posnum], self.str_format("fklfront2[{}]", kdata.posnum))
        self.accuracy("1 FINE")
        self.lappro(kdata.fkl[kdata.posnum], self.str_format("fkl[{}]", kdata.posnum), 100)
        self.signal(1, -2, 214)
        self.swait(1001, -1002)
        self.speed("30")
        self.accuracy("1 FINE")
        self.lmove(kdata.fkl[kdata.posnum], self.str_format("fkl[{}]", kdata.posnum))
        self.lappro(kdata.fkl[kdata.posnum], self.str_format("fkl[{}]", kdata.posnum), -1.5)
        self.break_()
        self.twait(0.5)
        self.accuracy("5")
        self.lappro(kdata.fkl[kdata.posnum], self.str_format("fkl[{}]", kdata.posnum), 240)
        self.tool(kdata.tooltet_k, "tooltet_k")
        self.accuracy("50")
        self.lappro(kdata.dq[kdata.posnum], self.str_format("dq[{}]", kdata.posnum), 250)
        self.accuracy("1 FINE")
        self.lmove(kdata.dq[kdata.posnum], self.str_format("dq[{}]", kdata.posnum))
        self.twait(1.5)
        self.accuracy("1")
        self.lappro(kdata.dq[kdata.posnum], self.str_format("dq[{}]", kdata.posnum), 300)
        self.signal(199)
        self.lmove(kdata.jxfront[kdata.jonum], self.str_format("jxfront[{}]", kdata.jonum))
        self.break_()
        if kdata.jonum == 1:
            kdata.rxstart[kdata.posnum] = self.shift(kdata.ldrx2[kdata.posnum], 0, -70, 0)
        else:
            kdata.rxstart[kdata.posnum] = self.shift(kdata.ldrx2[kdata.posnum], 0, 70, 0)
        self.accuracy("1")
        self.lmove(kdata.jxup[kdata.jonum], self.str_format("jxup[{}]", kdata.jonum))
        self.accuracy("1")
        self.lmove(kdata.jx[kdata.jonum], self.str_format("jx[{}]", kdata.jonum))
        self.accuracy("1")
        self.lmove(kdata.jx1[kdata.jonum], self.str_format("jx1[{}]", kdata.jonum))
        self.break_()
        self.signal(3, -4)
        self.swait(-1004, 1005)
        self.signal(226)
        self.swait(1218)
        self.accuracy("1 FINE")
        self.lappro(kdata.jx1[kdata.jonum], self.str_format("jx1[{}]", kdata.jonum), 80)
        self.signal(1, -2)
        self.swait(1001, -1002)
        self.lmove(kdata.rxstart[kdata.posnum], self.str_format("rxstart[{}]", kdata.posnum))
        self.signal(243)
        self.accuracy("1 FINE")
        self.lappro(kdata.rxstart[kdata.posnum], self.str_format("rxstart[{}]", kdata.posnum), -65)
        self.break_()
        self.tool(kdata.tooltet_k, "tooltet_k")
        self.signal(227)
        if kdata.jonum == 1:
            self.draw(",-15,,,170,,100 MM/S")
        else:
            self.draw(",15,,,-160,,100 MM/S")
        self.break_()
        self.here
        if kdata.jonum == 1:
            kdata.rxztf[kdata.posnum] = self.shift(kdata.rxzt[kdata.posnum], 140, -77, 0)
            self.break_()
            self.draw(",-80,,,177,,150 MM/S")
            self.break_()
            self.signal(227, 242)
            self.draw("137,-15,,,,,150 MM/S")
        else:
            kdata.rxztf[kdata.posnum] = self.shift(kdata.rxzt[kdata.posnum], -140, 77, 0)
            self.break_()
            self.draw(",80,,,-167,,200 MM/S")
            self.break_()
            self.signal(227, 242)
            self.draw("-137,15,,,,,200 MM/S")
        self.break_()
        self.tool(kdata.tooltet_t, "tooltet_t")
        self.speed("3")
        self.accuracy("1")
        self.lmove(kdata.rxgd[kdata.jonum], self.str_format("rxgd[{}]", kdata.jonum))
        self.break_()
        self.tool(kdata.tooltet_k, "tooltet_k")
        self.speed("3")
        self.accuracy("10")
        self.lmove(kdata.rxztf[kdata.posnum], self.str_format("rxztf[{}]", kdata.posnum))
        self.signal(237)
        self.break_()
        self.accuracy("1")
        if kdata.jonum == 1:
            self.draw("-300,-20,,,,,150 MM/S")
        else:
            self.draw("290,20,,,,,150 MM/S")
        self.signal(-243)
        self.break_()
        self.signal(-227, 228, 223, -242)
        self.swait(1219)
        self.twait(1)
        self.pulse(224, 8)
        self.twait(1)
        self.draw(",,180,,,,150 MM/S")
        self.break_()
        self.signal(-3, 4)
        self.accuracy("10")
        self.ldepart(350)
        self.swait(1004, -1005)
        self.break_()
        self.signal(224, -214, -236, -237)
        self.base(None, "NULL")
        self.tool(None, "NULL")
        self.jmove(kdata._guodu[kdata.jonum], self.str_format("#guodu[{}]", kdata.jonum))
        self.jmove(kdata._pounce[kdata.jonum], self.str_format("#pounce[{}]", kdata.jonum))
        self.home(1)
        self.break_()
        self.twait(2)
        self.signal(-199, -226, -228, -223, -227)
        kdata.posnum = 0
        kdata.picknum = -1
        return
        kdata.picknum = -1

    @allow_goto
    def pickc4q(self):
        self.rpicknum()
        self.base(None, "NULL")
        self.tool(kdata.tooltet_m, "tooltet_m")
        self.weight("90,,,582")
        self.accuracy("100 ALWAYS")
        self.speed("100 ALWAYS")
        if kdata.picknum == 0:
            self.lappro(kdata.dg[kdata.posnum * 10 + 1], self.str_format("dg[{}]", kdata.posnum * 10 + 1), 650)
            self.signal(1, -2, -255, 236)
            self.swait(1001, -1002)
            self.speed("20")
            self.accuracy("1 FINE")
            self.lmove(kdata.dg[kdata.posnum * 10 + 1], self.str_format("dg[{}]", kdata.posnum * 10 + 1))
            self.break_()
            self.signal(-1, 2)
            self.twait(0.3)
            self.swait(-1001, 1002)
            label.replay1
            if self.sig(1003):
                self.type("工字轮抓取_OK")
            else:
                self.type("未检测到工字轮,抓取_NG")
                self.type("工字轮可能掉落,请检查!!!")
                self.pause()
                goto.replay1
            self.speed("20")
            self.lappro(kdata.dg[kdata.posnum * 10 + 1], self.str_format("dg[{}]", kdata.posnum * 10 + 1), 100)
            self.accuracy("1")
            self.lappro(kdata.dg[kdata.posnum * 10 + 1], self.str_format("dg[{}]", kdata.posnum * 10 + 1), 650)
        else:
            self.lmove(kdata._pounce[kdata.jonum], self.str_format("#pounce[{}]", kdata.jonum))
            self.jmove(kdata._buffer, "#buffer")
            self.signal(214)
            self.picka2()
            self.break_()

    @allow_goto
    def pickd(self):
        self.base(None, "NULL")
        self.tool(kdata.tooltet_m, "tooltet_m")
        self.speed("100 ALWAYS")
        self.accuracy("100 ALWAYS")
        self.accuracy("5")
        self.jmove(kdata._pounce[kdata.jonum], self.str_format("#pounce[{}]", kdata.jonum))
        self.accuracy("10")
        self.lappro(kdata.dg[kdata.posnum * 10 + kdata.mlhnum],
                    self.str_format("dg[{}]", kdata.posnum * 10 + kdata.mlhnum), 850)
        self.signal(-255, 214)
        self.speed("80")
        self.accuracy("10")
        self.lmove(kdata.dg[kdata.posnum * 10 + kdata.mlhnum],
                   self.str_format("dg[{}]", kdata.posnum * 10 + kdata.mlhnum))
        self.break_()
        self.signal(-1, 2)
        self.twait(0.1)
        self.swait(-1001, 1002, 1003)
        self.speed("20")
        self.accuracy("0")
        self.lappro(kdata.dg[kdata.posnum * 10 + kdata.mlhnum],
                    self.str_format("dg[{}]", kdata.posnum * 10 + kdata.mlhnum), 200)
        self.speed("100")
        self.accuracy("10")
        self.lappro(kdata.dg[kdata.posnum * 10 + kdata.mlhnum],
                    self.str_format("dg[{}]", kdata.posnum * 10 + kdata.mlhnum), 850)
        self.signal(256)
        self.lmove(kdata._pounce[kdata.jonum], self.str_format("#pounce[{}]", kdata.jonum))
        self.jmove(kdata._buffer, "#buffer")
        self.pickf()
        self.jmove(kdata._buffer, "#buffer")
        self.home(1)

    @allow_goto
    def pickf(self):
        self.base(None, "NULL")
        self.tool(None, "NULL")
        self.speed("100 ALWAYS")
        self.accuracy("100 ALWAYS")
        label.x160
        self.rpalletnum()
        if kdata.palletnum == 0:
            self.twait(0.5)
            kdata.in_robot_change[1033] = 0
            goto.x160
        kdata.in_robot_change[1033] = 0
        if kdata.palletnum > 32 and kdata.palletnum < 49:
            kdata.palletnum2 = kdata.palletnum - 32
            kdata.aa[kdata.palletnum] = self.shift(kdata.aa[kdata.palletnum2], 0, 0, 420)
            kdata.a = kdata.aa[kdata.palletnum]
            kdata.pallet[kdata.palletnum] = kdata.a + kdata.cam
            kdata.aguodu[0] = kdata.pallet[kdata.palletnum][0]
            kdata.aguodu[3] = kdata.pallet[kdata.palletnum][3]
            kdata.aguodu[4] = kdata.pallet[kdata.palletnum][4]
            kdata.aguodu[5] = kdata.pallet[kdata.palletnum][5]
            kdata.up_limit = 350
            kdata.down_limit = -5
            self.lmove(kdata.aguodu, "aguodu")
            self.accuracy("1")
            self.jappro(kdata.pallet[kdata.palletnum], self.str_format("pallet[{}]", kdata.palletnum), kdata.up_limit)
            self.signal(237)
            self.signal(-256)
            self.speed("100")
            self.accuracy("1")
            self.lmove(kdata.pallet[kdata.palletnum], self.str_format("pallet[{}]", kdata.palletnum))
            self.speed("40")
            self.lappro(kdata.pallet[kdata.palletnum], self.str_format("pallet[{}]", kdata.palletnum), kdata.down_limit)
            self.break_()
            self.signal(1, -2)
            self.twait(0.1)
            self.swait(1001, -1002)
            self.signal(206)
            self.speed("20")
            self.lmove(kdata.pallet[kdata.palletnum], self.str_format("pallet[{}]", kdata.palletnum))
            self.accuracy("1")
            self.lappro(kdata.pallet[kdata.palletnum], self.str_format("pallet[{}]", kdata.palletnum), kdata.up_limit)
            self.signal(-237, -214, 255)
            self.break_()
            self.accuracy("20")
            self.signal(209)
            self.twait(0.1)
            return
        if kdata.palletnum > 16 and kdata.palletnum < 33:
            kdata.palletnum3 = kdata.palletnum - 16
            kdata.aa[kdata.palletnum] = self.shift(kdata.aa[kdata.palletnum3], 0, 0, 210)
            kdata.a = kdata.aa[kdata.palletnum]
            kdata.pallet[kdata.palletnum] = kdata.a + kdata.cam
            kdata.aguodu[0] = kdata.aa[kdata.palletnum][0]
            kdata.aguodu[3] = kdata.aa[kdata.palletnum][3]
            kdata.aguodu[4] = kdata.aa[kdata.palletnum][4]
            kdata.aguodu[5] = kdata.aa[kdata.palletnum][5]
            kdata.up_limit = 350
            kdata.down_limit = -5
            self.accuracy("1")
            self.jappro(kdata.pallet[kdata.palletnum], self.str_format("pallet[{}]", kdata.palletnum), kdata.up_limit)
            self.signal(237)
            self.break_()
            self.signal(-256)
            self.speed("100")
            self.accuracy("1")
            self.lmove(kdata.pallet[kdata.palletnum], self.str_format("pallet[{}]", kdata.palletnum))
            self.speed("40")
            self.lappro(kdata.pallet[kdata.palletnum], self.str_format("pallet[{}]", kdata.palletnum), kdata.down_limit)
            self.break_()
            self.signal(1, -2)
            self.twait(0.1)
            self.swait(1001, -1002)
            self.signal(206)
            self.speed("40")
            self.lmove(kdata.pallet[kdata.palletnum], self.str_format("pallet[{}]", kdata.palletnum))
            self.accuracy("1")
            self.lappro(kdata.pallet[kdata.palletnum], self.str_format("pallet[{}]", kdata.palletnum), kdata.up_limit)
            self.break_()
            self.signal(-237, -214, 255)
            self.accuracy("20")
            self.signal(209)
            self.twait(0.1)
            return
        if kdata.palletnum > 0 and kdata.palletnum < 17:
            kdata.a = kdata.aa[kdata.palletnum]
            kdata.pallet[kdata.palletnum] = kdata.a + kdata.cam
            kdata.aguodu[0] = kdata.aa[kdata.palletnum][0]
            kdata.aguodu[3] = kdata.aa[kdata.palletnum][3]
            kdata.aguodu[4] = kdata.aa[kdata.palletnum][4]
            kdata.aguodu[5] = kdata.aa[kdata.palletnum][5]
            kdata.up_limit = 690
            kdata.down_limit = -5
            self.accuracy("1")
            self.jappro(kdata.pallet[kdata.palletnum], self.str_format("pallet[{}]", kdata.palletnum), kdata.up_limit)
            self.signal(237)
            self.signal(-256)
            self.speed("100")
            self.accuracy("1")
            self.lmove(kdata.pallet[kdata.palletnum], self.str_format("pallet[{}]", kdata.palletnum))
            self.speed("40")
            self.lappro(kdata.pallet[kdata.palletnum], self.str_format("pallet[{}]", kdata.palletnum), kdata.down_limit)
            self.break_()
            self.signal(1, -2)
            self.twait(0.5)
            self.swait(1001, -1002)
            self.signal(206)
            self.speed("20")
            self.lmove(kdata.pallet[kdata.palletnum], self.str_format("pallet[{}]", kdata.palletnum))
            self.accuracy("1")
            self.lappro(kdata.pallet[kdata.palletnum], self.str_format("pallet[{}]", kdata.palletnum), kdata.up_limit)
            self.break_()
            self.signal(-237, -214, 255)
            self.signal(209)
            self.twait(0.1)
            return
        return

    @allow_goto
    def printng(self):
        kdata.vision_want_cou = kdata.vision_ng_count
        for kdata.k in range(kdata.vision_ng_count):
            print("更换第", kdata.vision_ng[kdata.k], "号轮")
            self.twait(0.2)
            kdata.vision_want[kdata.k] = kdata.vision_ng[kdata.k]

    @allow_goto
    def qingling(self):
        for kdata.picknum in range(48):
            kdata.pos[kdata.picknum, 1] = 0
            kdata.pos[kdata.picknum, 2] = 0
        self.twait(0.3)
        kdata.picknum = -1
        kdata.posnum1 = 0
        kdata.vision_want[1] = 33
        kdata.vision_want[2] = 34
        kdata.vision_want[3] = 35
        kdata.vision_want[4] = 36
        kdata.vision_want[5] = 40
        kdata.vision_want[6] = 39
        kdata.vision_want[7] = 38
        kdata.vision_want[8] = 37
        kdata.vision_want[9] = 41
        kdata.vision_want[10] = 42
        kdata.vision_want[11] = 43
        kdata.vision_want[12] = 44
        kdata.vision_want[13] = 48
        kdata.vision_want[14] = 47
        kdata.vision_want[15] = 46
        kdata.vision_want[16] = 45

    @allow_goto
    def qingling_type_p(self):
        kdata.vision_want[1] = 48
        kdata.vision_want[2] = 47
        kdata.vision_want[3] = 46

    @allow_goto
    def rmlhnum(self):
        label.x628
        kdata.mlhnum = self.bits(1161, 8)
        if kdata.mlhnum <= 0 or kdata.mlhnum > 5:
            print("mlhnum", kdata.mlhnum)
            self.twait(0.5)
            kdata.in_robot_change[1161] = 0
            goto.x628
        else:
            print("mlhnum", kdata.mlhnum)
            kdata.in_robot_change[1161] = 1

    @allow_goto
    def rpalletnum(self):
        label.x732
        kdata.palletnum = self.bits(1033, 8)
        if kdata.palletnum < 0 or kdata.palletnum > 48:
            self.twait(0.5)
            kdata.in_robot_change[1033] = 0
            goto.x732
        else:
            print("palletnum", kdata.palletnum)
            kdata.in_robot_change[1033] = 1

    @allow_goto
    def rpicknum(self):
        label.x632
        kdata.picknum = self.bits(1185, 8)
        if kdata.picknum < 0 or kdata.picknum > 48:
            self.twait(0.5)
            kdata.in_robot_change[1185] = 0
            goto.x632
        else:
            print("picknum", kdata.picknum)
            kdata.in_robot_change[1185] = 1

    @allow_goto
    def rposnum(self):
        label.x588
        kdata.posnum = self.bits(1177, 8)
        if kdata.posnum <= 0 or kdata.posnum > 66:
            print("posnum", kdata.posnum)
            self.twait(0.5)
            kdata.in_robot_change[1177] = 0
            goto.x588
        else:
            print("posnum", kdata.posnum)
            kdata.in_robot_change[1177] = 1
        if kdata.posnum % 2 == 1:
            kdata.jonum = 1
        else:
            kdata.jonum = 2

    @allow_goto
    def socket1(self):
        kdata.recv_buf[1]=""
        self.stscus_tcp()
        self.com_init()
        self.open_socket()
        if kdata.sock_id < 0:
            pass
        kdata.text_id=0
        kdata.tout=60
        kdata.ret=0
        self.communicate1()
        label.recv_rt
        #self.tcp_recv(kdata.ret,kdata.sock_id,kdata.recv_buf[1],kdata.num,kdata.tout_rec,kdata.max_length)
        if kdata.ret < 0:
            if kdata.socket_connect < 3:
                self.cam_rlink_gv()
                print("相机重启ing")
                kdata.socket_connect=kdata.socket_connect + 1
                print("相机重启",kdata.socket_connect," 次")
            else:
                print("检测相机连接是否正常")
                print("重启视觉软件并运行")
                self.pause()
        else:
            if kdata.ret == 0:
                kdata.temp = self.decode(kdata.recv_buf[1], "#", 0)
                kdata.temp = self.decode(kdata.recv_buf[1], "#", 1)
                kdata.temp = self.decode(kdata.recv_buf[1], "@", 0)
                # self.tcp_send(kdata.sret,kdata.sock_id,kdata.recv_buf[1],1,kdata.tout)
                kdata.type = self.val(kdata.temp)
                if kdata.type == 117:
                    self.decode()
                    self.close_socket()
                else:
                    #####goto.recv_rt
                    pass

    @allow_goto
    def socket2(self):
        kdata.recv_buf[1]=""
        self.stscus_tcp()
        self.com_init()
        self.open_socket()
        if kdata.sock_id < 0:
            pass
        kdata.text_id=0
        kdata.tout=60
        kdata.ret=0
        self.communicate2()
        label.recv_rt
        #self.tcp_recv(kdata.ret,kdata.sock_id,kdata.recv_buf[1],kdata.num,kdata.tout_rec,kdata.max_length)
        if kdata.ret < 0:
            if kdata.socket_connect < 3:
                print("相机重启ing")
                self.cam_rlink_gv()
                kdata.socket_connect=kdata.socket_connect + 1
                print("相机重启",kdata.socket_connect," 次")
            else:
                print("检测相机连接是否正常")
                print("重启视觉软件并运行")
                self.pause()
        else:
            if kdata.ret == 0:
                kdata.temp = self.decode(kdata.recv_buf[1], "#", 0)
                kdata.temp = self.decode(kdata.recv_buf[1], "#", 1)
                kdata.temp = self.decode(kdata.recv_buf[1], "@", 0)
                # self.tcp_send(kdata.sret,kdata.sock_id,kdata.recv_buf[1],1,kdata.tout)
                kdata.type = self.val(kdata.temp)
                if kdata.type == 117:
                    self.decode()
                    self.close_socket()
                else:
                    ####goto.recv_rt
                    pass

    @allow_goto
    def socket3(self):
        kdata.recv_buf[1] = ""
        self.stscus_tcp()
        self.com_init()
        self.open_socket()
        if kdata.sock_id < 0:
            pass
        kdata.text_id = 0
        kdata.tout = 60
        kdata.ret = 0
        self.communicate3()
        self.twait(0.6)
        self.close_socket()

    @allow_goto
    def socket4(self):
        kdata.recv_buf[1] = ""
        self.stscus_tcp()
        self.com_init()
        self.open_socket()
        if kdata.sock_id < 0:
            pass
        kdata.text_id = 0
        kdata.tout = 60
        kdata.ret = 0
        self.communicate4()
        self.twait(0.6)
        self.close_socket()

    @allow_goto
    def socket5(self):
        kdata.recv_buf[1] = ""
        self.stscus_tcp()
        self.com_init()
        self.open_socket()
        if kdata.sock_id < 0:
            pass
        kdata.text_id = 0
        kdata.tout = 60
        kdata.ret = 0
        self.communicate5()
        self.twait(0.6)
        self.close_socket()

    @allow_goto
    def socket6(self):
        kdata.recv_buf[1] = ""
        self.stscus_tcp()
        self.com_init()
        self.open_socket()
        if kdata.sock_id < 0:
            pass
        kdata.text_id = 0
        kdata.tout = 60
        kdata.ret = 0
        self.communicate6()
        self.twait(0.6)
        self.close_socket()

    @allow_goto
    def speed50200(self):
        self.speed("100 ALWAYS")
        self.weight("90,0,0,334,0,0,0")
        self.accuracy("200 ALWAYS")
        kdata.tooltet_k = [0, 0, 371, 0, 0, 0]
        kdata.tooltet_m = [0, 0, 582, 0, 0, 0]
        kdata.tooltet_t = [110, -80, 455, 0, 0, 0]

    @allow_goto
    def stscus_tcp(self):
        tcp_cnt = 0
        #self.tcp_status(tcp_cnt,port1[0],sock1[0],err1[0],sub1[0],kdata.ip_add1[0])
        if tcp_cnt != 0:
            for kdata.i in range(tcp_cnt):
                kdata.j=kdata.i-1
                #print(port1[kdata.j])
                kdata.i=kdata.i + 1
            self.close_socket()

    @allow_goto
    def vision_one(self):
        self.base(None, "NULL")
        self.tool(None, "NULL")
        self.speed("100 ALWAYS")
        self.accuracy("100 ALWAYS")
        self.accuracy("1")
        kdata.socket_connect = 2
        kdata.vision_ok = -1
        self.signal(6)
        self.signal(-1, 2)
        self.twait(0.1)
        self.swait(-1001, 1002)
        self.lappro(kdata.pick_original[kdata.picknum], self.str_format("pick_original[{}]", kdata.picknum), 420)
        self.lmove(kdata.pick_original[kdata.picknum], self.str_format("pick_original[{}]", kdata.picknum))
        self.break_()
        self.twait(0.8)
        self.socket1()
        if kdata.vision_ok == 1:
            kdata.pos[kdata.picknum, 1] = kdata.p - kdata.pos_recv[2]
            kdata.pos[kdata.picknum, 2] = kdata.q - kdata.pos_recv[3]
            print("偏移值", kdata.pos[kdata.picknum, 1], kdata.pos[kdata.picknum, 2])
            kdata.aa[kdata.picknum] = self.shift(kdata.pick_original[kdata.picknum], -kdata.pos[kdata.picknum, 1],
                                                 -kdata.pos[kdata.picknum, 2], 0)
        else:
            self.twait(0.3)
            self.socket2()
            if kdata.vision_ok == 1:
                kdata.pos[kdata.picknum, 1] = kdata.p - kdata.pos_recv[2]
                kdata.pos[kdata.picknum, 2] = kdata.q - kdata.pos_recv[3]
                print("偏移值", kdata.pos[kdata.picknum, 1], kdata.pos[kdata.picknum, 2])
                kdata.aa[kdata.picknum] = self.shift(kdata.pick_original[kdata.picknum], -kdata.pos[kdata.picknum, 1],
                                                     -kdata.pos[kdata.picknum, 2], 0)
            else:
                print("第", kdata.picknum, "号轮未识别")
        self.signal(-6)

    @allow_goto
    def work2(self):
        kdata.in_robot_change[1041] = 1
        self.signal(-25, 26)
        self.bits_assign(kdata.jobnum, 41, 8)
        self.swait(kdata.jobnum_check)
        print("jobnum", kdata.jobnum)
        self.bits_assign(0, 41, 8)
        self.signal(-26)
        self.rposnum()
        self.pickc1()
        self.pickc2()
        self.pickc3()
        self.pickc4q()
        self.pickc4f()
        self.signal(27)
        self.swait(1025)
        self.signal(-209, -202, -27, -217, -218)

    @allow_goto
    def work3(self):
        kdata.in_robot_change[1041] = 1
        self.signal(-25, 26)
        self.bits_assign(kdata.jobnum, 41, 8)
        self.swait(kdata.jobnum_check)
        print("jobnum", kdata.jobnum)
        self.bits_assign(0, 41, 8)
        self.signal(-26)
        self.picka1()
        label.x757
        self.rpicknum()
        if kdata.picknum == 0:
            print("picknum", kdata.picknum)
            kdata.in_robot_change[1185] = 0
            goto.x757
        self.picka2()
        self.rposnum()
        if kdata.posnum == kdata.posnum1:
            print("检查第", kdata.posnum, "台空轮缓存位光电")
            self.pause()
        self.pickb()
        self.signal(-26, 27)
        self.swait(1025)
        self.signal(-27)

    @allow_goto
    def work4(self):
        kdata.in_robot_change[1041] = 1
        self.signal(-25, 26)
        self.bits_assign(kdata.jobnum, 41, 8)
        self.swait(kdata.jobnum_check)
        print("jobnum", kdata.jobnum)
        self.bits_assign(0, 41, 8)
        self.signal(-26)
        self.rposnum()
        self.rmlhnum()
        self.pickd()
        self.signal(-26, 27)
        self.swait(1025)
        self.signal(-209, -27)

    @allow_goto
    def work5(self):
        kdata.in_robot_change[1041] = 1
        self.signal(-25, 26)
        self.bits_assign(kdata.jobnum, 41, 8)
        self.swait(kdata.jobnum_check)
        print("jobnum", kdata.jobnum)
        self.bits_assign(0, 41, 8)
        self.signal(-26)
        self.picka1()
        self.signal(-26, 27)
        self.swait(1025)
        self.signal(-27)

    @allow_goto
    def work6(self):
        kdata.in_robot_change[1041] = 1
        self.signal(-25, 26)
        self.signal(26)
        self.bits_assign(kdata.jobnum, 41, 8)
        self.swait(kdata.jobnum_check)
        print("jobnum", kdata.jobnum)
        self.bits_assign(0, 41, 8)
        self.signal(-26)
        self.rposnum()
        self.pickdg_tech()
        self.home(1)
        self.break_()
        self.signal(-26, 27, 234)
        self.swait(1025)
        self.signal(-27)

