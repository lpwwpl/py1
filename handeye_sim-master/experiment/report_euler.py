# _*_ coding:utf-8 _*_
# @time: 2020/10/8 下午3:18
# @author: 张新新
# @email: 1262981714@qq.com
import os
import cv2
import transforms3d
import numpy as np
import matplotlib.pyplot as plt
from pylab import mpl

def getErrorList(fileList,euler_camera2end_gt,t_camera2end_gt,euler_obj2base_gt,t_obj2base_gt):
    euler_camera2end_error_mutlti = np.array([])
    t_camera2end_error_mutlti = np.array([])
    euler_obj2base_error_mutlti = np.array([])
    t_obj2base_error_mutlti = np.array([])
    for file in fileList:
        fs = cv2.FileStorage(file, cv2.FileStorage_READ)
        euler_camera2end_error = np.array([])
        t_camera2end_error = np.array([])
        euler_obj2base_error = np.array([])
        t_obj2base_error = np.array([])
        for i in range(25):
            Hcamera2end_temp = fs.getNode("Hcamera2end{0}".format(i)).mat()
            euler_camera2end_temp = transforms3d.euler.mat2euler(Hcamera2end_temp[:3, :3])
            t_camera2end_temp = Hcamera2end_temp[:3, 3]
            euler_camera2end_error_rx = min(abs(euler_camera2end_temp[0]-euler_camera2end_gt[0]),
                                            abs(euler_camera2end_temp[0]-euler_camera2end_gt[0]+2*3.1415926),
                                            abs(euler_camera2end_temp[0] - euler_camera2end_gt[0] - 2 * 3.1415926))
            euler_camera2end_error_ry = min(abs(euler_camera2end_temp[1] - euler_camera2end_gt[1]),
                                            abs(euler_camera2end_temp[1] - euler_camera2end_gt[1] + 2 * 3.1415926),
                                            abs(euler_camera2end_temp[1] - euler_camera2end_gt[1] - 2 * 3.1415926))
            euler_camera2end_error_rz = min(abs(euler_camera2end_temp[2] - euler_camera2end_gt[2]),
                                            abs(euler_camera2end_temp[2] - euler_camera2end_gt[2] + 2 * 3.1415926),
                                            abs(euler_camera2end_temp[2] - euler_camera2end_gt[2] - 2 * 3.1415926))
            euler_camera2end_error = np.append(euler_camera2end_error,np.array([euler_camera2end_error_rx,euler_camera2end_error_ry,euler_camera2end_error_rz]))
            t_camera2end_error = np.append(t_camera2end_error, np.abs(t_camera2end_temp - t_camera2end_gt))

            Hobj2base_temp = fs.getNode("Hobj2base{0}".format(i)).mat()
            euler_obj2base_temp = transforms3d.euler.mat2euler(Hobj2base_temp[:3, :3])
            # if np.linalg.norm(euler_obj2base_temp - euler_obj2base_gt) > np.linalg.norm(euler_obj2base_temp + euler_obj2base_gt):
            #     euler_obj2base_temp = -euler_obj2base_temp
            t_obj2base_temp = Hobj2base_temp[:3, 3]
            #euler_obj2base_error = np.append(euler_obj2base_error, np.abs(euler_obj2base_temp - euler_obj2base_gt))
            euler_obj2base_error_rx = min(abs(euler_obj2base_temp[0]-euler_obj2base_gt[0]),
                                          abs(euler_obj2base_temp[0]-euler_obj2base_gt[0]+2*3.1415926),
                                          abs(euler_obj2base_temp[0] - euler_obj2base_gt[0] - 2 * 3.1415926))
            euler_obj2base_error_ry = min(abs(euler_obj2base_temp[1] - euler_obj2base_gt[1]),
                                          abs(euler_obj2base_temp[1] - euler_obj2base_gt[1] + 2 * 3.1415926),
                                          abs(euler_obj2base_temp[1] - euler_obj2base_gt[1] - 2 * 3.1415926))
            euler_obj2base_error_rz = min(abs(euler_obj2base_temp[2] - euler_obj2base_gt[2]),
                                          abs(euler_obj2base_temp[2] - euler_obj2base_gt[2] + 2 * 3.1415926),
                                          abs(euler_obj2base_temp[2] - euler_obj2base_gt[2] - 2 * 3.1415926))
            euler_obj2base_error = np.append(euler_obj2base_error, np.array([euler_obj2base_error_rx,euler_obj2base_error_ry,euler_obj2base_error_rz]))
            t_obj2base_error = np.append(t_obj2base_error, np.abs(t_obj2base_temp - t_obj2base_gt))
        euler_camera2end_error = euler_camera2end_error.reshape([-1, 3])
        t_camera2end_error = t_camera2end_error.reshape([-1, 3])
        euler_obj2base_error = euler_obj2base_error.reshape([-1, 3])
        t_obj2base_error = t_obj2base_error.reshape([-1, 3])
        euler_camera2end_error_mutlti = np.append(euler_camera2end_error_mutlti, euler_camera2end_error)
        t_camera2end_error_mutlti = np.append(t_camera2end_error_mutlti, t_camera2end_error)
        euler_obj2base_error_mutlti = np.append(euler_obj2base_error_mutlti, euler_obj2base_error)
        t_obj2base_error_mutlti = np.append(t_obj2base_error_mutlti, t_obj2base_error)
    euler_camera2end_error_mutlti = euler_camera2end_error_mutlti.reshape([-1, 25, 3])
    t_camera2end_error_mutlti = t_camera2end_error_mutlti.reshape([-1, 25, 3])
    euler_obj2base_error_mutlti = euler_obj2base_error_mutlti.reshape([-1, 25, 3])
    t_obj2base_error_mutlti = t_obj2base_error_mutlti.reshape([-1, 25, 3])
    euler_camera2end_error_mean = np.mean(euler_camera2end_error_mutlti, axis=0)
    t_camera2end_error_mean = np.mean(t_camera2end_error_mutlti, axis=0)
    euler_obj2base_error_mean = np.mean(euler_obj2base_error_mutlti, axis=0)
    t_obj2base_error_mean = np.mean(t_obj2base_error_mutlti, axis=0)
    return euler_camera2end_error_mean,t_camera2end_error_mean,euler_obj2base_error_mean,t_obj2base_error_mean

rootdir  = "../result"
filelist = os.listdir(rootdir)
nolocal_std_list = []
random_list = []

for file in filelist:
    if file.startswith("noLocal_std"):
        nolocal_std_list.append(os.path.join(rootdir,file))
    if file.startswith("random"):
        random_list.append(os.path.join(rootdir,file))


fs = cv2.FileStorage("../config/handineye_gt.yml", cv2.FileStorage_READ)
Hcamera2end_gt = fs.getNode("Hcamera2end").mat()
Hobj2base_gt = fs.getNode("Hobj2base").mat()
fs.release()
euler_camera2end_gt = transforms3d.euler.mat2euler(Hcamera2end_gt[:3, :3])
t_camera2end_gt = Hcamera2end_gt[:3, 3]
euler_obj2base_gt = transforms3d.euler.mat2euler(Hobj2base_gt[:3, :3])
t_obj2base_gt = Hobj2base_gt[:3, 3]
nolocal_std_list.sort()
no_local_std_euler_camera2end_error,no_local_std_t_camera2end_error,no_local_std_euler_obj2base_error,no_local_std_t_obj2base_error = \
    getErrorList(nolocal_std_list[0:len(nolocal_std_list)-1],euler_camera2end_gt,t_camera2end_gt,euler_obj2base_gt,t_obj2base_gt)
random_euler_camera2end_error,random_t_camera2end_error,random_euler_obj2base_error,random_t_obj2base_error = \
    getErrorList(random_list,euler_camera2end_gt,t_camera2end_gt,euler_obj2base_gt,t_obj2base_gt)

no_local_std_rx_error = no_local_std_euler_camera2end_error[:,0]
random_rx_error = random_euler_camera2end_error[:,0]
no_local_std_ry_error = no_local_std_euler_camera2end_error[:,1]
random_ry_error = random_euler_camera2end_error[:,1]
no_local_std_rz_error = no_local_std_euler_camera2end_error[:,2]
random_rz_error = random_euler_camera2end_error[:,2]
# no_local_std_qw_error = no_local_std_euler_camera2end_error[:,3]
# random_qw_error = random_euler_camera2end_error[:,3]

no_local_std_tx_error = no_local_std_t_camera2end_error[:,0]
random_tx_error = random_t_camera2end_error[:,0]
no_local_std_ty_error = no_local_std_t_camera2end_error[:,1]
random_ty_error = random_t_camera2end_error[:,1]
no_local_std_tz_error = no_local_std_t_camera2end_error[:,2]
random_tz_error = random_t_camera2end_error[:,2]

#mpl.rcParams['font.sans-serif'] = ['Times New Roman'] # 指定默认字体
mpl.rcParams['axes.unicode_minus'] = False # 解决保存图像是负号'-'显示为方块的问题

x_range = np.arange(5,30,1)
plt.rcParams['figure.figsize'] = (6.0, 4.0)
plt.rcParams['image.interpolation'] = 'nearest' # 设置 interpolation style
#plt.rcParams['image.cmap'] = 'gray' # 设置 颜色 style
plt.rcParams['savefig.dpi'] = 2080#图片像素
plt.rcParams['figure.dpi'] = 300 #分辨率
plt.title("camera2end error")

fig,ax = plt.subplots(2,3,sharex='col',sharey='row')
#ax[0,0].figsize=(56,56)
ins0=ax[0,0].plot(x_range,no_local_std_rx_error, label = 'no_local_std',)
ins1=ax[0,0].plot(x_range,random_rx_error, label = 'random')
# lns = ins0+ins1
# labs = [l.get_label() for l in lns]
# ax[0,0].legend(lns, labs, loc="upper right")
# #ax[0,0].set_xlabel("number")
ax[0,0].set_ylabel("Error")
ax[0,0].set_title("qx")
# ax[0,0].set_xticks([5,10,15,20,25])
#ax[0,0].set_yticks([0.001,0.002,0.003,0.004])

#ax[0,1].figsize=(128,128)
ins0=ax[0,1].plot(x_range,no_local_std_ry_error, label = 'no_local_std')
ins1=ax[0,1].plot(x_range,random_ry_error, label = 'random')
# lns = ins0+ins1
# labs = [l.get_label() for l in lns]
# ax[0,1].legend(lns, labs, loc="upper right")
# ax[0,1].set_xlabel("number")
# ax[0,1].set_ylabel("Error")
ax[0,1].set_title("qy")
# ax[0,1].set_xticks([5,10,15,20,25])
# ax[0,1].set_yticks([0.001,0.002,0.003,0.004])

#ax[1,0].figsize=(128,128)
ins0=ax[0,2].plot(x_range,no_local_std_rz_error, label = 'no_local_std')
ins1=ax[0,2].plot(x_range,random_rz_error, label = 'random')
# lns = ins0+ins1
# labs = [l.get_label() for l in lns]
# ax[0,2].legend(lns, labs, loc="upper right")
# ax[0,2].set_xlabel("number")
# ax[0,2].set_ylabel("Error")
ax[0,2].set_title("qz")


ins0=ax[1,0].plot(x_range,no_local_std_tx_error, label = 'no_local_std')
ins1=ax[1,0].plot(x_range,random_tx_error, label = 'random')
# lns = ins0+ins1
# labs = [l.get_label() for l in lns]
# ax[1,0].legend(lns, labs, loc="upper right")
# ax[1,0].set_xlabel("number")
ax[1,0].set_ylabel("Error")
ax[1,0].set_title("tx")
ax[1,0].set_xticks([5,10,15,20,25])
# ax[1,0].set_yticks([0.001,0.002,0.003,0.004])

ins0=ax[1,1].plot(x_range,no_local_std_ty_error, label = 'no_local_std')
ins1=ax[1,1].plot(x_range,random_ty_error, label = 'random')
# lns = ins0+ins1
# labs = [l.get_label() for l in lns]
# ax[1,1].legend(lns, labs, loc="upper right")
# ax[1,1].set_xlabel("number")
# ax[1,1].set_ylabel("Error")
ax[1,1].set_title("ty")
ax[1,1].set_xticks([5,10,15,20,25])
# ax[1,1].set_yticks([0.001,0.002,0.003,0.004])

ins0=ax[1,2].plot(x_range,no_local_std_tz_error, label = 'no_local_std')
ins1=ax[1,2].plot(x_range,random_tz_error, label = 'random')
# lns = ins0+ins1
# labs = [l.get_label() for l in lns]
# ax[1,2].legend(lns, labs, loc="upper right")
# ax[1,2].set_xlabel("number")
# ax[1,2].set_ylabel("Error")
ax[1,2].set_title("tz")
ax[1,2].set_xticks([5,10,15,20,25])
# ax[1,2].set_yticks([0.001,0.002,0.003,0.004])

# lastSubplot = plt.subplot(subcol,subrow,subcol*subrow)
# ax[1,3].set_frame_on(False)
# ax[1,3].get_xaxis().set_visible(False)
# ax[1,3].get_yaxis().set_visible(False)
# plt.plot([], [], label = 'no_local_std')
# plt.plot([], [], label = 'random')
# lns = ins0+ins1
# labs = [l.get_label() for l in lns]
# ax[1,3].legend(lns, labs, loc="upper right")
# ax[1,3].legend(loc = 'lower right')
plt.savefig("euler_camera2end_error_10_10.png")
plt.show()
no_local_std_rx_error = no_local_std_euler_obj2base_error[:,0]
random_rx_error = random_euler_obj2base_error[:,0]
no_local_std_ry_error = no_local_std_euler_obj2base_error[:,1]
random_ry_error = random_euler_obj2base_error[:,1]
no_local_std_rz_error = no_local_std_euler_obj2base_error[:,2]
random_rz_error = random_euler_obj2base_error[:,2]
# no_local_std_qw_error = no_local_std_euler_camera2end_error[:,3]
# random_qw_error = random_euler_camera2end_error[:,3]

no_local_std_tx_error = no_local_std_t_obj2base_error[:,0]
random_tx_error = random_t_obj2base_error[:,0]
no_local_std_ty_error = no_local_std_t_obj2base_error[:,1]
random_ty_error = random_t_obj2base_error[:,1]
no_local_std_tz_error = no_local_std_t_obj2base_error[:,2]
random_tz_error = random_t_obj2base_error[:,2]

#mpl.rcParams['font.sans-serif'] = ['Times New Roman'] # 指定默认字体
mpl.rcParams['axes.unicode_minus'] = False # 解决保存图像是负号'-'显示为方块的问题

x_range = np.arange(5,30,1)
plt.rcParams['figure.figsize'] = (6.0, 4.0)
# plt.rcParams['image.interpolation'] = 'nearest' # 设置 interpolation style
# plt.rcParams['image.cmap'] = 'gray' # 设置 颜色 style
plt.rcParams['savefig.dpi'] = 2080#图片像素
plt.rcParams['figure.dpi'] = 300 #分辨率
plt.title(" obj2base error")


fig,ax = plt.subplots(2,3,sharex='col',sharey='row')
#ax[0,0].figsize=(56,56)
ins0=ax[0,0].plot(x_range,no_local_std_rx_error, label = 'no_local_std',)
ins1=ax[0,0].plot(x_range,random_rx_error, label = 'random')
# lns = ins0+ins1
# labs = [l.get_label() for l in lns]
# ax[0,0].legend(lns, labs, loc="upper right")
# #ax[0,0].set_xlabel("number")
ax[0,0].set_ylabel("Error")
ax[0,0].set_title("qx")
# ax[0,0].set_xticks([5,10,15,20,25])
#ax[0,0].set_yticks([0.001,0.002,0.003,0.004])

#ax[0,1].figsize=(128,128)
ins0=ax[0,1].plot(x_range,no_local_std_ry_error, label = 'no_local_std')
ins1=ax[0,1].plot(x_range,random_ry_error, label = 'random')
# lns = ins0+ins1
# labs = [l.get_label() for l in lns]
# ax[0,1].legend(lns, labs, loc="upper right")
# ax[0,1].set_xlabel("number")
# ax[0,1].set_ylabel("Error")
ax[0,1].set_title("qy")
# ax[0,1].set_xticks([5,10,15,20,25])
# ax[0,1].set_yticks([0.001,0.002,0.003,0.004])

#ax[1,0].figsize=(128,128)
ins0=ax[0,2].plot(x_range,no_local_std_rz_error, label = 'no_local_std')
ins1=ax[0,2].plot(x_range,random_rz_error, label = 'random')
# lns = ins0+ins1
# labs = [l.get_label() for l in lns]
# ax[0,2].legend(lns, labs, loc="upper right")
# ax[0,2].set_xlabel("number")
# ax[0,2].set_ylabel("Error")
ax[0,2].set_title("qz")
# ax[0,2].set_xticks([5,10,15,20,25])
# ax[0,2].set_yticks([0.001,0.002,0.003,0.004])

#ax[1,1].figsize=(128,128)
# ins0=ax[0,3].plot(x_range,no_local_std_qw_error, label = 'no_local_std')
# ins1=ax[0,3].plot(x_range,random_qw_error, label = 'random')
# lns = ins0+ins1
# labs = [l.get_label() for l in lns]
# ax[0,3].legend(lns, labs, loc="upper right")
# ax[0,3].set_xlabel("number")
# ax[0,3].set_ylabel("Error")
# ax[0,3].set_title("qw")
# ax[0,3].set_xticks([5,10,15,20,25])
# ax[0,3].set_yticks([0.001,0.002,0.003,0.004])

ins0=ax[1,0].plot(x_range,no_local_std_tx_error, label = 'no_local_std')
ins1=ax[1,0].plot(x_range,random_tx_error, label = 'random')
# lns = ins0+ins1
# labs = [l.get_label() for l in lns]
# ax[1,0].legend(lns, labs, loc="upper right")
# ax[1,0].set_xlabel("number")
ax[1,0].set_ylabel("Error")
ax[1,0].set_title("tx")
ax[1,0].set_xticks([5,10,15,20,25])
# ax[1,0].set_yticks([0.001,0.002,0.003,0.004])

ins0=ax[1,1].plot(x_range,no_local_std_ty_error, label = 'no_local_std')
ins1=ax[1,1].plot(x_range,random_ty_error, label = 'random')

ax[1,1].set_title("ty")
ax[1,1].set_xticks([5,10,15,20,25])

ins0=ax[1,2].plot(x_range,no_local_std_tz_error, label = 'no_local_std')
ins1=ax[1,2].plot(x_range,random_tz_error, label = 'random')

ax[1,2].set_title("tz")
ax[1,2].set_xticks([5,10,15,20,25])

plt.savefig("euler_obj2base_error_10_10.png")
plt.show()
print(0)






