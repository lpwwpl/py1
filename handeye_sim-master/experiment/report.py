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

def getErrorList(fileList,q_camera2end_gt,t_camera2end_gt,q_obj2base_gt,t_obj2base_gt):
    q_camera2end_error_mutlti = np.array([])
    t_camera2end_error_mutlti = np.array([])
    q_obj2base_error_mutlti = np.array([])
    t_obj2base_error_mutlti = np.array([])
    for file in fileList:
        fs = cv2.FileStorage(file, cv2.FileStorage_READ)
        q_camera2end_error = np.array([])
        t_camera2end_error = np.array([])
        q_obj2base_error = np.array([])
        t_obj2base_error = np.array([])
        for i in range(25):
            Hcamera2end_temp = fs.getNode("Hcamera2end{0}".format(i)).mat()
            q_camera2end_temp = transforms3d.quaternions.mat2quat(Hcamera2end_temp[:3, :3])
            if np.linalg.norm(q_camera2end_temp - q_camera2end_gt) > np.linalg.norm(
                    q_camera2end_temp + q_camera2end_gt):
                q_camera2end_temp = -q_camera2end_temp
            t_camera2end_temp = Hcamera2end_temp[:3, 3]
            q_camera2end_error = np.append(q_camera2end_error, np.abs(q_camera2end_temp - q_camera2end_gt))
            t_camera2end_error = np.append(t_camera2end_error, np.abs(t_camera2end_temp - t_camera2end_gt))

            Hobj2base_temp = fs.getNode("Hobj2base{0}".format(i)).mat()
            q_obj2base_temp = transforms3d.quaternions.mat2quat(Hobj2base_temp[:3, :3])
            if np.linalg.norm(q_obj2base_temp - q_obj2base_gt) > np.linalg.norm(q_obj2base_temp + q_obj2base_gt):
                q_obj2base_temp = -q_obj2base_temp
            t_obj2base_temp = Hobj2base_temp[:3, 3]
            q_obj2base_error = np.append(q_obj2base_error, np.abs(q_obj2base_temp - q_obj2base_gt))
            t_obj2base_error = np.append(t_obj2base_error, np.abs(t_obj2base_temp - t_obj2base_gt))
        q_camera2end_error = q_camera2end_error.reshape([-1, 4])
        t_camera2end_error = t_camera2end_error.reshape([-1, 3])
        q_obj2base_error = q_obj2base_error.reshape([-1, 4])
        t_obj2base_error = t_obj2base_error.reshape([-1, 3])
        q_camera2end_error_mutlti = np.append(q_camera2end_error_mutlti, q_camera2end_error)
        t_camera2end_error_mutlti = np.append(t_camera2end_error_mutlti, t_camera2end_error)
        q_obj2base_error_mutlti = np.append(q_obj2base_error_mutlti, q_obj2base_error)
        t_obj2base_error_mutlti = np.append(t_obj2base_error_mutlti, t_obj2base_error)
    q_camera2end_error_mutlti = q_camera2end_error_mutlti.reshape([-1, 25, 4])
    t_camera2end_error_mutlti = t_camera2end_error_mutlti.reshape([-1, 25, 3])
    q_obj2base_error_mutlti = q_obj2base_error_mutlti.reshape([-1, 25, 4])
    t_obj2base_error_mutlti = t_obj2base_error_mutlti.reshape([-1, 25, 3])
    q_camera2end_error_mean = np.mean(q_camera2end_error_mutlti, axis=0)
    t_camera2end_error_mean = np.mean(t_camera2end_error_mutlti, axis=0)
    q_obj2base_error_mean = np.mean(q_obj2base_error_mutlti, axis=0)
    t_obj2base_error_mean = np.mean(t_obj2base_error_mutlti, axis=0)
    return q_camera2end_error_mean,t_camera2end_error_mean,q_obj2base_error_mean,t_obj2base_error_mean

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
q_camera2end_gt = transforms3d.quaternions.mat2quat(Hcamera2end_gt[:3, :3])
t_camera2end_gt = Hcamera2end_gt[:3, 3]
q_obj2base_gt = transforms3d.quaternions.mat2quat(Hobj2base_gt[:3, :3])
t_obj2base_gt = Hobj2base_gt[:3, 3]
nolocal_std_list.sort()
no_local_std_q_camera2end_error,no_local_std_t_camera2end_error,no_local_std_q_obj2base_error,no_local_std_t_obj2base_error = \
    getErrorList(nolocal_std_list[0:len(nolocal_std_list)-1],q_camera2end_gt,t_camera2end_gt,q_obj2base_gt,t_obj2base_gt)
random_q_camera2end_error,random_t_camera2end_error,random_q_obj2base_error,random_t_obj2base_error = \
    getErrorList(random_list,q_camera2end_gt,t_camera2end_gt,q_obj2base_gt,t_obj2base_gt)

no_local_std_qx_error = no_local_std_q_camera2end_error[:,0]
random_qx_error = random_q_camera2end_error[:,0]
no_local_std_qy_error = no_local_std_q_camera2end_error[:,1]
random_qy_error = random_q_camera2end_error[:,1]
no_local_std_qz_error = no_local_std_q_camera2end_error[:,2]
random_qz_error = random_q_camera2end_error[:,2]
no_local_std_qw_error = no_local_std_q_camera2end_error[:,3]
random_qw_error = random_q_camera2end_error[:,3]

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
# plt.rcParams['image.interpolation'] = 'nearest' # 设置 interpolation style
# plt.rcParams['image.cmap'] = 'gray' # 设置 颜色 style
plt.rcParams['savefig.dpi'] = 2080#图片像素
plt.rcParams['figure.dpi'] = 300 #分辨率


fig,ax = plt.subplots(2,4,sharex='col',sharey='row')
#ax[0,0].figsize=(56,56)
ins0=ax[0,0].plot(x_range,no_local_std_qx_error, label = 'no_local_std',)
ins1=ax[0,0].plot(x_range,random_qx_error, label = 'random')
# lns = ins0+ins1
# labs = [l.get_label() for l in lns]
# ax[0,0].legend(lns, labs, loc="upper right")
# #ax[0,0].set_xlabel("number")
ax[0,0].set_ylabel("Error")
ax[0,0].set_title("qx")
# ax[0,0].set_xticks([5,10,15,20,25])
ax[0,0].set_yticks([0.001,0.002,0.003,0.004])

#ax[0,1].figsize=(128,128)
ins0=ax[0,1].plot(x_range,no_local_std_qy_error, label = 'no_local_std')
ins1=ax[0,1].plot(x_range,random_qy_error, label = 'random')
# lns = ins0+ins1
# labs = [l.get_label() for l in lns]
# ax[0,1].legend(lns, labs, loc="upper right")
# ax[0,1].set_xlabel("number")
# ax[0,1].set_ylabel("Error")
ax[0,1].set_title("qy")
# ax[0,1].set_xticks([5,10,15,20,25])
# ax[0,1].set_yticks([0.001,0.002,0.003,0.004])

#ax[1,0].figsize=(128,128)
ins0=ax[0,2].plot(x_range,no_local_std_qz_error, label = 'no_local_std')
ins1=ax[0,2].plot(x_range,random_qz_error, label = 'random')
# lns = ins0+ins1
# labs = [l.get_label() for l in lns]
# ax[0,2].legend(lns, labs, loc="upper right")
# ax[0,2].set_xlabel("number")
# ax[0,2].set_ylabel("Error")
ax[0,2].set_title("qz")
# ax[0,2].set_xticks([5,10,15,20,25])
# ax[0,2].set_yticks([0.001,0.002,0.003,0.004])

#ax[1,1].figsize=(128,128)
ins0=ax[0,3].plot(x_range,no_local_std_qw_error, label = 'no_local_std')
ins1=ax[0,3].plot(x_range,random_qw_error, label = 'random')
# lns = ins0+ins1
# labs = [l.get_label() for l in lns]
# ax[0,3].legend(lns, labs, loc="upper right")
# ax[0,3].set_xlabel("number")
# ax[0,3].set_ylabel("Error")
ax[0,3].set_title("qw")
ax[0,3].set_xticks([5,10,15,20,25])
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
ax[1,3].set_frame_on(False)
ax[1,3].get_xaxis().set_visible(False)
ax[1,3].get_yaxis().set_visible(False)
plt.plot([], [], label = 'no_local_std')
plt.plot([], [], label = 'random')
lns = ins0+ins1
labs = [l.get_label() for l in lns]
ax[1,3].legend(lns, labs, loc="upper right")

plt.savefig("q_error_10_10.png")
plt.show()
print(0)






