# _*_ coding:utf-8 _*_
# @time: 2020/10/16 上午9:19
# @author: 张新新
# @email: 1262981714@qq.com
import os
import cv2
import transforms3d
import numpy as np
import matplotlib
import matplotlib.pyplot as plt
import matplotlib.gridspec as gridspec
from auto import utils
from pylab import mpl
colorMap ={
        "no_Local":"black",
        "std":"blue",
        "no_local_std":"gray",
        "rme":"green",
        "no_lock_rme":"red",
        "random":"yellow",
        'ias':"pink"
    }
cali_type = 0
if cali_type==0:
    Hx = "Hcamera2end"
    Hy = "Hobj2base"
else:
    Hx = "Hcamera2base"
    Hy = "Hobj2end"


def getErrorList(fileList,euler_hx_gt,t_hx_gt,euler_hy_gt,t_hy_gt):
    euler_hx_error_mutlti = np.array([])
    t_hx_error_mutlti = np.array([])
    euler_hy_error_mutlti = np.array([])
    t_hy_error_mutlti = np.array([])
    for file in fileList:
        result = utils.json_load(file)
        if not result[0]["simu"]==0.002:
            continue
        euler_hx_error = np.array([])
        t_hx_error = np.array([])
        euler_hy_error = np.array([])
        t_hy_error = np.array([])
        for i in range(5,31):
            for dict in result:
                if dict["image_number"]==i:
                    Hx_temp = dict[Hx]
                    Hy_temp = dict[Hy]
            euler_hx_temp = transforms3d.euler.mat2euler(Hx_temp[:3, :3])
            t_hx_temp = Hx_temp[:3, 3]
            euler_hx_error_rx = min(abs(euler_hx_temp[0]-euler_hx_gt[0]),
                                            abs(euler_hx_temp[0]-euler_hx_gt[0]+2*3.1415926),
                                            abs(euler_hx_temp[0] - euler_hx_gt[0] - 2 * 3.1415926))
            euler_hx_error_ry = min(abs(euler_hx_temp[1] - euler_hx_gt[1]),
                                            abs(euler_hx_temp[1] - euler_hx_gt[1] + 2 * 3.1415926),
                                            abs(euler_hx_temp[1] - euler_hx_gt[1] - 2 * 3.1415926))
            euler_hx_error_rz = min(abs(euler_hx_temp[2] - euler_hx_gt[2]),
                                            abs(euler_hx_temp[2] - euler_hx_gt[2] + 2 * 3.1415926),
                                            abs(euler_hx_temp[2] - euler_hx_gt[2] - 2 * 3.1415926))
            euler_hx_error = np.append(euler_hx_error,np.array([euler_hx_error_rx,euler_hx_error_ry,euler_hx_error_rz]))
            t_hx_error = np.append(t_hx_error, np.abs(t_hx_temp - t_hx_gt))

            euler_hy_temp = transforms3d.euler.mat2euler(Hy_temp[:3, :3])
            # if np.linalg.norm(euler_hy_temp - euler_hy_gt) > np.linalg.norm(euler_hy_temp + euler_hy_gt):
            #     euler_hy_temp = -euler_hy_temp
            t_hy_temp = Hy_temp[:3, 3]
            #euler_hy_error = np.append(euler_hy_error, np.abs(euler_hy_temp - euler_hy_gt))
            euler_hy_error_rx = min(abs(euler_hy_temp[0]-euler_hy_gt[0]),
                                          abs(euler_hy_temp[0]-euler_hy_gt[0]+2*3.1415926),
                                          abs(euler_hy_temp[0] - euler_hy_gt[0] - 2 * 3.1415926))
            euler_hy_error_ry = min(abs(euler_hy_temp[1] - euler_hy_gt[1]),
                                          abs(euler_hy_temp[1] - euler_hy_gt[1] + 2 * 3.1415926),
                                          abs(euler_hy_temp[1] - euler_hy_gt[1] - 2 * 3.1415926))
            euler_hy_error_rz = min(abs(euler_hy_temp[2] - euler_hy_gt[2]),
                                          abs(euler_hy_temp[2] - euler_hy_gt[2] + 2 * 3.1415926),
                                          abs(euler_hy_temp[2] - euler_hy_gt[2] - 2 * 3.1415926))
            euler_hy_error = np.append(euler_hy_error, np.array([euler_hy_error_rx,euler_hy_error_ry,euler_hy_error_rz]))
            t_hy_error = np.append(t_hy_error, np.abs(t_hy_temp - t_hy_gt))
        euler_hx_error = euler_hx_error.reshape([-1, 3])
        t_hx_error = t_hx_error.reshape([-1, 3])
        euler_hy_error = euler_hy_error.reshape([-1, 3])
        t_hy_error = t_hy_error.reshape([-1, 3])
        euler_hx_error_mutlti = np.append(euler_hx_error_mutlti, euler_hx_error)
        t_hx_error_mutlti = np.append(t_hx_error_mutlti, t_hx_error)
        euler_hy_error_mutlti = np.append(euler_hy_error_mutlti, euler_hy_error)
        t_hy_error_mutlti = np.append(t_hy_error_mutlti, t_hy_error)
    euler_hx_error_mutlti = euler_hx_error_mutlti.reshape([-1, 26, 3])
    t_hx_error_mutlti = t_hx_error_mutlti.reshape([-1, 26, 3])
    euler_hy_error_mutlti = euler_hy_error_mutlti.reshape([-1, 26, 3])
    t_hy_error_mutlti = t_hy_error_mutlti.reshape([-1, 26, 3])
    return euler_hx_error_mutlti,t_hx_error_mutlti,euler_hy_error_mutlti,t_hy_error_mutlti

def draw_error(ax_list,euler_error,t_error,label,x_range):
    rx_error = euler_error[:,0]
    ry_error = euler_error[:,1]
    rz_error = euler_error[:,2]

    tx_error = t_error[:, 0]
    ty_error = t_error[:, 1]
    tz_error = t_error[:, 2]

    ax_list[0].plot(x_range, rx_error,color=colorMap[label], label=label)
    ax_list[0].set_title("rx")
    ax_list[1].plot(x_range, ry_error,color=colorMap[label],  label=label)
    ax_list[1].set_title("ry")
    ax_list[2].plot(x_range, rz_error,color=colorMap[label], label=label)
    ax_list[2].set_title("rz")
    ax_list[3].plot(x_range, tx_error,color=colorMap[label], label=label)
    ax_list[3].set_title("tx")
    ax_list[4].plot(x_range, ty_error,color=colorMap[label], label=label)
    ax_list[4].set_title("ty")
    ax_list[5].plot(x_range, tz_error,color=colorMap[label], label=label)
    ax_list[5].set_title("tz")


def init_set():
    plt.rcParams['figure.figsize'] = (6.0, 4.0)
    plt.rcParams['image.interpolation'] = 'nearest'  # 设置 interpolation style
    plt.rcParams['savefig.dpi'] = 1024  # 图片像素
    plt.rcParams['figure.dpi'] = 300  # 分辨率
    #x_range = np.arange(5, 31, 1)

    fig1, f1_axes = plt.subplots(ncols=4, nrows=2, constrained_layout=True, sharex='col', sharey='row')
    # spec2 = gridspec.GridSpec(ncols=4, nrows=2, figure=fig1)
    ax_list = []

    ax_list.append(f1_axes[0, 0])
    ax_list.append(f1_axes[0, 1])
    ax_list.append(f1_axes[0, 2])
    ax_list.append(f1_axes[1, 0])
    ax_list.append(f1_axes[1, 1])
    ax_list.append(f1_axes[1, 2])
    label_ax = fig1.add_subplot(f1_axes[0, 3])
    plain_ax = fig1.add_subplot(f1_axes[1, 3])
    plain_ax.get_xaxis().set_visible(False)
    plain_ax.get_yaxis().set_visible(False)
    plain_ax.set_frame_on(False)

    # 设置label

    label_ax.set_frame_on(False)
    label_ax.get_xaxis().set_visible(False)
    label_ax.get_yaxis().set_visible(False)
    lns = []
    for label in method_list:
        lns = lns + plt.plot([], [], color=colorMap[label], label=label)
    labs = [l.get_label() for l in lns]
    label_ax.legend(lns, labs, loc="upper right")
    return ax_list






if __name__ == '__main__':
    method_list = ["no_Local", "std", 'no_local_std', "random",'ias']
    #文件划分
    method_file_list = []
    for i in range(len(method_list)):
        method_file_list.append([])
    root_dir = "../result/11_9"
    files = os.listdir(root_dir)
    for file in files:
        for i in range(len(method_list)):
            if file.startswith(method_list[i]):
                method_file_list[i].append(os.path.join(root_dir,file))
    fs = cv2.FileStorage("../config/handineye_gt.yml", cv2.FileStorage_READ)
    Hx_gt = fs.getNode(Hx).mat()
    Hy_gt = fs.getNode(Hy).mat()
    fs.release()
    euler_hx_gt = transforms3d.euler.mat2euler(Hx_gt[:3, :3])
    t_hx_gt = Hx_gt[:3, 3]
    euler_hy_gt = transforms3d.euler.mat2euler(Hy_gt[:3, :3])
    t_hy_gt = Hy_gt[:3, 3]
    x_range = np.arange(5, 31, 1)
    #error 统计
    method_error = []
    method_std = []
    ## test

    # method_file_list = ["../result/11_8/random_11_12_20_16.json"]
    # euler_hx_error, t_hx_error, euler_hy_error, t_hy_error = \
    #     getErrorList(method_file_list, euler_hx_gt, t_hx_gt, euler_hy_gt, t_hy_gt)

    for i in range(len(method_list)):
        euler_hx_error, t_hx_error, euler_hy_error, t_hy_error = \
            getErrorList(method_file_list[i],euler_hx_gt,t_hx_gt,euler_hy_gt,t_hy_gt)
        method_error.append([np.mean(euler_hx_error,axis=0), np.mean(t_hx_error,axis=0), np.mean(euler_hy_error,axis=0), np.mean(t_hy_error,axis=0)])
        method_std.append([np.std(euler_hx_error,axis=0), np.std(t_hx_error,axis=0), np.std(euler_hy_error,axis=0), np.std(t_hy_error,axis=0)])
    ax_list = init_set()
    for i in range(len(method_list)):
        draw_error(ax_list,method_error[i][0], method_error[i][1], method_list[i], x_range)
    plt.savefig(os.path.join(root_dir, "E{0}_mean.png".format(Hx)))
    plt.show()
    ax_list = init_set()
    for i in range(len(method_list)):
        draw_error(ax_list,method_std[i][0], method_std[i][1], method_list[i], x_range)
    plt.savefig(os.path.join(root_dir, "E{0}_std.png").format(Hx))
    plt.show()
    ax_list = init_set()
    for i in range(len(method_list)):
        draw_error(ax_list,method_error[i][2], method_error[i][3], method_list[i], x_range)
    plt.savefig(os.path.join(root_dir, "E{0}_mean.png").format(Hy))
    plt.show()
    ax_list = init_set()
    for i in range(len(method_list)):
        draw_error(ax_list,method_std[i][2], method_std[i][3], method_list[i], x_range)
    plt.savefig(os.path.join(root_dir, "E{0}_std.png").format(Hy))
    plt.show()



