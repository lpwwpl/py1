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

def getErrorList(fileList,euler_camera2end_gt,t_camera2end_gt,euler_obj2base_gt,t_obj2base_gt):
    euler_camera2end_error_mutlti = np.array([])
    t_camera2end_error_mutlti = np.array([])
    euler_obj2base_error_mutlti = np.array([])
    t_obj2base_error_mutlti = np.array([])
    for file in fileList:
        result = utils.json_load(file)
        if not result[0]["simu"]==0.002:
            continue
        euler_camera2end_error = np.array([])
        t_camera2end_error = np.array([])
        euler_obj2base_error = np.array([])
        t_obj2base_error = np.array([])
        for i in range(5,31):
            for dict in result:
                if dict["image_number"]==i:
                    Hcamera2end_temp = dict["Hcamera2end"]
                    Hobj2base_temp = dict["Hobj2base"]
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
    euler_camera2end_error_mutlti = euler_camera2end_error_mutlti.reshape([-1, 26, 3])
    t_camera2end_error_mutlti = t_camera2end_error_mutlti.reshape([-1, 26, 3])
    euler_obj2base_error_mutlti = euler_obj2base_error_mutlti.reshape([-1, 26, 3])
    t_obj2base_error_mutlti = t_obj2base_error_mutlti.reshape([-1, 26, 3])
    return euler_camera2end_error_mutlti,t_camera2end_error_mutlti,euler_obj2base_error_mutlti,t_obj2base_error_mutlti

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
    root_dir = "../result/10_30"
    files = os.listdir(root_dir)
    for file in files:
        for i in range(len(method_list)):
            if file.startswith(method_list[i]):
                method_file_list[i].append(os.path.join(root_dir,file))
    fs = cv2.FileStorage("../config/handineye_gt.yml", cv2.FileStorage_READ)
    Hcamera2end_gt = fs.getNode("Hcamera2end").mat()
    Hobj2base_gt = fs.getNode("Hobj2base").mat()
    fs.release()
    euler_camera2end_gt = transforms3d.euler.mat2euler(Hcamera2end_gt[:3, :3])
    t_camera2end_gt = Hcamera2end_gt[:3, 3]
    euler_obj2base_gt = transforms3d.euler.mat2euler(Hobj2base_gt[:3, :3])
    t_obj2base_gt = Hobj2base_gt[:3, 3]
    x_range = np.arange(5, 31, 1)
    #error 统计
    method_error = []
    method_std = []
    for i in range(len(method_list)):
        euler_camera2end_error, t_camera2end_error, euler_obj2base_error, t_obj2base_error = \
            getErrorList(method_file_list[i],euler_camera2end_gt,t_camera2end_gt,euler_obj2base_gt,t_obj2base_gt)
        method_error.append([np.mean(euler_camera2end_error,axis=0), np.mean(t_camera2end_error,axis=0), np.mean(euler_obj2base_error,axis=0), np.mean(t_obj2base_error,axis=0)])
        method_std.append([np.std(euler_camera2end_error,axis=0), np.std(t_camera2end_error,axis=0), np.std(euler_obj2base_error,axis=0), np.std(t_obj2base_error,axis=0)])
    ax_list = init_set()
    for i in range(len(method_list)):
        draw_error(ax_list,method_error[i][0], method_error[i][1], method_list[i], x_range)
    plt.savefig(os.path.join(root_dir, "Ecamera2end_mean.png"))
    plt.show()
    ax_list = init_set()
    for i in range(len(method_list)):
        draw_error(ax_list,method_std[i][0], method_std[i][1], method_list[i], x_range)
    plt.savefig(os.path.join(root_dir, "Ecamera2end_std.png"))
    plt.show()
    ax_list = init_set()
    for i in range(len(method_list)):
        draw_error(ax_list,method_error[i][2], method_error[i][3], method_list[i], x_range)
    plt.savefig(os.path.join(root_dir, "Eobj2base_mean.png"))
    plt.show()
    ax_list = init_set()
    for i in range(len(method_list)):
        draw_error(ax_list,method_std[i][2], method_std[i][3], method_list[i], x_range)
    plt.savefig(os.path.join(root_dir, "Eobj2base_std.png"))
    plt.show()



