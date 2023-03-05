# _*_ coding:utf-8 _*_
# @time: 2020/11/23 上午9:21
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
import pandas as pd
import seaborn as sns
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


def getErrorList(rootdir,fileList,euler_hx_gt,t_hx_gt,euler_hy_gt,t_hy_gt,method_list,name_list):
    # hx_df_euler = pd.DataFrame(columns=('image number', 'euler_error', 'method'))
    # hy_df_euler = pd.DataFrame(columns=('image number', 'euler_error', 'method'))
    # hx_df_t = pd.DataFrame(columns=('image number', 't_error', 'method'))
    # hy_df_t = pd.DataFrame(columns=('image number', 't_error', 'method'))

    image_number = []
    hx_euler = []
    hy_euler = []
    hx_t= []
    hy_t = []
    method = []


    for file in fileList:
        name=None
        for i in range(len(method_list)):
            if file.startswith(method_list[i]):
                name = name_list[i]
        if name is None:
            continue
        result = utils.json_load(os.path.join(rootdir,file))
        if not result[0]["simu"]==0.002:
            continue
        for dict in result:
            number = dict["image_number"]
            if number<5:
                continue
            Hx_temp = dict[Hx]
            Hy_temp = dict[Hy]
            euler_hx_temp = transforms3d.euler.mat2euler(Hx_temp[:3, :3])
            t_hx_temp = Hx_temp[:3, 3]
            euler_hx_error_rx = min(abs(euler_hx_temp[0] - euler_hx_gt[0]),
                                    abs(euler_hx_temp[0] - euler_hx_gt[0] + 2 * 3.1415926),
                                    abs(euler_hx_temp[0] - euler_hx_gt[0] - 2 * 3.1415926))
            euler_hx_error_ry = min(abs(euler_hx_temp[1] - euler_hx_gt[1]),
                                    abs(euler_hx_temp[1] - euler_hx_gt[1] + 2 * 3.1415926),
                                    abs(euler_hx_temp[1] - euler_hx_gt[1] - 2 * 3.1415926))
            euler_hx_error_rz = min(abs(euler_hx_temp[2] - euler_hx_gt[2]),
                                    abs(euler_hx_temp[2] - euler_hx_gt[2] + 2 * 3.1415926),
                                    abs(euler_hx_temp[2] - euler_hx_gt[2] - 2 * 3.1415926))
            if max(euler_hx_error_rx,euler_hx_error_ry,euler_hx_error_rz)>0.08 :
                print(file)
                continue
            hx_euler_error = np.mean(np.array([euler_hx_error_rx, euler_hx_error_ry, euler_hx_error_rz]))
            hx_t_error = np.mean(np.abs(t_hx_temp - t_hx_gt))
            image_number.append(number)

            hx_euler.append(hx_euler_error)
            hx_t.append(hx_t_error)
            method.append(name)
            # hx_df_euler = hx_df_euler.append({'image number':number,"euler_error":hx_euler_error,"method":name},ignore_index=True)
            # hx_df_t = hx_df_t.append({'image number':number, 't_error':hx_t_error, 'method':name},ignore_index=True)
            euler_hy_temp = transforms3d.euler.mat2euler(Hy_temp[:3, :3])
            t_hy_temp = Hy_temp[:3, 3]
            euler_hy_error_rx = min(abs(euler_hy_temp[0] - euler_hy_gt[0]),
                                    abs(euler_hy_temp[0] - euler_hy_gt[0] + 2 * 3.1415926),
                                    abs(euler_hy_temp[0] - euler_hy_gt[0] - 2 * 3.1415926))
            euler_hy_error_ry = min(abs(euler_hy_temp[1] - euler_hy_gt[1]),
                                    abs(euler_hy_temp[1] - euler_hy_gt[1] + 2 * 3.1415926),
                                    abs(euler_hy_temp[1] - euler_hy_gt[1] - 2 * 3.1415926))
            euler_hy_error_rz = min(abs(euler_hy_temp[2] - euler_hy_gt[2]),
                                    abs(euler_hy_temp[2] - euler_hy_gt[2] + 2 * 3.1415926),
                                    abs(euler_hy_temp[2] - euler_hy_gt[2] - 2 * 3.1415926))
            hy_euler_error = np.mean(np.array([euler_hy_error_rx, euler_hy_error_ry, euler_hy_error_rz]))
            hy_t_error = np.mean(np.abs(t_hy_temp - t_hy_gt))
            #
            hy_euler.append(hy_euler_error)
            hy_t.append(hy_t_error)
    hx_df_euler = pd.DataFrame({"image number":image_number,"error":hx_euler,"method":method})
    hx_df_t = pd.DataFrame({"image number":image_number,"error":hx_t,"method":method})
    hy_df_euler = pd.DataFrame({"image number":image_number,"error":hy_euler,"method":method})
    hy_df_t = pd.DataFrame({"image number":image_number,"error":hy_t,"method":method})
    return hx_df_euler,hy_df_euler,hx_df_t,hy_df_t









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
    plt.rcParams['figure.figsize'] = (8.0, 4.0)
    fig1, f1_axes = plt.subplots(ncols=2, nrows=1, constrained_layout=True)
    ax_list = []

    ax_list.append(f1_axes[0])
    ax_list.append(f1_axes[1])
    ax_list[0].set_ylabel("Absulute Rotation error(rad)")
    ax_list[1].set_ylabel("Absulute Position error(m)")
    return ax_list



def draw_error_seaborn_ax(ax,error_dic,x_range,fileName=None):
    df = pd.DataFrame(error_dic, index=x_range)
    if not (fileName is None):
        df.to_csv(fileName)
    sns.lineplot(data=df, ax=ax)

def get_mean_band(error_df, methodlist):
    error_dic = {}
    for method in method_list:
        error = np.empty([31-5,3])
        for i in range(5,31):
            h5 = hx_df_euler[(hx_df_euler["image number"] == i) & (hx_df_euler["method"] == method)]["error"]
            error[i-5,0] = h5.mean(axis=0)
            error[i-5,1] = h5.std(axis=0)
        error_dic[method]=error
    return error_dic











if __name__ == '__main__':
    method_list = [ 'no_local_std', "random",'ias']
    name_list = ['no_local_std', "random",'ias']
    #文件划分

    method_file_list = []
    sns.set_style("whitegrid")
    for i in range(len(method_list)):
        method_file_list.append([])
    root_dir = "../result/11_33"
    files = os.listdir(root_dir)
    if cali_type==0:
        fs = cv2.FileStorage("../config/handineye_gt.yml", cv2.FileStorage_READ)
    else:
        fs = cv2.FileStorage("../config/handtoeye_gt.yml", cv2.FileStorage_READ)
    Hx_gt = fs.getNode(Hx).mat()
    Hy_gt = fs.getNode(Hy).mat()
    fs.release()
    euler_hx_gt = transforms3d.euler.mat2euler(Hx_gt[:3, :3])
    t_hx_gt = Hx_gt[:3, 3]
    euler_hy_gt = transforms3d.euler.mat2euler(Hy_gt[:3, :3])
    t_hy_gt = Hy_gt[:3, 3]
    x_range = np.arange(5, 31, 1)
    filePath = []

    hx_df_euler, hy_df_euler, hx_df_t, hy_df_t = getErrorList(root_dir,files,euler_hx_gt,t_hx_gt,euler_hy_gt,t_hy_gt,method_list,name_list)

    ax_list = init_set()
    error_dic = get_mean_band(hx_df_euler,method_list)
    scale = 0.1
    for method in method_list:
        y = error_dic[method][:,0]
        std = error_dic[method][:,1]
        ax_list[0].plot(x_range,y,color=colorMap[method],label=method)
        ax_list[0].fill_between(x_range,y+scale*std,y-scale*std,alpha=0.2)
    error_dic = get_mean_band(hx_df_t, method_list)
    for method in method_list:
        y = error_dic[method][:, 0]
        std = error_dic[method][:, 1]
        ax_list[1].plot(x_range, y, color=colorMap[method], label=method)
        ax_list[1].fill_between(x_range, y + scale*std, y - scale*std, alpha=0.2)
    plt.show()

    # #print(hx_df_euler)
    # for i in range(5,26):
    #     h5 = hx_df_euler[(hx_df_euler["image number"]==i) & (hx_df_euler["method"]=="no_local_std")]
    #     print(i,h5.mean())
    # for i in range(5,30):
    #     df = hx_df_euler[(hx_df_euler["image number"]==i) & (hx_df_euler["method"]=="no_local_std")]
    #
    # sns.lineplot(x="image number", y="euler_error", hue="method", data=hx_df_euler,ax=ax_list[0],estimator="mean",ci='sd')
    # sns.lineplot(x="image number", y="t_error", hue="method", data=hx_df_t,ax=ax_list[1],ci='sd')
    # # draw_error_seaborn_ax(ax_list[0], hx_df_euler, x_range,os.path.join(root_dir, "{0}_mean_r.csv".format(Hx)))
    # # draw_error_seaborn_ax(ax_list[1], hx_df_t, x_range,os.path.join(root_dir, "{0}_mean_t.csv".format(Hx)))
    # # for i in range(5):
    # #     ax_list[i+1].legend_.remove()
    # # ax_list[0].set_ylabel("error(rad)")
    # # ax_list[3].set_ylabel("error(m)")
    # # ax_list[3].set_xlabel("iter")
    # # ax_list[4].set_xlabel("iter")
    # # ax_list[5].set_xlabel("iter")
    # plt.savefig(os.path.join(root_dir, "E{0}2.png").format(Hx))
    # plt.show()
    #
    # ax_list = init_set()
    # sns.lineplot(x="image number", y="euler_error", hue="method", data=hy_df_euler, ax=ax_list[0],ci=100)
    # sns.lineplot(x="image number", y="t_error", hue="method", data=hy_df_t, ax=ax_list[1],ci=99)
    # # for i in range(5):
    # #     ax_list[i+1].legend_.remove()
    # plt.savefig(os.path.join(root_dir, "E{0}2.png").format(Hy))
    # plt.show()
