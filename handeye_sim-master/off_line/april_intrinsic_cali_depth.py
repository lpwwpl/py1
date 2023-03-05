#-*- coding:utf-8 -*-
# @time: 
# @author:张新新
# @email: 1262981714@qq.com
import os
from board import apriltagboard
from scipy import optimize as op
from PIL import Image
import cv2
import numpy as np
from utils import depthUtils



def getImgList(root_dir):
    file_list = os.listdir(root_dir)
    rgb_list = []
    depth_list = []
    for file in file_list:
        if file.endswith("_color.bmp"):
            rgb_list.append(os.path.join(root_dir,file))
        elif file.endswith("_depth.png"):
            depth_list.append(os.path.join(root_dir,file))
    return rgb_list,depth_list

def main():
    board = apriltagboard.AprilTagBoard("../config/apriltag_real.yml", "../config/tagId.csv")
    #root_dir = "../real_data/12_16_20_55"
    root_dir = "../real_data/intrinisc_data"
    img_list, depth_image_list = getImgList(root_dir)
    if len(img_list) != len(depth_image_list):
        print("numer of img and depth not same")
        return 0
    img_list.sort()
    depth_image_list.sort()
    print(img_list)
    print(depth_image_list)
    # if len(img_list)<10:
    #     print("number of img not enough")
    #     return 0
    depth_list = []
    image_size = tuple([1920,1080])
    image_list = []
    obj_list = []
    for i in range(len(img_list)):
        image = cv2.imread(img_list[i])
        flag,objpoints,imagepoints = board.getObjImgPointList(image)
        if not flag:
            continue
        depth_points,rejectids = depthUtils.get_depth(imagepoints,depth_image_list[i])
        if len(rejectids) == np.size(len(depth_points)):
            continue
        image_list.append(np.delete(imagepoints,rejectids,axis=0))
        depth_list.append(np.delete(depth_points,rejectids,axis=0))
        obj_list.append(np.delete(objpoints,rejectids,axis=0))
    rme, intrinsic,discoff = board.intrinsic(image_list,obj_list,image_size)
    print(intrinsic,rme)


    intrinsic,discoff = board.intrinsic_depth_opt4(obj_list,image_list,depth_list,intrinsic,discoff)
    print(intrinsic, rme)
    for i in range(len(obj_list)):
        extrinsic = board.extrinsic(image_list[i],obj_list[i],intrinsic,discoff)
        extrinsic_opt = board.extrinsic_opt(intrinsic,discoff,extrinsic,image_list[i],obj_list[i])
        flag,extrinsic_depth = board.extrisic_depth(obj_list[i],image_list[i],depth_list[i],intrinsic,discoff)
        print("extrinsic",extrinsic)
        print("extrinsic_opt",extrinsic_opt)
        print("extrinsic_depth",extrinsic_depth)
    fs = cv2.FileStorage(root_dir+"/intrinsic.yml", cv2.FileStorage_WRITE)
    fs.write("cameraMatrix", intrinsic)
    print(intrinsic)
    fs.write("discoff", discoff)
    fs.release()
    return 0

if __name__ == "__main__":
    main()
