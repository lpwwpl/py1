#-*- coding:utf-8 -*-
# @time: 
# @author:张新新
# @email: 1262981714@qq.com
import os
from board import arucoboard
from PIL import Image
import cv2
import numpy as np
from utils import depthUtils
def getImgList(root_dir):
    file_list = os.listdir(root_dir)
    rgb_list = []
    depth_list = []
    for file in file_list:
        if file.endswith("_color.png"):
            rgb_list.append(os.path.join(root_dir,file))
        elif file.endswith("_depth.png"):
            depth_list.append(os.path.join(root_dir,file))
    return rgb_list,depth_list

def main():
    board = arucoboard.arucoboard("../config/aruco.yml")
    root_dir = "D:\\data\\fbs_data\\1129\\kinect"
    img_list, depth_image_list = getImgList(root_dir)
    if len(img_list) != len(depth_image_list):
        print("numer of img and depth not same")
        return 0
    # if len(img_list)<10:
    #     print("number of img not enough")
    #     return 0
    depth_list = []
    image_size = tuple([640,480])
    image_list = []
    obj_list = []
    print(board.GetBoardAllPoints())
    for i in range(len(img_list)):
        image = cv2.imread(img_list[i])
        imagepoints,objpoints = board.getObjImgPointList(image)
        depth_points,rejectids = depthUtils.get_depth(imagepoints,depth_image_list[i])
        image_list.append(np.delete(imagepoints,rejectids,axis=0))
        depth_list.append(np.delete(depth_points,rejectids,axis=0))
        obj_list.append(np.delete(objpoints,rejectids,axis=0))
    rme, intrinsic,discoff = board.intrinsic(image_list,obj_list,image_size)
    print(intrinsic,rme)
    intrinsic,discoff = board.intrinsic_depth_opt(obj_list,image_list,depth_list,intrinsic,discoff)
    fs = cv2.FileStorage(root_dir+"/intrinsic.yml", cv2.FileStorage_WRITE)
    fs.write("cameraMatrix", intrinsic)
    fs.write("discoff", discoff)
    fs.release()
    return 0

if __name__ == "__main__":
    main()
