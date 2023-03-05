#-*- coding:utf-8 -*-
# @time: 
# @author:张新新
# @email: 1262981714@qq.com
import os
from board import apriltagboard
from PIL import Image
import sys
ros_cv2_path = '/opt/ros/kinetic/lib/python2.7/dist-packages'
if ros_cv2_path in sys.path:
    sys.path.remove(ros_cv2_path)
    import cv2
    sys.path.append(ros_cv2_path)
else:
    import cv2
import numpy as np
from utils import depthUtils
def getImgList(root_dir):
    file_list = os.listdir(root_dir)
    rgb_list = []
    for file in file_list:
        if file.endswith("_color.bmp"):
            rgb_list.append(os.path.join(root_dir,file))
    return rgb_list

def main():
    board = apriltagboard.AprilTagBoard("../config/apriltag_real.yml", "../config/tagId.csv")
    root_dir = "../real_data/12_16_20_55"
    img_list = getImgList(root_dir)

    # if len(img_list)<10:
    #     print("number of img not enough")
    #     return 0

    image_size = tuple([1920, 1080])
    image_list = []
    obj_list = []
    print(board.GetBoardAllPoints())
    for i in range(len(img_list)):
        image = cv2.imread(img_list[i])
        flag,imagepoints, objpoints = board.getObjImgPointList(image)
        if not flag:
            continue
        image_list.append(imagepoints)
        obj_list.append(objpoints)
    rme, intrinsic, discoff = board.intrinsic(image_list, obj_list, image_size)
    print(intrinsic, rme)
    fs = cv2.FileStorage(root_dir + "/intrinsic_rgb.yml", cv2.FileStorage_WRITE)
    fs.write("cameraMatrix", intrinsic)
    fs.write("discoff", discoff)
    fs.release()
    return 0

if __name__ == "__main__":
    main()
