# -*- coding:utf-8 -*-
import sys
ros_cv2_path = '/opt/ros/kinetic/lib/python2.7/dist-packages'


import numpy as np

from PIL import Image


# from pykinect2 import PyKinectV2
def get_depth(image_points,depth_path):
    depth_image = Image.open(depth_path)
    depth_image = np.array(depth_image)
    depth_point = np.empty([image_points.shape[0],1])
    rejectid = []
    for j in range(image_points.shape[0]):
        try:
            depth_point[j,0] = depth_image[int(image_points[j, 1]), int(image_points[j, 0])] / 1000.0
        except Exception:
            depth_point[j, 0] = 0
        if depth_point[j,0]<0.3:
            rejectid.append(j)
    return depth_point,rejectid
# from pykinect2 import PyKinectV2
def get_depth2(image_points,depth_image):

    depth_point = np.empty([image_points.shape[0],1])
    rejectid = []
    for j in range(image_points.shape[0]):
        try:
            depth_point[j,0] = depth_image[int(image_points[j, 1]), int(image_points[j, 0])] / 1000.0
        except Exception:
            depth_point[j, 0] = 0
        if depth_point[j,0]==0:
            rejectid.append(j)
    return depth_point,rejectid
