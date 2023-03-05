import sys
import os
rootpath = str("/home/speedbot/code/handeye_sim")
syspath = sys.path
sys.path.append(rootpath)
dirs = os.listdir(rootpath)
for dir in dirs:
    sys.path.append(os.path.join(rootpath,dir))

from board import apriltagboard
ros_cv2_path = '/opt/ros/kinetic/lib/python2.7/dist-packages'
if ros_cv2_path in sys.path:
    sys.path.remove(ros_cv2_path)
    import cv2
    sys.path.append(ros_cv2_path)
else:
    import cv2
from camera import kinect
from robot import aubo
import numpy as np
from utils import depthUtils

camera = kinect.Kinect(1026,"../config/intrinsic_1216.yml")
board = apriltagboard.AprilTagBoard("../config/apriltag_real.yml", "../config/tagId.csv")
# camera = kinect.Kinect(1026,"../config/intrinsic_rgb.yml")
# camera = kinect.Kinect(1026,"../real_data/intrinisc_data/intrinsic.yml")


flag,img,depth  = camera.get_rgb_depth_image()
flag, objpoint, imgpoint = board.getObjImgPointList(img, verbose=0)
camerapose = board.extrinsic(imgpoint, objpoint, camera.intrinsic, camera.dist)
camerapose1 = board.extrinsic_opt(camera.intrinsic, camera.dist,camerapose,imgpoint,objpoint)

depth_points,rejectids = depthUtils.get_depth2(imgpoint,depth)
imgpoint = np.delete(imgpoint,rejectids,axis=0)
depth_points = np.delete(depth_points,rejectids,axis=0)
objpoint = np.delete(objpoint,rejectids,axis=0)
flag,extrinsic = board.extrisic_depth(objpoint,imgpoint,depth_points,camera.intrinsic, camera.dist)
print(camerapose1,extrinsic)
