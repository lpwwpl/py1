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
from auto import utils
'''
get Hpencil2cam 
'''
board = apriltagboard.AprilTagBoard("../config/apriltag_real.yml", "../config/tagId.csv")
# camera = kinect.Kinect(1026,"../config/intrinsic_rgb.yml")
# camera = kinect.Kinect(1026,"../real_data/intrinisc_data/intrinsic.yml")
# camera = kinect.Kinect(1026,"../config/intrinsic_1217.yml")
result = utils.json_load("../real_data/2020_12_19_15_30/result.json")
#result = utils.json_load("../real_data/2020_12_16_12_17/result.json")
Hcam2end = result[-1]["Hcamera2end"]
Hobj2base = result[-1]["Hobj2base"]
robot = aubo.robot(1025)

#camerapose1 = board.extrinsic_opt(camera.intrinsic, camera.dist,extrinsic_depth,imgpoint,objpoint)
falg,robot_pose = robot.get_pose()
pencil_end = np.array([-board.tag_size/2,-board.tag_size/2,0])
Hobj2pencil = np.identity(4)
Hobj2pencil[:3,3] = -pencil_end[:]

Hpencil2end = np.dot(np.dot(np.linalg.inv(robot_pose),Hobj2base),np.linalg.inv(Hobj2pencil))
print(Hpencil2end)
fp = cv2.FileStorage("../config/Hpencil2end.yml",cv2.FileStorage_WRITE)
fp.write("Hpencil2end",Hpencil2end)
fp.release()