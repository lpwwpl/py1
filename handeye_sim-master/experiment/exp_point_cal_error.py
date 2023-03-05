from auto import utils
import os
import random
from camera import kinect
from robot import aubo
import sys
import transforms3d
import numpy as np
ros_cv2_path = '/opt/ros/kinetic/lib/python2.7/dist-packages'
if ros_cv2_path in sys.path:
    sys.path.remove(ros_cv2_path)
    import cv2
    sys.path.append(ros_cv2_path)
else:
    import cv2
from utils import depthUtils
from board import apriltagboard
import numpy as np
def get_robot_pose(Hcam2end,Hpencil2cam,cur_Hend2base,cur_Hobj2camera,point):
    Hobj2pencil = np.identity(4)
    Hobj2pencil[:3,3] = -point[:]
    Hrobot = np.dot(np.dot(cur_Hend2base,np.dot(Hcam2end,cur_Hobj2camera)),
                    np.linalg.inv(np.dot(Hcam2end,np.dot(Hpencil2cam,Hobj2pencil))))
    return Hrobot
result = utils.json_load("../real_data/2020_12_17_17_59/result.json")
#result = utils.json_load("../real_data/2020_12_16_12_17/result.json")
Hcam2end = result[-1]["Hcamera2end"]
board = apriltagboard.AprilTagBoard("../config/apriltag_paper.yml", "../config/tagId.csv")
camera = kinect.Kinect(1026,"../config/intrinsic_1217.yml")
robot = aubo.robot(port=1025)
Hobj2base = result[-1]["Hobj2base"]
border_center = np.array([(board.tag_size+board.markerSeparation)*3,(board.tag_size+board.markerSeparation)*2,0,1]).reshape([4,1])
border_cneter_ = np.dot(Hobj2base,border_center)
width = (board.tag_size+board.markerSeparation)*7
height = (board.tag_size+board.markerSeparation)*5
q = transforms3d.quaternions.mat2quat(Hobj2base[:3,:3])
robot.build_board(border_cneter_[0,0],border_cneter_[1,0],border_cneter_[2,0],q[0],q[1],q[2],q[3],width,height)

fs = cv2.FileStorage("../temp/campose_pen.yml",cv2.FileStorage_READ)
camerapose_for = fs.getNode("campose").mat()
robotpose_for = fs.getNode("robotpose").mat()
fs.release()
temp =robotpose_for.copy()
temp[2,3] = temp[2,3]+0.2
#robot.move(temp)

flag,cur_Hend2base = robot.get_pose()
flag,img,depth  = camera.get_rgb_depth_image()
flag, objpoint, imgpoint = board.getObjImgPointList(img, verbose=0)
depth_points,rejectids = depthUtils.get_depth2(imgpoint,depth)

camerapose1 = board.extrinsic(imgpoint, objpoint, camera.intrinsic, camera.dist)
camerapose = board.extrinsic_opt(camera.intrinsic, camera.dist,camerapose1,imgpoint,objpoint)
imgpoint = np.delete(imgpoint,rejectids,axis=0)
depth_points = np.delete(depth_points,rejectids,axis=0)
objpoint = np.delete(objpoint,rejectids,axis=0)
flag,extrinsic_depth = board.extrisic_depth(objpoint,imgpoint,depth_points,camera.intrinsic, camera.dist)

fs = cv2.FileStorage("../config/Hpencil2camera.yml",cv2.FileStorage_READ)
Hpencil2cam = fs.getNode("Hpencil2camera").mat()
fs.release()
point = np.array([-board.tag_size/2+(board.tag_size+board.markerSeparation)*3,-board.tag_size/2,0])

robot_pose = get_robot_pose(Hcam2end,Hpencil2cam,cur_Hend2base,extrinsic_depth,point)


robotpose2 = np.dot(np.dot(cur_Hend2base,np.dot(Hcam2end,extrinsic_depth)),np.linalg.inv(np.dot(Hcam2end,camerapose_for)))
robotpose3 = np.dot(np.dot(cur_Hend2base,np.dot(Hcam2end,camerapose1)),np.linalg.inv(np.dot(Hcam2end,camerapose_for)))
print(np.dot(cur_Hend2base,np.dot(Hcam2end,extrinsic_depth)))
print(np.dot(robotpose_for,np.dot(Hcam2end,camerapose_for)))
print(np.dot(robotpose2,np.dot(Hcam2end,camerapose_for)))


robot.move(robotpose2)

flag,pose_cur = robot.get_pose()
flag,img,depth  = camera.get_rgb_depth_image()
flag, objpoint, imgpoint = board.getObjImgPointList(img, verbose=0)
depth_points,rejectids = depthUtils.get_depth2(imgpoint,depth)

camerapose1 = board.extrinsic(imgpoint, objpoint, camera.intrinsic, camera.dist)
camerapose = board.extrinsic_opt(camera.intrinsic, camera.dist,camerapose1,imgpoint,objpoint)
imgpoint = np.delete(imgpoint,rejectids,axis=0)
depth_points = np.delete(depth_points,rejectids,axis=0)
objpoint = np.delete(objpoint,rejectids,axis=0)
flag,extrinsic_depth2 = board.extrisic_depth(objpoint,imgpoint,depth_points,camera.intrinsic, camera.dist)
print(0)