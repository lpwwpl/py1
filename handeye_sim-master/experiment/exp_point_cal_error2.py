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
import time
def get_robot_pose(Hpencil2end,Hcam2end,cur_Hend2base,cur_Hobj2camera,point):
    Hobj2pencil = np.identity(4)
    Hobj2pencil[:3,3] = -point[:]
    Hrobot = np.dot(np.dot(cur_Hend2base,np.dot(Hcam2end,cur_Hobj2camera)),np.linalg.inv(np.dot(Hpencil2end,Hobj2pencil)))
    return Hrobot
result = utils.json_load("../real_data/2020_12_19_17_10/result.json")
#result = utils.json_load("../real_data/2020_12_16_12_17/result.json")
Hcam2end = result[-1]["Hcamera2end"]
board = apriltagboard.AprilTagBoard("../config/apriltag_paper.yml", "../config/tagId.csv")
camera = kinect.Kinect(1026,"../real_data/intrinisc_data/intrinsic.yml")
robot = aubo.robot(port=1025)
Hobj2base = result[-1]["Hobj2base"]
# build board
border_center = np.array([(board.tag_size+board.markerSeparation)*3,(board.tag_size+board.markerSeparation)*2,0,1]).reshape([4,1])
border_cneter_ = np.dot(Hobj2base,border_center)
width = (board.tag_size+board.markerSeparation)*7
height = (board.tag_size+board.markerSeparation)*5
q = transforms3d.quaternions.mat2quat(Hobj2base[:3,:3])
robot.build_board(border_cneter_[0,0],border_cneter_[1,0],border_cneter_[2,0]-0.03,q[0],q[1],q[2],q[3],width,height)
border_center = np.array([(board.tag_size+board.markerSeparation)*3,(board.tag_size+board.markerSeparation)*2,0,1]).reshape([4,1])
#robot.move(temp)
point0 = np.array([-board.tag_size/2,-board.tag_size/2,0])
point1 = np.array([board.tag_size/2+(board.tag_size+board.markerSeparation)*(board.marker_X-1),
                   -board.tag_size/2,0])
point2 = np.array([board.tag_size/2+(board.tag_size+board.markerSeparation)*(board.marker_X-1),
                   board.tag_size/2+(board.tag_size+board.markerSeparation)*(board.marker_Y-1),0])
point3 = np.array([-board.tag_size/2,
                   board.tag_size/2+(board.tag_size+board.markerSeparation)*(board.marker_Y-1),0])
fs = cv2.FileStorage("../config/Hpencil2end.yml",cv2.FileStorage_READ)
Hpencil2end = fs.getNode("Hpencil2end").mat()
fs.release()
points = []
points.append(point0)
#points.append(point1)
points.append(point2)
points.append(point3)
for point in points:
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

    #point = np.array([-board.tag_size/2+(board.tag_size+board.markerSeparation)*3,-board.tag_size/2,0])
    robot_pose = get_robot_pose(Hpencil2end,Hcam2end,cur_Hend2base,camerapose1,point)
    robot.move(robot_pose)
    time.sleep(5)
    robot.move(cur_Hend2base)
# robotpose2 = np.dot(np.dot(cur_Hend2base,np.dot(Hcam2end,extrinsic_depth)),np.linalg.inv(np.dot(Hcam2end,camerapose_for)))
# print(np.dot(cur_Hend2base,np.dot(Hcam2end,extrinsic_depth)))
# print(np.dot(robotpose_for,np.dot(Hcam2end,camerapose_for)))
# print(np.dot(robotpose2,np.dot(Hcam2end,camerapose_for)))
# robotpose3 = robot.get_pose()
#
# robot.move(robotpose2)
#
# flag,pose_cur = robot.get_pose()
# flag,img,depth  = camera.get_rgb_depth_image()
# flag, objpoint, imgpoint = board.getObjImgPointList(img, verbose=0)
# depth_points,rejectids = depthUtils.get_depth2(imgpoint,depth)
#
# camerapose1 = board.extrinsic(imgpoint, objpoint, camera.intrinsic, camera.dist)
# camerapose = board.extrinsic_opt(camera.intrinsic, camera.dist,camerapose1,imgpoint,objpoint)
# imgpoint = np.delete(imgpoint,rejectids,axis=0)
# depth_points = np.delete(depth_points,rejectids,axis=0)
# objpoint = np.delete(objpoint,rejectids,axis=0)
# flag,extrinsic_depth2 = board.extrisic_depth(objpoint,imgpoint,depth_points,camera.intrinsic, camera.dist)
# print(0)