from auto import utils
import os
import random
from camera import kinect
from robot import aubo
import sys
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
robot = aubo.robot(port=1025)
fs = cv2.FileStorage("../temp/campose_pen.yml",cv2.FileStorage_READ)

camerapose_for = fs.getNode("campose").mat()
robotpose_for = fs.getNode("robotpose").mat()
robotpose_for[2,3] = robotpose_for[2,3]+0.1
fs.release()
flag,robot_pose = robot.move(robotpose_for)
print(robotpose_for)
print(robot_pose)
robot.realease()
