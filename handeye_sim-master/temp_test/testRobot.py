from robot import aubo
ros_cv2_path = '/opt/ros/kinetic/lib/python2.7/dist-packages'
import sys
import transforms3d as t3d
import numpy as np
if ros_cv2_path in sys.path:
    sys.path.remove(ros_cv2_path)
    import cv2
    sys.path.append(ros_cv2_path)
else:
    import cv2
fs = cv2.FileStorage("../config/auto_set_real.yml", cv2.FILE_STORAGE_READ)
init_pose = fs.getNode("init_robot_pose").mat()
q = init_pose[3:7].flatten()
pose_r = t3d.quaternions.quat2mat(q)
init_robot_pose = np.identity(4)
init_robot_pose[:3, :3] = pose_r[:, :]
init_robot_pose[0, 3] = init_pose[0]
init_robot_pose[1, 3] = init_pose[1]
init_robot_pose[2, 3] = init_pose[2]
fs.release()
robot = aubo.robot(1025)
flag = robot.move(init_robot_pose)
print("move to first pose:",flag)
euler = np.array([0,30,0])*3.1415926/180
r = t3d.euler.euler2mat(euler[0],euler[1],euler[2])
pose2 = np.identity(4)
pose2[:3,:3] = r[:,:]
pose2 = np.dot(pose2,init_robot_pose)
robot.move(pose2)
print("move to second pose: ",flag)
robot.realease()

