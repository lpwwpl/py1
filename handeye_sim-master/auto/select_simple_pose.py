# _*_ coding:utf-8 _*_
# @time: 2020/10/4 上午10:30
# @author: 张新新
# @email: 1262981714@qq.com
ros_cv2_path = '/opt/ros/kinetic/lib/python2.7/dist-packages'
if ros_cv2_path in sys.path:
    sys.path.remove(ros_cv2_path)
    import cv2
    sys.path.append(ros_cv2_path)
else:
    import cv2
import numpy as np

import transforms3d
def select_pose_by_view(pose,camera_intrinsic,imgsize,board):
    board_max_x = board.marker_X*(board.markerSeparation+board.tag_size)
    board_max_y = board.marker_Y * (board.markerSeparation + board.tag_size)
    mid_x = board_max_x/2
    mid_y = board_max_y/2

    mid_points = np.array([[0,mid_x,mid_x,board_max_x],
                           [mid_y,0,board_max_y,mid_y],
                           [0,0,0,0],
                           [1,1,1,1]])
    select_pose= []
    all=0
    camera_intrinsic = np.append(camera_intrinsic,np.zeros([3,1]),1)
    for i in range(len(pose)):
        camera_pose = pose[i]
        points = np.dot(camera_intrinsic,np.dot(camera_pose,mid_points))
        if (points[2, 0] < 0):
            continue
        points[:,:] = points[:,:]/points[2,:]

        t=0
        for j in range(4):
            if points[0,j]>0 and points[0,j]<imgsize[0] and points[1,j]>0 and points[1,j]<imgsize[1]:
                t=t+1
        if t>3:
            select_pose.append(camera_pose)
        if t==4:
            all = all+1
    #
    # t = np.array([])
    # for pose in select_pose:
    #     t = np.append(t,pose[:3,3])
    # t=t.reshape([-1,3])
    return select_pose
