# -*- coding: utf-8 -*-
"""
Use two cameras to get the 3D distance of points

For point1, get its u, v (pixel position) in left and right images independently
then write it as point1 = [ul, vl, ur, vr]
point2, point3, ..., pointn are similar.

Input: points position in two images (from left and right cameras)
Output: 3D distance between points
"""

import numpy as np
import cv2

# checkerboard corner point position (ul, vl, ur, vr)
# l for group_5_19_left.jpg, r for group_5_19_right.jpg
# The 3D distance of the following two points are 20mm (ground truth)
points = [
[167,	131,	206,	131],
[167,	116,	206,	116],
]
# [167,	131,	206,	131],
# [167,	115,	206,	116],
# intrinsic matrix of the left and right cameras (got from around 20 images)
intrinsic_left = np.array([[6.12692566e+02,0,3.23764923e+02],[0,6.12443115e+02,2.33248459e+02],[0,0,1]])
intrinsic_right = np.array([[6.12692566e+02,0,3.23764923e+02],[0,6.12443115e+02,2.33248459e+02],[0,0,1]])

distortion_left = np.array([0,-0,0,-0,0])
distortion_right = np.array([0,-0,0,0,0])

# rotation and translation matrix of camera1&2 related to checkerboard (group_5_18)
Rl = cv2.Rodrigues(np.array([0.0163251,-0.08265144,1.57490133]))[0]
Tl = [[0.02180753],[-0.06759886],[0.39119994]]

Rr = cv2.Rodrigues(np.array([0.00532589, -0.08043565, 1.57453466]))[0]
Tr = [[0.02143023],[-0.07702609],[0.39410995]]

def point2d_3d(ul, vl, ur, vr):
    """ 2D position in images to real world 3D position
    A function transfer the coordinate of a point 
    in left and right images (1280x720) to real world coordinate
    Input: the position of a point in left and right images (pixel)
    Output: 3D position of the point in real world (mm), 
            according to the left camera position
    """

    # Zc_left * [[ul],[vl],[1]] = Pl * [[X],[Y],[Z],[1]]
    Pl = np.dot(intrinsic_left, np.hstack((Rl, Tl)))
    Pr = np.dot(intrinsic_right, np.hstack((Rr, Tr)))

    # solve AX = B
    A_eq = [[ul*Pl[2][0]-Pl[0][0], ul*Pl[2][1]-Pl[0][1], ul*Pl[2][2]-Pl[0][2]],\
        [vl*Pl[2][0]-Pl[1][0], vl*Pl[2][1]-Pl[1][1], vl*Pl[2][2]-Pl[1][2]],\
        [ur*Pr[2][0]-Pr[0][0], ur*Pr[2][1]-Pr[0][1], ur*Pr[2][2]-Pr[0][2]],\
        [vr*Pr[2][0]-Pr[1][0], vr*Pr[2][1]-Pr[1][1], vr*Pr[2][2]-Pr[1][2]]] 
    B_eq = [Pl[0][3]-ul*Pl[2][3], Pl[1][3]-vl*Pl[2][3], Pr[0][3]-ur*Pr[2][3], Pr[1][3]-vr*Pr[2][3]]

    answer = np.linalg.lstsq(A_eq, B_eq, rcond=-1)
    X = 24*answer[0][0]
    Y = 24*answer[0][1]
    Z = 24*answer[0][2]
    # print(X,Y,Z,end='\n')
    # print(np.dot(A_eq, [[X],[Y],[Z]]))

    return X, Y, Z

def undistort_image(img, intrinsic, distortion):
    h, w = img.shape[:2]
    newcameramtx, roi=cv2.getOptimalNewCameraMatrix(intrinsic,distortion,(w,h),0,(w,h))
    dst = cv2.undistort(img, intrinsic, distortion, None, newcameramtx)
    # crop the image
    x,y,w,h = roi
    dst = dst[y:y+h, x:x+w]
    return dst

def distance_3D(point0_2d, point1_2d):
    X0, Y0, Z0 = point2d_3d(point0_2d[0],point0_2d[1],point0_2d[2],point0_2d[3])
    X1, Y1, Z1 = point2d_3d(point1_2d[0],point1_2d[1],point1_2d[2],point1_2d[3])

    distance = np.sqrt((X0 - X1)*(X0 - X1) + (Y0 - Y1)*(Y0 - Y1) + (Z0 - Z1)*(Z0 - Z1))
    print(distance)
#l
# [[ 0.0163251 ]
#  [-0.08265144]
#  [ 1.57490133]]
# tvecs
# [[ 0.02180753]
#  [-0.06759886]
#  [ 0.39119994]]

#r
# rvecs
# [[ 0.00532589]
#  [-0.08043565]
#  [ 1.57453466]]
# tvecs
# [[ 0.02143023]
#  [-0.07702609]
#  [ 0.39410995]]

def main():
    print("Distance in 3D (mm)") 
    for point_2d in points[1:]:
        distance_3D(points[0], point_2d)


if __name__ == '__main__':
    main()
    