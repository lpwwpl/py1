import cv2
import numpy as np

left_camera_matrix = np.array([[ 6.12692566e+02,    0,          23764923e+02 ],
                               [   0,          6.12443115e+02,  2.33248459e+02],
                               [   0,            0,            1        ]])

left_distortion = np.array([  0,  0,   0,  0,   0])

right_camera_matrix = np.array([[ 6.12692566e+02,    0,          23764923e+02 ],
                               [   0,          6.12443115e+02,  2.33248459e+02],
                               [   0,            0,            1        ]])

right_distortion = np.array([  0,  0,   0,  0,   0])

om = np.array([0.03944465, 0.20144, 1.52740131])
R = cv2.Rodrigues(om)[0]
T = np.array([5.62233136, -2.98129403, 25.81993809])
'''
R = np.array([[ 0.55203161, -0.05852514, 0.83176674],
             [-0.13085881,  0.97909187, 0.1557404 ],
             [-0.82349079, -0.19481763, 0.53283113]])
T = np.array([-20.71161089,  -3.72345748, 12.57491925])
'''
size = (640,480)

R1,R2,P1,P2,Q,roi1,roi2 = cv2.stereoRectify(left_camera_matrix,left_distortion,
                                            right_camera_matrix,right_distortion,
                                            size,R,T)

newcameramatrix1,roi1 = cv2.getOptimalNewCameraMatrix(left_camera_matrix,left_distortion,size,0,size)
newcameramatrix2,roi2 = cv2.getOptimalNewCameraMatrix(right_camera_matrix,right_distortion,size,0,size)

left_map1,left_map2 = cv2.initUndistortRectifyMap(left_camera_matrix,left_distortion,None,newcameramatrix1,size,cv2.CV_16SC2)
right_map1,right_map2 = cv2.initUndistortRectifyMap(right_camera_matrix,right_distortion,None,newcameramatrix2,size,cv2.CV_16SC2)

