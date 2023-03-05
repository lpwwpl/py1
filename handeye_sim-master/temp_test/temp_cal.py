# _*_ coding:utf-8 _*_
# @time: 2020/9/29 下午4:41
# @author: 张新新
# @email: 1262981714@qq.com
import math
import numpy as np
import cv2
w = 640
h = 480
theta = 57*math.pi/180
Df = 3.5
Dn = 0.01
fx=fy = w/(2*(math.tan(theta/2)))
u0 = w/2
v0 = h/2
print(fx)
intrinsic =np.array([[fx,0,u0],
                     [0,fy,v0],
                     [0,0,1]])
dist = np.array([0,0,0,0,0])
fs = cv2.FileStorage("../config/intrinsic_gt.yml",cv2.FileStorage_WRITE)
fs.write("intrinsic",intrinsic)
fs.write("dist",dist)




