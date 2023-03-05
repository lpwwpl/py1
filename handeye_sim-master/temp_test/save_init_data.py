# _*_ coding:utf-8 _*_
# @time: 2020/9/28 下午4:40
# @author: 张新新
# @email: 1262981714@qq.com
import cv2
import numpy as np
import transforms3d
import math
euler = [0,0,0]
#euler = [math.pi/2,0,math.pi]
#euler = [math.pi/4,-math.pi/3,math.pi/6]
init_t = np.array([0,0.4,0.64])
#init_t = np.array([0,-0.4,0.30])
q = transforms3d.euler.euler2quat(euler[0],euler[1],euler[2],"rxyz")
print(transforms3d.euler.euler2mat(euler[0],euler[1],euler[2],"rxyz"))
print(q)
#q = np.array([q[1],q[2],q[3],q[0]])
fs = cv2.FileStorage("../config/init_handtoeye_robot_pose.yml",cv2.FileStorage_WRITE)
fs.write("initpose",np.append(q,init_t))
fs.release()

