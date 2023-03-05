from PyQt5.QtCore import QSemaphore
import numpy as np
from enum import Enum
zero=np.zeros((480,640,3), np.uint8)
depth_scale = 0.001
colorImg=np.zeros((480,640,3), np.uint8)
depthImg=np.zeros((480,640), np.uint16)
inferImg=np.zeros((480,640,3), np.uint8)
clusters=np.zeros((480,640,3), np.uint8)
pc1 = np.zeros((480,640,3),np.uint8)
pc2 = np.zeros((480,640,3),np.uint8)
axisPoints = np.asarray([])
axisPointsColor = np.asarray([])
camPointsColor = np.asarray([])
camPointsColor_clusters = np.asarray([])
color_streaming=np.zeros((480,640,3), np.uint8)
depth_streaming=np.zeros((480,640,3), np.uint8)

color_l_streaming=np.zeros((480,640,3), np.uint8)
color_l_streaming=np.zeros((480,640,3), np.uint8)
color_r_streaming=np.zeros((480,640,3), np.uint8)
color_r_streaming=np.zeros((480,640,3), np.uint8)

sem = QSemaphore(1)
sem_streaming = QSemaphore(1)

torch_device = None
net = None
softmax = None
x=0.0
y=0.0
z=0.0
r=0.0
p=0.0
z=0.0
isReal=False
isVirt=False
class ROBOT_TYPE(Enum):
    UR=1
    ABB=2
    NoType=3