# _*_ coding:utf-8 _*_
# @time: 2020/9/28 上午10:06
# @author: 张新新
# @email: 1262981714@qq.com
from Vrep import vrep
from Vrep import vrep_connect
import time
import numpy as np
import cv2
import transforms3d
import random
class Camera():
    def __init__(self,clientId,config):
        self.clientID = clientId
        errorCode, self.KinectRgbHandle = vrep.simxGetObjectHandle(self.clientID, 'kinect_rgb',
                                                                     vrep.simx_opmode_blocking)
        errorCode, self.KinectDepthHandle = vrep.simxGetObjectHandle(self.clientID, 'kinect_depth',
                                                               vrep.simx_opmode_oneshot_wait)
        self.imgsize = (640, 480)
        fs2 = cv2.FileStorage(config, cv2.FileStorage_READ)
        self.intrinsic_gt = fs2.getNode("intrinsic").mat()
        self.intrinsic = fs2.getNode("intrinsic").mat()
        self.dist = fs2.getNode("dist").mat()
        fs2.release()

    def move(self,pose):
        euler = transforms3d.euler.mat2euler(pose[:3, :3], 'rxyz')
        vrep.simxSetObjectPosition(self.clientID, self.KinectRgbHandle, -1, pose[:3, 3],
                                   vrep.simx_opmode_oneshot_wait)
        time.sleep(0.5)
        vrep.simxSetObjectOrientation(self.clientID, self.KinectRgbHandle, -1, euler, vrep.simx_opmode_oneshot_wait)
        # q0 = transforms3d.quaternions.mat2quat(pose[:3,:3])
        # q = np.array([q0[1],q0[2],q0[3],q0[0]])
        # errorcode, q1 = vrep.simxGetObjectQuaternion(self.clientID, self.dummyTipHandle, -1,
        #                                              vrep.simx_opmode_oneshot_wait)
        # vrep.simxSetObjectQuaternion(self.clientID,self.dummyTargetHandle,-1,q0,vrep.simx_opmode_oneshot_wait)
        # time.sleep(1)

        time.sleep(0.5)
    def add_noise(self):
        f_random = (random.random()-0.5)*20
        self.intrinsic[0, 0] = self.intrinsic_gt[0,0]+f_random
        self.intrinsic[1, 1] = self.intrinsic_gt[1,1]+f_random
        self.intrinsic[0, 2] = self.intrinsic_gt[0,2]+(random.random()-0.5)*2
        self.intrinsic[1, 2] = self.intrinsic_gt[1,2]+(random.random()-0.5)*2

    def get_rgb_image(self):
        # Get color image from simulation

        sim_ret, resolution, raw_image = vrep.simxGetVisionSensorImage(self.clientID, self.KinectRgbHandle, 0,
                                                                       vrep.simx_opmode_blocking)
        color_img = np.asarray(raw_image)
        color_img.shape = (resolution[1], resolution[0], 3)
        color_img = color_img.astype(np.float) / 255
        color_img[color_img < 0] += 1
        color_img *= 255
        color_img = np.fliplr(color_img)
        color_img = color_img.astype(np.uint8)
        center = (resolution[0]/2,resolution[1]/2)
        size = (resolution[0],resolution[1])
        rotateMat = cv2.getRotationMatrix2D(center, 180, 1)
        result_img = cv2.warpAffine(color_img, rotateMat, size)

        return result_img

    def get_depth_image(self):
        sim_ret, resolution, depth_buffer = vrep.simxGetVisionSensorDepthBuffer(self.clientID, self.KinectDepthHandle,
                                                                                vrep.simx_opmode_blocking)
        depth_img = np.asarray(depth_buffer)
        depth_img.shape = (resolution[1], resolution[0])
        depth_img = np.fliplr(depth_img)
        return depth_img
if __name__ == '__main__':
    camera = Camera(vrep_connect.getVrep_connect())
    rgb = camera.get_rgb_image()
    cv2.imshow("rgb",rgb)
    cv2.waitKey(0)
    cv2.destroyAllWindows()
