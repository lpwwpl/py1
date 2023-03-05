# _*_ coding:utf-8 _*_
# @time: 2020/9/26 上午9:59
# @author: 张新新
# @email: 1262981714@qq.com
from Vrep import vrep
import time
import transforms3d
import numpy as np
import math
import random
import cv2
class Robot():
    def __init__(self,clinetID):
        self.clientID = clinetID
        errorCode, self.dummyTargetHandle = vrep.simxGetObjectHandle(self.clientID, 'Dummy_target', vrep.simx_opmode_oneshot_wait)
        errorCode, self.dummyTipHandle = vrep.simxGetObjectHandle(self.clientID, 'Dummy_tip', vrep.simx_opmode_oneshot_wait)
        fs = cv2.FileStorage("../config/Robot_set.yml",cv2.FileStorage_READ)
        self.mu_q = fs.getNode("mu_q").real()
        self.simu_q = fs.getNode("simu_q").real()
        self.mu_t = fs.getNode("mu_t").real()
        self.simu_t = fs.getNode("simu_t").real()
        fs.release()
        errorcode, self.home_euler = vrep.simxGetObjectOrientation(self.clientID, self.dummyTipHandle, -1,
                                                     vrep.simx_opmode_oneshot_wait)
        errorcode,self.home_position = vrep.simxGetObjectPosition(self.clientID, self.dummyTipHandle, -1,
                                                     vrep.simx_opmode_oneshot_wait)

    def go_home(self):
        vrep.simxSetObjectOrientation(self.clientID, self.dummyTargetHandle, -1,self.home_euler,vrep.simx_opmode_oneshot_wait)
        time.sleep(0.5)
        vrep.simxSetObjectPosition(self.clientID, self.dummyTargetHandle, -1, self.home_position,
                                   vrep.simx_opmode_oneshot_wait)
        time.sleep(0.5)
        return

    def go_guodudian(self):
        guodudian_position = np.array([-0.1, -0.1, 0.9])
        vrep.simxSetObjectPosition(self.clientID, self.dummyTargetHandle, -1, guodudian_position,
                                   vrep.simx_opmode_oneshot_wait)
        time.sleep(2)
        guodudian_euler = np.array([0, -math.pi / 2, math.pi / 2])
        vrep.simxSetObjectOrientation(self.clientID, self.dummyTargetHandle, -1, guodudian_euler,
                                      vrep.simx_opmode_oneshot_wait)
        time.sleep(2)

        guodudian_position = np.array([0.2, 0.0, 0.5])
        vrep.simxSetObjectPosition(self.clientID, self.dummyTargetHandle, -1, guodudian_position,
                                   vrep.simx_opmode_oneshot_wait)
        time.sleep(2)
        # _, position = vrep.simxGetObjectPosition(self.clientID, self.dummyTipHandle, -1,
        #                                          vrep.simx_opmode_oneshot_wait)

        # for i in range(10):
        #
        #     time.sleep(2)
        #     guodudian_position = np.array([-0.3,0.0,0.9-i*0.05])
        #     vrep.simxSetObjectPosition(self.clientID, self.dummyTargetHandle, -1, guodudian_position,
        #                                    vrep.simx_opmode_oneshot_wait)
        #
        #     time.sleep(2)
        # _,position = vrep.simxGetObjectPosition(self.clientID,self.dummyTipHandle,-1,vrep.simx_opmode_oneshot_wait)
        #     if np.linalg.norm(guodudian_position-position)>0.01:
        #         self.go_home()
        #     else:
        #         break

    def move(self,pose):
        '''
        将末端移动到相应的位置
        :param pose:4*4
        :param mu_q:四元数高斯噪声的均值
        :param simu_q:四元数噪声的方差
        :param mu_t:position高斯噪声的均值
        :param simu_q:position四元数噪声的方差
        '''
        euler = transforms3d.euler.mat2euler(pose[:3,:3],'rxyz')
        vrep.simxSetObjectPosition(self.clientID, self.dummyTargetHandle, -1, pose[:3, 3],
                                   vrep.simx_opmode_oneshot_wait)
        time.sleep(2)
        vrep.simxSetObjectOrientation(self.clientID, self.dummyTargetHandle, -1,euler,vrep.simx_opmode_oneshot_wait)
        time.sleep(2)
        q0 = transforms3d.quaternions.mat2quat(pose[:3,:3])
        q = np.array([q0[1],q0[2],q0[3],q0[0]])
        errorcode, q1 = vrep.simxGetObjectQuaternion(self.clientID, self.dummyTipHandle, -1,
                                                     vrep.simx_opmode_oneshot_wait)
        # vrep.simxSetObjectQuaternion(self.clientID,self.dummyTargetHandle,-1,q0,vrep.simx_opmode_oneshot_wait)
        #time.sleep(1)


        errorcode,q1 = vrep.simxGetObjectQuaternion(self.clientID,self.dummyTipHandle,-1,vrep.simx_opmode_oneshot_wait)
        errorcode, euler1 = vrep.simxGetObjectOrientation(self.clientID, self.dummyTipHandle, -1,
                                                     vrep.simx_opmode_oneshot_wait)
        errorcode,t1 = vrep.simxGetObjectPosition(self.clientID,self.dummyTipHandle,-1,vrep.simx_opmode_oneshot_wait)

        norm_q = min(np.linalg.norm(q1-q),np.linalg.norm(q1+q))
        norm_t = np.linalg.norm(pose[:3,3]-t1)
        if not (norm_q<0.01 and norm_t<0.001):
            vrep.simxSetObjectOrientation(self.clientID, self.dummyTargetHandle, -1, euler,
                                          vrep.simx_opmode_oneshot_wait)
            time.sleep(2)
            errorcode, q1 = vrep.simxGetObjectQuaternion(self.clientID, self.dummyTipHandle, -1,
                                                         vrep.simx_opmode_oneshot_wait)
            norm_q = min(np.linalg.norm(q1 - q), np.linalg.norm(q1 + q))
            if not norm_q < 0.01:
                print("error move,q_dis:{0},t_dis:{1}".format(norm_q, norm_t))
                return False
        q = np.empty([4],np.float32)
        q[0] = q1[0]+random.gauss(self.mu_q,self.simu_q)
        q[1] = q1[1]+random.gauss(self.mu_q,self.simu_q)
        q[2] = q1[2]+random.gauss(self.mu_q,self.simu_q)
        q[3] = q1[3]+random.gauss(self.mu_q,self.simu_q)
        q0 = q/np.linalg.norm(q)
        t = np.empty([3],np.float32)
        t[0] = t1[0]+random.gauss(self.mu_t,self.simu_t)
        t[1] = t1[1]+random.gauss(self.mu_t,self.simu_t)
        t[2] = t1[2]+random.gauss(self.mu_t,self.simu_t)
        vrep.simxSetObjectQuaternion(self.clientID, self.dummyTargetHandle, -1, q0, vrep.simx_opmode_oneshot_wait)
        vrep.simxSetObjectPosition(self.clientID, self.dummyTargetHandle, -1, t, vrep.simx_opmode_oneshot_wait)
        time.sleep(1.5)
        return True

    def moveable(self,pose):
        '''
        给定pose，判断是否可达
        :param pose: 4*4
        :return:
        '''
        t = pose[:3,3]
        distance = np.linalg.norm(t)
        if distance>0.85:
            return False
        else:
            return True


if __name__ == '__main__':
    robot = Robot(0)
    pose = np.zeros([4,4])
    pose_r = transforms3d.euler.euler2mat(math.pi/6,math.pi/4,math.pi/3,'rxyz')
    q = transforms3d.euler.euler2quat(math.pi/6,math.pi/4,math.pi/3,'rxyz')
    print(q)
    euler = np.array([math.pi/6,math.pi/4,math.pi/3])
    pose_t = np.array([0.3,0.3,0.3])
    pose[:3,:3] = pose_r[:,:]
    pose[:3,3] = pose_t[:]

    robot.move(pose)