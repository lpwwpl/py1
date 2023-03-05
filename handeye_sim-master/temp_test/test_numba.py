# _*_ coding:utf-8 _*_
# @time: 2020/11/13 下午8:49
# @author: 张新新
# @email: 1262981714@qq.com

from numba import jit
import numba as nb
import numpy as np
import random
import math
import transforms3d
import time
@jit(nopython=True)
def score_std_numba(expect_camera_list,Hend2base,Hobj2camera,Hx):
    expect_robot_pose = np.zeros((expect_camera_list.shape[0],4,4),dtype=nb.float32)
    score = np.zeros((expect_camera_list.shape[0],1),dtype=nb.float32)
    expect_robot_q0 = np.zeros((Hend2base.shape[0],1),dtype=nb.float32)
    expect_robot_q1 = np.zeros((Hend2base.shape[0],1),dtype=nb.float32)
    expect_robot_q2 = np.zeros((Hend2base.shape[0],1),dtype=nb.float32)
    expect_robot_q3 = np.zeros((Hend2base.shape[0],1),dtype=nb.float32)
    expect_robot_t0 = np.zeros((Hend2base.shape[0],1),dtype=nb.float32)
    expect_robot_t1 = np.zeros((Hend2base.shape[0],1),dtype=nb.float32)
    expect_robot_t2 = np.zeros((Hend2base.shape[0],1),dtype=nb.float32)
    for i in range(expect_camera_list.shape[0]):
        for j in range(Hend2base.shape[0]):
            robot_pose= np.dot(Hend2base[j], np.dot(Hx, np.dot(Hobj2camera[j], np.dot(
                    np.linalg.inv(expect_camera_list[i]), np.linalg.inv(Hx)))))
            R = robot_pose[:3,:3]
            Qxx, Qyx, Qzx, Qxy, Qyy, Qzy, Qxz, Qyz, Qzz = R.flat
            K = np.array([
                [Qxx - Qyy - Qzz, 0, 0, 0],
                [Qyx + Qxy, Qyy - Qxx - Qzz, 0, 0],
                [Qzx + Qxz, Qzy + Qyz, Qzz - Qxx - Qyy, 0],
                [Qyz - Qzy, Qzx - Qxz, Qxy - Qyx, Qxx + Qyy + Qzz]],dtype=nb.float32
            ) / 3.0
            vals, vecs = np.linalg.eigh(K)
            q = vecs[:,np.argmax(vals)]
            if j==0:
                expect_robot_q0[j,0]=q[3]
                expect_robot_q1[j,0]=q[0]
                expect_robot_q2[j,0]=q[1]
                expect_robot_q3[j,0]=q[2]
            else:
                sub = abs(q[3]-expect_robot_q0[0,0])+abs(q[0]-expect_robot_q1[0,0])+abs(q[1]-expect_robot_q2[0,0])+abs(q[2]-expect_robot_q3[0,0])
                sum = abs(q[3]-expect_robot_q0[0,0])+abs(q[0]-expect_robot_q1[0,0])+abs(q[1]-expect_robot_q2[0,0])+abs(q[2]-expect_robot_q3[0,0])
                if sub<sum:
                    expect_robot_q0[j, 0] = q[3]
                    expect_robot_q1[j, 0] = q[0]
                    expect_robot_q2[j, 0] = q[1]
                    expect_robot_q3[j, 0] = q[2]
                else:
                    expect_robot_q0[j, 0] = -q[3]
                    expect_robot_q1[j, 0] = -q[0]
                    expect_robot_q2[j, 0] = -q[1]
                    expect_robot_q3[j, 0] = -q[2]

            # if 1 + R[0, 0] + R[1, 1] + R[2, 2] > 0:
            #     q0 = 0.5 * math.sqrt(1 + R[0, 0] + R[1, 1] + R[2, 2])
            #     expect_robot_q0[j, 0] = q0
            #     expect_robot_q1[j, 0] = (R[2, 1] - R[1, 2]) / (4 * q0)
            #     expect_robot_q2[j, 0] = (R[0, 2] - R[2, 0]) / (4 * q0)
            #     expect_robot_q3[j, 0] = (R[1, 0] - R[0, 1]) / (4 * q0)
            # else:
            #     if max(R[0, 0], R[1, 1], R[2, 2]) == R[0, 0]:
            #         t = math.sqrt(1 + R[0, 0] - R[1, 1] - R[2, 2])
            #         expect_robot_q0[j, 0] = (R[2, 1] - R[1, 2]) / t
            #         expect_robot_q1[j, 0] = t / 4
            #         expect_robot_q2[j, 0] = (R[0, 2] + R[2, 0]) / t
            #         expect_robot_q3[j, 0] = (R[0, 1] + R[1, 0]) / t
            #     elif max(R[0, 0], R[1, 1], R[2, 2]) == R[1, 1]:
            #         t = math.sqrt(1 - R[0, 0] + R[1, 1] - R[2, 2])
            #         expect_robot_q0[j, 0] = (R[0, 2] - R[2, 0]) / t
            #         expect_robot_q1[j, 0] = (R[0, 1] + R[1, 0]) / t
            #         expect_robot_q2[j, 0] = t / 4
            #         expect_robot_q3[j, 0] = (R[1, 2] + R[2, 1]) / t
            #     else:
            #         t = math.sqrt(1 - R[0, 0] - R[1, 1] + R[2, 2])
            #         expect_robot_q0[j, 0] = (R[1, 0] - R[0, 1]) / t
            #         expect_robot_q1[j, 0] = (R[0, 2] + R[2, 0]) / t
            #         expect_robot_q2[j, 0] = (R[1, 2] - R[2, 1]) / t
            #         expect_robot_q3[j, 0] = t / 4
            expect_robot_t0[j,0]=robot_pose[0,3]
            expect_robot_t1[j,0]=robot_pose[1,3]
            expect_robot_t2[j,0]=robot_pose[2,3]
        expect_robot_q0_std = np.std(expect_robot_q0)
        expect_robot_q1_std = np.std(expect_robot_q1)
        expect_robot_q2_std = np.std(expect_robot_q2)
        expect_robot_q3_std = np.std(expect_robot_q3)
        expect_robot_t0_std = np.std(expect_robot_t0)
        expect_robot_t1_std = np.std(expect_robot_t1)
        expect_robot_t2_std = np.std(expect_robot_t2)
        expect_robot_q0_mean = np.mean(expect_robot_q0)
        expect_robot_q1_mean = np.mean(expect_robot_q1)
        expect_robot_q2_mean = np.mean(expect_robot_q2)
        expect_robot_q3_mean = np.mean(expect_robot_q3)
        expect_robot_t0_mean = np.mean(expect_robot_t0)
        expect_robot_t1_mean = np.mean(expect_robot_t1)
        expect_robot_t2_mean = np.mean(expect_robot_t2)

        score[i,0] = expect_robot_q0_std+expect_robot_q1_std+expect_robot_q2_std+\
                expect_robot_q3_std+expect_robot_t0_std+expect_robot_t1_std+expect_robot_t1_std+expect_robot_t2_std
        w = expect_robot_q0_mean
        x = expect_robot_q1_mean
        y = expect_robot_q2_mean
        z = expect_robot_q3_mean
        Nq = w * w + x * x + y * y + z * z
        R = np.zeros((3,3),dtype=nb.float32)
        if Nq < 10^-6:
            R =  np.eye(3,dtype=nb.float32)
        else:
            s = 2.0 / Nq
            X = x * s
            Y = y * s
            Z = z * s
            wX = w * X
            wY = w * Y
            wZ = w * Z
            xX = x * X
            xY = x * Y
            xZ = x * Z
            yY = y * Y
            yZ = y * Z
            zZ = z * Z
            R = np.array(
               [[ 1.0-(yY+zZ), xY-wZ, xZ+wY ],
                [ xY+wZ, 1.0-(xX+zZ), yZ-wX ],
                [ xZ-wY, yZ+wX, 1.0-(xX+yY) ]],dtype=nb.float32)

        expect_robot_pose[i,:3,:3] = R[:,:]
        expect_robot_pose[i,0,3] = expect_robot_t0_mean
        expect_robot_pose[i,1,3] = expect_robot_t1_mean
        expect_robot_pose[i,2,3] = expect_robot_t2_mean
        expect_robot_pose[i,2,4] = 1
    return score,expect_robot_pose



@jit(nopython=True)
def score_no_local_numba(expect_robot_pose,Hend2base):
    score = np.zeros((expect_robot_pose.shape[0], 1), dtype=nb.float32)
    K = np.zeros((4, 4), dtype=nb.float32)
    for i in range(expect_robot_pose.shape[0]):
        min_score = 0
        R = expect_robot_pose[i, :3, :3]
        Qxx, Qyx, Qzx, Qxy, Qyy, Qzy, Qxz, Qyz, Qzz = R.flat

        K[0, 0] = Qxx - Qyy - Qzz
        K[1, 0] = Qyx + Qxy
        K[1, 1] = Qyy - Qxx - Qzz
        K[2, 0] = Qzx + Qxz
        K[2, 1] = Qzy + Qyz
        K[2, 2] = Qzz - Qxx - Qyy
        K[3, 0] = Qyz - Qzy
        K[3, 1] = Qzx - Qxz
        K[3, 2] = Qxy - Qyx
        K[3, 3] = Qxx + Qyy + Qzz
        # K = K / 3.0
        vals, vecs = np.linalg.eigh(K)
        q0 = vecs[:, np.argmax(vals)]
        t0 = expect_robot_pose[i, :3, 3]
        for j in range(Hend2base.shape[0]):
            R = Hend2base[j, :3, :3]
            Qxx, Qyx, Qzx, Qxy, Qyy, Qzy, Qxz, Qyz, Qzz = R.flat
            K = np.array([
                [(Qxx - Qyy - Qzz) / 3.0, 0, 0, 0],
                [(Qyx + Qxy) / 3.0, (Qyy - Qxx - Qzz) / 3.0, 0, 0],
                [(Qzx + Qxz) / 3.0, (Qzy + Qyz) / 3.0, (Qzz - Qxx - Qyy) / 3.0, 0],
                [(Qyz - Qzy) / 3.0, (Qzx - Qxz) / 3.0, (Qxy - Qyx) / 3.0, (Qxx + Qyy + Qzz) / 3.0]], dtype=nb.float32
            )
            vals, vecs = np.linalg.eigh(K)
            q = vecs[:, np.argmax(vals)]
            t = Hend2base[j, :3, 3]
            q_dis = min(np.linalg.norm(q + q0), np.linalg.norm(q - q0))
            t_dis = np.linalg.norm(t - t0)
            score_t = -(math.pow(math.e, -0.5 * abs(q_dis)) + 1 * math.pow(math.e, -0.5 * abs(t_dis)))
            print(i, j, q_dis,t_dis,score_t)
            if score_t < min_score:
                min_score = score_t
        score[i, 0] = min_score
    return score


def score_no_local(expect_robot_pose,Hend2base):
    score = np.array([])
    for i in range(len(expect_robot_pose)):
        q0 = transforms3d.quaternions.mat2quat(expect_robot_pose[i][:3, :3])
        t0 = expect_robot_pose[i][:3, 3]
        min_score = 0
        for j in range(len(Hend2base)):
            q = transforms3d.quaternions.mat2quat(Hend2base[j][:3, :3])
            if np.linalg.norm(q - q0) > np.linalg.norm(q + q0):
                q = -q
            t = Hend2base[j][:3, 3]
            q_dis = np.linalg.norm(q - q0)
            t_dis = np.linalg.norm(t - t0)
            score_t = -(math.pow(math.e, -0.5 * abs(q_dis)) + 1 * math.pow(math.e, -0.5 * abs(t_dis)))
            print(i, j, q_dis,t_dis,score_t)
            if score_t < min_score:
                min_score = score_t
        score=np.append(score,min_score)
    return score


def score_std(expect_camera_list,Hend2base,Hobj2camera,Hx):
    expect_robot_pose = np.empty([len(expect_camera_list),len(Hend2base),4,4])
    score = np.array([])
    robot_pose = []
    for i in range(len(expect_camera_list)):
        q = np.array([])
        t = np.array([])
        for j in range(len(Hend2base)):
            expect_robot_pose = np.dot(Hend2base[j], np.dot(Hx, np.dot(Hobj2camera[j], np.dot(
                    np.linalg.inv(expect_camera_list[i]), np.linalg.inv(Hx)))))
            q = np.append(q, transforms3d.quaternions.mat2quat(expect_robot_pose[:3, :3]))
            t = np.append(t, expect_robot_pose[:3, 3])
        q = q.reshape([-1, 4])
        t = t.reshape([-1, 3])
        for i in range(1, q.shape[0]):
            if abs(np.linalg.norm(q[0, :] - q[i, :])) > abs(np.linalg.norm(q[0, :] + q[i, :])):
                q[i, :] = -q[i, :]
        mean_q = np.mean(q, 0)
        mean_t = np.mean(t, 0)
        std_q = np.std(q, axis=0)
        std_t = np.std(t, axis=0)
        expect_robot_pose = np.append(transforms3d.quaternions.quat2mat(mean_q), np.transpose([mean_t]), 1)
        expect_robot_pose = np.append(expect_robot_pose, np.array([[0, 0, 0, 1]]), 0)
        score_t = np.sum(std_q) + np.sum(std_t)
        robot_pose.append(expect_robot_pose)
        score = np.append(score,score_t)
    return expect_robot_pose

if __name__ == '__main__':
    expect_camera_list = []
    for i in range(10000):
        ax = random.random()*math.pi
        ay = random.random()*math.pi
        az = random.random()*math.pi
        r = transforms3d.euler.euler2mat(ax,ay,az)
        RT = np.identity(4)
        RT[:3,:3]=r[:,:]
        RT[0,3] = random.random()
        RT[1,3] = random.random()
        RT[2,3] = random.random()
        expect_camera_list.append(RT)
    Hend2base = []
    for i in range(10):
        ax = random.random()*math.pi
        ay = random.random()*math.pi
        az = random.random()*math.pi
        r = transforms3d.euler.euler2mat(ax,ay,az)
        RT = np.identity(4)
        RT[:3,:3]=r[:,:]
        RT[0,3] = random.random()
        RT[1,3] = random.random()
        RT[2,3] = random.random()
        Hend2base.append(RT)
    Hobj2camera = []
    for i in range(10):
        ax = random.random()*math.pi
        ay = random.random()*math.pi
        az = random.random()*math.pi
        r = transforms3d.euler.euler2mat(ax,ay,az)
        RT = np.identity(4)
        RT[:3,:3]=r[:,:]
        RT[0,3] = random.random()
        RT[1,3] = random.random()
        RT[2,3] = random.random()
        Hobj2camera.append(RT)
    ax = random.random() * math.pi
    ay = random.random() * math.pi
    az = random.random() * math.pi
    r = transforms3d.euler.euler2mat(ax, ay, az)
    Hx = np.identity(4)
    Hx[:3, :3] = r[:, :]
    Hx[0, 3] = random.random()
    Hx[1, 3] = random.random()
    Hx[2, 3] = random.random()
    score = score_no_local(expect_camera_list[:2],Hend2base)
    expect_cameras = np.array(expect_camera_list[:2])
    Hend2bases = np.array(Hend2base)
    Hobj2cameras = np.array(Hobj2camera)
    score2 = score_no_local_numba(expect_cameras,Hend2bases)
    print(0)
    # time1 = time.time()
    # #x = score_std(expect_camera_list,Hend2base,Hobj2camera,Hx)
    # time2 = time.time()
    # print("no numba time:",time2-time1)
    # expect_robot_pose = np.empty([len(expect_camera_list), len(Hend2base), 4, 4])
    # expect_cameras = np.array(expect_camera_list)
    # Hend2bases = np.array(Hend2base)
    # Hobj2cameras = np.array(Hobj2camera)
    # score_no_local(expect_cameras, Hend2bases)
    # time3 = time.time()
    # score_std_numba(expect_cameras,Hend2bases,Hobj2cameras,Hx)
    # time4 = time.time()
    # print("numba time:", time4 - time3)






