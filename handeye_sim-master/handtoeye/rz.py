# -*- coding:utf-8 -*-
import numpy as np
from utils import tranformUtils as utils
from scipy import optimize as op
def loss_function(X,poseList, extrinsicList, real_coor):
    rotationX = utils.q2dcm(X[0:4].reshape(1, -1))
    Hx = np.append(rotationX, np.transpose([X[4:7]]), 1)
    Hx = np.append(Hx, np.array([[0, 0, 0, 1]]), 0)
    rotationY = utils.q2dcm(X[7:11].reshape(1, -1))
    Hy = np.append(rotationY, np.transpose([X[11:14]]), 1)
    Hy = np.append(Hy, np.array([[0, 0, 0, 1]]), 0)
    n = len(poseList)

    error = RMSE(Hx,Hy,poseList,extrinsicList, real_coor)
    return error

def RMSE(Hx,Hy,poseList, extrinsicList, real_coor):
    n = len(poseList)
    error = np.array([])
    a = np.size(real_coor, 0)
    real_coor = np.append(real_coor.T, np.zeros([1, a]), 0)
    real_coor = np.append(real_coor, np.ones([1, a]), 0)
    for i in range(n):
        Hbh = np.array(poseList[i])
        HT1 = np.dot(poseList[i],Hy)
        HT2 = np.dot(np.linalg.inv(Hx),HT1)
        HT3 = np.dot(np.linalg.inv(extrinsicList[i]),HT2)
        proj = np.dot(HT3,real_coor)

        proj[:, :] = proj[:, :] / proj[3, :]
        error = np.append(error,proj[0:3,:]-real_coor[0:3,:])
    return error
def proj_error(Hx,Hy,poseList, extrinsicList, objpoints):
    return RMSE(Hx,Hy,poseList, extrinsicList, objpoints)

def RMSE2(Hx,Hy,poseList, extrinsicList, real_coor):
    n = len(poseList)
    error = np.array([])
    a = np.size(real_coor, 0)
    real_coor = np.append(real_coor.T, np.zeros([1, a]), 0)
    real_coor = np.append(real_coor, np.ones([1, a]), 0)
    for i in range(n):
        Hbh = np.array(poseList[i])
        HT1 = np.dot(poseList[i],Hy)
        HT2 = np.dot(np.linalg.inv(Hx),HT1)
        HT3 = np.dot(np.linalg.inv(extrinsicList[i]),HT2)
        proj = np.dot(HT3,real_coor)

        proj[:, :] = proj[:, :] / proj[3, :]
        error = np.append(error,np.mean(np.linalg.norm(proj[0:3,:]-real_coor[0:3,:],axis=0)))
    return error

def refine(X,Y,poseList, extrinsicList, real_coor):
    qx = utils.dcm2q(X[:3, :3])
    qy = utils.dcm2q(Y[:3, :3])
    initx = np.append(qx, X[:3, 3])
    inity = np.append(qy, Y[:3, 3])
    init = np.append(initx, inity)
    solver = op.root(loss_function, init, args=(poseList, extrinsicList, real_coor), method="lm")
    X = solver.x
    rotationX = utils.q2dcm(X[0:4].reshape(1, -1))
    Hx = np.append(rotationX, np.transpose([X[4:7]]), 1)
    Hx = np.append(Hx, np.array([[0, 0, 0, 1]]), 0)
    rotationY = utils.q2dcm(X[7:11].reshape(1, -1))
    Hy = np.append(rotationY, np.transpose([X[11:14]]), 1)
    Hy = np.append(Hy, np.array([[0, 0, 0, 1]]), 0)
    return Hx, Hy