import numpy as np
import scipy
from method import tranformUtils as utils
# Calculates the least squares solution of
# AX = XB
#
#  Hand-Eye Calibration
#  R. Horaud and F. Dornaika
#
#  Mili Shah
#  July 2014
def q2rot(q1,q2,q3,q4):
    R = np.identity(3)
    R[0, 0] = q4 ^ 2 + q1 ^ 2 - q2 ^ 2 - q3 ^ 2
    R[1, 1] = q4 ^ 2 - q1 ^ 2 + q2 ^ 2 - q3 ^ 2
    R[2, 2] = q4 ^ 2 - q1 ^ 2 - q2 ^ 2 + q3 ^ 2

    R[1, 2] = 2 *( -q4 * q3 + q1 * q2 )
    R[2, 1] = 2 *( q4 * q3 + q1 * q2 )

    R[1, 3] = 2 *( q4 * q2 + q1 * q3)
    R[3, 1] = 2 *( -q4 * q2 + q1 * q3)

    R[2, 3] = 2 *( -q4 * q1 + q2 * q3)
    R[3, 2] = 2 *( q4 * q1 + q2 * q3)
    return R

def calibration(motionAs, motionBs):
    size = len(motionAs)
    AA = np.zeros([4,4])
    for i in range(size):
        A1 = scipy.linalg.logm(motionAs[i][:3,:3])
        B1 = scipy.linalg.logm(motionBs[i][:3,:3])
        a = np.array([A1[2,1],A1[0,2],A1[1,0]])
        a = a/np.linalg.norm(a)
        b = np.array([B1[2,1],B1[0,2],B1[1,0]])
        AA1 = np.array([[0,-a[0]+b[0],-a[1]+b[1],-a[2]+b[2]],
                            [a[0]-b[0],0,-a[2]-b[2],a[1]+b[1]],
                            [a[1]-b[1],a[2]+b[2],0,-a[0]-b[0]],
                            [a[2]-b[2],-a[1]-b[1],a[0]+b[0],0]])
        AA = AA+ np.dot(AA1.T,AA1)
    V, D = np.linalg.eig(AA)
    ind = np.argsort(V)
    V = V[ind]
    D = D[:, ind]
    R = q2rot(D[1],D[2],D[3],V[0])
    C = np.zeros([3*size,3])
    d = np.zeros([3*size,3])
    I = np.identity(3)
    for i in range(size):
        C[3*i:3*i+3,:] = I-motionAs[i][:3,:3]
        d[3*i:3*i+3,:] = motionAs[i][:3,3]-np.dot(R,motionBs[i][:3,3])
    t = np.linalg.lstsq(C,d, rcond=-1)[0]
    return np.append(np.append(R,t,1),np.array([[0,0,0,1]]))

