import numpy as np
import cv2
from UtilSet import *
import math
import os
from scipy import linalg
# 旋转矩阵转rpy欧拉角
def rm2rpy(R):
    # sy = np.sqrt(R[0][0] * R[0][0] + R[1][0] * R[1][0])
    sy = np.sqrt(R[2][1] * R[2][1] + R[2][2] * R[2][2])
    singular = sy < 1e-6

    if not singular:
        rotatex = np.arctan2(R[2][1], R[2][2])
        rotatey = np.arctan2(-R[2][0], sy)
        rotatez = np.arctan2(R[1][0], R[0][0])
    else:
        rotatex = np.arctan2(-R[1][2], R[1][1])
        rotatey = np.arctan2(-R[2][0], sy)
        rotatez = 0

    return np.asarray([R[0][3], R[1][3], R[2][3], rotatex, rotatey, rotatez])


def Rypr(y, p, r):
    '''
    Rotationsmatrix für y=yaw, p=pitch, r=roll in degrees
    '''
    # from Degree to Radians
    # y = y * np.pi / 180.0
    # p = p * np.pi / 180.0
    # r = r * np.pi / 180.0

    rx = y
    ry = p
    rz = r

    rxs = np.sin(rx)
    rxc = np.cos(rx)
    rys = np.sin(ry)
    ryc = np.cos(ry)
    rzs = np.sin(rz)
    rzc = np.cos(rz)

    Rr = np.matrix([[1.0, 0.0, 0.0], [0.0, rxc, -rxs], [0.0, rxs, rxc]])
    Rp = np.matrix([[ryc, 0.0, rys], [0.0, 1.0, 0.0], [-rys, 0.0, ryc]])
    Ry = np.matrix([[rzc, -rzs, 0.0], [rzs, rzc, 0.0], [0.0, 0.0, 1.0]])

    return Rr * Rp * Ry

# rpy转旋转矩阵
def rpy2rm(rpy):
    # Rx = np.zeros((3, 3), dtype=rpy.dtype)
    # Ry = np.zeros((3, 3), dtype=rpy.dtype)
    # Rz = np.zeros((3, 3), dtype=rpy.dtype)

    R0 = np.zeros((4, 4))

    x = rpy[0]
    y = rpy[1]
    z = rpy[2]
    thetaX = rpy[3]
    thetaY = rpy[4]
    thetaZ = rpy[5]

    cx = np.cos(thetaX)
    sx = np.sin(thetaX)

    cy = np.cos(thetaY)
    sy = np.sin(thetaY)

    cz = np.cos(thetaZ)
    sz = np.sin(thetaZ)

    R0[0][0] = cz * cy
    R0[0][1] = cz * sy * sx - sz * cx
    R0[0][2] = cz * sy * cx + sz * sx
    R0[0][3] = x
    R0[1][0] = sz * cy
    R0[1][1] = sz * sy * sx + cz * cx
    R0[1][2] = sz * sy * cx - cz * sx
    R0[1][3] = y
    R0[2][0] = -sy
    R0[2][1] = cy * sx
    R0[2][2] = cy * cx
    R0[2][3] = z
    R0[3][3] = 1
    return R0

def rot2quat(r):
    rv,_ = cv2.Rodrigues(r)
    theta = np.linalg.norm(rv)
    l = rv / theta
    q = np.concatenate([[np.cos(theta/2)], np.sin(theta/2) * l.reshape(-1)])
    return q

def mat_to_r_t(mat):
    r = mat[:3,:3]
    t = mat[:3,3].reshape(-1)
    return r,t

def rot2dualquat(r,t):
    q = rot2quat(r)
    qprime = 0.5 * quat_mul(np.concatenate([[0],t]),q)
    return q, qprime

def cross_mat(v):
    x,y,z = v.tolist()
    mat = np.array(
        [
            [0,-z,y],
            [z,0,-x],
            [-y,x,0]
        ]
    )
    return mat

def dualquat2r_t(q1, q1prime):
    r = quat2rot(q1)
    t = (2 * quat_mul(q1prime, quat_con(q1)))[1:]
    return r, t

def quat2rot(q):
    w,x,y,z = q.tolist()
    r = np.array(
        [
            [1-2*(y*y+z*z), 2*(x*y-z*w),    2*(x*z+y*w)],
            [2*(x*y+z*w),   1-2*(x*x+z*z),  2*(y*z-x*w)],
            [2*(x*z-y*w),   2*(y*z+x*w),    1-2*(x*x+y*y)]
        ]
    )
    return r

def quat_con(q):
    q_inv = -q
    q_inv[0] = - q_inv[0]
    return q_inv

def get_sub_mat(qa, qa_prime, qb, qb_prime):
    mat = np.zeros((6,8))
    # equation 1
    mat[:3, 0] = qa[1:] - qb[1:]
    mat[:3, 1:4] = cross_mat(qa[1:] + qb[1:])
    # equation 2
    mat[3:, 0] = qa_prime[1:] - qb_prime[1:]
    mat[3:, 1:4] = cross_mat(qa_prime[1:] + qb_prime[1:])
    mat[3:, 4] = qa[1:] - qb[1:]
    mat[3:, 5:] = cross_mat(qa[1:] + qb[1:])
    return mat

def quat_mul(q1, q2):
    q3 = np.zeros_like(q1)
    q3[0] = q1[0]*q2[0] - np.dot(q1[1:], q2[1:])
    q3[1:] = np.cross(q1[1:], q2[1:]) + q1[0]*q2[1:] + q2[0]*q1[1:]
    return q3

class HandEyeCalibration:

    def __init__(self):
        # input variables for handeye calibration
        self.R_gripper2base = []
        self.t_gripper2base = []
        self.gripper2base = []
        self.R_target2cam = []
        self.t_target2cam = []
        self.target2cam = []
        self.R_cam2gripper = []
        self.t_cam2gripper = []

        self.AlgorithmTest = False




    def captureHandEyeInputs(self, robotXYZABC, camRVec, camTVec):
        # prepare Gripper2Base inputs
        hmRobot = HMUtil.convertXYZABCtoHMRad(robotXYZABC)
        self.R_gripper2base.append(hmRobot[0:3, 0:3])
        self.t_gripper2base.append(hmRobot[0:3, 3])
        self.gripper2base.append(hmRobot)
        # prepare Target2Cam inputs
        camRMatrix = np.zeros(shape=(3, 3))
        cv2.Rodrigues(camRVec, camRMatrix)
        hmCam = HMUtil.makeHM(camRMatrix, camTVec)
        hmCam = HMUtil.inverseHM(hmCam)
        self.target2cam.append(hmCam)
        self.R_target2cam.append(hmCam[0:3, 0:3])
        self.t_target2cam.append(hmCam[0:3, 3])

    def getHandEyeResultMatrixUsingOpenCV(self):
        methodHE = [cv2.CALIB_HAND_EYE_TSAI, cv2.CALIB_HAND_EYE_PARK, cv2.CALIB_HAND_EYE_HORAUD,
                    cv2.CALIB_HAND_EYE_ANDREFF, cv2.CALIB_HAND_EYE_DANIILIDIS]
        # methodHE = [cv2.CALIB_HAND_EYE_TSAI]
        if (self.AlgorithmTest == True):
            fsHandEyeTest = cv2.FileStorage("HandEyeTestData.xml", cv2.FILE_STORAGE_WRITE)

        for mth in methodHE:
            self.R_cam2gripper, self.t_cam2gripper = cv2.calibrateHandEye(self.R_gripper2base, self.t_gripper2base,
                                                                          self.R_target2cam, self.t_target2cam, None,
                                                                          None, mth)
            # output results
            print("--------------------------------------")
            print("Method %d" % mth)
            print(self.R_cam2gripper)
            print(self.t_cam2gripper)
            print("--------------------------------------")
            print("Distance: %f" % math.sqrt(
                math.pow(self.t_cam2gripper[0], 2.0) + math.pow(self.t_cam2gripper[1], 2.0) + math.pow(
                    self.t_cam2gripper[2], 2.0)))
            print("--------------------------------------")

            # CALIB_HAND_EYE_HORAUD
            # if (mth == cv2.CALIB_HAND_EYE_TSAI):
            # for idx in range(len(self.R_gripper2base)):
            #     print("######")
            # make a homogeneous matrix from Target(Calibration) to Gripper(TCP)
            hmT2G = HMUtil.makeHM(self.R_cam2gripper, self.t_cam2gripper.T)
            print("cam2gripper: ")
            print(hmT2G)
            # make a homogeneous matrix from Gripper(TCP) to Robot Base
            hmG2B = HMUtil.makeHM(self.R_gripper2base[0], self.t_gripper2base[0].reshape(1, 3))
            # make a homogeneous matrix from Camera to Target(Target)
            hmC2T = HMUtil.makeHM(self.R_target2cam[0], self.t_target2cam[0].reshape(1, 3))

            # Final HM(Camera to Robot Base)
            # H(C2B) = H(G2B)H(T2G)H(C2T)
            hmResultTransform = np.dot(hmG2B, hmT2G)
            hmResultTransform = np.dot(hmResultTransform, hmC2T)

        if (self.AlgorithmTest == True):
            fsHandEyeTest.release()

        print("Final HM(Camera to Robot Base): ")
        print(hmResultTransform)
        return hmResultTransform

    def calculateTransformMatrixUsing3Points(self, p, p_prime):
        # construct intermediate matrix
        Q = p[1:] - p[0]
        Q_prime = p_prime[1:] - p_prime[0]

        # calculate rotation matrix
        R = np.dot(np.linalg.inv(np.row_stack((Q, np.cross(*Q)))),
                   np.row_stack((Q_prime, np.cross(*Q_prime))))

        # calculate translation vector
        t = p_prime[0] - np.dot(p[0], R)

        # calculate affine transformation matrix
        return np.column_stack((np.row_stack((R, t)),
                                (0, 0, 0, 1)))

    # deprecated...
    def calculateTransformMatrix(self, srcPoints, dstPoints):
        assert (len(srcPoints) == len(dstPoints))

        p = np.ones([len(srcPoints), 4])
        p_prime = np.ones([len(dstPoints), 4])
        for idx in range(len(srcPoints)):
            p[idx][0] = srcPoints[idx][0]
            p[idx][1] = srcPoints[idx][1]
            p[idx][2] = srcPoints[idx][2]

            p_prime[idx][0] = dstPoints[idx][0]
            p_prime[idx][1] = dstPoints[idx][1]
            p_prime[idx][2] = dstPoints[idx][2]

        trMatrix = cv2.solve(p, p_prime, flags=cv2.DECOMP_SVD)
        return trMatrix

    train_size = 50
    sample_size = 20
    iteration = 100
    score_threshold = 0.02
    scalar_threshold = 0.0005

    def matlab_svd(self,mat):
        m, n = mat.shape[:2]
        U, sdiag, VH = np.linalg.svd(mat)
        # U, sdiag, VH = scipy.linalg.svd(mat, lapack_driver='gesvd')
        S = np.zeros((m, n))
        np.fill_diagonal(S, sdiag)
        V = VH.T.conj()
        return U, S, V

    def AXXB_Solver(self,AA, BB):
        np.set_printoptions(precision=6)
        m, n = AA.shape[:2]
        n //= 4
        A = np.zeros((9 * n, 9))
        b = np.zeros((9 * n, 1))
        eye = np.eye(3)
        for i in range(n):
            Ra = AA[:3, 4 * i:4 * i + 3]
            Rb = BB[:3, 4 * i:4 * i + 3]
            A[9 * i:9 * i + 9, :] = np.kron(Ra, eye) - np.kron(eye, Rb.T)
        u, s, v = self.matlab_svd(A)
        x = v[:, -1]
        R = x.reshape(3, 3)
        R = np.sign(np.linalg.det(R)) / pow(abs(np.linalg.det(R)), 1 / 3) * R
        u, s, v = self.matlab_svd(R)
        R = np.matmul(u, v.T)
        if np.linalg.det(R) < 0:
            R = np.matmul(u, np.diag([1, 1, -1]), v.T)
        C = np.zeros((3 * n, 3))
        d = np.zeros((3 * n, 1))
        for i in range(n):
            C[3 * i:3 * i + 3] = eye - AA[:3, 4 * i:4 * i + 3]
            d[3 * i:3 * i + 3] = (AA[:3, 4 * i + 3] - np.matmul(R, BB[:3, 4 * i + 3])).reshape(3, 1)
        # t = np.linalg.solve(C, d)
        t = np.linalg.lstsq(C, d,rcond=-1)[0]
        return np.vstack([np.hstack([R, t]), np.array([0, 0, 0, 1])])

    def get_sub_mat(qa, qa_prime, qb, qb_prime):
        mat = np.zeros((6, 8))
        # equation 1
        mat[:3, 0] = qa[1:] - qb[1:]
        mat[:3, 1:4] = cross_mat(qa[1:] + qb[1:])
        # equation 2
        mat[3:, 0] = qa_prime[1:] - qb_prime[1:]
        mat[3:, 1:4] = cross_mat(qa_prime[1:] + qb_prime[1:])
        mat[3:, 4] = qa[1:] - qb[1:]
        mat[3:, 5:] = cross_mat(qa[1:] + qb[1:])
        return mat

    # solve AX=XB by dual quaternion
    def dual_quaternion_approach(self,motionAs, motionBs):
        size = motionAs.shape[0]
        T = []
        for j in range(size):
            motionA = motionAs[j]
            motionB = motionBs[j]
            ra, ta = mat_to_r_t(motionA)
            rb, tb = mat_to_r_t(motionB)
            qa, qa_prime = rot2dualquat(ra, ta)
            qb, qb_prime = rot2dualquat(rb, tb)
            T.append(get_sub_mat(qa, qa_prime, qb, qb_prime))
        T = np.concatenate(T)

        T=np.nan_to_num(T)
        U, s, V = np.linalg.svd(T)
        idx1, idx2 = np.argsort(s)[:2].tolist()
        v7 = V[idx1]
        v8 = V[idx2]

        u1 = v7[:4]
        v1 = v7[4:]
        u2 = v8[:4]
        v2 = v8[4:]

        a = np.dot(u1, v1)
        b = np.dot(u1, v2) + np.dot(u2, v1)
        c = np.dot(u2, v2)

        s1 = (-b + np.sqrt(b * b - 4 * a * c)) / (2 * a)
        s2 = (-b - np.sqrt(b * b - 4 * a * c)) / (2 * a)

        x1 = s1 ** 2 * np.dot(u1, u1) + 2 * s1 * np.dot(u1, u2) + np.dot(u2, u2)
        x2 = s2 ** 2 * np.dot(u1, u1) + 2 * s2 * np.dot(u1, u2) + np.dot(u2, u2)
        (x, s) = (x1, s1) if x1 >= x2 else (x2, s2)

        lambda2 = np.sqrt(1 / x)
        lambda1 = s * lambda2

        q = lambda1 * u1 + lambda2 * u2
        q_ = lambda1 * v1 + lambda2 * v2

        r_ba, t_ba = dualquat2r_t(q, q_)
        return r_t_to_mat(r_ba, t_ba), s

    def get_error(self,T_world_cam, motionAs, motionBs, score_threshold):
        motionAs_ = np.matmul(T_world_cam, motionBs)
        motionAs_ = np.matmul(motionAs_, np.linalg.inv(T_world_cam))
        error = np.linalg.norm(motionAs - motionAs_) / np.linalg.norm(motionAs)
        tAs_ = motionAs_[:, :3, 3]
        tAs = motionAs[:, :3, 3]
        error = np.linalg.norm(tAs_ - tAs) / np.linalg.norm(tAs)
        return error

    def ransac_for_calibration(self,motionAs, motionBs, sample_size=20, iteration=10, score_threshold=0.002, show=False):
        best_error = np.inf
        best_result = None

        for i in range(iteration):
            sample_idxs = np.random.randint(0, motionAs.shape[0], size=(1, sample_size))
            sampled_motionAs = motionAs[sample_idxs.ravel().tolist()]
            sampled_motionBs = motionBs[sample_idxs.ravel().tolist()]

            result, singular_values = self.dual_quaternion_approach(sampled_motionAs, sampled_motionBs)
            error = self.get_error(result, motionAs, motionBs, score_threshold)

            if error < best_error:
                best_error = error
                best_result = result
            if show:
                print("iter ", i, "error: ", error)
        return best_result, best_error

    def get_motion(self,A, B, scalar_threshold=0.0005, train_size=20, show=False):
        size = A.shape[0]
        motionAs = []
        motionBs = []
        for i in range(size):
            Ai = A[i]
            Bi = B[i]

            for j in range(i + 1, size):
                Aj = A[j]
                Bj = B[j]

                motionA = np.matmul(np.linalg.inv(Aj), Ai)
                motionB = np.matmul(Bj, np.linalg.inv(Bi))
                ra, ta = mat_to_r_t(motionA)
                rb, tb = mat_to_r_t(motionB)
                qa, qa_prime = rot2dualquat(ra, ta)
                qb, qb_prime = rot2dualquat(rb, tb)

                # check scalar be equivalent
                diff_scalar = np.abs(qa[0] - qb[0])
                diff_scalar_ = np.abs(qa_prime[0] - qb_prime[0])
                # if show:
                #     print(j, diff_scalar, diff_scalar_)
                # if (diff_scalar < scalar_threshold and diff_scalar_ < scalar_threshold):
                motionAs.append(motionA)
                motionBs.append(motionB)
        shuffle_idxs = [i for i in range(len(motionAs))]
        np.random.shuffle(shuffle_idxs)
        if show:
            print('valid motion size: ', len(motionAs))
            print('train size: ', train_size)
        return np.stack(motionAs)[shuffle_idxs][:train_size], np.stack(motionBs)[shuffle_idxs][:train_size]

    def hand_eye_calibration(self,A, B, sample_size=20, iteration=10, score_threshold=0.002, scalar_threshold=0.0005,
                             train_size=20, show=False):
        motionAs, motionBs = self.get_motion(
            A,
            B,
            scalar_threshold=scalar_threshold,
            train_size=train_size,
            show=show
        )
        T_world_cam, error = self.ransac_for_calibration(
            motionAs,
            motionBs,
            sample_size,
            iteration,
            score_threshold,
            show=show
        )
        if show:
            print('best error:', error)
        return T_world_cam


def r_t_to_mat(r, t):
    mat = np.zeros((4,4))
    mat[:3,:3] = r
    mat[:3,3] = t.T
    mat[3,3] = 1
    return mat

def ShowImage(Image, windowsizewidth,windowsizeheight,Time=1):
    cv2.namedWindow('output', cv2.WINDOW_NORMAL)
    cv2.imshow('output', Image)
    # Resize the Window
    cv2.resizeWindow('output', np.int16(windowsizewidth), np.int16(windowsizeheight))
    cv2.waitKey(Time)

    # cv2.destroyAllWindows()


folder='runs/calibration_collect/exp15'
w_size = 3
h_size = 3

CheckerboardSquareSize=0.025

FactorPictureScaling = 1
# [[-9.99620769e-01  2.61081138e-02  8.75695863e-03  5.24925386e-04]
#  [-2.59364125e-02 -9.99479597e-01  1.91790879e-02  7.31528968e-02]
#  [ 9.25313129e-03  1.89446905e-02  9.99777714e-01  6.42897076e-02]
#  [ 0.00000000e+00  0.00000000e+00  0.00000000e+00  1.00000000e+00]]


# folder='runs/calibration_collect/exp8'
# w_size = 9
# h_size = 6
# CheckerboardSquareSize=0.024
# FactorPictureScaling = 1
# [[-0.44018531  0.80445955 -0.39885051 -0.03312723]
#  [-0.22881199 -0.53003324 -0.81652302  0.07664526]
#  [ 0.86826377  0.26815965 -0.41738283  0.09094993]
#  [ 0.          0.          0.          1.        ]]
if __name__ == '__main__':
    ##################################
    pose_file = open('{}/data_robotxyzrpy.txt'.format(folder), 'r')
    cali_file = open('{}/data_cali.txt'.format(folder), )

    positionList=[]
    try:
        index = 0
        while True:
            text_line = pose_file.readline()
            if text_line:
                worlds = text_line.split(',')
                robotXYZABC = []
                for c in worlds:
                    robotXYZABC.append(float(c))
                positionList.append(np.array(robotXYZABC))
                index = index + 1
            else:
                break
    finally:
        pose_file.close()

    corner_num = w_size * h_size
    cameraMatrix=np.asarray([[612.69256660, 0.0, 323.764923],
                            [0.0, 612.443115, 233.248459],
                            [0.0, 0.0, 1.0]])
    distCoeffs = np.asarray([0.0, 0.0, 0.0, 0.0, 0.0])

    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
    imgpoints = []  # 2d points in image plane.
    objpoints = []

    objp = np.zeros((w_size * h_size, 3), np.float32)*CheckerboardSquareSize
    objp[:, :2] = np.mgrid[0:w_size, 0:h_size].T.reshape(-1, 2) * CheckerboardSquareSize  # 25mm
    images = []
    UsefulPositions = []
    image_count=0
    for fname in cali_file.readlines():
        fname = fname.replace('\n', '')
        fname_glob = os.path.join(os.path.dirname(__file__), fname)
        fname_glob = fname_glob.replace('\\', '/')
        img = cv2.imread(fname_glob)
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        # findChessboardCorners
        ret, corners = cv2.findChessboardCorners(gray, (w_size, h_size), None, cv2.CALIB_CB_ADAPTIVE_THRESH)
        if ret == True:
            corners2 = cv2.cornerSubPix(gray, corners, (1, 1), (-1, -1), criteria)
            imgpoints.append(corners2)
            objpoints.append(objp)
            cv2.drawChessboardCorners(img, (w_size, h_size), corners2, ret)
            images.append(img)
            UsefulPositions.append(positionList[image_count])
            cv2.imshow('Images', img)
            cv2.waitKey(0)
            image_count = image_count+1
    # cv2.destroyAllWindows()
    ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(objpoints, imgpoints, gray.shape[::-1], cameraMatrix=cameraMatrix, distCoeffs=distCoeffs)
    print("rvecs: ", rvecs)
    print("tvecs:", tvecs)
    print("mtx",mtx)
    # Undistort the picture Method 1, Easy Method
    img = images[0]

    width = int(img.shape[1] / FactorPictureScaling)
    height = int(img.shape[0] / FactorPictureScaling)

    dim = (width, height)

    # resize image
    img = cv2.resize(img, dim, interpolation=cv2.INTER_AREA)

    h, w = img.shape[:2]
    newcameramtx, roi = cv2.getOptimalNewCameraMatrix(mtx, dist, (w, h), 1, (w, h))
    # Find the rotation and translation vectors.

    print("newcameramtx:",newcameramtx)
    # undistort
    undstlist = []
    idx = 0
    # images = glob.glob('D:/Scripts/PythonScripts/*.jpg') #skal fjernes
    for img2 in images:
        # for img2 in images: #skal fjernes
        # img = cv2.imread(img2,1)# skal fjernes
        img = img2  # skal fjernes
        width = int(img.shape[1] / FactorPictureScaling)
        height = int(img.shape[0] / FactorPictureScaling)

        dim = (width, height)
        # resize image
        img = cv2.resize(img, dim, interpolation=cv2.INTER_AREA)
        dst = cv2.undistort(img, mtx, dist, None, newcameramtx)
        # crop the image
        x, y, w, h = roi
        dst = dst[y:y + h, x:x + w]
        undstlist.append(dst)
        ShowImage(dst,width,height)
        cv2.waitKey(1)
        filename = 'CalibratedResultNr%s.png' % (idx)

        path = os.path.abspath(os.path.join(executable_path(), "{}/{}".format(folder,filename)))
        cv2.imwrite(path, dst)
        # cv2.imshow('imgshow',dst)
        idx = idx + 1

    cv2.destroyAllWindows()

    mean_error = 0
    for i in range(len(objpoints)):
        imgpoints2, _ = cv2.projectPoints(objpoints[i], rvecs[i], tvecs[i], mtx, dist)
        error = cv2.norm(imgpoints[i], imgpoints2, cv2.NORM_L2) / len(imgpoints2)
        mean_error += error
    print("total pixel error: ", mean_error / len(objpoints))


    # handeye = HandEyeCalibration()
    # try:
    #     index = 0
    #     while True:
    #         text_line = pose_file.readline()
    #         if text_line:
    #             worlds = text_line.split(',')
    #             robotXYZABC = []
    #             for c in worlds:
    #                 robotXYZABC.append(float(c))
    #             handeye.captureHandEyeInputs(robotXYZABC, rvecs[index], tvecs[index])
    #             index = index + 1
    #         else:
    #             break
    # finally:
    #     pose_file.close()


    robotrvecs = []
    robottvecs = []
    for i in range(0, len(UsefulPositions)):
        # temp = UsefulPositions[i][3:6]
        a = np.asarray(UsefulPositions[i][3:6])
        # a = a / 180 * 3.1415926
        robotrvecs.append(a)
        # robotrvecs.append(UsefulPositions[i][3:6])
        temp1 = np.array(UsefulPositions[i][0:2].reshape(1, 2))
        temp2 = np.array(UsefulPositions[i][2].reshape(1, 1))
        temp1 = np.concatenate((temp1, temp2), axis=1).reshape(3, )

        robottvecs.append(temp1.copy())


    for j in range(len(rvecs)):
        r, _ = cv2.Rodrigues(rvecs[j])
        rvecs[j] = r
        t = tvecs[j].reshape(-1)
        tvecs[j] = t

    for j in range(len(robotrvecs)):
        # r, _ = cv2.Rodrigues(robotrvecs[j])
        # robotrvecs[j] = r
        a = [0,0,0,robotrvecs[j][0],robotrvecs[j][1],robotrvecs[j][2]]
        rm = rpy2rm(a)
        rm = Rypr(a[3],a[4],a[5])
        # robotrvecs[j]=rm[:3,:3]
        t = robottvecs[j].reshape(-1)
        robottvecs[j] = t

    homogenousmatrixplaceholder = np.identity(4)

    rm1, tm1 = cv2.calibrateHandEye(robotrvecs,robottvecs, rvecs, tvecs, method=cv2.CALIB_HAND_EYE_PARK)

    # print(np.linalg.inv(Rm))
    homogenousmatrixplaceholder = np.identity(4)
    homogenousmatrixplaceholder[0:3, 0:3] = rm1
    homogenousmatrixplaceholder[0:3, 3] = tm1.ravel()
    print("Park-Martin Calibration Matrix (1):")
    print(homogenousmatrixplaceholder)
    print("")

    rm2, tm2 = cv2.calibrateHandEye(robotrvecs,robottvecs, rvecs, tvecs, method=cv2.CALIB_HAND_EYE_TSAI)

    homogenousmatrixplaceholder = np.identity(4)
    homogenousmatrixplaceholder[0:3, 0:3] = rm2
    homogenousmatrixplaceholder[0:3, 3] = tm2.ravel()
    print("Tsai Calibration Matrix (2):")
    print(homogenousmatrixplaceholder)
    print("")

    rm3, tm3 = cv2.calibrateHandEye(robotrvecs,robottvecs, rvecs, tvecs, method=cv2.CALIB_HAND_EYE_HORAUD)

    homogenousmatrixplaceholder = np.identity(4)
    homogenousmatrixplaceholder[0:3, 0:3] = rm3
    homogenousmatrixplaceholder[0:3, 3] = tm3.ravel()
    print("Horaud Calibration Matrix (3):")
    print(homogenousmatrixplaceholder)
    print("")

    rm4, tm4 = cv2.calibrateHandEye(robotrvecs,robottvecs
                                    , rvecs, tvecs, method=cv2.CALIB_HAND_EYE_DANIILIDIS)

    homogenousmatrixplaceholder = np.identity(4)
    homogenousmatrixplaceholder[0:3, 0:3] = rm4
    homogenousmatrixplaceholder[0:3, 3] = tm4.ravel()
    print("Daniilidis Calibration Matrix (4):")
    print(homogenousmatrixplaceholder)
    print("")

    calibrationchoice = int(input("Choose Calibration method (1-4): "))
    if (calibrationchoice == 1):
        print("Park-Martin Method")
        cTee = np.identity(4)
        cTee[0:3, 0:3] = rm1
        cTee[0:3, 3] = tm1.ravel()
        print(cTee)

    elif (calibrationchoice == 2):
        print("Tsai Method")
        cTee = np.identity(4)
        cTee[0:3, 0:3] = rm2
        cTee[0:3, 3] = tm2.ravel()
        print(cTee)

    elif (calibrationchoice == 3):
        print("Horaud Method")
        cTee = np.identity(4)
        cTee[0:3, 0:3] = rm3
        cTee[0:3, 3] = tm3.ravel()
        print(cTee)

    elif (calibrationchoice == 4):
        print("Daniilidis Method")
        cTee = np.identity(4)
        cTee[0:3, 0:3] = rm4
        cTee[0:3, 3] = tm4.ravel()

    else:
        # default park-martin
        print("Default Park-Martin Method")
        cTee = np.identity(4)
        cTee[0:3, 0:3] = rm1
        cTee[0:3, 3] = tm1.ravel()
        print(cTee)

    print("")

    print("Image Distortion Parameters:")
    print(dist)
    print("")

    print("Camera Intrinsic Matrix:")
    print(mtx)
    print("")

    print("Optimal Camera Intrinsic Matrix")
    print(newcameramtx)
    print("")
