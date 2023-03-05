import numpy as np
import cv2
from UtilSet import *
import math
import os
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

class HandEyeCalibration:

    def __init__(self):
        # input variables for handeye calibration
        self.R_gripper2base = []
        self.t_gripper2base = []
        self.R_target2cam = []
        self.t_target2cam = []
        self.R_cam2gripper = []
        self.t_cam2gripper = []

        self.AlgorithmTest = False

    def captureHandEyeInputs(self, robotXYZABC, camRVec, camTVec):
        # prepare Gripper2Base inputs
        hmRobot = HMUtil.convertXYZABCtoHMDeg(robotXYZABC)
        self.R_gripper2base.append(hmRobot[0:3, 0:3])
        self.t_gripper2base.append(hmRobot[0:3, 3])

        # prepare Target2Cam inputs
        camRMatrix = np.zeros(shape=(3, 3))
        cv2.Rodrigues(camRVec, camRMatrix)
        hmCam = HMUtil.makeHM(camRMatrix, camTVec)
        # hmCam = HMUtil.inverseHM(hmCam)
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

def r_t_to_mat(r, t):
    mat = np.zeros((4,4))
    mat[:3,:3] = r
    mat[:3,3] = t.T
    mat[3,3] = 1
    return mat

folder='runs/calibration_collect/temp'
w_size = 8
h_size = 6
CheckerboardSquareSize=0.024
# [[ 0.23565236 -0.96858357 -0.07945958  0.00351234]
#  [ 0.95083089  0.21287888  0.22495153 -0.09916614]
#  [-0.20096909 -0.12856298  0.9711246   0.41279517]
#  [ 0.          0.          0.          1.        ]]
if __name__ == '__main__':
    ##################################
    objpoints = []  # 3d point in real world space
    imgpoints = []  # 2d points in image plane.
    objp = np.zeros((w_size * h_size, 3), np.float32)
    objp[:, :2] = np.mgrid[0:w_size, 0:h_size].T.reshape(-1, 2)*CheckerboardSquareSize
    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
    T_cam_obj_r = []
    T_cam_obj_t = []
    ###########################

    handeye = HandEyeCalibration()

    #############################
    pose_file = open('{}/data_robotxyzrpy.txt'.format(folder), 'r')
    cali_file = open('{}/data_cali.txt'.format(folder), )
    count1 = len(pose_file.readlines())
    count2 = len(cali_file.readlines())
    pose_file.close()
    cali_file.close()


    if(count1 != count2):
        pass
    else:
        pose_file = open('{}/data_robotxyzrpy.txt'.format(folder), 'r')
        cali_file = open('{}/data_cali.txt'.format(folder), )
        for fname in cali_file.readlines():
            fname = fname.replace('\n', '')
            fname_glob = os.path.join(os.path.dirname(__file__),  fname)
            fname_glob = fname_glob.replace('\\', '/')
            img = cv2.imread(fname_glob)
            gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

            # Find the chess board corners
            # ret, corners = cv2.findCirclesGrid(gray, (7, 7), None)
            ret, corners = cv2.findChessboardCorners(gray, (w_size, h_size), None, cv2.CALIB_CB_ADAPTIVE_THRESH)
            # If found, add object points, image points (after refining them)
            if ret == True:
                # append the set of object points like (0,0,0), (1,0,0), (2,0,0), ...
                objpoints.append(objp)
                # refine corners coordinates
                corners2 = cv2.cornerSubPix(gray, corners, (1, 1), (-1, -1), criteria)
                imgpoints.append(corners2)
                cv2.drawChessboardCorners(img, (w_size, h_size), corners2, ret)
                cv2.imshow('Images', img)
                cv2.waitKey(1)

        cali_file.close()
        ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(objpoints, imgpoints, gray.shape[::-1], None, None)
        # T_cam_obj_r.append(rvecs)
        # T_cam_obj_t.append(tvecs)
        # for j in range(len(rvecs)):
        #     r, _ = cv2.Rodrigues(rvecs[j])
        #     t = tvecs[j].reshape(-1)
        #     mat = r_t_to_mat(r, t)
        #     T_cam_obj_r.append(rvecs)
        try:
            index = 0
            while True:
                text_line = pose_file.readline()
                if text_line:
                    worlds = text_line.split(',')
                    robotXYZABC = []
                    for c in worlds:
                        robotXYZABC.append(float(c))
                    handeye.captureHandEyeInputs(robotXYZABC, rvecs[index],tvecs[index])
                    index = index + 1
                else:
                    break
        finally:
            pose_file.close()

        handeye.getHandEyeResultMatrixUsingOpenCV()

        cam3DTestPoints = []
        robot3DTestPoints = []
        cam3DTestPoints.append([-0.10259, 0.07283, 0.40900])
        cam3DTestPoints.append([0.14604, 0.00431, 0.42700])
        cam3DTestPoints.append([-0.00145, 0.10705, 0.31100])
        cam3DTestPoints.append([-0.10259, 0.07283, 0.40900])
        cam3DTestPoints.append([0.14604, 0.00431, 0.42700])
        cam3DTestPoints.append([-0.00145, 0.10705, 0.31100])

        robot3DTestPoints.append([-0.18101, -0.52507, 0.01393])
        robot3DTestPoints.append([0.06137, -0.68306, 0.01546])
        robot3DTestPoints.append([-0.18807, -0.66342, 0.01510])
        robot3DTestPoints.append([-0.18101, -0.52507, 0.01393])
        robot3DTestPoints.append([0.06137, -0.68306, 0.01546])
        robot3DTestPoints.append([-0.18807, -0.66342, 0.01510])
        # result = handeye.calculateTransformMatrixUsing3Points(
        #     np.array(((-0.10259, 0.07283, 0.40900), (0.14604, 0.00431, 0.42700), (-0.00145, 0.10705, 0.31100))),
        #     np.array(((-0.18101, -0.52507, 0.01393), (0.06137, -0.68306, 0.01546), (-0.18807, -0.66342, 0.01510)))
        # )
        # print(result)
        # print(result.shape)
        #
        # camC = np.array(((cam3DTestPoints[0]), (cam3DTestPoints[1]), (cam3DTestPoints[2])))
        # print(camC.shape)
        # robotC = np.array(((robot3DTestPoints[0]), (robot3DTestPoints[1]), (robot3DTestPoints[2])))
        # result = handeye.calculateTransformMatrixUsing3Points(camC, robotC)
        # print(result)
        #
        # result = handeye.calculateTransformMatrix(cam3DTestPoints, robot3DTestPoints)
        # print(result)
        #
        # print(np.dot(np.array([-0.10259, 0.07283, 0.40900, 1]).reshape(1, 4), result[1]))



