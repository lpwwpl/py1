#-*- coding:utf-8 -*-
# @time: 
# @author:张新新
# @email: 1262981714@qq.com
from board import board as board
import cv2.aruco as aruco
import cv2
import numpy as np
class arucoboard(board.board):
    def __init__(self,configFile):
        self.getParameter(configFile)

    def getParameter(self,configFile):
        fs = cv2.FileStorage(configFile,cv2.FileStorage_READ)
        self.marker_X = int(fs.getNode("marker_X").real())

        self.marker_Y = int(fs.getNode("marker_Y").real())
        self.markerSeparation = fs.getNode("markerSeparation").real()
        self.tag_size = fs.getNode("tag_size").real()
        self.dictionary = aruco.Dictionary_get(aruco.DICT_6X6_100)
        self.board = aruco.GridBoard_create(self.marker_X, self.marker_Y, self.tag_size, self.markerSeparation, self.dictionary, 0)
        self.parameters = aruco.DetectorParameters_create()

    def GetBoardAllPoints(self):
        objpoint = np.array(self.board.objPoints).reshape(-1,3)
        return objpoint[:,:2]
    def getObjImgPointList(self,image,verbose=0):
        img_corners, ids, rejectedImgPoints = aruco.detectMarkers(image, self.dictionary, parameters=self.parameters)
        image_points = np.empty([4*len(img_corners),2])
        obj_points = np.empty([4*len(img_corners),2])
        obj_ids = self.board.ids
        obj_corners = self.board.objPoints
        for j in range(ids.shape[0]):
            for i in range(len(obj_ids)):
                if ids[j,0]==obj_ids[i]:
                    break
            image_points[4*j:4*(j+1),:] = img_corners[j][:,:]
            obj_points[4*j:4*(j+1),:] = obj_corners[i][:,:2]
        if verbose==1:
            aruco.drawDetectedMarkers(image, img_corners, ids, (0, 255, 0))
            cv2.namedWindow("Marker Detection")
            cv2.imshow("Marker Detection", image)
            cv2.waitKey(0)
            cv2.destroyAllWindows()
        return image_points,obj_points











