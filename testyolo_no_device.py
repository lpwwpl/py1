import os
os.environ['QT_API'] = 'pyside'
os.environ["FORCE_CPU"] = 'True'
import cv2
import argparse
import time
import cv2 as cv
import torch
from utils.torch_utils import select_device, load_classifier, time_sync
from utils.general import check_img_size, check_requirements, check_imshow, colorstr, non_max_suppression, \
    apply_classifier, scale_coords, xyxy2xywh, strip_optimizer, set_logging, increment_path, save_one_box
from utils.plots import colors, plot_one_box
from models.experimental import attempt_load
# import open3d as o3d
from PyQt5 import QtCore, QtGui, QtWidgets
from PyQt5.QtWidgets import *
from PyQt5.QtGui import *
from PyQt5.QtCore import *
from UtilSet import *
# from Kawasaki_Control import Kawasaki_Control
from Kawasaki_Control import Kawasaki_cntl_diag
from ThreeDSurfaceGraphWindow import ThreeDSurfaceGraphWindow
# from PySide2 import QtCore, QtGui, QtWidgets
# from PySide2.QtWidgets import *
# from PySide2.QtGui import *
# from PySide2.QtCore import *

# from traits.api import HasTraits, Instance, on_trait_change
# from traitsui.api import View, Item,Group
# from mayavi.core.ui.api import MayaviScene, MlabSceneModel, \
#         SceneEditor

from sklearn.decomposition import PCA
from scipy.spatial.transform import Rotation as R
import scipy.cluster.hierarchy as sch
import pdb
import threading
import kdata_lx
from environment import Environment

# from Robot import *
# from Part import *
# from FreeCAD import *
# import FreeCAD as App

pca = PCA()
currentSpeed = "300"
currentAccuracy = "300"
timestamp = None


# def Fun_Tool2Base_Shooting():
#     Tool2Base_Shooting = [0.10767868646835144*1000, -0.3787937230921638*1000, 0.15660759*1000, 0.009104541870731944,
#                           3.130781700067779, 0.05941563583525477]
#     return Tool2Base_Shooting



def letterbox(im, new_shape=(640, 640), color=(114, 114, 114), auto=True, scaleFill=False, scaleup=True, stride=32):
    # Resize and pad image while meeting stride-multiple constraints
    shape = im.shape[:2]  # current shape [height, width]
    if isinstance(new_shape, int):
        new_shape = (new_shape, new_shape)

    # Scale ratio (new / old)
    r = min(new_shape[0] / shape[0], new_shape[1] / shape[1])
    if not scaleup:  # only scale down, do not scale up (for better val mAP)
        r = min(r, 1.0)

    # Compute padding
    ratio = r, r  # width, height ratios
    new_unpad = int(round(shape[1] * r)), int(round(shape[0] * r))
    dw, dh = new_shape[1] - new_unpad[0], new_shape[0] - new_unpad[1]  # wh padding
    if auto:  # minimum rectangle
        dw, dh = np.mod(dw, stride), np.mod(dh, stride)  # wh padding
    elif scaleFill:  # stretch
        dw, dh = 0.0, 0.0
        new_unpad = (new_shape[1], new_shape[0])
        ratio = new_shape[1] / shape[1], new_shape[0] / shape[0]  # width, height ratios

    dw /= 2  # divide padding into 2 sides
    dh /= 2

    if shape[::-1] != new_unpad:  # resize
        im = cv2.resize(im, new_unpad, interpolation=cv2.INTER_LINEAR)
    top, bottom = int(round(dh - 0.1)), int(round(dh + 0.1))
    left, right = int(round(dw - 0.1)), int(round(dw + 0.1))
    im = cv2.copyMakeBorder(im, top, bottom, left, right, cv2.BORDER_CONSTANT, value=color)  # add border
    return im, ratio, (dw, dh)


class KCore(QtCore.QThread):
    pic_signal = pyqtSignal()

    def __init__(self, parent=None):
        super(KCore, self).__init__(parent)
        self.working = True
        #self.cam = CamHandler()
        self.exec_path = executable_path()
        # intriFileName = os.path.abspath(os.path.join(self.exec_path, "picsYolo/test-camera-intrinsics.txt"))
        # self.colorCamIntrinsics = np.loadtxt(intriFileName, dtype=float)

    def __del__(self):
        self.working = False
        #self.cam.stop_cam()

    def bbox2points(self, bbox):
        """
        From bounding box yolo format
        to corner points cv2 rectangle
        """
        x, y, w, h = bbox
        xmin = int(round(x - (w / 2)))
        xmax = int(round(x + (w / 2)))
        ymin = int(round(y - (h / 2)))
        ymax = int(round(y + (h / 2)))
        return xmin, ymin, xmax, ymax

    def point2bbox(self, point):
        x1, y1, x2, y2 = point
        x = (x1 + x2) / 2
        y = (y1 + y2) / 2
        w = (x2 - x1)
        h = (y2 - y1)
        return (x, y, w, h)

    @torch.no_grad()
    def InitYolo(self,
                weights='yolov5s.pt',  # model.pt path(s)
                imgsz=640,  # inference size (pixels)
                device='',  # cuda device, i.e. 0 or 0,1,2,3 or cpu
                view_img=True,  # show results
                half=False,  # use FP16 half-precision inference
                ):
            self.view_img = view_img
            self.augment=False
            self.imgsz = imgsz
            self.device = select_device(device)
            self.half = half&(self.device.type != 'cpu')  # half precision only supported on CUDA
            self.line_thickness=1
            w = weights[0] if isinstance(weights, list) else weights
            self.classify, self.pt, self.onnx = False, w.endswith('.pt'), w.endswith('.onnx')  # inference type
            stride, self.names = 64, [f'class{i}' for i in range(1000)]  # assign defaults
            if self.pt:
                self.model = attempt_load(weights, map_location=self.device)  # load FP32 model
                stride = int(self.model.stride.max())  # model stride
                self.names = self.model.module.names if hasattr(self.model, 'module') else self.model.names  # get class names
                if self.half:
                    self.model.half()  # to FP16
                if self.classify:  # second-stage classifier
                    modelc = load_classifier(name='resnet50', n=2)  # initialize
                    modelc.load_state_dict(torch.load('resnet50.pt', map_location=self.device)['model']).to(self.device).eval()
            elif self.onnx:
                check_requirements(('onnx', 'onnxruntime'))
                import onnxruntime
                session = onnxruntime.InferenceSession(w, None)
            self.imgsz = check_img_size(imgsz, s=stride)  # check image size


    @torch.no_grad()
    def pred(self,source):
        project = 'runs/detect'
        name = 'exp'
        img0 = source
        img = letterbox(img0,  stride=32)[0]
        img = img.transpose((2, 0, 1))[::-1]  # HWC to CHW, BGR to RGB
        img = np.ascontiguousarray(img)
        # Run inference
        if self.pt and self.device.type != 'cpu':
            self.model(torch.zeros(1, 3, self.imgsz, self.imgsz).to(self.device).type_as(next(self.model.parameters())))  # run once
        t0 = time.time()

        if self.pt:
            img = torch.from_numpy(img).to(self.device)
            img = img.half() if self.half else img.float()  # uint8 to fp16/32
        elif self.onnx:
            img = img.astype('float32')
        img /= 255.0  # 0 - 255 to 0.0 - 1.0
        if len(img.shape) == 3:
            img = img[None]  # expand for batch dim
            # Inference
        t1 = time_sync()
        if self.pt:
            # visualize = increment_path(save_dir / Path(path).stem, mkdir=True)
            pred = self.model(img, augment=self.augment, visualize=False)[0]
        elif self.onnx:
            pred = torch.tensor(session.run([session.get_outputs()[0].name], {session.get_inputs()[0].name: img}))

        conf_thres = 0.25
        iou_thres = 0.45
        agnostic_nms = False
        max_det = 1000
        classes = None
        hide_labels = True
        hide_conf = False
        # NMS
        pred = non_max_suppression(pred, conf_thres, iou_thres, classes, agnostic_nms, max_det=max_det)
        t2 = time_sync()
        # Second-stage classifier (optional)
        if self.classify:
            pred = apply_classifier(pred, modelc, img, im0s)
        # Process predictions
        detections = []
        for i, det in enumerate(pred):  # detections per image
            s, im0 = '', img0.copy()
            gn = torch.tensor(im0.shape)[[1, 0, 1, 0]]  # normalization gain whwh
            if len(det):
                # Rescale boxes from img_size to im0 size
                det[:, :4] = scale_coords(img.shape[2:], det[:, :4], im0.shape).round()

                # Write results
                for *xyxy, conf, cls in reversed(det):
                    if self.view_img:  # Add bbox to image
                        c = int(cls)  # integer class
                        # label = None if hide_labels else (names[c] if hide_conf else f'{names[c]} {conf:.2f}')
                        label = None
                        if not hide_labels and not hide_conf:
                            label = f'{self.names[c]} {conf:.2f}'
                        if not hide_labels and hide_conf:
                            label = f'{self.names[c]}'
                        if hide_labels and not hide_conf:
                            label = f'{conf:.2f}'

                        plot_one_box(xyxy, im0, label=label, color=colors(c, True), line_thickness=self.line_thickness)
                        c1, c2 = (int(xyxy[0]), int(xyxy[1])), (int(xyxy[2]-xyxy[0]), int(xyxy[3]-xyxy[1]))
                        detection = (f'{conf:.2f}',self.names[c],c1,c2)
                        detections.append(detection)
            kdata_lx.inferImg = im0
            # if self.view_img:
            #     cv2.imshow("test",im0)
            #     cv2.waitKey(0)

        return detections


    def image_detection(self,cameraIntrinsics,colorImg_uint8,depthImg_uint16):
        # [image_path, frameName, data, colorImg_uint8, depthImg_uint16, cameraIntrinsics, _, _] = get_stream()

        # image_bgr = cv2.imread("/home/jihua/桌面/picsYolo/frame-20210420_095757.color.png")
        # image_depth = cv2.imread("/home/jihua/桌面/picsYolo/frame-20210420_095757.depth.png", -1)
        # cameraIntrinsics = np.array([(6.12692566e+02, 0.00000000e+00, 3.23764923e+02),
        #                              (0.00000000e+00, 6.12443115e+02, 2.33248459e+02),
        #                              (0.00000000e+00, 0.00000000e+00, 1.00000000e+00)])

        # image_bgr = colorImg_uint8
        image_bgr = cv.cvtColor(colorImg_uint8,cv.COLOR_BGR2RGB)
        image_depth = depthImg_uint16
        depth = cv2.split(image_depth)[0] / 10000
        # print("depth", type(depth))
        # 判断输入图像是否为3通道
        if len(image_bgr.shape) == 2:
            image_bgr = np.stack([image_bgr] * 3, axis=-1)
        # 获取原始图像大小
        orig_h, orig_w = image_bgr.shape[:2]


        detections = self.pred(image_bgr)

        new_detections = []
        camPoints_clusters = []
        width = 640
        height = 480
        for detection in detections:
            pred_conf,pred_label, (x, y), (w, h) = detection
            new_x = x / width * orig_w
            new_y = y / height * orig_h
            new_w = w / width * orig_w
            new_h = h / height * orig_h

            # 可以约束一下
            (x1, y1, x2, y2) = self.bbox2points((new_x, new_y, new_w, new_h))
            x1 = x1 if x1 > 0 else 0
            x2 = x2 if x2 < orig_w else orig_w
            y1 = y1 if y1 > 0 else 0
            y2 = y2 if y2 < orig_h else orig_h

            (new_x, new_y, new_w, new_h) = self.point2bbox((x1, y1, x2, y2))
            new_detections.append((pred_label, pred_conf, (new_x, new_y, new_w, new_h)))
            pixX_clusters = int(new_x + new_w / 2)
            pixY_clusters = int(new_y + new_h / 2)
            pixZ_clusters_1 = depth[(pixY_clusters - 10):(pixY_clusters + 10),
                              (pixX_clusters - 10):(pixX_clusters + 10)]
            pixZ_clusters_2 = ((pixZ_clusters_1 < 1) & (pixZ_clusters_1 > 0))
            pixZ_clusters_3 = pixZ_clusters_1 * pixZ_clusters_2
            if np.all(pixZ_clusters_3 == 0):
                continue
            else:
                pixZ_clusters = np.mean(pixZ_clusters_3.ravel()[np.flatnonzero(pixZ_clusters_3)])
                # print(pixZ_clusters_1)
                # print(pixZ_clusters)
                camX_clusters = (pixX_clusters - cameraIntrinsics[0][2]) * pixZ_clusters / cameraIntrinsics[0][0]
                camY_clusters = (pixY_clusters - cameraIntrinsics[1][2]) * pixZ_clusters / cameraIntrinsics[1][1]
                camZ_clusters = pixZ_clusters
                camPoints_clusters.append([camX_clusters, camY_clusters, camZ_clusters, 0, 0, 0])
        if not camPoints_clusters:
            pass
            # image = darknet.draw_boxes(new_detections, image_rgb, class_colors)
            # return image, new_detections, camPoints_clusters
        else:
            camPoints_clusters1 = np.array(camPoints_clusters)
            # print(camPoints_clusters1)
            idex = np.lexsort(camPoints_clusters1.T[:3, :])
            # idex = np.lexsort(-1*camPoints_clusters1[:, 2])

            sorted_camPoints_clusters = camPoints_clusters1[idex, :]
            sorted_camPoints_clusters1 = sorted_camPoints_clusters[~(sorted_camPoints_clusters == 0).all(1)]
            # image = darknet.draw_boxes(new_detections, image_rgb, class_colors)
            # print(camPoints_clusters)
            # return cv2.cvtColor(image, cv2.COLOR_RGB2BGR), new_detections
            # print('trans',trans,'\n euler:', euler)
            # return camPoints_clusters
            # darknet.print_detections(detections, True)
            # cv2.imshow('picture', image)
            # cv2.waitKey()
            Object2Cam = np.array(sorted_camPoints_clusters1)
            image_bgr = cv.cvtColor(colorImg_uint8, cv.COLOR_BGR2RGB)
            for i in range(Object2Cam.shape[0]):
                imageAxisPix = self.calImageAxis(Object2Cam[i], 0.06, cameraIntrinsics)
                cv2.circle(image_bgr,(int(imageAxisPix[0,0]), int(imageAxisPix[0,1])), 5, (255,255,255))
                # cv2.line(image_bgr, (int(imageAxisPix[0, 0]), int(imageAxisPix[0, 1])),
                #          (int(imageAxisPix[1, 0]), int(imageAxisPix[1, 1])), (255, 0, 0))
                # cv2.line(image_bgr, (int(imageAxisPix[0, 0]), int(imageAxisPix[0, 1])),
                #          (int(imageAxisPix[2, 0]), int(imageAxisPix[2, 1])), (0, 255, 0))
                # cv2.line(image_bgr, (int(imageAxisPix[0, 0]), int(imageAxisPix[0, 1])),
                #          (int(imageAxisPix[3, 0]), int(imageAxisPix[3, 1])), (0, 0, 255))
            # colorImg_uint8.write('D:/lpw.jpg')
            kdata_lx.colorImg = image_bgr
            kdata_lx.depthImg = depthImg_uint16


            # kdata_lx.sem.release()
            self.pic_signal.emit()
            return new_detections, list(sorted_camPoints_clusters1[0])

    # Object2Cam
    def calImageAxis(self, Object2Cam, length, cameraIntrinsics):
        centroid = np.asarray([Object2Cam[0],Object2Cam[1],Object2Cam[2]])
        rotation = np.asarray([Object2Cam[3],Object2Cam[4],Object2Cam[5]])
        o = centroid.reshape(3, 1)
        xyz = rotation * length + np.tile(centroid, (3, 1)).T

        oxyz = np.hstack((o, xyz))
        imageAxisPix = np.dot(cameraIntrinsics, oxyz)
        imageAxisPix = imageAxisPix / np.tile(imageAxisPix[2], (3, 1))
        return imageAxisPix[0:2].T

    def client_srv(self,prefix=None):
        #self.cam.getFrame()
        exec_path = executable_path()

        if prefix:
            colorFileName = os.path.abspath(os.path.join(exec_path, "picsYolo/{}.color.png".format(prefix)))
            deptFileName = os.path.abspath(os.path.join(exec_path, "picsYolo/{}.depth.png".format(prefix)))
            color_img = cv.cvtColor(cv.imread(colorFileName, flags=1), cv.COLOR_RGB2BGR)
            depth_img = cv.imread(deptFileName, flags=-1)
        else:
            pass

        cameraIntrinsics = np.array([(6.12692566e+02, 0.00000000e+00, 3.23764923e+02),
                                     (0.00000000e+00, 6.12443115e+02, 2.33248459e+02),
                                     (0.00000000e+00, 0.00000000e+00, 1.00000000e+00)])

        _, Object2Cam = self.image_detection(cameraIntrinsics,color_img,depth_img)

        return Object2Cam


    def Fun_Suction_Grip(self):
        if kdata_lx.isVirt:
            self.env.gripper(0)
        if kdata_lx.isReal:
            self.controlDialog.gripper(0)

    def Fun_Suction_Release(self):
        if kdata_lx.isVirt:
            self.env.gripper(1)
        else:
            self.controlDialog.gripper(1)

    def Fun_Eyeinhand_Shooting(self,flag):
        if flag == False:
            Tool2Base_Shooting = [0.15802303 , -0.32541972 , 0.1903955 , 0.00931757, 3.13076056,
                                  0.05933887]
        else:
            Tool2Base_Shooting = [-0.26879975 , -0.32541972 , 0.1903955 , 0.009104541870731944,
                                  3.130781700067779,
                                  0.05941563583525477]
        return Tool2Base_Shooting

    def text_read(self,):
        # exec_path = executable_path()
        # camFileName = os.path.abspath(os.path.join(exec_path, 'picsYolo/Cam2Tool.txt'))
        # with open(camFileName, "r") as f:
        #    Cam2BaseTxt1 = f.read().replace('[', '').replace(']', '')
        # Cam2BaseTxt2 = Cam2BaseTxt1.splitlines()
        # Cam2BaseTxt = [row.split() for row in Cam2BaseTxt2 if row.strip()]
        # col = len(Cam2BaseTxt[0])
        # row = len(Cam2BaseTxt)
        # Cam2Base = list(range(6))
        # for j in range(col):
        #    Cam2Base[j] = float(Cam2BaseTxt[0][j])
        # Cam2Base_rm = rpy2rm(Cam2Base)
        Cam2Base_rm = [[0.99958932, -0.02117886, 0.01930385, -0.06328652],
                       [0.0221976, 0.99828422, -0.0541838, -0.16513022],
                       [-0.01812318, 0.05459005, 0.99834437, -0.11598357], [0, 0, 0, 1]]
        return Cam2Base_rm

    def Fun_Tool2Base_Suction_Interim(self,Object2Cam):
        # Object2Cam = client_srv()
        Object2Cam_rm = rpy2rm(Object2Cam)

        Cam2Tool_rm = self.text_read()
        Tool2Base = self.get_current_tcp()
        Tool2Base_rm = rv2rm(Tool2Base)
        Object2Base = Tool2Base_rm.dot(Cam2Tool_rm).dot(Object2Cam_rm)
        Tool2Base_Suction_rm = Object2Base
        Tool2Base_Suction = rm2rv(Tool2Base_Suction_rm)
        Move_rm = np.array([[1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 1, -0.05 ], [0, 0, 0, 1]])
        Tool2Base_Interim1_rm = Tool2Base_Suction_rm.dot(Move_rm)
        Tool2Base_Interim1 = rm2rv(Tool2Base_Interim1_rm)
        Tool2Base_Interim2 = Tool2Base_Suction + np.array([0, 0, 0.15 , 0, 0, 0])
        return Tool2Base_Suction, Tool2Base_Interim1, Tool2Base_Interim2

    def Fun_Eyeinhand_Release(self,flag):
        x = 0
        y = 0
        z = 0
        if flag == False:
            Tool2Base_Release = [(-0.16 - 0.12 * x) , (-0.56 + 0.08 * y) , (0.075 + 0.05 * z) ,
                                 0.00909552, 3.13077267,
                                 0.05922008]
        else:
            Tool2Base_Release = [(0.17 + 0.12 * x) , (-0.56 + 0.08 * y) , (0.075 + 0.05 * z) ,
                                 0.00909552,
                                 3.13077267,
                                 0.05922008]
        Tool2Base_Release_Interim1 = Tool2Base_Release + np.array([0, 0, 0.01 , 0, 0, 0])
        Tool2Base_Release_Interim2 = Tool2Base_Release + np.array([0, 0, 0.12 , 0, 0, 0])
        return Tool2Base_Release, Tool2Base_Release_Interim1, Tool2Base_Release_Interim2


    def movej(self,tcp,acc,vel):
        if kdata_lx.isVirt:
            self.env.moveJ(tcp,acc,vel)
        if kdata_lx.isReal:
            self.controlDialog.moveJ(tcp,acc,vel)

    def get_current_tcp(self):
        if kdata_lx.isVirt:
            return self.env.get_current_tcp()
        else:
            return self.controlDialog.get_current_tcp()

    def move_to_tcp(self, target_tcp, tool_acc, tool_vel):
        tool_pos_tolerance = [0.01, 0.01, 0.01, 0.05, 0.05, 0.05]

        actual_pos = self.get_current_tcp()  # target_tcp
        t1 = time.time()
        while not (all([np.abs(actual_pos[j] - target_tcp[j]) < tool_pos_tolerance[j] for j in range(3)])
                   and all(
                    [np.abs(actual_pos[j + 3] - target_tcp[j + 3]) < tool_pos_tolerance[j + 3] for j in range(3)])):
            # [410.72627118678747, -57.835160228166224, 403.55144691507746, -0.2824616264256288, 0.8656988465117846, -0.4132565023628232]
            actual_pos = self.get_current_tcp()
            t2 = time.time()
            if (t2 - t1) > 3:
                return
            time.sleep(0.01)

    def suction_process(self):
        target_tcp_shooting = self.Fun_Eyeinhand_Shooting(False)
        # self.move_to_tcp(target_tcp_shooting, 1.5, 1)
        Object2Cam = self.client_srv()
        if Object2Cam:
            [target_tcp_suction, target_tcp_interim1,
             target_tcp_interim2] = self.Fun_Tool2Base_Suction_Interim(Object2Cam)
            self.move_to_tcp(target_tcp_interim1, 1.5, 1)
            self.move_to_tcp(target_tcp_suction, 0.5, 0.5)
            self.Fun_Suction_Grip()
            self.move_to_tcp(target_tcp_interim2, 0.5, 1)
            [target_tcp_release, target_tcp_interim_1, target_tcp_interim_2] = self.Fun_Eyeinhand_Release(False)
            self.move_to_tcp(target_tcp_interim_2, 1.5, 1)
            self.move_to_tcp(target_tcp_release, 0.5, 0.5)
            self.Fun_Suction_Release()
            time.sleep(1)
            self.move_to_tcp(target_tcp_interim_1, 0.5, 0.5)


    def run(self):
        while self.working == True:
            self.suction_process()
            time.sleep(0.5)



class Kawasaki_yolo(QWidget):

    def __init__(self, opt=None, parent=None):
        QtWidgets.QWidget.__init__(self, parent)
        # super(Kawasaki, self).__init__(parent)
        self.opt = opt
        self.core = KCore()
        self.core.InitYolo(**vars(self.opt))

        self.core.pic_signal.connect(self.showImage)
        self.initUI()
        self.initCaptureList()

        # self.core.start()

    def capture(self): # real signature unknown; restored from __doc__
        self.core.suction_process()

    def Auto(self):
        pass

    def viewItem(self):
        if self.captureList.currentText() == 'None':
            kdata_lx.depthImg = kdata_lx.zero
            kdata_lx.colorImg = kdata_lx.zero
            kdata_lx.inferImg = kdata_lx.zero
            self.showImage()
            return
        self.core.client_srv(self.captureList.currentText())

    def virt_capture(self):
        if kdata_lx.isVirt and self.env:
            ret = self.env.capture()
            filepath = "picsYolo/lpw.color.png"
            deptfilepath = "picsYolo/lpw.depth.png"
            cv.imwrite(filepath, kdata_lx.colorImg)
            depth_mat = np.zeros((480, 640), np.float32)

            for y in range(480):
                for x in range(640):
                    depth_short = kdata_lx.depthImg[y, x] * 10000
                    depth_mat[y, x] = depth_short
            cv.imwrite(deptfilepath, depth_mat)
            self.extrinsic = self.env.extrinsic
            return ret
        return False

    def initCaptureList(self):
        files_path ="./picsYolo"
        strList = ['None']
        for fl in os.listdir(files_path):
            fileInfo = QFileInfo(fl)
            suffix = fileInfo.suffix()
            if suffix == 'png':
                strList.append(fileInfo.baseName())
        strList = set(strList)
        self.captureList.blockSignals(True)
        self.captureList.addItems(strList)
        self.captureList.blockSignals(False)
        try:
            self.captureList.setCurrentText('None')
        except Exception as e:
            print(e)

    def initUI(self):
        self.captureList = QComboBox()
        self.captureList.currentIndexChanged.connect(self.viewItem)
        self.captureList.setMinimumWidth(640)
        self.btnCapture = QPushButton("Capture")
        self.btnAuto = QPushButton("Auto")
        self.btnRealControl = QPushButton("Real")
        self.btnVirtControl = QPushButton("Virt")

        self.grid = QtWidgets.QGridLayout()
        self.setLayout(self.grid)

        self.label1 = QtWidgets.QLabel()
        self.label2 = QtWidgets.QLabel()
        self.label3 = QtWidgets.QLabel()
        self.label1.resize(640, 480)
        self.label2.resize(640, 480)
        self.label3.resize(640, 480)
        self.plot_container_pc1 = ThreeDSurfaceGraphWindow()
        self.plot_container_pc2 = ThreeDSurfaceGraphWindow()

        self.grid.setContentsMargins(0, 0, 0, 0)
        self.env = Environment()
        # self.control = Kawasaki_Control()
        self.controlDialog = Kawasaki_cntl_diag()
        gBox = QGridLayout()
        widgetOpt = QWidget()
        widgetOpt.setLayout(gBox)
        self.btnCapture.setFixedSize(160, 50)
        self.btnAuto.setFixedSize(160, 50)
        self.btnRealControl.setFixedSize(160, 50)
        self.btnVirtControl.setFixedSize(160, 50)
        self.btnVirtControl.clicked.connect(self.virt)
        self.btnRealControl.clicked.connect(self.real)
        gBox.addWidget(self.btnCapture, 0, 0, 1, 1)
        gBox.addWidget(self.btnAuto, 0, 1, 1, 1)
        gBox.addWidget(self.btnRealControl, 0, 2, 1, 1)
        gBox.addWidget(self.btnVirtControl, 0, 3, 1, 1)
        gBox.addWidget(self.captureList, 0, 4, 1, 4)
        self.grid.addWidget(widgetOpt, 0, 1, 1, 2)
        self.grid.addWidget(self.label1, 1, 0)
        self.grid.addWidget(self.label2, 1, 1)
        self.grid.addWidget(self.label3, 1, 2)
        self.grid.addWidget(self.plot_container_pc1, 2, 0)
        self.grid.addWidget(self.plot_container_pc2, 2, 1)

        tmp = cv2.cvtColor(kdata_lx.depthImg.astype(np.uint8), cv2.COLOR_BGR2RGB).astype('uint8')
        qimg = QtGui.QImage(tmp, 640, 480, QtGui.QImage.Format_RGB888)
        img = QtGui.QPixmap.fromImage(qimg)
        self.label1.setPixmap(img)
        self.label2.setPixmap(img)
        self.label3.setPixmap(img)
        self.setSizePolicy(QSizePolicy.Fixed, QSizePolicy.Fixed)

    def virt(self):
        # self.control.hide()
        self.controlDialog.close()
        self.env.start_manual_control()
        kdata_lx.isVirt = True

    def real(self):
        self.controlDialog.show()
        kdata_lx.isReal = True

    def closeEvent(self, event):
        self.core.working = False
        self.core.wait()
        # self.control.close()
        self.controlDialog.close()
        event.accept()

    def showImage(self):
        # kdata_lx.sem.tryAcquire()
        tmp1 = cv2.cvtColor(kdata_lx.colorImg, cv2.COLOR_BGR2RGB).astype('uint8')
        qImg1 = QtGui.QImage(tmp1, 640, 480, QtGui.QImage.Format_RGB888)
        tmp2 = cv2.cvtColor(kdata_lx.depthImg.astype(np.uint8), cv2.COLOR_BGR2RGB).astype('uint8')
        qImg2 = QtGui.QImage(tmp2, 640, 480, QtGui.QImage.Format_RGB888)
        tmp3 = cv2.cvtColor(kdata_lx.inferImg.astype(np.uint8), cv2.COLOR_BGR2RGB).astype('uint8')
        qImg3 = QtGui.QImage(tmp3, 640, 480, QtGui.QImage.Format_RGB888)

        img1 = QtGui.QPixmap.fromImage(qImg1)
        self.label1.setPixmap(img1)

        img2 = QtGui.QPixmap.fromImage(qImg2)
        self.label2.setPixmap(img2)

        img3 = QtGui.QPixmap.fromImage(qImg3)
        self.label3.setPixmap(img3)

        # self.plot_container_pc1.draw_graph(kdata_lx.pc1[0], kdata_lx.pc1[1], kdata_lx.pc1[2], kdata_lx.camPointsColor,kdata_lx.axisPointsColor)
        # self.plot_container_pc2.draw_graph(kdata_lx.pc2[0], kdata_lx.pc2[1], kdata_lx.pc2[2], kdata_lx.camPointsColor_clusters,kdata_lx.axisPointsColor)
        # kdata_lx.sem.release()


def parse_opt():
    # weights = 'yolov5s.pt',  # model.pt path(s)
    # imgsz = 640,  # inference size (pixels)
    # device = '',  # cuda device, i.e. 0 or 0,1,2,3 or cpu
    # view_img = True,  # show results
    # nosave = False,  # do not save images/videos
    # half = False,  # use FP16 half-precision inference
    parser = argparse.ArgumentParser()
    parser.add_argument('--weights', nargs='+', type=str, default='weights/best.pt', help='model.pt path(s)')
    parser.add_argument('--imgsz', '--img', '--img-size', type=int, default=640, help='inference size (pixels)')
    parser.add_argument('--device', default='', help='cuda device, i.e. 0 or 0,1,2,3 or cpu')
    parser.add_argument('--view-img', default=True,action='store_true', help='show results')
    parser.add_argument('--half', action='store_true', help='use FP16 half-precision inference')
    opt = parser.parse_args()
    return opt

if __name__ == "__main__":

    app = QApplication.instance()
    if app is None:
        app = QApplication(sys.argv)
    opt = parse_opt()
    kawasaki = Kawasaki_yolo(opt)
    kawasaki.show()

    sys.exit(app.exec_())