
from UtilSet import *
from PyQt5 import QtCore, QtGui, QtWidgets
from PyQt5.QtWidgets import *
from PyQt5.QtGui import *
from PyQt5.QtCore import *
from ctypes import *
timestamp = None
from ThreeDSurfaceGraphWindow import ThreeDSurfaceGraphWindow
# import halcon as ha

class Kawasaki_calibration(QWidget):

    def __init__(self, parent=None):
        QtWidgets.QWidget.__init__(self, parent)
        self.initUI()
        # self.core.start()

    def capture(self): # real signature unknown; restored from __doc__
        self.core.suction_process()

    def reset(self):
        pass

    def initUI(self):
        self.grid = QtWidgets.QGridLayout()
        self.setLayout(self.grid)

        self.labelPics = QLabel("Pics:")
        self.labelPos = QLabel("Pose:")
        self.lineEditPics = QLineEdit()
        self.lineEditPics.setMinimumWidth(960)
        self.lineEditPos = QLineEdit()
        self.lineEditPos.setMinimumWidth(960)
        self.btnSetting = QPushButton("Set")

        self.grid.setContentsMargins(0, 0, 0, 0)
        self.grid.addWidget(self.labelPics, 0, 1, 1, 1)
        self.grid.addWidget(self.lineEditPics, 0, 2, 1, 4)
        self.grid.addWidget(self.labelPos, 1, 1, 1, 1)
        self.grid.addWidget(self.lineEditPos, 1, 2, 1, 4)
        self.grid.addWidget(self.btnSetting, 2, 2, 1, 1)
        self.btnSetting.clicked.connect(self.set)
        self.setSizePolicy(QSizePolicy.Fixed,QSizePolicy.Fixed)

    def set(self):
        picsfileOrPath = self.lineEditPics.text()
        posesfileOrPath = self.lineEditPos.text()

        picsFile = QFile(picsfileOrPath)
        poseFile = QFile(posesfileOrPath)
        if not picsFile.exists() or not poseFile.exists():
            return

        # picsPath

    def closeEvent(self, event):
        self.close()

    def showImage(self):
        pass
        # kdata_lx.sem.tryAcquire()

# class halconpy():
#     def __init__(self):
#         pass
#
#     def calibration_ha_program(self):
#         pass
#         # engine = ha.HDevEngine()
#         # engine.add_procedure_path('E:/Program Files/MVTec/HALCON-20.11-Steady/procedures')
#         # engine.add_procedure_path('D:/halcon_20210721')
#         #
#         # aug=engine.get_procedure_names()
#         # print(aug)
#         #
#         # program = ha.HDevProgram('D:/halcon_20210721/calibration_eyeinhand.hdev')
#         # aug=program.get_local_procedure_names()
#         # print(aug)
#         # aug = ha.HDevProcedure.load_local(program, 'calibration')
#         # aug_call = ha.HDevProcedureCall(aug)
#         # aug_call.execute()
#
#         # self.procedure=ha.HDevProcedure()
#         # self.procedure = self.procedure.load_external('calibration')
#         #
#         # aug_call = ha.HDevProcedureCall(self.procedure)
#         # aug_call.set_input_control_param_by_name('WindowHandle', None)
#         # # aug_call.set_input_control_param_by_name('CamCalibError', None)
#         # # aug_call.set_input_control_param_by_name('Errors', None)
#         # aug_call.execute()
#
#     def calibration_hamodule(self):
#
#         engine = ha.HDevEngine()
#         engine.add_procedure_path('E:/Program Files/MVTec/HALCON-20.11-Steady/procedures')
#
#
#         ImageNameStart = 'D:/halcon_202107271/images/Color_'
#         DataNameStart = 'D:/halcon_202107271/result/movingcam_'
#         NumImages = 8
#
#         Image = ha.read_image(ImageNameStart + '00')
#
#         # ha.close_window()  dev_close_window ()
#         width, height = ha.get_image_size_s(Image)
#         WindowHandle = ha.open_window(0, 0, width, height, father_window=0 , mode='visible',machine='')
#         ha.set_line_width(WindowHandle,2)
#         ha.set_draw(WindowHandle,'margin')
#         ha.disp_obj(Image,WindowHandle)
#         # set_display_font (WindowHandle, 14, 'mono', 'true', 'false')
#         ParamName = ['color_0', 'color_1', 'color_2', 'color_3', 'color_4', 'color_5', 'color_6', 'alpha_6']
#         ParamValue = ['red', 'green', 'blue', 'red', 'green', 'blue', 'white', 0.7]
#         Labels = ha.tuple_gen_const(7, '')
#         Labels[0] = 'Robot\'s Tool'
#         Labels[3] = 'Robot\'s Base'
#         Instructions=['','','']
#         Instructions[0] = 'Rotate: Left button'
#         Instructions[1] = 'Zoom:   Shift + left button'
#         Instructions[2] = 'Move:   Ctrl  + left button'
#         ArrowThickness = 0.005
#         ArrowLength = 0.05
#         procedure=ha.HDevProcedure()
#         procedure = procedure.load_external('gen_robot_tool_and_base_object_model_3d')
#
#         aug_call = ha.HDevProcedureCall(procedure)
#         aug_call.set_input_control_param_by_name('ArrowThickness', ArrowThickness)
#         aug_call.set_input_control_param_by_name('ArrowLength', ArrowLength)
#         aug_call.execute()
#         OM3DToolOrigin = aug_call.get_output_control_param_by_name('OM3DToolOrigin')
#         OM3DBase = aug_call.get_output_control_param_by_name('OM3DBase')
#
#         CalTabFile = 'D:\\halcon_20210721\\caltab.descr'
#         StartCamParam = ha.read_cam_par('D:\\halcon_20210721\\movingcam_start_campar.dat')
#         CalibDataID = ha.create_calib_data('hand_eye_moving_cam', 1, 1)
#         ha.set_calib_data_cam_param(CalibDataID, 0, [], StartCamParam)
#         ha.set_calib_data_calib_object(CalibDataID, 0, CalTabFile)
#         ha.set_calib_data(CalibDataID, 'model', 'general', 'optimization_method', 'nonlinear')
#         # procedure = procedure.load_external('disp_message')
#         # aug_call = ha.HDevProcedureCall(procedure)
#         # aug_call.set_input_control_param_by_name("WindowHandle",WindowHandle)
#         # aug_call.set_input_control_param_by_name("String", 'The calibration data model was created')
#         # aug_call.set_input_control_param_by_name("CoordSystem", 'window')
#         # aug_call.set_input_control_param_by_name("Row", 12)
#         # aug_call.set_input_control_param_by_name("Column", 12)
#         # aug_call.set_input_control_param_by_name("Color", 'black')
#         # aug_call.set_input_control_param_by_name("Box", 'true')
#         # aug_call.execute()
#         WindowHandleR = ha.open_window(0, width + 10, width, height,  father_window=0 , mode='visible',machine='')
#         # ha.disp_message(WindowHandle, 'The calibration data model was created', 'window', 12, 12, 'black', 'true')
#         for I in range(NumImages):
#             # ha.set_window(WindowHandle)
#             # ha.clear_window()
#             Image=ha.read_image("{}{:0>2d}".format(ImageNameStart,I))
#             ha.find_calib_object (Image, CalibDataID, 0, 0, I, [], [])
#             Caltab=ha.get_calib_data_observ_contours ( CalibDataID, 'caltab', 0, 0, I)
#             # procedure = procedure.load_external('get_calib_data_observ_contours')
#             # aug_call = ha.HDevProcedureCall(procedure)
#             # aug_call.set_input_control_param_by_name("CalibDataID",CalibDataID)
#             # aug_call.set_input_control_param_by_name("CountourName", 'caltab')
#             # aug_call.set_input_control_param_by_name("CameraIdx", 0)
#             # aug_call.set_input_control_param_by_name("CalibObjIdx", 0)
#             # aug_call.set_input_control_param_by_name("CalibObjPoseIdx", I)
#             # aug_call.execute()
#             # Caltab = aug_call.get_output_control_param_by_name('Countours')
#             RCoord, CCoord, Index, PoseForCalibrationPlate = ha.get_calib_data_observ_points(CalibDataID, 0, 0, I)
#             # procedure = procedure.load_external('get_calib_data_observ_points')
#             # aug_call = ha.HDevProcedureCall(procedure)
#             # aug_call.set_input_control_param_by_name("CalibDataID",CalibDataID)
#             # aug_call.set_input_control_param_by_name("CameraIdx", 0)
#             # aug_call.set_input_control_param_by_name("CalibObjIdx", 0)
#             # aug_call.set_input_control_param_by_name("CalibObjPoseIdx", I)
#             # aug_call.execute()
#             # RCoord = aug_call.get_output_control_param_by_name('Row')
#             # CCoord = aug_call.get_output_control_param_by_name('Column')
#             # Index = aug_call.get_output_control_param_by_name('Index')
#             # Pose = aug_call.get_output_control_param_by_name('Pose')
#             # ha.set_window(WindowHandle)
#             # ha.clear_window()
#
#             # ha.set_color(WindowHandle,'green')
#             # ha.disp_obj(Image,WindowHandle)
#             # ha.disp_obj(Caltab,WindowHandle)
#             # ha.set_color(WindowHandle,'yellow')
#             # ha.disp_cross(WindowHandle, RCoord, CCoord, 6, 0)
#             # ha.set_colored (WindowHandle,3)
#             # procedure = procedure.load_external('disp_3d_coord_system')
#             # aug_call = ha.HDevProcedureCall(procedure)
#             # aug_call.set_input_control_param_by_name("WindowHandle",WindowHandle)
#             # aug_call.set_input_control_param_by_name("CamParam", StartCamParam)
#             # aug_call.set_input_control_param_by_name("Pose", PoseForCalibrationPlate)
#             # aug_call.set_input_control_param_by_name("CoordAxesLength", 0.01)
#             # aug_call.execute()
#
#             pose_file = "{}{:0>2d}.dat".format('D:\\halcon_20210721\\data_eyeinhand\\movingcam_robot_pose_', I)
#             ToolInBasePose = ha.read_pose(pose_file)
#             if I == 0:
#                 PoseIn = [-0.006, -0.296, 12, 178, 2, 270, 0]
#             else:
#                 PoseIn = PoseOut
#
#             OM3DTool=ha.rigid_trans_object_model_3d(OM3DToolOrigin, ToolInBasePose)
#
#             ha.set_system('use_window_thread','true')
#             procedure = procedure.load_external('visualize_object_model_3d')
#             aug_call = ha.HDevProcedureCall(procedure)
#             aug_call.set_input_control_param_by_name("WindowHandle",WindowHandleR)
#             aug_call.set_input_control_param_by_name("ObjectModel3D", OM3DTool + OM3DBase)
#             aug_call.set_input_control_param_by_name("CamParam", [])
#             aug_call.set_input_control_param_by_name("PoseIn", PoseIn)
#             aug_call.set_input_control_param_by_name("GenParamName", ParamName)
#             aug_call.set_input_control_param_by_name("GenParamValue", ParamValue)
#             aug_call.set_input_control_param_by_name("Title", 'Position of robot tool coordinate system in robot base coordinate system')
#             aug_call.set_input_control_param_by_name("Label",Labels)
#             aug_call.set_input_control_param_by_name("Information", Instructions)
#             aug_call.execute()
#             PoseOut = aug_call.get_output_control_param_by_name('PosOut')
#             # PoseOut = ha.visualize_object_model_3d(WindowHandleR, [OM3DTool, OM3DBase], PoseIn, ParamName, ParamValue,
#             #                           'Position of robot tool coordinate system in robot base coordinate system',
#             #                           Labels, Instructions)
#
#             ha.set_calib_data(CalibDataID, 'tool', I, 'tool_in_base_pose',ToolInBasePose)
#
#             # procedure = procedure.load_external('disp_3d_coord_system')
#             # aug_call = ha.HDevProcedureCall(procedure)
#             # aug_call.set_input_control_param_by_name("WindowHandle",WindowHandleR)
#             # aug_call.set_input_control_param_by_name("CamParam", StartCamParam)
#             # aug_call.set_input_control_param_by_name("Pose", PoseForCalibrationPlate)
#             # aug_call.set_input_control_param_by_name("CoordAxesLength", 0.01)
#             # aug_call.execute()
#             # ha.disp_3d_coord_system(WindowHandle, StartCamParam, PoseForCalibrationPlate, 0.01)
#             # ha.disp_obj(Image,WindowHandleR)
#
#         # dev_set_window(WindowHandleR)
#         # dev_close_window()
#         Warnings = ha.check_hand_eye_calibration_input_poses(CalibDataID, 0.05, 0.005)
#         Errors = ha.calibrate_hand_eye(CalibDataID)
#         CamCalibError = ha.get_calib_data(CalibDataID, 'model', 'general', 'camera_calib_error')
#
#         CamParam = ha.get_calib_data(CalibDataID, 'camera', 0, 'params')
#         ToolInCamPose = ha.get_calib_data(CalibDataID, 'camera', 0, 'tool_in_cam_pose')
#         CalObjInBasePose=ha.get_calib_data(CalibDataID, 'calib_obj', 0, 'obj_in_base_pose')
#         PlaneInBasePose = ha.set_origin_pose(CalObjInBasePose, 0, 0, 0.005)
#         try:
#             ha.write_pose(ToolInCamPose, DataNameStart + 'final_pose_cam_tool.dat')
#             ha.write_pose(CalObjInBasePose, DataNameStart + 'final_pose_base_calplate.dat')
#             ha.write_pose(PlaneInBasePose, DataNameStart + 'final_pose_base_plane.dat')
#         except Exception as e:
#             print(e)
#
#     def calibration(self):
#         # with calibration images and data files
#         ImageNameStart = 'D:/halcon_20210721/images/Color_'
#         DataNameStart = 'D:/halcon_202107271/result/movingcam_'
#         NumImages = 8
#         path = ImageNameStart + '00.png'
#         cpath=c_char_p(bytes(str(path), 'utf-8'))
#
#         value=c_long(0)
#         image=pointer(value)
#         ha.read_image(byref(image),cpath)
#         cheight=c_long(0)
#         cwidth=c_long(0)
#         ha.get_image_size (image,byref(cheight), byref(cwidth))
#         height=cheight.value
#         width=cwidth.value
#         # dll.tuple_gen_const(7, '', Labels)
#         # Labels[0] = 'Robot\'s Tool'
#         # Labels[3] = 'Robot\'s Base'
#         # dev_close_window()
#         # get_image_size(Image, Width, Height)
#         # dev_open_window(0, 0, Width, Height, 'black', WindowHandle)
#         # dev_set_line_width(2)
#         # dev_set_draw('margin')

if __name__ == "__main__":
    # test = halconpy()
    # test.calibration_hamodule()
    app = QApplication(sys.argv)
    kawasaki = Kawasaki_calibration()
    kawasaki.show()
    #
    sys.exit(app.exec())