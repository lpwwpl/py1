import cv2
import math
import time
import threading

import json
import numpy as np
import pybullet as p
import pybullet_data
from scipy.spatial.transform import Rotation as R

from perception import Camera, CameraIntrinsic, Frame
from ur5 import UR5
from PyQt5 import QtCore
import kdata_lx

def load_scenes():
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    plane = p.loadURDF('plane.urdf', [0, 0, -0.63], [0, 0, 0, 1])
    table0 = p.loadURDF('table/table.urdf', [0, 0.5, -0.63], [0, 0, 0, 1])
    table1 = p.loadURDF('table/table.urdf', [0, -0.5, -0.63], [0, 0, 0, 1])
    bucket = p.loadURDF('tray/tray.urdf', [-0.4, 0.5, 0], [0, 0, 0, 1])

    def rand_distribute(file_name, x_min=-0.2, x_max=0.1, y_min=-0.1, y_max=0.1, z_min=0.2, z_max=0.8, scale=1.0):
        xyz = np.random.uniform([x_min, y_min, z_min], [x_max, y_max, z_max], size=3)
        rpy = np.random.uniform(-np.pi, np.pi, size=3)
        orn = p.getQuaternionFromEuler(rpy)
        object_id = p.loadURDF(file_name, xyz, orn, globalScaling=scale)

        return object_id

    def distribute(file_name, x_min=-0.2, x_max=0.1, y_min=-0.1, y_max=0.1, z_min=0.2, z_max=0.8, scale=1.0):
        xyz = [0,0,0]
        rpy = [0, 0, 0]
        orn = p.getQuaternionFromEuler(rpy)
        object_id = p.loadURDF(file_name, xyz, orn, globalScaling=scale)

        return object_id

    objects_id = []
    for i in range(3):
        objects_id.append(rand_distribute('lego/lego.urdf'))

    for i in range(2):
        objects_id.append(rand_distribute('jenga/jenga.urdf'))

    for i in range(2):
        objects_id.append(rand_distribute('cube_small.urdf'))
    # for i in range(1):
    #     objects_id.append(distribute('./meshes/objects/chessboard.urdf'))

    return objects_id





class DebugAxes(object):
    """
    可视化某个局部坐标系, 红色x轴, 绿色y轴, 蓝色z轴
    """
    def __init__(self):
        self.uids = [-1, -1, -1]

    def update(self, pos, orn):
        """
        Arguments:
        - pos: len=3, position in world frame
        - orn: len=4, quaternion (x, y, z, w), world frame
        """
        pos = np.asarray(pos)
        rot3x3 = R.from_quat(orn).as_matrix()
        axis_x, axis_y, axis_z = rot3x3.T
        self.uids[0] = p.addUserDebugLine(pos, pos + axis_x * 0.05, [1, 0, 0], replaceItemUniqueId=self.uids[0])
        self.uids[1] = p.addUserDebugLine(pos, pos + axis_y * 0.05, [0, 1, 0], replaceItemUniqueId=self.uids[1])
        self.uids[2] = p.addUserDebugLine(pos, pos + axis_z * 0.05, [0, 0, 1], replaceItemUniqueId=self.uids[2])


class Environment(object):
    def __init__(self):
        self.urdf_file = "./urdf/real_arm.urdf"
        self.camera_config = "./setup.json"
        self.stope_ = False
        with open(self.camera_config, "r") as j:
            config = json.load(j)
        self.camera_intrinsic = CameraIntrinsic.from_dict(config["intrinsic"])

        self.update_camera_image_thread = None
        self.update_debug_axes_thread = None
        self.update_arm_thread = None
        self.frame = None


        # self.open()

    def open(self):
        try:
            self.client = p.connect(p.GUI)
            # p.setRealTimeSimulation(1)

            p.setGravity(0, 0, -9.81)
            p.resetDebugVisualizerCamera(1.674, 70, -50.8, [0, 0, 0])

            self.objects = load_scenes()
            self.camera = Camera(self.camera_intrinsic)
            self.arm = UR5(self.urdf_file)

            self.end_axes = DebugAxes()  # 机械臂末端的局部坐标系
            self.camera_axes = DebugAxes()  # 相机坐标系

            init_xyz = [0.08, -0.20, 0.6]
            init_rpy = [0, math.pi / 2., 0]
            self.param_ids = self.setup_target_pose_params(init_xyz, init_rpy)
            # p.stepSimulation()

        except Exception as e:
            print(e)
            kdata_lx.isVirt = False

    def __del__(self):
        p.disconnect(p.GUI)

    def setup_target_pose_params(self,initial_xyz, initial_rpy):
        initial_x, initial_y, initial_z = initial_xyz
        initial_roll, initial_pitch, initial_yaw = initial_rpy

        param_ids = [
            p.addUserDebugParameter('x', -1, 1, initial_x),
            p.addUserDebugParameter('y', -1, 1, initial_y),
            p.addUserDebugParameter('z', 0, 1, initial_z),
            p.addUserDebugParameter('roll', -math.pi, math.pi, initial_roll),
            p.addUserDebugParameter('pitch', -math.pi, math.pi, initial_pitch),
            p.addUserDebugParameter('yaw', -math.pi, math.pi, initial_yaw),
            p.addUserDebugParameter('finger openness', 0, 1, 1)
        ]

        return param_ids

    def read_user_params(self,param_ids):
        return [p.readUserDebugParameter(param_id) for param_id in param_ids]

    def _bind_camera_to_end(self, end_pos, end_orn):
        """设置相机坐标系与末端坐标系的相对位置
        
        Arguments:
        - end_pos: len=3, end effector position
        - end_orn: len=4, end effector orientation, quaternion (x, y, z, w)

        Returns:
        - wcT: shape=(4, 4), transform matrix, represents camera pose in world frame
        """
        relative_offset = [-0.05, 0, 0.1]  # 相机原点相对于末端执行器局部坐标系的偏移量
        end_orn = R.from_quat(end_orn).as_matrix()
        end_x_axis, end_y_axis, end_z_axis = end_orn.T

        wcT = np.eye(4)  # w: world, c: camera, ^w_c T
        wcT[:3, 0] = -end_y_axis  # camera x axis
        wcT[:3, 1] = -end_z_axis  # camera y axis
        wcT[:3, 2] = end_x_axis  # camera z axis
        wcT[:3, 3] = end_orn.dot(relative_offset) + end_pos  # eye position
        return wcT

    def update_debug_axes(self):
        try:
            while True:
                if self.stope_:
                    break
                if not p.isConnected():
                    break
                # update debug axes and camera position
                end_pos, end_orn = self.arm.get_end_state()
                if not p.isConnected():
                    break
                self.end_axes.update(end_pos, end_orn)
                if not p.isConnected():
                    break
                wcT = self._bind_camera_to_end(end_pos, end_orn)
                if not p.isConnected():
                    break
                self.camera_axes.update(
                    pos=wcT[:3, 3],
                    orn=R.from_matrix(wcT[:3, :3]).as_quat()
                )

        except Exception as e:
            self.stop_manual_control()
            print(e)
            kdata_lx.isVirt = False

    def capture(self):
        if self.frame:
            kdata_lx.colorImg = self.frame.color_image()
            kdata_lx.depthImg = self.frame.depth_image()
            self.extrinsic = self.frame.extrinsic

    def update_camera_image(self):
        try:
            while True:
                if self.stope_:
                    break
                if not p.isConnected():
                    break
                end_pos, end_orn = self.arm.get_end_state()
                if not p.isConnected():
                    break
                wcT = self._bind_camera_to_end(end_pos, end_orn)
                cwT = np.linalg.inv(wcT)
                if not p.isConnected():
                    break
                self.frame = self.camera.render(cwT)
                assert isinstance(self.frame, Frame)

                if not p.isConnected():
                    break
                rgb = self.frame.color_image()  # 这里以显示rgb图像为例, frame还包含了深度图, 也可以转化为点云
                bgr = np.ascontiguousarray(rgb[:, :, ::-1])  # flip the rgb channel
                # cv2.imshow("image", bgr)
                # key = cv2.waitKey(1)
                time.sleep(0.02)
        except Exception as e:
            print(e)
            self.stop_manual_control()
            kdata_lx.isVirt = False

    def stop_manual_control(self):
        # self.client.close()
        self.stope_ = True
        if self.update_camera_image_thread and self.update_camera_image_thread.is_alive():
            self.update_camera_image_thread.join()
        if self.update_camera_image_thread and self.update_camera_image_thread.is_alive():
            self.update_debug_axes_thread.join()
        kdata_lx.isVirt = False




    def get_current_tcp(self):
        end_pos, end_orn= self.arm.get_end_state()
        pose = [end_pos[0],end_pos[1],end_pos[2],end_orn[0],end_orn[1],end_orn[2]]
        pose = np.asarray(pose)
        print("cur:{}".format(pose))
        return pose

    def gripper(self,openess):
        self.arm.control_gripper(openess)

    def move_old_ver(self,target_pose,acc=None,vel=0.01):
        # xyz = [target_pose[0],target_pose[1],target_pose[2]]
        # xyz = np.asarray(xyz)
        # rpy = [target_pose[3],target_pose[4],target_pose[5]]
        # rpy = np.asarray(rpy)
        # pose = [xyz,rpy]
        # self.movep(pose,vel)
        # print("target"+target_pose)
        # target_pose = self.read_user_params(self.param_ids)  # [x, y, z, roll, pitch, yaw, finger openness]
        self.arm.move_to(target_pose[:3],
                        p.getQuaternionFromEuler(target_pose[3:6]))
        # self.arm.control_gripper(target_pose[-1])

    def is_static(self):
        """Return true if objects are no longer moving."""
        v = [np.linalg.norm(p.getBaseVelocity(i)[0])
             for i in self.obj_ids['rigid']]
        return all(np.array(v) < 5e-3)

    # while not self.is_static:
    #   p.stepSimulation()




    def movep(self,target_pose,acc=None,vel=0.01):
        self.arm.move_to(target_pose[:3],p.getQuaternionFromEuler(target_pose[3:6]))
            # self.arm.movep(target_pose[:3],p.getQuaternionFromEuler(target_pose[3:6]),vel)


    def start_manual_control(self):
        self.stope_ = False
        self.open()
        # thread for updatig debug axes
        if not self.update_debug_axes_thread or not self.update_debug_axes_thread.is_alive():
            self.update_debug_axes_thread = threading.Thread(
                target=self.update_debug_axes)
            # self.update_debug_axes_thread.setDaemon(True)
            self.update_debug_axes_thread.start()

        # thread for updating camera image
        if not self.update_camera_image_thread or not self.update_camera_image_thread.is_alive():
            self.update_camera_image_thread = threading.Thread(
                target=self.update_camera_image)
            # self.update_camera_image_thread.setDaemon(True)
            self.update_camera_image_thread.start()

        if not self.update_arm_thread or not self.update_arm_thread.is_alive():
            self.update_arm_thread = threading.Thread(
                target=self.update_arm)
            # self.update_arm_thread.setDaemon(True)
            self.update_arm_thread.start()

        kdata_lx.isVirt = True

    def update_arm(self):
        try:
            while True:
                if self.stope_:
                    break
                if not p.isConnected():
                    break
                p.stepSimulation()
                # target_pose = read_user_params(self.param_ids)  # [x, y, z, roll, pitch, yaw, finger openness]
                # if not p.isConnected():
                #     break
                # self.arm.move_to(target_pose[:3],
                #                 p.getQuaternionFromEuler(target_pose[3:6]))
                # if not p.isConnected():
                #     break
                # self.arm.control_gripper(target_pose[-1])
                time.sleep(0.01)

        except Exception as e:
            print(e)
            # p.setRealTimeSimulation(0)
            self.stop_manual_control()

# if __name__ == "__main__":
#     env = Environment()
#     time.sleep(2)
#
#     print("[INFO] Start manual control!")
#     env.start_manual_control()

