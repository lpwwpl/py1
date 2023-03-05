import cv2
import math
import time
import threading
import gym
import json
import numpy as np
import pybullet as p
import pybullet_data
from scipy.spatial.transform import Rotation as R
from utils import pybullet_utils
from perception import Camera, CameraIntrinsic, Frame
# from ur5 import UR5
from PyQt5 import QtCore
import kdata_lx
from grippers import Suction
from primitives import PickPlace
import cameras
import os
from utils import utils
from UtilSet import *
PLANE_URDF_PATH = 'plane/plane.urdf'
UR5_WORKSPACE_URDF_PATH = 'ur5/workspace.urdf'
UR5_URDF_PATH = 'ur5/ur5.urdf'
def get_random_pose(x_min=-0.1, x_max=0.1, y_min=-0.1, y_max=0.1, scale=1.0):
    xyz = np.random.uniform([x_min, y_min, 0], [x_max, y_max, 0], size=3)
    rpy = np.random.uniform(-np.pi, np.pi, size=3)
    orn = p.getQuaternionFromEuler(rpy)
    return [xyz,orn]

    return object_id

def rand_distribute(file_name, x_min=-0.2, x_max=0.1, y_min=-0.1, y_max=0.1, z_min=0.0, z_max=0.8, scale=1.0):
    xyz = np.random.uniform([x_min, y_min, z_min], [x_max, y_max, z_max], size=3)
    rpy = np.random.uniform(-np.pi, np.pi, size=3)
    orn = p.getQuaternionFromEuler(rpy)
    object_id = p.loadURDF(file_name, xyz, orn, globalScaling=scale)

    return object_id


    # for i in range(2):
    #     objects_id.append(rand_distribute('jenga/jenga.urdf'))
    #
    # for i in range(2):
    #     objects_id.append(rand_distribute('cube_small.urdf'))




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
        self.uids[0] = p.addUserDebugLine(pos, pos + axis_x * 0.15, [1, 0, 0], replaceItemUniqueId=self.uids[0])
        self.uids[1] = p.addUserDebugLine(pos, pos + axis_y * 0.15, [0, 1, 0], replaceItemUniqueId=self.uids[1])
        self.uids[2] = p.addUserDebugLine(pos, pos + axis_z * 0.15, [0, 0, 1], replaceItemUniqueId=self.uids[2])

ASSETS_PATH = './assets/'
class Environment_Pick_Place(gym.Env):
    def __init__(self):
        self.assets_root = ASSETS_PATH
        self.urdf_file = "./assets/ur5/ur5.urdf"
        self.stope_ = False
        self.camera_config = "./setup.json"
        with open(self.camera_config, "r") as j:
            config = json.load(j)
        self.camera_intrinsic = CameraIntrinsic.from_dict(config["intrinsic"])

        self.update_camera_image_thread = None
        self.update_debug_axes_thread = None
        self.update_arm_thread = None

        self.ee_tip = 10  # Link ID of suction cup.
        self.ee_tool_fixed_joint = 9  # Link ID of suction cup.
        self.pick_place = PickPlace()
        # self.agent_cams = cameras.RealSenseD435.CONFIG
        self.pix_size = 0.003125
        self.homej = np.array([-1, -0.5, 0.5, -0.5, -0.5, 0]) * np.pi
        # self.open()
        self.oracle_cams = cameras.Oracle.CONFIG

        self.pix_size = 0.003125
        self.bounds = np.array([[0.25, 0.75], [-0.5, 0.5], [0, 0.3]])
        self.obj_ids = {'fixed': [], 'rigid': [], 'deformable': []}
        self.agent_cams = cameras.RealSenseD435.CONFIG
        self.task = None
        self.isReset = False



    def add_object(self, urdf, pose, category='rigid'):
        """List of (fixed, rigid, or deformable) objects in env."""
        fixed_base = 1 if category == 'fixed' else 0
        obj_id = pybullet_utils.load_urdf(
            p,
            os.path.join(self.assets_root, urdf),
            pose[0],
            pose[1],
            useFixedBase=fixed_base)
        self.obj_ids[category].append(obj_id)
        return obj_id

    # def get_random_pose(self,  obj_size):
    #     """Get random collision-free object pose within workspace bounds."""
    #
    #     # Get erosion size of object in pixels.
    #     max_size = np.sqrt(obj_size[0] ** 2 + obj_size[1] ** 2)
    #     erode_size = int(np.round(max_size / self.pix_size))
    #
    #     _, hmap, obj_mask = self.get_true_image(env)
    #
    #     # Randomly sample an object pose within free-space pixels.
    #     free = np.ones(obj_mask.shape, dtype=np.uint8)
    #     for obj_ids in env.obj_ids.values():
    #         for obj_id in obj_ids:
    #             free[obj_mask == obj_id] = 0
    #     free[0, :], free[:, 0], free[-1, :], free[:, -1] = 0, 0, 0, 0
    #     free = cv2.erode(free, np.ones((erode_size, erode_size), np.uint8))
    #     if np.sum(free) == 0:
    #         return None, None
    #     pix = utils.sample_distribution(np.float32(free))
    #     pos = utils.pix_to_xyz(pix, hmap, self.bounds, self.pix_size)
    #     pos = (pos[0], pos[1], obj_size[2] / 2)
    #     theta = np.random.rand() * 2 * np.pi
    #     rot = utils.eulerXYZ_to_quatXYZW((0, 0, theta))
    #     return pos, rot
    def get_random_size(self, min_x, max_x, min_y, max_y, min_z, max_z):
        """Get random box size."""
        size = np.random.rand(3)
        size[0] = size[0] * (max_x - min_x) + min_x
        size[1] = size[1] * (max_y - min_y) + min_y
        size[2] = size[2] * (max_z - min_z) + min_z
        return tuple(size)

    def get_random_pose(self, env, obj_size):
        """Get random collision-free object pose within workspace bounds."""

        # Get erosion size of object in pixels.
        max_size = np.sqrt(obj_size[0] ** 2 + obj_size[1] ** 2)
        erode_size = int(np.round(max_size / self.pix_size))

        _, hmap, obj_mask = self.get_true_image(env)

        # Randomly sample an object pose within free-space pixels.
        free = np.ones(obj_mask.shape, dtype=np.uint8)
        for obj_ids in env.obj_ids.values():
            for obj_id in obj_ids:
                free[obj_mask == obj_id] = 0
        free[0, :], free[:, 0], free[-1, :], free[:, -1] = 0, 0, 0, 0
        free = cv2.erode(free, np.ones((erode_size, erode_size), np.uint8))
        if np.sum(free) == 0:
            return None, None
        pix = utils.sample_distribution(np.float32(free))
        pos = utils.pix_to_xyz(pix, hmap, self.bounds, self.pix_size)
        pos = (pos[0], pos[1], obj_size[2] / 2)
        theta = np.random.rand() * 2 * np.pi
        rot = utils.eulerXYZ_to_quatXYZW((0, 0, theta))
        return pos, rot

    def reset(self):
        # super().reset(env)

        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        print(pybullet_data.getDataPath())
        # plane = p.loadURDF('plane.urdf', [0, 0, -0.63], [0, 0, 0, 1])
        # table0 = p.loadURDF('table/table.urdf', [0, 0.5, -0.63], [0, 0, 0, 1])
        # table1 = p.loadURDF('table/table.urdf', [0, -0.5, -0.63], [0, 0, 0, 1])
        # bucket = p.loadURDF('tray/tray.urdf', [-0.4, 0.5, 0], [0, 0, 0, 1])
        # Add blocks.
        block_size = (0.04, 0.04, 0.04)
        block_urdf = 'block/block.urdf'

        for i in range(6):
            self.add_object(block_urdf,get_random_pose())

        zone_size = self.get_random_size(0.05, 0.3, 0.05, 0.3, 0.05, 0.05)
        zone_pose = self.get_random_pose(zone_size)
        container_template = 'container/container-template.urdf'
        half = np.float32(zone_size) / 2
        replace = {'DIM': zone_size, 'HALF': half}

        container_urdf = self.fill_template(container_template, replace)
        self.add_object(container_urdf, zone_pose, 'fixed')
        os.remove(container_urdf)
        # for i in range(6):
        #     self.add_object('stacking/stand.urdf',get_random_pose())
        # chess_old='chessboard.urdf'
        # chess_id = p.loadURDF(chess_old, [0, 0.3, 0.2], [0, 0, 0, 1])
        # chess_urdf = 'chess/chessboard.urdf'
        # pose_chess = [[0.1, 0.0, 0.32], [0, 0, 0,1]]
        # chess_id=self.add_object(chess_urdf, pose_chess)


        # for i in range(2):
        #     self.add_object('cube_small.urdf', get_random_pose())

    def get_ee_pose(self):
        return p.getLinkState(self.ur5, self.ee_tip)[0:2]

    def set_task(self, task):
        # task.set_assets_root(self.assets_root)
        self.task = task

    def open(self):
        try:
            self.client = p.connect(p.GUI)
            # p.setRealTimeSimulation(1)
            p.resetSimulation(p.RESET_USE_DEFORMABLE_WORLD)
            p.setGravity(0, 0, -9.81)
            p.resetDebugVisualizerCamera(1.674, 70, -50.8, [0., 0, 0])

            p.configureDebugVisualizer(p.COV_ENABLE_GUI, 0)
            p.setPhysicsEngineParameter(enableFileCaching=0)
            p.setAdditionalSearchPath(ASSETS_PATH)

            self.oracle_cams = cameras.Oracle.CONFIG
            self.camera = Camera(self.camera_intrinsic)
            # self.arm = UR5(self.urdf_file)

            self.end_axes = DebugAxes()  # 机械臂末端的局部坐标系
            self.camera_axes = DebugAxes()  # 相机坐标系

            init_xyz = [0.0, -0.0, 0.50]
            init_rpy = [0, math.pi / 2., 0]
            self.param_ids = self.setup_target_pose_params(init_xyz, init_rpy)
            # p.stepSimulation()
            # p.setRealTimeSimulation(1)
            # self.ur5 = pybullet_utils.load_urdf(p, self.urdf_file)
            base_pos = [-0.5, 0.0, -0.0]
            base_orn = [0, 0, 0, 1]  # quaternion (x, y, z, w)
            pybullet_utils.load_urdf(p, os.path.join(self.assets_root, PLANE_URDF_PATH),
                                     [0, 0, -0.001])
            pybullet_utils.load_urdf(
                p, os.path.join(self.assets_root, UR5_WORKSPACE_URDF_PATH), [0.0, 0, 0])
            self.ur5 = p.loadURDF(self.urdf_file, base_pos, base_orn,
                                  flags=p.URDF_USE_INERTIA_FROM_FILE)
            # self.ur5 = pybullet_utils.load_urdf(p, os.path.join(self.assets_root, UR5_URDF_PATH))
            n_joints = p.getNumJoints(self.ur5)
            joints = [p.getJointInfo(self.ur5, i) for i in range(n_joints)]
            self.joints = [j[0] for j in joints if j[2] == p.JOINT_REVOLUTE]

            # Move robot to home joint configuration.
            for i in range(len(self.joints)):
                p.resetJointState(self.ur5, self.joints[i], self.homej[i])
            if self.task:
                self.ee = self.task.ee(self.assets_root, self.ur5, self.ee_tool_fixed_joint)
                self.ee.release()
            self.reset()

            # while not self.is_static:
            #     p.stepSimulation()
            self.isReset = True
        except Exception as e:
            print(e)
            kdata_lx.isVirt = False

    def __del__(self):
        p.disconnect(p.GUI)

    def pick_place_ex(self, action=None):
        """Execute action with specified primitive.

        Args:
          action: action to execute.

        Returns:
          (obs, reward, done, info) tuple containing MDP step data.
        """
        if action is not None:
            timeout = self.pick_place(self.movej, self.movep, self.ee, **action)

            # Exit early if action times out. We still return an observation
            # so that we don't break the Gym API contract.
            # if timeout:
            #     obs = self._get_obs()
                # return obs, 0.0, True, self.info

        # Step simulator asynchronously until objects settle.
        while not self.is_static:
            p.stepSimulation()

        # Get task rewards.
        # reward, info = self.task.reward() if action is not None else (0, {})
        # done = self.task.done()
        #
        # # Add ground truth robot state into info.
        # info.update(self.info)
        #
        # obs = self._get_obs()
        #
        # return obs

    # def _get_obs(self):
    #     # Get RGB-D camera image observations.
    #     obs = {'color': (), 'depth': ()}
    #     for config in self.agent_cams:
    #         color, depth, _ = self.render_camera(config)
    #         obs['color'] += (color,)
    #         obs['depth'] += (depth,)
    #
    #     return obs

    def render_camera(self, config):
        """Render RGB-D image with specified camera configuration."""

        # OpenGL camera settings.
        lookdir = np.float32([0, 0, 1]).reshape(3, 1)
        updir = np.float32([0, -1, 0]).reshape(3, 1)
        rotation = p.getMatrixFromQuaternion(config['rotation'])
        rotm = np.float32(rotation).reshape(3, 3)
        lookdir = (rotm @ lookdir).reshape(-1)
        updir = (rotm @ updir).reshape(-1)
        lookat = config['position'] + lookdir
        focal_len = config['intrinsics'][0]
        znear, zfar = config['zrange']
        viewm = p.computeViewMatrix(config['position'], lookat, updir)
        fovh = (config['image_size'][0] / 2) / focal_len
        fovh = 180 * np.arctan(fovh) * 2 / np.pi

        # Notes: 1) FOV is vertical FOV 2) aspect must be float
        aspect_ratio = config['image_size'][1] / config['image_size'][0]
        projm = p.computeProjectionMatrixFOV(fovh, aspect_ratio, znear, zfar)

        # Render with OpenGL camera settings.
        _, _, color, depth, segm = p.getCameraImage(
            width=config['image_size'][1],
            height=config['image_size'][0],
            viewMatrix=viewm,
            projectionMatrix=projm,
            shadow=1,
            flags=p.ER_SEGMENTATION_MASK_OBJECT_AND_LINKINDEX,
            # Note when use_egl is toggled, this option will not actually use openGL
            # but EGL instead.
            renderer=p.ER_BULLET_HARDWARE_OPENGL)

        # Get color image.
        color_image_size = (config['image_size'][0], config['image_size'][1], 4)
        color = np.array(color, dtype=np.uint8).reshape(color_image_size)
        color = color[:, :, :3]  # remove alpha channel
        if config['noise']:
            color = np.int32(color)
            color += np.int32(self._random.normal(0, 3, config['image_size']))
            color = np.uint8(np.clip(color, 0, 255))

        # Get depth image.
        depth_image_size = (config['image_size'][0], config['image_size'][1])
        zbuffer = np.array(depth).reshape(depth_image_size)
        depth = (zfar + znear - (2. * zbuffer - 1.) * (zfar - znear))
        depth = (2. * znear * zfar) / depth
        if config['noise']:
            depth += self._random.normal(0, 0.003, depth_image_size)

        # Get segmentation image.
        segm = np.uint8(segm).reshape(depth_image_size)

        return color, depth, segm

    def movej(self, targj, speed=0.001, timeout=10):
        """Move UR5 to target joint configuration."""
        t0 = time.time()
        # gains = np.ones(len(self.joints))
        # p.setJointMotorControlArray(
        #     bodyIndex=self.ur5,
        #     jointIndices=self.joints,
        #     controlMode=p.POSITION_CONTROL,
        #     targetPositions=targj,  # stepj,
        #     positionGains=gains)
        # p.stepSimulation()
        # for _ in range(480):
        #     p.stepSimulation()

        # targ_pose = pose
        # while not self.ee.detect_contact():  # and target_pose[2] > 0:
        #     targ_pose = utils.multiply(targ_pose, delta)
        #     self.movep(targ_pose)

        while (time.time() - t0) < timeout:
            currj = [p.getJointState(self.ur5, i)[0] for i in self.joints]
            currj = np.array(currj)
            diffj = targj - currj
            if all(np.abs(diffj) < 1e-2):
                # print("movej:{}".format(currj))
                return False

            # Move with constant velocity
            norm = np.linalg.norm(diffj)
            v = diffj / norm if norm > 0 else 0
            stepj = currj + v * speed

            gains = np.ones(len(self.joints))

            p.setJointMotorControlArray(
                bodyIndex=self.ur5,
                jointIndices=self.joints,
                controlMode=p.POSITION_CONTROL,
                targetPositions= stepj,#,
                positionGains=gains)
            time.sleep(0.01)

            p.stepSimulation()

        print(f'Warning: movej exceeded {timeout} second timeout. Skipping.')
        return True

    def movep_6dof(self, pose, acc=0.01, speed=0.01,active=False):
        """Move UR5 to target end effector pose."""
        # xyz = pose[:3]
        xyz = (pose[0],pose[1],pose[2])
        qua = p.getQuaternionFromEuler(pose[3:6])
        qua =(0.0, 0.0, 0.0, 1.0)
        pose = (xyz,qua)
        # targj = self.solve_ik(pose)

        # timeout = self.movep(prepick_pose)
        delta = (np.float32([0, 0, -0.001]),
                 utils.eulerXYZ_to_quatXYZW((0, 0, 0)))

        if active:

            targ_pose = pose
            while not self.ee.detect_contact():  # and target_pose[2] > 0:

                targ_pose = utils.multiply(targ_pose, delta)
                if targ_pose[0][2]<=0.001:
                    break
                # print('movep:{}'.format(targ_pose))
                self.movep(targ_pose)
        else:
            self.movep(pose)
        # return self.movej(targj, speed)

    def movep(self, pose, speed=0.01):
        """Move UR5 to target end effector pose."""
        targj = self.solve_ik(pose)
        return self.movej(targj, speed)

    def solve_ik(self, pose):
        """Calculate joint configuration with inverse kinematics."""
        joints = p.calculateInverseKinematics(
            bodyUniqueId=self.ur5,
            endEffectorLinkIndex=self.ee_tip,
            targetPosition=pose[0],
            targetOrientation=pose[1],
            lowerLimits=[-3 * np.pi / 2, -2.3562, -17, -17, -17, -17],
            upperLimits=[-np.pi / 2, 0, 17, 17, 17, 17],
            jointRanges=[np.pi, 2.3562, 34, 34, 34, 34],  # * 6,
            restPoses=np.float32(self.homej).tolist(),
            maxNumIterations=100,
            residualThreshold=1e-5
        )
        joints = np.float32(joints)
        joints[2:] = (joints[2:] + np.pi) % (2 * np.pi) - np.pi

        return joints

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


    def t2c(self, end_pos, end_orn):
        relative_offset = [0.05, 0, 0.05]
        end_orn = R.from_quat(end_orn).as_matrix()
        end_x_axis, end_y_axis, end_z_axis = end_orn.T
        wcT = np.eye(4)  # w: world, c: camera, ^w_c T
        wcT[:3, 0] = end_x_axis  # camera x axis
        wcT[:3, 1] = -end_y_axis  # camera y axis
        wcT[:3, 2] = -end_z_axis  # camera z axis

        wcT[:3, 3] = end_orn.dot(relative_offset)   # eye position

        return wcT

    def _bind_camera_to_end(self, end_pos, end_orn):
        """设置相机坐标系与末端坐标系的相对位置

        Arguments:
        - end_pos: len=3, end effector position
        - end_orn: len=4, end effector orientation, quaternion (x, y, z, w)

        Returns:
        - wcT: shape=(4, 4), transform matrix, represents camera pose in world frame
        """
        relative_offset = [0.05, 0, 0.05]  # 相机原点相对于末端执行器局部坐标系的偏移量
        end_orn = R.from_quat(end_orn).as_matrix()
        end_x_axis, end_y_axis, end_z_axis = end_orn.T


        wcT = np.eye(4)  # w: world, c: camera, ^w_c T
        wcT[:3, 0] = end_x_axis  # camera x axis
        wcT[:3, 1] = -end_y_axis  # camera y axis
        wcT[:3, 2] = -end_z_axis  # camera z axis


        # wcT[:3, :3] = end_orn

        wcT[:3, 3] = end_orn.dot(relative_offset) + end_pos  # eye position

        return wcT

    def get_end_state(self):
        """Get the position and orientation of the end effector.

        Returns:
        - end_pos: len=3, (x, y, z) in world coordinate system
        - end_orn: len=4, orientation in quaternion representation (x, y, z, w)
        """
        end_state = p.getLinkState(self.ur5, self.ee_tip)
        end_pos = end_state[0]
        end_orn = end_state[1]

        return end_pos, end_orn

    def update_debug_axes(self):
        try:
            while True:
                if self.stope_:
                    break
                if not p.isConnected():
                    break
                # update debug axes and camera position
                end_pos, end_orn = self.get_end_state()
                # end_pos, end_orn = self.arm.get_end_state()
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
        # imgs = self._get_obs()
        # kdata_lx.colorImg = imgs['color']
        # kdata_lx.depthImg = imgs['depth']
        end_pos, end_orn = self.get_end_state()

        wcT = self._bind_camera_to_end(end_pos, end_orn)

        # cwT = np.linalg.inv(wcT)
        if not p.isConnected():
            return
        # self.frame = self.camera.render(cwT)
        self.frame = self.camera.render(wcT)

        rpy = rm2rpy(wcT[:3,])

        self.oracle_cams[0]['position'] = (end_pos[0]+0.05, end_pos[1], end_pos[2]+0.05)
        self.oracle_cams[0]['rotation'] = p.getQuaternionFromEuler((rpy[3],rpy[4],rpy[5]))

        pose = self.get_current_tcp()

        color, depth, segm = self.render_camera(self.oracle_cams[0])

        kdata_lx.colorImg = cv2.cvtColor(color,cv2.COLOR_RGB2BGRA)
        kdata_lx.depthImg = depth


        # kdata_lx.colorImg = self.frame.color_image()
        # kdata_lx.depthImg = self.frame.depth_image()
        # kdata_lx.color_streaming
        self.extrinsic = self.frame.t2c
        return True,pose

    def getobjs(self):
        for obj_id in self.obj_ids['rigid']:
            pos, rot = p.getBasePositionAndOrientation(obj_id)

            print(pos, rot)

    def update_camera_image(self):
        try:
            while True:
                if self.stope_:
                    break
                if not p.isConnected():
                    break
                end_pos, end_orn = self.get_end_state()
                if not p.isConnected():
                    break
                wcT = self._bind_camera_to_end(end_pos, end_orn)

                # cwT = np.linalg.inv(wcT)
                if not p.isConnected():
                    break
                # self.frame = self.camera.render(cwT)
                self.frame = self.camera.render(wcT)


                t2c = self.t2c(end_pos, end_orn)
                self.frame.t2c = t2c

                ###################################
                rpy = rm2rpy(wcT[:3, ])
                self.oracle_cams[0]['position'] = (end_pos[0] + 0.05, end_pos[1], end_pos[2] + 0.05)
                self.oracle_cams[0]['rotation'] = p.getQuaternionFromEuler((rpy[3], rpy[4], rpy[5]))

                # imgs = self._get_obs()
                assert isinstance(self.frame, Frame)

                if not p.isConnected():
                    break
                # rgb = self.frame.color_image()  # 这里以显示rgb图像为例, frame还包含了深度图, 也可以转化为点云
                # rgb=imgs['color']
                # bgr = np.ascontiguousarray(rgb[:, :, ::-1])  # flip the rgb channel

                color, depth, segm = self.render_camera(self.oracle_cams[0])
                kdata_lx.color_streaming = color
                kdata_lx.depth_streaming = depth
                # kdata_lx.depth_streaming =
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



    def isConnect(self):
        return p.isConnected()

    def get_current_tcp(self):
        end_pos, end_orn= self.get_end_state()
        pose = [end_pos[0],end_pos[1],end_pos[2],end_orn[0],end_orn[1],end_orn[2]]
        pose = np.asarray(pose)
        return pose

    def gripper(self,openess):
        if openess == 0:
            self.ee.activate()
            print("ee activate")
        else:
            self.ee.release()
            print("ee release")

    # def move_old_ver(self,target_pose,acc=None,vel=0.01):
    #     # xyz = [target_pose[0],target_pose[1],target_pose[2]]
    #     # xyz = np.asarray(xyz)
    #     # rpy = [target_pose[3],target_pose[4],target_pose[5]]
    #     # rpy = np.asarray(rpy)
    #     # pose = [xyz,rpy]
    #     # self.movep(pose,vel)
    #     # print("target"+target_pose)
    #     # target_pose = self.read_user_params(self.param_ids)  # [x, y, z, roll, pitch, yaw, finger openness]
    #     self.arm.move_to(target_pose[:3],
    #                     p.getQuaternionFromEuler(target_pose[3:6]))
    #     # self.arm.control_gripper(target_pose[-1])

    def is_static(self):
        """Return true if objects are no longer moving."""
        v = [np.linalg.norm(p.getBaseVelocity(i)[0])
             for i in self.joints]
        return all(np.array(v) < 5e-3)

    # while not self.is_static:
    #   p.stepSimulation()




    # def movep(self,target_pose,acc=None,vel=0.01):
    #     self.arm.move_to(target_pose[:3],target_pose[3:5],acc=None,vel=0.01)
    #         # self.arm.movep(target_pose[:3],p.getQuaternionFromEuler(target_pose[3:6]),vel)


    def start_manual_control(self):
        self.stope_ = False
        self.open()
        # thread for updatig debug axes
        # if not self.update_debug_axes_thread or not self.update_debug_axes_thread.is_alive():
        #     self.update_debug_axes_thread = threading.Thread(
        #         target=self.update_debug_axes)
        #     # self.update_debug_axes_thread.setDaemon(True)
        #     self.update_debug_axes_thread.start()

        # thread for updating camera image
        if not self.update_camera_image_thread or not self.update_camera_image_thread.is_alive():
            self.update_camera_image_thread = threading.Thread(
                target=self.update_camera_image)
            # self.update_camera_image_thread.setDaemon(True)
            self.update_camera_image_thread.start()

        # if not self.update_arm_thread or not self.update_arm_thread.is_alive():
        #     self.update_arm_thread = threading.Thread(
        #         target=self.update_arm)
        #     # self.update_arm_thread.setDaemon(True)
        #     self.update_arm_thread.start()

        kdata_lx.isVirt = True
        p.setTimeStep(1. / 240)
        self.extrinsic = None

        pose = [(0.000, -0.000, 0.4010000000000003), (0, 0, 0,1)]
        self.movep(pose)
        while not self.is_static:
            p.stepSimulation()


    def _get_obs(self):
        # Get RGB-D camera image observations.
        obs = {'color': (), 'depth': ()}
        for config in self.agent_cams:
            color, depth, _ = self.render_camera(config)
            obs['color'] += (color,)
            obs['depth'] += (depth,)

        return obs

    def render_camera(self, config):
        """Render RGB-D image with specified camera configuration."""

        # OpenGL camera settings.
        lookdir = np.float32([0, 0, 1]).reshape(3, 1)
        updir = np.float32([0, -1, 0]).reshape(3, 1)
        rotation = p.getMatrixFromQuaternion(config['rotation'])
        rotm = np.float32(rotation).reshape(3, 3)
        lookdir = (rotm @ lookdir).reshape(-1)
        updir = (rotm @ updir).reshape(-1)
        lookat = config['position'] + lookdir
        focal_len = config['intrinsics'][0]
        znear, zfar = config['zrange']
        viewm = p.computeViewMatrix(config['position'], lookat, updir)
        fovh = (config['image_size'][0] / 2) / focal_len
        fovh = 180 * np.arctan(fovh) * 2 / np.pi

        # Notes: 1) FOV is vertical FOV 2) aspect must be float
        aspect_ratio = config['image_size'][1] / config['image_size'][0]
        projm = p.computeProjectionMatrixFOV(fovh, aspect_ratio, znear, zfar)

        # Render with OpenGL camera settings.
        _, _, color, depth, segm = p.getCameraImage(
            width=config['image_size'][1],
            height=config['image_size'][0],
            viewMatrix=viewm,
            projectionMatrix=projm,
            shadow=1,
            flags=p.ER_SEGMENTATION_MASK_OBJECT_AND_LINKINDEX,
            # Note when use_egl is toggled, this option will not actually use openGL
            # but EGL instead.
            renderer=p.ER_BULLET_HARDWARE_OPENGL)

        # Get color image.
        color_image_size = (config['image_size'][0], config['image_size'][1], 4)
        color = np.array(color, dtype=np.uint8).reshape(color_image_size)
        color = color[:, :, :3]  # remove alpha channel
        if config['noise']:
            color = np.int32(color)
            color += np.int32(self._random.normal(0, 3, config['image_size']))
            color = np.uint8(np.clip(color, 0, 255))

        # Get depth image.
        depth_image_size = (config['image_size'][0], config['image_size'][1])
        zbuffer = np.array(depth).reshape(depth_image_size)
        depth = (zfar + znear - (2. * zbuffer - 1.) * (zfar - znear))
        depth = (2. * znear * zfar) / depth
        if config['noise']:
            depth += self._random.normal(0, 0.003, depth_image_size)

        # Get segmentation image.
        segm = np.uint8(segm).reshape(depth_image_size)

        return color, depth, segm


    def move_to_pos_orn(self, pos, orn, acc=0.01, speed=0.01):
        """Move arm to provided pose.

        Arguments:
        - pos: len=3, position (x, y, z) in world coordinate system
        - orn: len=4, quaternion (x, y, z, w) in world coordinate system
        - finger_angle: numeric, gripper's openess

        Returns:
        - joints_pose: len=8, angles for 8 controllable joints
        """
        joints = p.calculateInverseKinematics(
            bodyUniqueId=self.ur5,
            endEffectorLinkIndex=self.ee_tip,
            targetPosition=pos,
            targetOrientation=orn,
            lowerLimits=[-3 * np.pi / 2, -2.3562, -17, -17, -17, -17],
            upperLimits=[-np.pi / 2, 0, 17, 17, 17, 17],
            jointRanges=[np.pi, 2.3562, 34, 34, 34, 34],  # * 6,
            restPoses=np.float32(self.homej).tolist(),
            maxNumIterations=100,
            residualThreshold=1e-5
        )
        joints = np.float32(joints)
        joints[2:] = (joints[2:] + np.pi) % (2 * np.pi) - np.pi
        self.movej(joints, speed)



    def update_arm(self):
        try:
            while True:
                if self.stope_:
                    break
                if not p.isConnected():
                    break
                p.stepSimulation()
                # target_pose = self.read_user_params(self.param_ids)  # [x, y, z, roll, pitch, yaw, finger openness]
                # self.move_to_pos_orn(target_pose[:3],
                #                      p.getQuaternionFromEuler(target_pose[3:6]))

                time.sleep(0.02)

                # target_pose = read_user_params(self.param_ids)  # [x, y, z, roll, pitch, yaw, finger openness]
                # if not p.isConnected():
                #     break
                # self.arm.move_to(target_pose[:3],
                #                 p.getQuaternionFromEuler(target_pose[3:6]))
                # if not p.isConnected():
                #     break
                # self.arm.control_gripper(target_pose[-1])

        except Exception as e:
            print(e)
            # p.setRealTimeSimulation(0)
            self.stop_manual_control()

class Task:
    def __init__(self, parent=None):
        self.ee = Suction


if __name__ == "__main__":
    env = Environment_Pick_Placeget_true_image()
    task = Task()
    env.set_task(task)
    env.start_manual_control()



    # env.reset()

    # print("[INFO] Start manual control!")
    # env.start_manual_control()

