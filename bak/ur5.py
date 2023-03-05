import numpy as np
import pybullet as p
from collections import namedtuple
import time

class UR5(object):
    def __init__(self, urdf_file: str):
        self.file_name = urdf_file
        self.base_pos = [-0.5, 0.0, -0.1]
        self.base_orn = [0, 0, 0, 1]  # quaternion (x, y, z, w)

        self.uid = p.loadURDF(self.file_name, self.base_pos, self.base_orn,
                              flags=p.URDF_USE_INERTIA_FROM_FILE)
        self.end_effector_id = 7
        self.joints_map, self.control_joints_name = self.setup_sisbot(self.uid)
        self.control_joints_id = [self.joints_map[name].id for name in self.control_joints_name]
        self.control_joints_maxF = [self.joints_map[name].maxForce for name in self.control_joints_name]
        self.reset_joints_pose()
        # self.homej = np.array([-1, -0.5, 0.5, -0.5, -0.5, 0]) * np.pi

        n_joints = p.getNumJoints(self.uid)
        joints = [p.getJointInfo(self.uid , i) for i in range(n_joints)]
        self.joints = [j[0] for j in joints if j[2] == p.JOINT_REVOLUTE]

        # Move robot to home joint configuration.
        # for i in range(len(self.joints)):
        #     p.resetJointState(self.uid , self.joints[i], self.homej[i])


    def reset_joints_pose(self):
        """Move to an ideal init point."""
        self.action([0.15328961509984124, -1.8, -1.5820032364177563,
                     -1.2879050862601897, 1.5824233979484994, 0.19581299859677043,
                     0.012000000476837159, -0.012000000476837159])

    def set_base_pose(self, pos, orn):
        """
        - pos: len=3, (x, y, z) in world coordinate system
        - orn: len=4, (x, y, z, w) orientation in quaternion representation
        """
        self.base_pos = pos
        self.base_orn = orn
        p.resetBasePositionAndOrientation(self.uid, pos, orn)

    def setup_sisbot(self, uid):
        control_joints_name = [
            'shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint',
            'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint',
            'left_gripper_motor', 'right_gripper_motor']

        joint_type_list = ['REVOLUTE', 'PRISMATIC', 'SPHERICAL', 'PLANAR', 'FIXED']
        JointInfo = namedtuple('JointInfo',
                               ['id', 'name', 'type', 'lowerLimit', 'upperLimit', 'maxForce', 'maxVelocity',
                                'controllable'])

        num_joints = p.getNumJoints(uid)
        joints = dict()
        for i in range(num_joints):
            info = p.getJointInfo(uid, i)
            joint_id = info[0]
            joint_name = info[1].decode('utf-8')
            joint_type = joint_type_list[info[2]]
            joint_lower_limit = info[8]
            joint_upper_limit = info[9]
            joint_max_force = info[10]
            joint_max_vel = info[11]
            controllable = True if joint_name in control_joints_name else False

            info = JointInfo(joint_id, joint_name, joint_type, joint_lower_limit, joint_upper_limit,
                             joint_max_force, joint_max_vel, controllable)
            if info.type == 'REVOLUTE':
                p.setJointMotorControl2(uid, info.id, p.VELOCITY_CONTROL, targetVelocity=0, force=0)
            joints[info.name] = info

        return joints, control_joints_name

    def action(self, cmds):
        """
        - cmds: len=8, target angles for 8 controllable joints
        """
        n_joints = len(cmds)
        p.setJointMotorControlArray(self.uid, self.control_joints_id[:n_joints], p.POSITION_CONTROL,
                                    targetPositions=cmds, targetVelocities=[0] * n_joints,
                                    positionGains=[0.03] * n_joints, forces=self.control_joints_maxF[:n_joints])

    def ikines(self, pos, orn):

        pos = np.clip(pos, a_min=[-0.4, -0.4, 0.001], a_max=[0.4, 0.4, 0.8])
        joints_pos = list(p.calculateInverseKinematics(self.uid, self.end_effector_id, pos, orn))[:6]  # len=8
        return joints_pos

    def movep(self, pos, orn,speed=0.01):
        """Move UR5 to target end effector pose."""
        targj = self.solve_ik(pos,orn)[:6]

        return self.movej(targj, speed)

    def solve_ik(self, pos,orn):
        """Calculate joint configuration with inverse kinematics."""
        joints = p.calculateInverseKinematics(
            bodyUniqueId=self.uid,
            endEffectorLinkIndex=self.end_effector_id,
            targetPosition=pos,
            targetOrientation=orn,
            lowerLimits=[-3 * np.pi / 2, -2.3562, -17, -17, -17, -17],
            upperLimits=[-np.pi / 2, 0, 17, 17, 17, 17],
            jointRanges=[np.pi, 2.3562, 34, 34, 34, 34],  # * 6,
            # restPoses=np.float32(self.homej).tolist(),
            maxNumIterations=100,
            residualThreshold=1e-5)
        joints = np.float32(joints)
        joints[2:] = (joints[2:] + np.pi) % (2 * np.pi) - np.pi
        return joints

    def movej(self, targj, speed=0.01, timeout=5):
        """Move UR5 to target joint configuration."""
        t0 = time.time()
        while (time.time() - t0) < timeout:

            currj = [p.getJointState(self.uid, i)[0] for i in self.joints]
            currj = np.array(currj)
            diffj = targj - currj

            # print()
            if all(np.abs(diffj) < 1e-2):
                return False

            # Move with constant velocity
            norm = np.linalg.norm(diffj)
            v = diffj / norm if norm > 0 else 0
            stepj = currj + v * speed
            print(stepj)
            gains = np.ones(len(self.joints))
            p.setJointMotorControlArray(
                bodyIndex=self.uid,
                jointIndices=self.joints,
                controlMode=p.POSITION_CONTROL,
                targetPositions=stepj,
                positionGains=gains)
            p.stepSimulation()
        print(f'Warning: movej exceeded {timeout} second timeout. Skipping.')
        return True


    def move_to(self, pos, orn):
        """Move arm to provided pose.

        Arguments:
        - pos: len=3, position (x, y, z) in world coordinate system
        - orn: len=4, quaternion (x, y, z, w) in world coordinate system
        - finger_angle: numeric, gripper's openess

        Returns:
        - joints_pose: len=8, angles for 8 controllable joints
        """
        pos = np.clip(pos, a_min=[-0.4, -0.4, 0.001], a_max=[0.4, 0.4, 0.8])
        # pos = np.clip(pos)
        joints_pos = self.ikines(pos, orn)
        self.action(joints_pos)

    def control_gripper(self, openness):
        """
        width: 0~1, 0->close, 1->open
        """
        openness = np.clip(openness, 0, 1)
        motor_pos = (1. - openness) / 25.
        gripper_id = [self.joints_map['left_gripper_motor'].id, self.joints_map['right_gripper_motor'].id]
        p.setJointMotorControlArray(self.uid, gripper_id, p.POSITION_CONTROL,
                                    targetPositions=[motor_pos, -motor_pos], targetVelocities=[0, 0],
                                    positionGains=[0.03, 0.03], forces=self.control_joints_maxF[-2:])

    def get_joints_state(self):
        """Get all joints' angles and velocities.

        Returns:
        - joints_pos: len=n_joints, angles for all joints
        - joints_vel: len=n_joints, velocities for all joints
        """
        joints_state = p.getJointStates(self.uid, self.control_joints_id)
        joints_pos = [s[0] for s in joints_state]
        joints_vel = [s[1] for s in joints_state]

        return joints_pos, joints_vel

    def get_end_state(self):
        """Get the position and orientation of the end effector.

        Returns:
        - end_pos: len=3, (x, y, z) in world coordinate system
        - end_orn: len=4, orientation in quaternion representation (x, y, z, w)
        """
        end_state = p.getLinkState(self.uid, self.end_effector_id)
        end_pos = end_state[0]
        end_orn = end_state[1]

        return end_pos, end_orn

