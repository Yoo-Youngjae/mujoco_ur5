import numpy as np
import math
import utils
from scipy.spatial.transform import Rotation as R
import ikpy

############ CONFIG ##############
home_pos = (-2.595, -4.255, 1.491, -1.485, -0.920, -2.040)
place_pose = (-1.643, -3.509, 1.643, -2.070, -1.512, -2.016)
place_pose2 = [-1.6461, -3.5756, 1.7393, -2.1001, -1.5096, -2.8140]
GLOBAL_VEL = 0.3
GLOBAL_ACC = 0.3
box_height = 1.11
L_B_D = [0,-1,0,-0.25, -0.7071,0,-0.7071,1.0323, 0.7071,0,-0.7071,-0.1838,0,0,0,1]
L_B_D = np.array(L_B_D).reshape(4,4)
acc = 0.3
vel = 0.3
#################################

class Agent:
    def __init__(self, controller):
        self.robot = controller
        self.L_B_D = L_B_D

    def movel(self, tcp, relative=False):
        if len(tcp) == 3:

            if relative:
                cur_tcp, rot_mat = self.getl()
                cur_tcp = np.array(cur_tcp)
                trans_tcp = cur_tcp[:3] + tcp
                print('tcp', cur_tcp)
            target_joint = self.robot.ee_chain.inverse_kinematics(target_position=trans_tcp,
                                                                  target_orientation=rot_mat,
                                                                  orientation_mode='all')
            print('before target_joint', target_joint)
            target_joint = target_joint[1:7]
            print('after target_joint', target_joint)
            self.robot.move_group_to_joint_target(group="Arm", target=target_joint, marker=False)

        elif len(tcp) == 6:
            self.robot.move_group_to_joint_target(group="Arm", target=tcp, marker=False)
        else:
            assert False, 'length of tcp input must be 3 or 6'

    def movej(self, joints, relative=False):
        if relative is True:
            cur_pos = self.getj()
            target_pose = cur_pos + joints
            self.robot.move_group_to_joint_target(group="Arm", target=target_pose, marker=False)
        else:
            self.robot.move_group_to_joint_target(group="Arm", target=joints, marker=False)

    def getj(self):
        current_joint_values = self.robot.sim.data.qpos[self.robot.actuated_joint_ids]
        # print('shoulder_pan_joint', self.robot.sim.data.qpos[0])
        # print('shoulder_lift_joint', self.robot.sim.data.qpos[1])
        # print('elbow_joint', self.robot.sim.data.qpos[2])
        # print('wrist_1_joint', self.robot.sim.data.qpos[3])
        # print('wrist_2_joint', self.robot.sim.data.qpos[4])
        # print('wrist_3_joint', self.robot.sim.data.qpos[5])
        # print('base_to_lik', self.robot.sim.data.qpos[6]) # gripper_motor
        return current_joint_values[:6]

    def getl(self) :
        cur_joints = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
        cur_joints[1:7] = self.getj()
        cur_tcp = self.robot.ee_chain.forward_kinematics(cur_joints)

        trans_vec, rot_mat = ikpy.utils.geometry.from_transformation_matrix(cur_tcp)
        # rot_euler = utils.rotation_matrix_to_euler_angles(rot_mat)
        rot_euler = R.from_matrix(rot_mat).as_rotvec()
        tcp = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
        tcp[:3] = trans_vec[:3]
        tcp[3:] = rot_euler
        return tcp, rot_mat

    def close_gripper(self):
        return self.robot.close_gripper()

    def open_gripper(self):
        return self.robot.open_gripper()


    def moveD(self, tcp, acc=None, vel=None, wait=True, relative=False):
        global GLOBAL_ACC, GLOBAL_VEL
        if acc == None :
            acc = GLOBAL_ACC
        if vel == None :
            vel = GLOBAL_VEL
        if len(tcp) == 3:
            if relative:
                dir = self.desk_to_base(direction=tcp)
                #print(dir)
                self.movel(dir, acc=acc, vel=vel, wait=wait, relative=True)
            else:
                pos = self.desk_to_base(position=tcp)
                self.movel(pos, acc=acc, vel=vel, wait=wait, relative=False)
        elif len(tcp) == 6:
            if relative:
                # print('0. current_tcp_D', self.getl())
                current_tcp_D = utils.affine_to_tcp(self.base_to_desk(affine=utils.tcp_to_affine(self.getl())))
                # print('1. current_tcp_D', current_tcp_D)
                tcp_D = current_tcp_D + tcp
                # print('2. tcp_D', tcp_D)
                tcp_B = utils.affine_to_tcp(self.desk_to_base(affine=utils.tcp_to_affine(tcp_D)))
                # print('3. tcp_B', tcp_B)
                self.movel(tcp_B, acc=acc, vel=vel, wait=wait, relative=False)
            else:
                tcp_B = utils.affine_to_tcp(self.desk_to_base(affine=utils.tcp_to_affine(tcp)))
                self.movel(tcp_B, acc=acc, vel=vel, wait=wait, relative=False)
        else:
            assert False, 'length of tcp input must be 3 or 6'

    def desk_to_base(self, position=None, direction=None, scipy_R=None, dcm=None, affine=None):
        return utils.transform(self.L_B_D, position, direction, scipy_R, dcm, affine)
    def base_to_desk(self, position=None, direction=None, scipy_R=None, dcm=None, affine=None):
        return utils.transform(np.linalg.inv(self.L_B_D), position, direction, scipy_R, dcm, affine)

    def calculate_dist_for_pick(self, pc):
        # robot tool: +x = right , + y = front, +z = up
        offset = [0.373, 0.005, 0.643] #
        pc = pc.copy()
        pc[0], pc[1], pc[2] = round(pc[0], 3), round(pc[1], 3), round(pc[2], 3)
        # camera side: +x = right, +y = back, +z = down
        dist_x = pc[0] - offset[0]
        dist_y = -pc[1] + offset[1]
        dist_z = -pc[2] + offset[2]
        return dist_x, dist_y, dist_z

    def calcuate_rpy_to_ur5_rxryrz(self, rpy):
        roll, pitch, yaw = rpy
        yawMatrix = np.matrix([
            [math.cos(yaw), -math.sin(yaw), 0],
            [math.sin(yaw), math.cos(yaw), 0],
            [0, 0, 1]
        ])

        pitchMatrix = np.matrix([
            [math.cos(pitch), 0, math.sin(pitch)],
            [0, 1, 0],
            [-math.sin(pitch), 0, math.cos(pitch)]
        ])

        rollMatrix = np.matrix([
            [1, 0, 0],
            [0, math.cos(roll), -math.sin(roll)],
            [0, math.sin(roll), math.cos(roll)]
        ])

        R = yawMatrix * pitchMatrix * rollMatrix

        theta = math.acos(((R[0, 0] + R[1, 1] + R[2, 2]) - 1) / 2)
        multi = 1 / (2 * math.sin(theta))

        rx = multi * (R[2, 1] - R[1, 2]) * theta
        ry = multi * (R[0, 2] - R[2, 0]) * theta
        rz = multi * (R[1, 0] - R[0, 1]) * theta

        return (rx, ry, rz)

    def rotate_moveD(self, rpy):
        # print('(rx, ry, rz)', (rx, ry, rz))
        # rot_mat = R.from_rotvec(np.array([rx, ry, rz])).as_dcm()
        # rot_mat = R.from_rotvec(np.array([0, -0.3, 0])).as_dcm()

        (roll, pitch, yaw) = rpy
        print('[roll, pitch, yaw]', rpy[0] * 180 / math.pi,
                                    rpy[1] * 180 / math.pi,
                                    rpy[2] * 180 / math.pi)
        roll = math.radians(math.degrees(roll))
        pitch = math.radians(math.degrees(pitch))
        yaw = math.radians(math.degrees(yaw))
        rot_mat = R.from_rotvec(np.array([pitch, yaw, roll])).as_dcm()

        current_pos_D = self.base_to_desk(affine=utils.tcp_to_affine(self.getl()))
        target_pos_D = utils.transform(rot_mat, dcm=current_pos_D)
        self.moveD(utils.affine_to_tcp(dcm=target_pos_D, t=self.base_to_desk(self.getl()[:3])))

        # print('base_link', self.robot.sim.data.body_xpos[self.robot.model.body_name2id("base_link")])
        # print('shoulder_link', self.robot.sim.data.body_xpos[self.robot.model.body_name2id("shoulder_link")])
        # print('upper_arm_link', self.robot.sim.data.body_xpos[self.robot.model.body_name2id("upper_arm_link")])
        # print('forearm_link', self.robot.sim.data.body_xpos[self.robot.model.body_name2id("forearm_link")])
        # print('wrist_1_link', self.robot.sim.data.body_xpos[self.robot.model.body_name2id("wrist_1_link")])
        # print('wrist_2_link', self.robot.sim.data.body_xpos[self.robot.model.body_name2id("wrist_2_link")])
        # print('wrist_3_link', self.robot.sim.data.body_xpos[self.robot.model.body_name2id("wrist_3_link")])
        # print('ee_link', self.robot.sim.data.body_xpos[self.robot.model.body_name2id("ee_link")])