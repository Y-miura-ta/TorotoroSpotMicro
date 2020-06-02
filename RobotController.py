# -*- coding: utf-8 -*-

from mpl_toolkits import mplot3d
import numpy as np
from math import *
import matplotlib.pyplot as plt

#制御の加わる周期[s]
control_interval = 10.0e-3

#ローパスフィルタの重み
k_lpf = 0.1

#姿勢角が横方向に倒れすぎると脚の接地位置を大きく出す
p_rough_control_z = 5.0/50
p_rough_control_x = 10.0/50

#関節の回転正方向を決める行列
theta_sign = np.array([
    [1, 1, 1], # lf
    [1, 1, 1], # rf
    [1, 1, 1], # lb
    [1, 1, 1]  # rb
])
# theta_sign_vec = theta_sign.flatten()

class balance_trot:

    def __init__(self, robot_param, body_height, float_height, time_phi=0.2, gamma=0.5):
        
        #ロボットを持ってくる
        self.robot = robot_param
        
        #bodyの地面からの高さ
        self.z0 = body_height

        #脚上げ高さ
        self.fh = float_height
        
        #遊脚時間[s]
        self.T_phi = time_phi
        self.T_phi_control_step = int(self.T_phi/control_interval)
        
        #遊脚の上昇時間の比率 降下時間は(1-gamma)
        self.gam = gamma
        self.up_control_step = int(gamma*self.T_phi_control_step)
        
        #何回制御関数が呼ばれたか
        self.call_num = 0

        #制御が何ステップ目か
        self.control_step = 0

        #どの脚が遊脚か
        self.is_floating = [0, 1, 1, 0]

        #以前の脚先目標位置
        self.p_leg_prev = [
            [-robot.Lx, robot.Ly, -self.z0],
            [-robot.Lx, -robot.Ly, -self.z0],
            [robot.Lx, robot.Ly, -self.z0],
            [robot.Lx, -robot.Ly, -self.z0]
        ]

        #おしりの位置の真下
        self.p_hip = [
            [-robot.Lx, robot.Ly, -self.z0],
            [-robot.Lx, -robot.Ly, -self.z0],
            [robot.Lx, robot.Ly, -self.z0],
            [robot.Lx, -robot.Ly, -self.z0]
        ]

        #v_bodyローパスフィルタ用
        self.v_body_LPF = [0.0, 0.0, 0.0]
        self.v_body_LPF_last = [0.0, 0.0, 0.0]

    def set_leg_theta(self, p_leg, leg_num):

        #hipの根元から見た座標に変換
        p_leg_hip = IK.convert_xyz_body2leg(p_leg[0], p_leg[1], p_leg[2], leg_num)

        #IK
        thetas = IK.calc_theta1(p_leg_hip[0], p_leg_hip[1], p_leg_hip[2])

        #符号
        signs = theta_sign(leg_num)

        #各関節に角度をセット
        for joint in range(3):
                
            p.setJointMotorControl2(
                bodyUniqueId=self.robot.ID,
                jointIndex=self.robot.joint_IDs[leg_num][joint],
                controlMode=p.POSITION_CONTROL,
                targetPosition=signs[joint]*thetas[joint],
                force=FORCE_1
            )

    def stance_leg(self, v_body_d, v_body, iteration, num_goal, leg_num):

        p_goal = [0.0, 0.0, 0.0]
        p_goal[0] = self.p_hip[leg_num][0] - self.T_phi/2*v_body_d[0] - aa*(self.z0/(-world.g))**0.5*(v_body[0] - v_body_d[0])
        p_goal[1] = self.p_hip[leg_num][1] - self.T_phi/2*v_body_d[1] - aa*(self.z0/(-world.g))**0.5*(v_body[1] - v_body_d[1])
        p_goal[2] = self.p_hip[leg_num][2] - self.T_phi/2*v_body_d[2] - aa*(self.z0/(-world.g))**0.5*(v_body[2] - v_body_d[2])
        
        p_next = lp.linear_interpolation(self.p_leg_prev[leg_num], p_goal, iteration, num_goal)

        self.p_leg_prev[leg_num][0] = p_next[0]
        self.p_leg_prev[leg_num][1] = p_next[1]
        self.p_leg_prev[leg_num][2] = p_next[2]

        return p_next

    def float_up_leg(self, v_body_d, v_body, iteration, num_goal, leg_num):

        #脚上げ位置はhipの真下
        p_goal = [self.p_hip[leg_num][0], self.p_hip[leg_num][1], self.p_hip[leg_num][2] + self.fh]
        
        p_next = lp.linear_interpolation(self.p_leg_prev[leg_num], p_goal, iteration, num_goal)

        self.p_leg_prev[leg_num][0] = p_next[0]
        self.p_leg_prev[leg_num][1] = p_next[1]
        self.p_leg_prev[leg_num][2] = p_next[2]

        return p_next

    def float_down_leg(self, v_body_d, v_body, iteration, num_goal, leg_num):
        
        p_goal = [0.0, 0.0, 0.0]
        p_goal[0] = self.p_hip[leg_num][0] + self.T_phi/2*v_body_d[0] + aa*(self.z0/(-world.g))**0.5*(v_body[0] - v_body_d[0])
        p_goal[1] = self.p_hip[leg_num][1] + self.T_phi/2*v_body_d[1] + aa*(self.z0/(-world.g))**0.5*(v_body[1] - v_body_d[1])
        p_goal[2] = self.p_hip[leg_num][2] + self.T_phi/2*v_body_d[2] + aa*(self.z0/(-world.g))**0.5*(v_body[2] - v_body_d[2])

        p_next = lp.linear_interpolation(self.p_leg_prev[leg_num], p_goal, iteration, num_goal)

        self.p_leg_prev[leg_num][0] = p_next[0]
        self.p_leg_prev[leg_num][1] = p_next[1]
        self.p_leg_prev[leg_num][2] = p_next[2]

        return p_next

    def float_down_leg2(self, body_roll, v_body_d, v_body, iteration, num_goal, leg_num):
        
        p_goal = [0.0, 0.0, 0.0]
        p_goal[0] = self.p_hip[leg_num][0] + self.T_phi/2*v_body_d[0] + aa*(self.z0/(-world.g))**0.5*(v_body[0] - v_body_d[0]) + P_X*body_roll
        p_goal[1] = self.p_hip[leg_num][1] + self.T_phi/2*v_body_d[1] + aa*(self.z0/(-world.g))**0.5*(v_body[1] - v_body_d[1])
        p_goal[2] = self.p_hip[leg_num][2] + self.T_phi/2*v_body_d[2] + aa*(self.z0/(-world.g))**0.5*(v_body[2] - v_body_d[2]) - abs(P_Z*body_roll)

        p_next = lp.linear_interpolation(self.p_leg_prev[leg_num], p_goal, iteration, num_goal)

        self.p_leg_prev[leg_num][0] = p_next[0]
        self.p_leg_prev[leg_num][1] = p_next[1]
        self.p_leg_prev[leg_num][2] = p_next[2]

        print(p_next)
        
        return p_next

    #mainで毎ステップ呼ばれる関数 何もしない場合もある
    def walking(self, v_body_d):

        #制御周期が来たら
        if(self.call_num == control_world_step):

            #初期化
            self.call_num = 0
            
            #bodyの速度を取得
            v_body_world = p.getBaseVelocity(self.robot.ID)[0]
            q_body_world = p.getBasePositionAndOrientation(self.robot.ID)[1]
            rpy_body_world = p.getEulerFromQuaternion(q_body_world)
            v_body = p.invertTransform([-v_body_world[0], -v_body_world[1], -v_body_world[2]], q_body_world)[0]

            print(rpy_body_world[1])

            #ローパスフィルタ
            self.v_body_LPF[0] += k_LPF*(v_body[0] - self.v_body_LPF_last[0])
            self.v_body_LPF[1] += k_LPF*(v_body[1] - self.v_body_LPF_last[1])
            self.v_body_LPF[2] += k_LPF*(v_body[2] - self.v_body_LPF_last[2])
            self.v_body_LPF_last[0] = self.v_body_LPF[0]
            self.v_body_LPF_last[1] = self.v_body_LPF[1]
            self.v_body_LPF_last[2] = self.v_body_LPF[2]

            #脚ごとに制御を分ける
            for leg_num in range(4):

                if(self.is_floating[leg_num]):
                    
                    if(self.control_step < self.up_control_step):
                        
                        p_leg = self.float_up_leg(v_body_d, self.v_body_LPF, self.control_step, self.up_control_step, leg_num)
                        self.set_leg_theta(p_leg, leg_num)
                    
                    else:
                        
                        if(abs(rpy_body_world[1]) < pi/12):

                            p_leg = self.float_down_leg(v_body_d, self.v_body_LPF, self.control_step, self.T_phi_control_step, leg_num)

                        else:
                            p_leg = self.float_down_leg(v_body_d, self.v_body_LPF, self.control_step, self.T_phi_control_step, leg_num)
                            #p_leg = self.float_down_leg2(rpy_body_world[1], v_body_d, self.v_body_LPF, self.control_step, self.T_phi_control_step, leg_num)
                            
                        self.set_leg_theta(p_leg, leg_num)

                else:
                    
                    p_leg = self.stance_leg(v_body_d, self.v_body_LPF, self.control_step, self.T_phi_control_step, leg_num)
                    self.set_leg_theta(p_leg, leg_num)

            
            #T_phi経過後
            if(self.control_step == self.T_phi_control_step):

                #初期化
                self.control_step = 0

                #立脚と遊脚を切り替える
                self.is_floating = [self.is_floating[0]^1,
                                    self.is_floating[1]^1, 
                                    self.is_floating[2]^1, 
                                    self.is_floating[3]^1]

            self.control_step += 1

        self.call_num += 1
