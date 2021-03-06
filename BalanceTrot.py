# -*- coding: utf-8 -*-

import numpy as np
from math import *
import time
import IK
import LegPath as lp
import RobotController as rc
import KalmanMPU as kmpu

# 重力加速度[mm/s2]
g = 9800

# 制御の加わる周期[s]
control_interval = 100.0e-3

# ローパスフィルタの重み
# k_lpf = 0.1

# 姿勢角が横方向に倒れすぎると脚の接地位置を大きく出す
th_rough_control_roll = 1000

# #関節の回転正方向を決める行列
# theta_sign = np.array([
#     [1, 1, 1], # lf
#     [1, 1, 1], # rf
#     [1, 1, 1], # lb
#     [1, 1, 1]  # rb
# ])
# # theta_sign_vec = theta_sign.flatten()

k_s = np.array([0.2,0.2,1])*0
k_fd = np.array([0.2,0.2,1])*15

class trot:
    def __init__(self, _body_center, _body_height, _step_height, _target_vel, _time_phi=0.5, _gamma=0.5):
        self.body_center = _body_center # 予めarrayで入ってくる
        self.body_height = _body_height
        self.step_height = _step_height
        self.target_vel = _target_vel # 予めarrayで入ってくる
        self.time_phi = _time_phi # 遊脚時間[s]
        self.gamma = _gamma # 遊脚の上昇時間の比率比率
        # 遊脚期制御ステップ数
        self.phi_control_step = int(self.time_phi/control_interval)
        # 遊脚期脚上昇時の制御ステップ数．降下時間は(1-gamma)
        self.gamma_control_step = int(self.gamma*self.phi_control_step)
        # 制御が何ステップ目か
        self.control_step = 0
        # どの脚が遊脚か
        self.is_floating = np.array([0, 1, 1, 0]) # lf, rf, lb, rb
        # おしりの位置の真下位置をセット
        self.floor_hip = np.array([
            [IK.L/2.0-20, IK.W/2.0+35, -self.body_height],
            [IK.L/2.0-20, -IK.W/2.0-35, -self.body_height],
            [-IK.L/2.0-20, IK.W/2.0+35, -self.body_height],
            [-IK.L/2.0-20, -IK.W/2.0-35, -self.body_height]
        ])
        # self.floor_hip = np.array([
        #     [IK.L/2.0-10, IK.W/2.0, -self.body_height],
        #     [IK.L/2.0-10, -IK.W/2.0, -self.body_height],
        #     [-IK.L/2.0-40, IK.W/2.0, -self.body_height],
        #     [-IK.L/2.0-40, -IK.W/2.0, -self.body_height]
        # ])
        # 以前の脚先目標位置
        self.p_leg_prev = self.floor_hip.copy()

        self.body_IK = rc.bodyIK([
            [IK.L/2.0, IK.W/2.0, -self.body_height, 1],
            [IK.L/2.0, -IK.W/2.0, -self.body_height, 1],
            [-IK.L/2.0, IK.W/2.0, -self.body_height, 1],
            [-IK.L/2.0, -IK.W/2.0, -self.body_height, 1]
        ])
        
        # v_bodyローパスフィルタ用
        # self.v_body_LPF = [0.0, 0.0, 0.0]
        # self.v_body_LPF_last = [0.0, 0.0, 0.0]
        
    def stance_leg(self, v_body_obs, iteration, num_goal, leg_num):
        p_start = self.p_leg_prev[leg_num]
        p_goal = self.floor_hip[leg_num] - self.time_phi/2*self.target_vel - ((self.body_height + IK.R_TIP_BALL)/g)**0.5*(v_body_obs*k_s - self.target_vel)
        #print(p_goal)
        div_num = num_goal - iteration
        if(div_num <= 0):
            print("W: stance_leg:{}, div_num:{}".format(leg_num, div_num))
        div_vec = lp.linearDiv(p_start, p_goal, div_num)
        p_next = p_start + div_vec
        self.p_leg_prev[leg_num] = p_next
        
        return p_next

    def float_up_leg(self, v_body_obs, iteration, num_goal, leg_num):
        p_start = self.p_leg_prev[leg_num]
        # 真上に上げるだけ
        p_goal = p_goal = self.floor_hip[leg_num] + np.array([0, 0, self.step_height])
        div_num = num_goal - iteration
        if(div_num <= 0):
            print("W: float_up_leg:{}, div_num:{}".format(leg_num, div_num))
        div_vec = lp.linearDiv(p_start, p_goal, div_num)
        p_next = p_start + div_vec
        self.p_leg_prev[leg_num] = p_next
        
        return p_next

    def float_down_leg(self, v_body_obs, iteration, num_goal, leg_num):
        p_start = self.p_leg_prev[leg_num]
        p_goal = self.floor_hip[leg_num] + self.time_phi/2*self.target_vel + ((self.body_height + IK.R_TIP_BALL)/g)**0.5*(v_body_obs*k_fd - self.target_vel)
        #print( ((self.body_height + IK.R_TIP_BALL)/g)**0.5*(v_body_obs - self.target_vel)*k_fd )
        div_num = num_goal - iteration
        if(div_num <= 0):
            print("W: float_down_leg:{}, div_num:{}".format(leg_num, div_num))
        div_vec = lp.linearDiv(p_start, p_goal, div_num)
        p_next = p_start + div_vec
        self.p_leg_prev[leg_num] = p_next
        
        return p_next

    def float_down_leg_rough(self, v_body_obs, rpy_body_obs, iteration, num_goal, leg_num):
        p_start = self.p_leg_prev[leg_num]
        p_goal = self.floor_hip[leg_num] + self.time_phi/2*self.target_vel + ((self.body_height + IK.R_TIP_BALL)/g)**0.5*(v_body_obs - self.target_vel) + np.array([0, -rpy_body_obs[0]*5, 0])
        div_num = num_goal - iteration
        if(div_num <= 0):
            print("W: float_down_leg_rough:{}, div_num:{}".format(leg_num, div_num))
        div_vec = lp.linearDiv(p_start, p_goal, div_num)
        p_next = p_start + div_vec
        self.p_leg_prev[leg_num] = p_next
        
        return p_next

    # mainで毎ステップ呼ばれる関数 何もしない場合もある
    def walking(self, v_body_obs, rpy_body_obs):
        # ローパスフィルタ
        # self.v_body_LPF[0] += k_LPF*(v_body[0] - self.v_body_LPF_last[0])
        # self.v_body_LPF[1] += k_LPF*(v_body[1] - self.v_body_LPF_last[1])
        # self.v_body_LPF[2] += k_LPF*(v_body[2] - self.v_body_LPF_last[2])
        # self.v_body_LPF_last[0] = self.v_body_LPF[0]
        # self.v_body_LPF_last[1] = self.v_body_LPF[1]
        # self.v_body_LPF_last[2] = self.v_body_LPF[2]

        points_next = []        
        # 脚ごとに制御を分ける
        for leg_num in range(4):
            if(self.is_floating[leg_num]):
                if(self.control_step < self.gamma_control_step):
                    p_next = self.float_up_leg(v_body_obs, self.control_step, self.gamma_control_step, leg_num)
                else:
                    if(abs(rpy_body_obs[0]) < th_rough_control_roll):
                        p_next = self.float_down_leg(v_body_obs, self.control_step, self.phi_control_step, leg_num)
                    else:
                        p_next = self.float_down_leg_rough(v_body_obs, rpy_body_obs, self.control_step, self.phi_control_step, leg_num)
            else:
                p_next = self.stance_leg(v_body_obs, self.control_step, self.phi_control_step, leg_num)
            points_next.append(p_next)
            
        self.body_IK.Llf = np.append(points_next[0].copy(), 1)
        #print("self.body_IK.Llf:{}".format(self.body_IK.Llf))
        self.body_IK.Lrf = np.append(points_next[1].copy(), 1)
        self.body_IK.Llb = np.append(points_next[2].copy(), 1)
        self.body_IK.Lrb = np.append(points_next[3].copy(), 1)
        self.body_IK.calcLegPoseLF()
        self.body_IK.calcLegPoseRF()
        self.body_IK.calcLegPoseLB()
        self.body_IK.calcLegPoseRB()
        self.body_IK.setThetas()
        
        # T_phi経過後
        if(self.control_step == self.phi_control_step-1):
            # 初期化
            self.control_step = 0
            # 立脚と遊脚を切り替える
            self.is_floating = self.is_floating^1
            
        self.control_step += 1

def main():
    body_center = np.array([0, 0, 0])
    body_height = 170
    step_height = 40
    target_vel = np.array([0, 0, 0])
    trot_walk = trot(body_center, body_height, step_height, target_vel)

    # t_kf = kmpu.timer_kalman()
    # for i in range(2000):
    #     v_body_obs = np.array([0,0,0])
    #     rpy_body_obs = np.array([0, 0]) # [t_kf._kalAngleX, t_kf._kalAngleY])
    #     # print("rpy_body_obs:{}".format(rpy_body_obs))
    #     trot_walk.walking(v_body_obs, rpy_body_obs)
    #     time.sleep(0.05)
    
    t_kf = kmpu.timer_kalman()
    time.sleep(1)
    for i in range(20000):
        v_body_obs = np.array([t_kf.v_glob[0], t_kf.v_glob[1], 0])#np.array([t_kf._raw_datas[2][1], -t_kf._raw_datas[2][0], t_kf._raw_datas[2][2]])
        print("v:{}".format(v_body_obs))
        rpy_body_obs = np.array([t_kf._kalAngleX, t_kf._kalAngleY])
        # print("rpy_body_obs:{}".format(rpy_body_obs))
        trot_walk.walking(v_body_obs, rpy_body_obs)
        time.sleep(0.05)
        
    t_kf.pause_timer_kalman()
    
if __name__ == '__main__':
    main()
