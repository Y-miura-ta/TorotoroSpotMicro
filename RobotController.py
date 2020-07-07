# -*- coding: utf-8 -*-

import numpy as np
from math import *
import time
import IK
import ServoController as sc

class bodyIK:
    def __init__(self, L_legs_start):
        self.Llf = L_legs_start[0]
        self.Lrf = L_legs_start[1]
        self.Llb = L_legs_start[2]
        self.Lrb = L_legs_start[3]

        self.Llf_ex = np.array([100, 100, -100, 1]) # leg_points[0]
        self.Lrf_ex = np.array([100, -100, -100, 1]) # leg_points[1]
        self.Llb_ex = np.array([-100, 100, -100, 1]) # leg_points[2]
        self.Lrb_ex = np.array([-100, -100, -100, 1]) # leg_points[3]

        self.thetas_lf = []
        self.thetas_rf = []
        self.thetas_lb = []
        self.thetas_rb = []
        self.thetas = []

        self.Ix = np.array([[-1,0,0,0],[0,1,0,0],[0,0,1,0],[0,0,0,1]])
        (self.Tlf, self.Trf, self.Tlb, self.Trb) = IK.bodyIK(0, 0, 0, 0, 0, 0)

    def setBodyPose(self, omega, phi, psi, xm, ym, zm):
        self.setT(omega, phi, psi, xm, ym, zm)
        self.calcLegPoseLF()
        self.calcLegPoseRF()
        self.calcLegPoseLB()
        self.calcLegPoseRB()

        self.setThetas()

    def setT(self, omega, phi, psi, xm, ym, zm):
        (self.Tlf, self.Trf, self.Tlb, self.Trb) = IK.bodyIK(omega, phi, psi, xm, ym, zm)

    # 逆運動学の解がない場合は例外用の座標を設定
    def calcLegPoseLF(self):
        try:
            self.thetas_lf = IK.legIK(np.linalg.inv(self.Tlf)@self.Llf)
            self.Llf_ex = self.Llf
        except ValueError:
            self.thetas_lf = IK.legIK(np.linalg.inv(self.Tlf)@self.Llf_ex)
            print("Left front leg uses except point")
            
    def calcLegPoseRF(self):
        try:
            self.thetas_rf = IK.legIK(self.Ix@np.linalg.inv(self.Trf)@self.Lrf)
            self.Lrf_ex = self.Lrf
        except ValueError:
            self.thetas_rf = IK.legIK(self.Ix@np.linalg.inv(self.Trf)@self.Lrf_ex)
            print("Right front leg uses except point")

    def calcLegPoseLB(self):
        try:
            self.thetas_lb = IK.legIK(np.linalg.inv(self.Tlb)@self.Llb)
            self.Llb_ex = self.Llb
        except ValueError:
            self.thetas_lb = IK.legIK(np.linalg.inv(self.Tlb)@self.Llb_ex)
            print("Left back leg uses except point")

    def calcLegPoseRB(self):
        try:
            self.thetas_rb = IK.legIK(self.Ix@np.linalg.inv(self.Trb)@self.Lrb)
            self.Lrb_ex = self.Lrb
        except ValueError:
            self.thetas_rb = IK.legIK(self.Ix@np.linalg.inv(self.Trb)@self.Lrb_ex)
            print("Right back leg uses except point")

    def setThetas(self):
        #1行のベクトルに直す
        self.thetas = np.array((self.thetas_lf, self.thetas_rf, self.thetas_lb, self.thetas_rb)).flatten()
        sc.setRadThetas(self.thetas)
        
def main():
    start_time = time.time()
    print("Start main RobotCntroller.py:{}".format(start_time))
    Lp=np.array([[100, 100, -10000, 1], [100, -100, -100, 1], [-100, 100, -100, 1], [-100, -100, -100, 1]])
    body = bodyIK(Lp)
    body.setBodyPose(0.4, 0.4, -0.4, 10.0, 0.0, 0.0)
    end_time = time.time()
    print("End main RobotCntroller.py:{}".format(end_time))
    print("IK dt:{} sec".format(end_time - start_time))
    # for i in range(10):
    #     body.setBodyPose(0.0, 0.4/100*i, 0.0, 0, 0, 0)
    #     time.sleep(0.01)
    # for i in range(100):
    #     body.setBodyPose(0.0, 0.4/100*(99-i), 0.0, 0, 0, 0)
    #     time.sleep(0.01)
    # for i in range(100):
    #     body.setBodyPose(0.0, 0.0, 0.4/100*i, 0, 0, 0)
    #     time.sleep(0.01)
    # for i in range(10):
    #     body.setBodyPose(0.0, 0.0, 0.4/100*(99-i), 0, 0, 0)
    #     time.sleep(0.01)
    
if __name__ == '__main__':
    main()
