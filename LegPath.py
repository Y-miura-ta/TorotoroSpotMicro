# -*- coding: utf-8 -*-

from mpl_toolkits import mplot3d
import numpy as np
from math import *
import matplotlib.pyplot as plt

# デバッグ用描画の設定
def setupView(limit):
    ax = plt.axes(projection="3d")
    ax.set_xlim(-limit, limit)
    ax.set_ylim(-limit, limit)
    ax.set_zlim(-limit, limit)
    ax.set_xlabel("X")
    ax.set_ylabel("Z")
    ax.set_zlabel("Y")
    return ax

# 目標位置までを分割数で割ったベクトルを返す
def linearDiv(p_start, p_goal, div_num):
    div_vec = (p_goal - p_start)/div_num

    return div_vec

# 分割ベクトルをゴールまで足していく
def vecIntegral(p_start, p_goal, p_now, div_vec, div_num, iter_now):
    # iter_nowは0スタートなので(div_num-1)でゴール位置に到達
    if(iter_now < div_num):
        p_next = p_now + div_vec

        return p_next
    else:
        p_next = p_goal

        return p_next


def main():
    # グラフで確認
    setupView(200).view_init(elev=12., azim=28)
    p_start = np.array([0, 100, 200])
    p_goal = np.array([10, 50, 80])
    div_num = 30

    div_vec = linearDiv(p_start, p_goal, div_num)
    p_next = p_start

    # 青:スタート、赤:補間点、緑:ゴール 
    plt.plot([p_start[0]], [p_start[1]], [p_start[2]], 'b*')

    for i in range(div_num):
        p_next = vecIntegral(p_start, p_goal, p_next, div_vec, div_num, i)
        plt.plot([p_next[0]], [p_next[1]], [p_next[2]], 'r*')
    
    plt.plot([p_goal[0]], [p_goal[1]], [p_goal[2]], 'g*')

    plt.show()
    
if __name__ == '__main__':
    main()