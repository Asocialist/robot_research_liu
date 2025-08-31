#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import pandas as pd
import matplotlib.pyplot as plt
import numpy as np
from pathlib import Path

def plot_xy(draw_orientation=True, step=20):
    # 获取当前脚本所在目录
    BASE_DIR = Path(__file__).resolve().parent
    DATA_DIR = BASE_DIR.parent / "data"   # ../data

    # CSV 文件路径
    person_csv = DATA_DIR / "person_traj.csv"
    robot_csv  = DATA_DIR / "robot_traj.csv"

    # 读取数据
    person = pd.read_csv(person_csv)
    robot  = pd.read_csv(robot_csv)

    # 提取列
    px, py, pt = person["field.x"], person["field.y"], person["field.theta"]
    rx, ry, rt = robot["field.x"],  robot["field.y"],  robot["field.theta"]

    # 绘图
    plt.figure(figsize=(7,7))
    plt.plot(rx, ry, label="Robot")
    plt.plot(px, py, label="Person")

    # 起止点标记
    plt.scatter([rx.iloc[0]],[ry.iloc[0]], marker="o", s=60, label="Robot Start")
    plt.scatter([rx.iloc[-1]],[ry.iloc[-1]], marker="s", s=60, label="Robot End")
    plt.scatter([px.iloc[0]],[py.iloc[0]], marker="o", s=60, label="Person Start")
    plt.scatter([px.iloc[-1]],[py.iloc[-1]], marker="s", s=60, label="Person End")

    # ===== 在轨迹上加箭头 =====
    if draw_orientation:
        # Robot
        for i in range(0, len(rx), step):
            dx, dy = 0.3*np.cos(rt.iloc[i]), 0.3*np.sin(rt.iloc[i])
            plt.arrow(rx.iloc[i], ry.iloc[i], dx, dy, 
                      head_width=0.1, color="blue", alpha=0.6)
        # Person
        for i in range(0, len(px), step):
            dx, dy = 0.3*np.cos(pt.iloc[i]), 0.3*np.sin(pt.iloc[i])
            plt.arrow(px.iloc[i], py.iloc[i], dx, dy, 
                      head_width=0.1, color="green", alpha=0.6)

    plt.gca().set_aspect("equal", adjustable="datalim")
    plt.xlabel("X")
    plt.ylabel("Y")
    plt.title("XY Trajectory (Robot & Person)")
    plt.legend(loc="best")
    plt.tight_layout()

    # 保存到 data 文件夹下
    out_path = DATA_DIR / "xy_trajectory_with_theta.png"
    plt.savefig(out_path, dpi=200)
    plt.show()
    print(f"✅ 带朝向的轨迹图已保存到: {out_path}")

if __name__ == "__main__":
    # 默认开启画朝向，步长=20（即每20帧画一个箭头）
    plot_xy(draw_orientation=True, step=20)
