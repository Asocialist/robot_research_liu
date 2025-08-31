#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from pathlib import Path


BASE_DIR = Path(__file__).resolve().parent
DATA_DIR = BASE_DIR.parent / "data"   # ../data

def load_robot_traj():
    robot_csv = DATA_DIR / "robot_traj.csv"
    if not robot_csv.exists():
        raise FileNotFoundError(f"找不到CSV: {robot_csv}")
    
    df = pd.read_csv(robot_csv)

    # 把原始列名改成更方便使用的名字
    df = df.rename(columns={
        "field.x": "x",
        "field.y": "y",
        "field.theta": "theta",
        "%time": "time"
    })

    return df

def plot_robot_traj(df):
    if df.empty:
        raise ValueError("DataFrame is empty exit")

    plt.figure(figsize=(6, 6))
    plt.xlabel("X position")
    plt.ylabel("Y position")
    plt.title("Robot Trajectory")

    # 绘制轨迹线
    plt.plot(df["x"], df["y"], marker=".", markersize=2, linewidth=0.5, label="robot trajectory")

    # 起点
    plt.scatter(df["x"].iloc[0], df["y"].iloc[0], c="green", s=80, marker="o", label="Start")
    # plt.text(df["x"].iloc[0], df["y"].iloc[0], " Start", color="green", fontsize=10)


    # 终点
    plt.scatter(df["x"].iloc[-1], df["y"].iloc[-1], c="red", s=80, marker="X", label="End")
    # plt.text(df["x"].iloc[-1], df["y"].iloc[-1], " End", color="red", fontsize=10)

    a_step = 25 
    x_q = df["x"][::a_step]
    y_q = df["y"][::a_step]
    theta_q = df["theta"][::a_step]

    # 2. 根据角度 theta 计算箭头的 X (u) 和 Y (v) 方向分量
    u_q = np.cos(theta_q)
    v_q = np.sin(theta_q)

    # 3. 使用 plt.quiver 绘制箭头
    #    scale 控制箭头的大小 (值越大，箭头越短)
    #    width 控制箭头的线条宽度
    plt.quiver(x_q, y_q, u_q, v_q, 
               color='blue', 
               scale=40, 
               width=0.005,
               label='Heading')
    

    # 图例放到右下角
    plt.legend(loc="upper center", bbox_to_anchor=(0.5, -0.07), ncol=4, frameon=False)

    plt.axis("equal")
    plt.grid(True)

    out_path = DATA_DIR / "robot_trajectory.png"
    plt.savefig(out_path, dpi=200)
    print(f"已保存: {out_path}")

    plt.show()


if __name__ == "__main__":   
    df = load_robot_traj()
    print("轨迹数据前5行：")
    print(df.head())
    plot_robot_traj(df)