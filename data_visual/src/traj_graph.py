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
        raise ValueError("DataFrame 为空，无法绘制。")

    plt.figure(figsize=(6, 6))

    # 绘制轨迹线
    plt.plot(df["x"], df["y"], marker="o", markersize=2, linewidth=1, label="robot trajectory")

    # 起点
    plt.scatter(df["x"].iloc[0], df["y"].iloc[0], c="green", s=80, marker="o", label="Start")
    plt.text(df["x"].iloc[0], df["y"].iloc[0], " Start", color="green", fontsize=10)

    # 终点
    plt.scatter(df["x"].iloc[-1], df["y"].iloc[-1], c="red", s=80, marker="X", label="End")
    plt.text(df["x"].iloc[-1], df["y"].iloc[-1], " End", color="red", fontsize=10)

    # 绘制机器人朝向箭头（稀疏取点避免太密）
    step = max(len(df) // 20, 1)  # 控制箭头数量，默认取大约20个
    plt.quiver(
        df["x"][::step], df["y"][::step],
        np.cos(df["theta"][::step]), np.sin(df["theta"][::step]),
        angles="xy", scale_units="xy", scale=5, width=0.003, color="blue", alpha=0.6,
        label="Direction"
    )

    plt.xlabel("X position")
    plt.ylabel("Y position")
    plt.title("Robot Trajectory")

    # 图例放到右下角
    plt.legend(loc="lower right")

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