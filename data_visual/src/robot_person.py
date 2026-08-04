#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import pandas as pd
import numpy as np
import argparse
import os
from pathlib import Path

import matplotlib

if not os.environ.get("DISPLAY"):
    matplotlib.use("Agg")
import matplotlib.pyplot as plt

def plot_xy(data_dir, draw_orientation=True, step=20, show=True):
    data_dir = Path(data_dir)

    # CSV 文件路径
    person_csv = data_dir / "person_traj.csv"
    robot_csv  = data_dir / "robot_traj.csv"

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
    out_path = data_dir / "xy_trajectory_with_theta.png"
    plt.savefig(out_path, dpi=200)
    if show:
        plt.show()
    plt.close()
    print(f"✅ 带朝向的轨迹图已保存到: {out_path}")

def main():
    parser = argparse.ArgumentParser(description="绘制基础版机器人/人物轨迹图")
    parser.add_argument("--data-dir", default=str(Path(__file__).resolve().parent.parent / "data"),
                        help="输入/输出数据目录")
    parser.add_argument("--step", type=int, default=20, help="箭头步长（每N点画一次）")
    parser.add_argument("--no-arrow", action="store_true", help="关闭朝向箭头")
    parser.add_argument("--no-show", action="store_true", help="仅保存图片，不弹出窗口")
    args = parser.parse_args()
    plot_xy(args.data_dir, draw_orientation=not args.no_arrow, step=args.step, show=not args.no_show)

if __name__ == "__main__":
    main()
