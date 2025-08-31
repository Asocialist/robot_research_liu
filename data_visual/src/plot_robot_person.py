#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
绘制“机器人 + 人物”的XY轨迹到同一张图
- 默认从 ../data/ 读取 robot_traj.csv, person_traj.csv
- 统一列名：field.x -> x, field.y -> y, field.theta -> theta, %time -> time
- 若存在 time 列，会按时间升序排序，确保首尾点是“真实起终点”
- 图例放在图表下方；等比例坐标；可选朝向箭头
"""

from pathlib import Path
import argparse
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
import matplotlib.patheffects as pe

def load_csv(path):
    df = pd.read_csv(path)
    # 统一列名视图（不改变原表）
    colmap = {"field.x": "x", "field.y": "y", "field.theta": "theta", "%time": "time"}
    for k, v in colmap.items():
        if k in df.columns and v not in df.columns:
            df[v] = df[k]
    # 只保留需要的列
    keep = [c for c in ["x", "y", "theta", "time"] if c in df.columns]
    if not keep:
        raise ValueError(f"{path} 不包含必须的列（field.x/field.y 或 x/y）")
    df = df[keep].dropna(subset=["x", "y"]).copy()
    # 如有时间列则排序，确保首尾点合理
    if "time" in df.columns:
        df = df.sort_values("time").reset_index(drop=True)
    return df

def plot_robot_person(draw_orientation=True, step=20, outname="xy_robot_person.png"):
    # 路径：脚本在 src，数据在 ../data
    base_dir = Path(__file__).resolve().parent
    data_dir = base_dir.parent / "data"
    person_csv = data_dir / "person_traj.csv"
    robot_csv  = data_dir / "robot_traj.csv"

    if not person_csv.exists() or not robot_csv.exists():
        raise FileNotFoundError(f"找不到CSV：\n - {person_csv}\n - {robot_csv}")

    p = load_csv(person_csv)
    r = load_csv(robot_csv)

    # 提取
    px, py = p["x"].to_numpy(), p["y"].to_numpy()
    rx, ry = r["x"].to_numpy(), r["y"].to_numpy()
    pt = p["theta"].to_numpy() if "theta" in p.columns else None
    rt = r["theta"].to_numpy() if "theta" in r.columns else None

    # 开图
    plt.figure(figsize=(7, 7))
    # 机器人轨迹（蓝）
    plt.plot(rx, ry, color="royalblue", linewidth=1.0, label="Robot")
    # 人物轨迹（暗黄）
    plt.plot(px, py, color="goldenrod", linewidth=1.0, label="Person")

    # 起止点
    rx0, ry0, rx1, ry1 = rx[0], ry[0], rx[-1], ry[-1]
    px0, py0, px1, py1 = px[0], py[0], px[-1], py[-1]

    plt.scatter(rx0, ry0, s=80, c="navy", marker="o", zorder=10,
                edgecolors="white", linewidths=1.2, label="Robot Start")
    plt.scatter(rx1, ry1, s=90, c="navy", marker="s", zorder=10,
                edgecolors="white", linewidths=1.2, label="Robot End")

    plt.scatter(px0, py0, s=80, c="green", marker="o", zorder=10,
                edgecolors="white", linewidths=1.2, label="Person Start")
    plt.scatter(px1, py1, s=90, c="red", marker="X", zorder=10,
                edgecolors="white", linewidths=1.2, label="Person End")

    # 文本标注（白描边避免被遮挡）
    for (tx, ty, txt, dy) in [
        (rx0, ry0, "R-Start", 6),
        (rx1, ry1, "R-End",   6),
        (px0, py0, "P-Start", 6),
        (px1, py1, "P-End",   6),
    ]:
        plt.annotate(txt, (tx, ty), xytext=(6, dy), textcoords="offset points",
                     path_effects=[pe.withStroke(linewidth=2, foreground="white")],
                     fontsize=9)

    # 朝向箭头（可选）
    if draw_orientation:
        st = max(1, int(step))
        if rt is not None:
            for i in range(0, len(rx), st):
                dx, dy = 0.3*np.cos(rt[i]), 0.3*np.sin(rt[i])
                plt.arrow(rx[i], ry[i], dx, dy, head_width=0.08,
                          color="blue", alpha=0.6, length_includes_head=True)
        if pt is not None:
            for i in range(0, len(px), st):
                dx, dy = 0.3*np.cos(pt[i]), 0.3*np.sin(pt[i])
                plt.arrow(px[i], py[i], dx, dy, head_width=0.08,
                          color="green", alpha=0.6, length_includes_head=True)

    # 轴、网格、图例
    plt.gca().set_aspect("equal", adjustable="datalim")
    plt.xlabel("X")
    plt.ylabel("Y")
    plt.title("XY Trajectory (Robot & Person)")
    plt.grid(True, linestyle="--", alpha=0.5)

    # 图例放在图表下方
    # ncol=4 适合：Robot / Person / Robot Start / Robot End / Person Start / Person End (+ 可能的 Heading)
    # 可按实际图例数量微调
    plt.legend(loc="upper center", bbox_to_anchor=(0.5, -0.12), ncol=4, frameon=False)
    plt.tight_layout(rect=[0, 0.08, 1, 1])  # 底部留白，避免图例被裁掉

    out_path = data_dir / outname
    plt.savefig(out_path, dpi=200)
    plt.show()
    print(f"[OK] 已保存: {out_path}")

def main():
    ap = argparse.ArgumentParser(description="绘制 机器人+人物 轨迹到一张图")
    ap.add_argument("--no-arrow", action="store_true", help="关闭朝向箭头")
    ap.add_argument("--step", type=int, default=20, help="箭头步长（每N点画一次）")
    ap.add_argument("--out", default="xy_robot_person.png", help="输出文件名（保存在 ../data/ 下）")
    args = ap.parse_args()
    plot_robot_person(draw_orientation=not args.no_arrow, step=args.step, outname=args.out)

if __name__ == "__main__":
    main()
