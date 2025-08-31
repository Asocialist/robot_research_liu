#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
import matplotlib.patheffects as pe
from pathlib import Path

BASE_DIR = Path(__file__).resolve().parent
DATA_DIR = BASE_DIR.parent / "data"   # ../data

def load_person_traj():
    person_csv = DATA_DIR / "person_traj.csv"
    if not person_csv.exists():
        raise FileNotFoundError(f"找不到CSV: {person_csv}")
    df = pd.read_csv(person_csv)

    # 统一列名
    df = df.rename(columns={
        "field.x": "x",
        "field.y": "y",
        "field.theta": "theta",
        "%time": "time"
    })
    # 仅保留必要列，丢弃非法行
    need_cols = [c for c in ["x", "y", "theta", "time"] if c in df.columns]
    df = df[need_cols].dropna(subset=["x", "y"]).copy()

    # 若有 time 列则按时间排，确保首尾是“真实”起终点
    if "time" in df.columns:
        df = df.sort_values("time").reset_index(drop=True)

    return df

def plot_person_traj(df):
    if df.empty:
        raise ValueError("DataFrame 为空，无法绘图。")

    plt.figure(figsize=(6, 6))
    plt.xlabel("X position")
    plt.ylabel("Y position")
    plt.title("Person Trajectory")

    # 轨迹线（人物用暗黄色 goldenrod）
    plt.plot(df["x"], df["y"],
             color="goldenrod", marker=".", markersize=2, linewidth=0.8,
             label="Person trajectory")

    # 起点 / 终点（放在最后绘制，zorder 更高，白描边）
    x0, y0 = df["x"].iloc[0], df["y"].iloc[0]
    x1, y1 = df["x"].iloc[-1], df["y"].iloc[-1]

    plt.scatter(x0, y0, s=120, c="green", marker="o", zorder=10,
                edgecolors="white", linewidths=1.5, label="Start")
    plt.scatter(x1, y1, s=150, c="red", marker="X", zorder=10,
                edgecolors="white", linewidths=1.5, label="End")

    # 文字标注（带白描边，防止被线条或底图遮住）
    plt.annotate("Start", (x0, y0), xytext=(6, 6), textcoords="offset points",
                 path_effects=[pe.withStroke(linewidth=2, foreground="white")])
    plt.annotate("End", (x1, y1), xytext=(6, -10), textcoords="offset points",
                 path_effects=[pe.withStroke(linewidth=2, foreground="white")])

    # 若存在角度列，可选画朝向箭头（稀疏采样）
    if "theta" in df.columns and not df["theta"].isna().all():
        step = 25
        xq = df["x"].iloc[::step].to_numpy()
        yq = df["y"].iloc[::step].to_numpy()
        th = df["theta"].iloc[::step].to_numpy()
        u, v = np.cos(th), np.sin(th)
        plt.quiver(xq, yq, u, v, color='blue', scale=40, width=0.005, label='Heading')

    plt.axis("equal")
    plt.grid(True, linestyle="--", alpha=0.5)

    # 图例移至图表下方并居中；ncol 可按实际图例数量调整
    plt.legend(loc="upper center", bbox_to_anchor=(0.5, -0.07), ncol=4, frameon=False)

    # 底部留白，避免图例被裁掉
    plt.tight_layout(rect=[0, 0.08, 1, 1])

    out_path = DATA_DIR / "person_trajectory.png"
    plt.savefig(out_path, dpi=200)
    print(f"已保存: {out_path}")
    plt.show()

if __name__ == "__main__":
    df = load_person_traj()
    print("轨迹数据前5行：")
    print(df.head())
    plot_person_traj(df)
