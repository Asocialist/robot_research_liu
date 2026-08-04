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
import os
import numpy as np
import pandas as pd
import matplotlib.patheffects as pe

import matplotlib

if not os.environ.get("DISPLAY"):
    matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.collections import LineCollection

DEFAULT_OBJECT_POINTS = {
    "001": (0.846185, 1.080893),
    "002": (-1.031481, -1.772665),
    "003": (3.465035, -1.117952),
}

def resolve_formation_csv(data_dir):
    candidates = [
        data_dir / "formation_topic.csv",
        data_dir.parent.parent / "bag_topic_exports" / data_dir.name / "formation_topic.csv",
    ]
    for path in candidates:
        if path.exists():
            return path
    return None

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

def match_reference_by_time(track_df, ref_df):
    if "time" not in track_df.columns or "time" not in ref_df.columns:
        raise ValueError("按时间对齐需要 track_df 和 ref_df 都包含 time 列")

    ref_time = ref_df["time"].to_numpy()
    track_time = track_df["time"].to_numpy()
    idx = np.searchsorted(ref_time, track_time, side="left")
    idx = np.clip(idx, 0, len(ref_df) - 1)
    prev_idx = np.clip(idx - 1, 0, len(ref_df) - 1)

    use_prev = np.abs(ref_time[prev_idx] - track_time) <= np.abs(ref_time[idx] - track_time)
    nearest_idx = np.where(use_prev, prev_idx, idx)
    return ref_df.iloc[nearest_idx].reset_index(drop=True)

def align_track_to_reference(track_df, ref_df, mode):
    if mode == "none":
        return track_df
    if mode == "start":
        dx = ref_df["x"].iloc[0] - track_df["x"].iloc[0]
        dy = ref_df["y"].iloc[0] - track_df["y"].iloc[0]
    elif mode == "end":
        dx = ref_df["x"].iloc[-1] - track_df["x"].iloc[-1]
        dy = ref_df["y"].iloc[-1] - track_df["y"].iloc[-1]
    else:
        raise ValueError(f"未知对齐模式: {mode}")

    aligned = track_df.copy()
    aligned["x"] = aligned["x"] + dx
    aligned["y"] = aligned["y"] + dy
    print(f"[INFO] apply alignment mode={mode}, dx={dx:.6f}, dy={dy:.6f}")
    return aligned

def transform_person_local_to_world(person_df, robot_df, mode):
    if mode == "none":
        return person_df
    if mode != "robot-local":
        raise ValueError(f"未知人物坐标系模式: {mode}")

    matched_robot = match_reference_by_time(person_df, robot_df)
    transformed = person_df.copy()

    cos_theta = np.cos(matched_robot["theta"].to_numpy())
    sin_theta = np.sin(matched_robot["theta"].to_numpy())
    px = person_df["x"].to_numpy()
    py = person_df["y"].to_numpy()

    transformed["x"] = matched_robot["x"].to_numpy() + cos_theta * px - sin_theta * py
    transformed["y"] = matched_robot["y"].to_numpy() + sin_theta * px + cos_theta * py
    if "theta" in transformed.columns:
        transformed["theta"] = matched_robot["theta"].to_numpy() + transformed["theta"].to_numpy()

    print(f"[INFO] transform person trajectory from {mode} to world coordinates")
    return transformed

def plot_object_points(object_points):
    for label, (x, y) in object_points.items():
        plt.scatter(
            x, y, s=110, c="black", marker="D", zorder=12,
            edgecolors="white", linewidths=1.0,
        )
        plt.annotate(
            label, (x, y), xytext=(6, 6), textcoords="offset points",
            path_effects=[pe.withStroke(linewidth=2, foreground="white")],
            fontsize=9,
        )

def first_f_formation_time(data_dir):
    formation_csv = resolve_formation_csv(data_dir)
    if formation_csv is None:
        print("[WARN] formation_topic.csv not found; robot trajectory will use a single color")
        return None

    df = pd.read_csv(formation_csv)
    if "data" not in df.columns or "time" not in df.columns:
        print(f"[WARN] invalid formation_topic.csv format: {formation_csv}")
        return None

    matches = df["data"].astype(str).str.contains("formation_flag: True", na=False)
    if not matches.any():
        print(f"[WARN] no F-formation entry found in {formation_csv}")
        return None

    event_time = float(df.loc[matches, "time"].iloc[0])
    print(f"[INFO] first F-formation time={event_time:.6f} from {formation_csv}")
    return event_time

def plot_robot_segments_by_time(x, y, time, split_time):
    points = np.column_stack([x, y])
    if len(points) < 2:
        plt.plot(x, y, color="royalblue", linewidth=1.0, label="Robot")
        return

    if split_time is None or time is None:
        plt.plot(x, y, color="royalblue", linewidth=1.0, label="Robot")
        return

    segments = np.stack([points[:-1], points[1:]], axis=1)
    seg_times = time[1:]
    before = seg_times < split_time
    after = ~before

    if before.any():
        before_lc = LineCollection(segments[before], colors="royalblue", linewidths=1.0, label="Robot")
        plt.gca().add_collection(before_lc)
    if after.any():
        after_lc = LineCollection(segments[after], colors="#f4a3a3", linewidths=1.6, label="Robot In F-formation")
        plt.gca().add_collection(after_lc)

def plot_robot_person(
    data_dir,
    draw_orientation=True,
    step=20,
    robot_step=60,
    outname="xy_robot_person.png",
    show=True,
    align_person="none",
    person_frame="robot-local",
    object_points=None,
):
    data_dir = Path(data_dir)
    person_csv = data_dir / "person_traj.csv"
    robot_csv  = data_dir / "robot_traj.csv"

    if not person_csv.exists() or not robot_csv.exists():
        raise FileNotFoundError(f"找不到CSV：\n - {person_csv}\n - {robot_csv}")

    p = load_csv(person_csv)
    r = load_csv(robot_csv)
    p = transform_person_local_to_world(p, r, person_frame)
    p = align_track_to_reference(p, r, align_person)

    # 提取
    px, py = p["x"].to_numpy(), p["y"].to_numpy()
    rx, ry = r["x"].to_numpy(), r["y"].to_numpy()
    pt = p["theta"].to_numpy() if "theta" in p.columns else None
    rt = r["theta"].to_numpy() if "theta" in r.columns else None
    rtime = r["time"].to_numpy() if "time" in r.columns else None
    f_formation_time = first_f_formation_time(data_dir)

    # 开图
    plt.figure(figsize=(7, 7))
    # 机器人轨迹：进入F阵型后改为浅红色
    plot_robot_segments_by_time(rx, ry, rtime, f_formation_time)
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

    if object_points:
        plot_object_points(object_points)

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
        robot_st = max(1, int(robot_step))
        person_st = max(1, int(step))
        if rt is not None:
            for i in range(0, len(rx), robot_st):
                dx, dy = 0.3*np.cos(rt[i]), 0.3*np.sin(rt[i])
                plt.arrow(rx[i], ry[i], dx, dy, head_width=0.08,
                          color="blue", alpha=0.6, length_includes_head=True)
        if pt is not None:
            for i in range(0, len(px), person_st):
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
    if show:
        plt.show()
    plt.close()
    print(f"[OK] 已保存: {out_path}")

def main():
    ap = argparse.ArgumentParser(description="绘制 机器人+人物 轨迹到一张图")
    ap.add_argument("--data-dir", default=str(Path(__file__).resolve().parent.parent / "data"),
                    help="输入/输出数据目录")
    ap.add_argument("--no-arrow", action="store_true", help="关闭朝向箭头")
    ap.add_argument("--no-show", action="store_true", help="仅保存图片，不弹出窗口")
    ap.add_argument("--step", type=int, default=30, help="箭头步长（每N点画一次）")
    ap.add_argument("--align-person", choices=["none", "start", "end"], default="none",
                    help="将人物轨迹平移到机器人轨迹上进行对齐")
    ap.add_argument("--person-frame", choices=["world", "robot-local"], default="robot-local",
                    help="人物轨迹输入坐标系；robot-local 会按机器人位姿转换到全局系")
    ap.add_argument("--out", default="xy_robot_person.png", help="输出文件名（保存在 data-dir 下）")
    args = ap.parse_args()
    plot_robot_person(args.data_dir, draw_orientation=not args.no_arrow, step=args.step,
                      outname=args.out, show=not args.no_show, align_person=args.align_person,
                      person_frame=args.person_frame, object_points=DEFAULT_OBJECT_POINTS)

if __name__ == "__main__":
    main()
