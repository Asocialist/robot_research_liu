#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import pandas as pd
import matplotlib.pyplot as plt
import numpy as np
from pathlib import Path
import matplotlib.patches as mpatches
from matplotlib.legend_handler import HandlerPatch


def normalize_angle(a: np.ndarray) -> np.ndarray:
    """Normalize angles to [-pi, pi]."""
    return (a + np.pi) % (2.0 * np.pi) - np.pi


def find_time_column(df: pd.DataFrame) -> str | None:
    """Try to find a timestamp column in typical ROS CSV exports."""
    candidates = [
        "%time", "time", "stamp",
        "field.header.stamp",
        "field.header.stamp.secs", "field.header.stamp.nsecs",
        "header.stamp.secs", "header.stamp.nsecs",
    ]
    for c in candidates:
        if c in df.columns:
            return c
    for c in df.columns:
        lc = c.lower()
        if "stamp" in lc or "time" in lc:
            return c
    return None


def build_time_array(df: pd.DataFrame) -> np.ndarray | None:
    """Build float time array (seconds). Supports secs+nsecs pair or a single time column."""
    if "field.header.stamp.secs" in df.columns and "field.header.stamp.nsecs" in df.columns:
        return (df["field.header.stamp.secs"].to_numpy(dtype=np.float64) +
                df["field.header.stamp.nsecs"].to_numpy(dtype=np.float64) * 1e-9)

    if "header.stamp.secs" in df.columns and "header.stamp.nsecs" in df.columns:
        return (df["header.stamp.secs"].to_numpy(dtype=np.float64) +
                df["header.stamp.nsecs"].to_numpy(dtype=np.float64) * 1e-9)

    tc = find_time_column(df)
    if tc is None:
        return None

    t = df[tc].to_numpy(dtype=np.float64)
    # Heuristic: nanoseconds-like -> seconds
    if np.nanmedian(t) > 1e12:
        t = t * 1e-9
    return t


def pick_xy_columns(df: pd.DataFrame):
    """Pick x/y column names from common patterns."""
    x_candidates = ["field.x", "x", "X"]
    y_candidates = ["field.y", "y", "Y"]
    xcol = next((c for c in x_candidates if c in df.columns), None)
    ycol = next((c for c in y_candidates if c in df.columns), None)
    if xcol is None or ycol is None:
        raise KeyError(f"Cannot find x/y columns in {list(df.columns)}")
    return xcol, ycol


def pick_name_column(df: pd.DataFrame) -> str | None:
    """Pick name column if present."""
    candidates = ["name", "field.name", "field.name.data"]
    return next((c for c in candidates if c in df.columns), None)


def match_indices_by_time(tp: np.ndarray, tr: np.ndarray, tol: float = 0.05):
    """
    For each person timestamp tp[k], find nearest robot timestamp in tr.
    Keep pairs whose |tp - tr| <= tol (seconds).

    Returns: (person_indices, robot_indices)
    """
    if tp is None or tr is None or len(tp) == 0 or len(tr) == 0:
        return None, None

    # sort robot time for fast search
    order = np.argsort(tr)
    tr_s = tr[order]

    idx = np.searchsorted(tr_s, tp, side="left")
    idx = np.clip(idx, 0, len(tr_s) - 1)

    prev = np.clip(idx - 1, 0, len(tr_s) - 1)
    choose_prev = np.abs(tp - tr_s[prev]) < np.abs(tp - tr_s[idx])
    idx = np.where(choose_prev, prev, idx)

    robot_idx = order[idx]
    ok = np.abs(tp - tr[robot_idx]) <= tol

    return np.where(ok)[0], robot_idx[ok]


def transform_person_local_to_global(person: pd.DataFrame, robot: pd.DataFrame) -> pd.DataFrame:
    """Transform person (x,y,theta) from robot-local to global using robot pose."""
    px = person["field.x"].to_numpy(dtype=np.float64)
    py = person["field.y"].to_numpy(dtype=np.float64)
    pt = person["field.theta"].to_numpy(dtype=np.float64)

    rx = robot["field.x"].to_numpy(dtype=np.float64)
    ry = robot["field.y"].to_numpy(dtype=np.float64)
    rt = robot["field.theta"].to_numpy(dtype=np.float64)

    tp = build_time_array(person)
    tr = build_time_array(robot)

    if tp is not None and tr is not None and len(tp) > 0 and len(tr) > 0:
        # Nearest neighbor sync by time
        order = np.argsort(tr)
        tr_s = tr[order]
        rx_s, ry_s, rt_s = rx[order], ry[order], rt[order]

        idx = np.searchsorted(tr_s, tp, side="left")
        idx = np.clip(idx, 0, len(tr_s) - 1)

        prev = np.clip(idx - 1, 0, len(tr_s) - 1)
        choose_prev = np.abs(tp - tr_s[prev]) < np.abs(tp - tr_s[idx])
        idx = np.where(choose_prev, prev, idx)

        rxi, ryi, rti = rx_s[idx], ry_s[idx], rt_s[idx]
    else:
        # Index-based alignment
        n = min(len(px), len(rx))
        px, py, pt = px[:n], py[:n], pt[:n]
        rxi, ryi, rti = rx[:n], ry[:n], rt[:n]

    c, s = np.cos(rti), np.sin(rti)
    gx = px * c - py * s + rxi
    gy = px * s + py * c + ryi
    gt = normalize_angle(pt + rti)

    out = person.copy()
    out["field.x"] = gx
    out["field.y"] = gy
    out["field.theta"] = gt
    return out


def load_objectpoints(data_dir: Path) -> pd.DataFrame | None:
    """
    Try to load objectpoint CSV from common filenames.
    Return dataframe or None if not found.
    """
    candidates = [
        data_dir / "objectpoint_traj.csv",
        data_dir / "closest_objectpoint.csv",
        data_dir / "objectpoint.csv",
    ]
    for p in candidates:
        if p.exists():
            df = pd.read_csv(p)
            df.attrs["__src_path__"] = str(p)
            return df
    return None


def plot_xy(
    draw_orientation: bool = True,
    robot_step: int = 20,          # ✅ robot arrows step
    person_step: int = 20,         # ✅ person arrows step
    person_is_local: bool = True,
    draw_objectpoint: bool = True,
    objectpoint_unique: bool = True,
    objectpoint_annotate_name: bool = False,
    heading_time_sync: bool = True,
    heading_tol_s: float = 0.1,
):
    BASE_DIR = Path(__file__).resolve().parent
    DATA_DIR = BASE_DIR.parent / "data"

    person_csv = DATA_DIR / "pose_person_following.csv"
    robot_csv  = DATA_DIR / "pose_particle_localizer.csv"

    person = pd.read_csv(person_csv)
    robot  = pd.read_csv(robot_csv)

    # Person local->global if needed
    if person_is_local:
        person = transform_person_local_to_global(person, robot)

    # Extract robot/person
    px, py, pt = person["field.x"], person["field.y"], person["field.theta"]
    rx, ry, rt = robot["field.x"],  robot["field.y"],  robot["field.theta"]

    plt.figure(figsize=(7, 7))
    plt.plot(rx, ry, label="Robot")
    plt.plot(px, py, label="Person (Global)")

    # Start/end markers
    plt.scatter([rx.iloc[0]], [ry.iloc[0]], marker="o", s=60, label="Robot Start")
    plt.scatter([rx.iloc[-1]], [ry.iloc[-1]], marker="s", s=60, label="Robot End")
    plt.scatter([px.iloc[0]], [py.iloc[0]], marker="o", s=60, label="Person Start")
    plt.scatter([px.iloc[-1]], [py.iloc[-1]], marker="s", s=60, label="Person End")

    # Objectpoints
    if draw_objectpoint:
        op = load_objectpoints(DATA_DIR)
        if op is None:
            print("⚠️  No objectpoint CSV found (objectpoint_traj.csv / closest_objectpoint.csv / objectpoint.csv). Skip drawing.")
        else:
            xcol, ycol = pick_xy_columns(op)
            namecol = pick_name_column(op)

            op_plot = op[[xcol, ycol] + ([namecol] if namecol else [])].copy()

            # Remove duplicates if the same objectpoint is published repeatedly
            if objectpoint_unique:
                op_plot["_xr"] = op_plot[xcol].round(3)
                op_plot["_yr"] = op_plot[ycol].round(3)
                if namecol:
                    op_plot = op_plot.drop_duplicates(subset=["_xr", "_yr", namecol])
                else:
                    op_plot = op_plot.drop_duplicates(subset=["_xr", "_yr"])
                op_plot = op_plot.drop(columns=["_xr", "_yr"])

            plt.scatter(op_plot[xcol], op_plot[ycol], marker="*", s=120, label="Objectpoint")

            if objectpoint_annotate_name and namecol:
                for _, r in op_plot.iterrows():
                    plt.text(r[xcol], r[ycol], str(r[namecol]), fontsize=8)

            print(f"✅ Objectpoints loaded from: {op.attrs.get('__src_path__')} (N={len(op_plot)})")

    # Orientation arrows
    if draw_orientation:
        if heading_time_sync:
            tp = build_time_array(person)
            tr = build_time_array(robot)
            p_idx, r_idx = match_indices_by_time(tp, tr, tol=heading_tol_s)

            if p_idx is not None and len(p_idx) > 0:
                # ✅ Robot arrows (use robot_step)
                for k in range(0, len(p_idx), max(1, robot_step)):
                    i = int(r_idx[k])  # robot index
                    dx, dy = 0.3 * np.cos(rt.iloc[i]), 0.3 * np.sin(rt.iloc[i])
                    plt.arrow(rx.iloc[i], ry.iloc[i], dx, dy,
                              head_width=0.1, color="blue", alpha=0.6)

                # ✅ Person arrows (use person_step)
                for k in range(0, len(p_idx), max(1, person_step)):
                    j = int(p_idx[k])  # person index
                    dx, dy = 0.3 * np.cos(pt.iloc[j]), 0.3 * np.sin(pt.iloc[j])
                    plt.arrow(px.iloc[j], py.iloc[j], dx, dy,
                              head_width=0.1, color="green", alpha=0.6)

                print(f"✅ Draw headings with time-sync (tol={heading_tol_s}s), matched N={len(p_idx)}")
            else:
                print("⚠️  No time-synced samples found. "
                      "Check timestamps or increase heading_tol_s. Fallback to index-based arrows.")

                # Fallback: index-based arrows with separate steps
                for i in range(0, len(rx), max(1, robot_step)):
                    dx, dy = 0.3 * np.cos(rt.iloc[i]), 0.3 * np.sin(rt.iloc[i])
                    plt.arrow(rx.iloc[i], ry.iloc[i], dx, dy, head_width=0.1, color="blue", alpha=0.6)

                for i in range(0, len(px), max(1, person_step)):
                    dx, dy = 0.3 * np.cos(pt.iloc[i]), 0.3 * np.sin(pt.iloc[i])
                    plt.arrow(px.iloc[i], py.iloc[i], dx, dy, head_width=0.1, color="green", alpha=0.6)
        else:
            # Original index-based drawing (but with separate steps)
            for i in range(0, len(rx), max(1, robot_step)):
                dx, dy = 0.3 * np.cos(rt.iloc[i]), 0.3 * np.sin(rt.iloc[i])
                plt.arrow(rx.iloc[i], ry.iloc[i], dx, dy, head_width=0.1, color="blue", alpha=0.6)

            for i in range(0, len(px), max(1, person_step)):
                dx, dy = 0.3 * np.cos(pt.iloc[i]), 0.3 * np.sin(pt.iloc[i])
                plt.arrow(px.iloc[i], py.iloc[i], dx, dy, head_width=0.1, color="green", alpha=0.6)

    plt.gca().set_aspect("equal", adjustable="datalim")
    plt.xlabel("X")
    plt.ylabel("Y")
    plt.title("XY Trajectory (Robot & Person in Global Frame)")

    # ====== Legend: add arrow proxies ======
    def make_legend_arrow(legend, orig_handle, xdescent, ydescent, width, height, fontsize):
        # 尽量拿到原 proxy 的颜色
        col = None
        try:
            col = orig_handle.get_edgecolor()
        except Exception:
            col = None
        if col is None or (hasattr(col, "__len__") and len(col) == 0):
            try:
                col = orig_handle.get_facecolor()
            except Exception:
                col = "k"

        return mpatches.FancyArrowPatch(
            (xdescent, ydescent + height / 2.0),
            (xdescent + width, ydescent + height / 2.0),
            arrowstyle='-|>',
            mutation_scale=fontsize * 1.3,
            lw=2,
            color=col
        )

    # proxies (legend only)
    robot_heading_proxy = mpatches.FancyArrowPatch((0, 0), (1, 0),
                                                  arrowstyle='-|>', mutation_scale=15,
                                                  color='blue', lw=2)
    person_heading_proxy = mpatches.FancyArrowPatch((0, 0), (1, 0),
                                                   arrowstyle='-|>', mutation_scale=15,
                                                   color='green', lw=2)

    ax = plt.gca()
    handles, labels = ax.get_legend_handles_labels()
    handles += [robot_heading_proxy, person_heading_proxy]
    labels  += ["Robot Heading", "Person Heading"]

    plt.legend(
        handles, labels,
        loc="upper center",
        bbox_to_anchor=(0.5, -0.10),
        ncol=3,
        frameon=False,
        handler_map={mpatches.FancyArrowPatch: HandlerPatch(patch_func=make_legend_arrow)}
    )

    plt.tight_layout()
    plt.subplots_adjust(bottom=0.22)

    out_path = DATA_DIR / "xy_trajectory_with_theta.png"
    plt.savefig(out_path, dpi=300, bbox_inches="tight")
    plt.show()
    print(f"✅ Saved: {out_path}")


if __name__ == "__main__":
    plot_xy(
        draw_orientation=True,
        robot_step=600,          # ✅ 机器人箭头步长
        person_step=1000,         # ✅ 人物箭头步长（可不同）
        person_is_local=True,
        draw_objectpoint=True,
        objectpoint_unique=True,
        objectpoint_annotate_name=False,
        heading_time_sync=True,   # ✅ 只显示时间一致（匹配到的点）的朝向
        heading_tol_s=0.1         # ✅ 时间匹配容忍（秒），匹配不到可改 0.2/0.3
    )
