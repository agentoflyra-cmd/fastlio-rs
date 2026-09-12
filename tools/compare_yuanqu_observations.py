#!/usr/bin/env python3
"""Compare yuanqu accepted observation dumps from two FAST-LIO runs.

The dump files are joined by (frame_index, point_index). SurfelID is not used
for cross-run matching because each replay builds an independent map.
"""

from __future__ import annotations

import argparse
import csv
from collections import defaultdict
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np


DEFAULT_DUMP_03 = Path("output/yuanqu-obs-03.csv")
DEFAULT_DUMP_08 = Path("output/yuanqu-obs-08.csv")
DEFAULT_TRAJ_03 = Path("output/trajectory-yuanqu-03.csv")
DEFAULT_TRAJ_08 = Path("output/trajectory-yuanqu-odom-max-plane-distance-08.csv")
DEFAULT_OUT_PREFIX = Path("output/yuanqu-observation-diff")


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--dump-a", type=Path, default=DEFAULT_DUMP_03)
    parser.add_argument("--dump-b", type=Path, default=DEFAULT_DUMP_08)
    parser.add_argument("--traj-a", type=Path, default=DEFAULT_TRAJ_03)
    parser.add_argument("--traj-b", type=Path, default=DEFAULT_TRAJ_08)
    parser.add_argument("--label-a", default="03")
    parser.add_argument("--label-b", default="08")
    parser.add_argument("--out-prefix", type=Path, default=DEFAULT_OUT_PREFIX)
    parser.add_argument("--top-extra", type=int, default=20)
    return parser.parse_args()


def read_trajectory(path: Path) -> dict[int, dict[str, float]]:
    out: dict[int, dict[str, float]] = {}
    with path.open(newline="") as file:
        for frame_index, row in enumerate(csv.DictReader(file)):
            parsed = {}
            for key, value in row.items():
                if value == "true":
                    parsed[key] = 1.0
                elif value == "false":
                    parsed[key] = 0.0
                else:
                    parsed[key] = float(value)
            out[frame_index] = parsed
    return out


def read_dump(path: Path) -> dict[int, dict[int, dict[str, float]]]:
    frames: dict[int, dict[int, dict[str, float]]] = defaultdict(dict)
    with path.open(newline="") as file:
        for row in csv.DictReader(file):
            frame_index = int(row["frame_index"])
            point_index = int(row["point_index"])
            frames[frame_index][point_index] = {
                "residual_abs": float(row["residual_abs"]),
                "residual_signed": float(row["residual_signed"]),
                "variance": float(row["variance"]),
                "plane_distance": float(row["plane_distance"]),
                "centroid_distance": float(row["surfel_centroid_distance"]),
                "planarity": float(row["planarity"]),
                "normal_z_abs": float(row["normal_z_abs"]),
                "normal_x": float(row["normal_w_x"]),
                "normal_y": float(row["normal_w_y"]),
                "normal_z": float(row["normal_w_z"]),
                "point_w_x": float(row["point_w_x"]),
                "point_w_y": float(row["point_w_y"]),
                "point_w_z": float(row["point_w_z"]),
            }
    return frames


def percentile(values: list[float], p: float) -> float:
    if not values:
        return 0.0
    return float(np.percentile(np.asarray(values, dtype=float), p))


def mean(values: list[float]) -> float:
    if not values:
        return 0.0
    return float(np.mean(np.asarray(values, dtype=float)))


def frame_rows(
    frames_a: dict[int, dict[int, dict[str, float]]],
    frames_b: dict[int, dict[int, dict[str, float]]],
    traj_a: dict[int, dict[str, float]],
    traj_b: dict[int, dict[str, float]],
) -> list[dict[str, float]]:
    rows: list[dict[str, float]] = []
    for frame_index in sorted(set(frames_a) | set(frames_b)):
        a = frames_a.get(frame_index, {})
        b = frames_b.get(frame_index, {})
        keys_a = set(a)
        keys_b = set(b)
        common = keys_a & keys_b
        only_a = keys_a - keys_b
        only_b = keys_b - keys_a

        residual_delta = [b[i]["residual_abs"] - a[i]["residual_abs"] for i in common]
        normal_dot = [
            abs(
                a[i]["normal_x"] * b[i]["normal_x"]
                + a[i]["normal_y"] * b[i]["normal_y"]
                + a[i]["normal_z"] * b[i]["normal_z"]
            )
            for i in common
        ]
        only_b_residual = [b[i]["residual_abs"] for i in only_b]
        only_b_centroid = [b[i]["centroid_distance"] for i in only_b]
        only_b_normal_z = [b[i]["normal_z_abs"] for i in only_b]
        only_b_z = [b[i]["point_w_z"] for i in only_b]

        ta = traj_a.get(frame_index, {})
        tb = traj_b.get(frame_index, {})
        rows.append(
            {
                "frame_index": float(frame_index),
                "timestamp_sec": ta.get("timestamp_sec", tb.get("timestamp_sec", 0.0)),
                "count_a": float(len(a)),
                "count_b": float(len(b)),
                "common": float(len(common)),
                "only_a": float(len(only_a)),
                "only_b": float(len(only_b)),
                "only_b_ratio": float(len(only_b) / max(len(b), 1)),
                "traj_a_obs_accepted": ta.get("obs_accepted", 0.0),
                "traj_b_obs_accepted": tb.get("obs_accepted", 0.0),
                "traj_a_no_assoc_ratio": ta.get("obs_no_association", 0.0)
                / max(ta.get("obs_input_points", 1.0), 1.0),
                "traj_b_no_assoc_ratio": tb.get("obs_no_association", 0.0)
                / max(tb.get("obs_input_points", 1.0), 1.0),
                "common_residual_delta_mean": mean(residual_delta),
                "common_residual_delta_p95": percentile(residual_delta, 95),
                "common_normal_dot_abs_p10": percentile(normal_dot, 10),
                "only_b_residual_p50": percentile(only_b_residual, 50),
                "only_b_residual_p95": percentile(only_b_residual, 95),
                "only_b_centroid_p95": percentile(only_b_centroid, 95),
                "only_b_normal_z_abs_mean": mean(only_b_normal_z),
                "only_b_normal_z_abs_p10": percentile(only_b_normal_z, 10),
                "only_b_point_w_z_mean": mean(only_b_z),
            }
        )
    return rows


def write_summary(path: Path, rows: list[dict[str, float]]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", newline="") as file:
        writer = csv.DictWriter(file, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)


def plot_summary(path: Path, rows: list[dict[str, float]], label_a: str, label_b: str) -> None:
    frame = np.array([row["frame_index"] for row in rows])
    trel = np.array([row["timestamp_sec"] for row in rows])
    trel = trel - trel[0]
    count_a = np.array([row["count_a"] for row in rows])
    count_b = np.array([row["count_b"] for row in rows])
    only_b = np.array([row["only_b"] for row in rows])
    only_b_ratio = np.array([row["only_b_ratio"] for row in rows])
    no_a = np.array([row["traj_a_no_assoc_ratio"] for row in rows])
    no_b = np.array([row["traj_b_no_assoc_ratio"] for row in rows])
    residual_p95 = np.array([row["only_b_residual_p95"] for row in rows])
    centroid_p95 = np.array([row["only_b_centroid_p95"] for row in rows])
    normal_z_p10 = np.array([row["only_b_normal_z_abs_p10"] for row in rows])
    normal_dot_p10 = np.array([row["common_normal_dot_abs_p10"] for row in rows])

    fig, axes = plt.subplots(3, 2, figsize=(14, 12), dpi=150)
    fig.suptitle(f"yuanqu observation diff | {label_a} vs {label_b}", fontsize=16)

    ax = axes[0, 0]
    ax.plot(trel, count_a, label=f"{label_a} accepted")
    ax.plot(trel, count_b, label=f"{label_b} accepted")
    ax.plot(trel, only_b, label=f"{label_b}-only")
    ax.set_title("Accepted Observations")
    ax.set_ylabel("count")
    ax.grid(alpha=0.25)
    ax.legend()

    ax = axes[0, 1]
    ax.plot(trel, no_a, label=f"{label_a} no assoc")
    ax.plot(trel, no_b, label=f"{label_b} no assoc")
    ax.plot(trel, only_b_ratio, label=f"{label_b}-only / {label_b}")
    ax.set_title("Association Ratios")
    ax.set_ylabel("ratio")
    ax.grid(alpha=0.25)
    ax.legend()

    ax = axes[1, 0]
    ax.plot(trel, residual_p95)
    ax.set_title(f"{label_b}-only Residual P95")
    ax.set_ylabel("m")
    ax.grid(alpha=0.25)

    ax = axes[1, 1]
    ax.plot(trel, centroid_p95)
    ax.set_title(f"{label_b}-only Surfel Centroid Distance P95")
    ax.set_ylabel("m")
    ax.grid(alpha=0.25)

    ax = axes[2, 0]
    ax.plot(trel, normal_z_p10)
    ax.set_title(f"{label_b}-only |normal_z| P10")
    ax.set_xlabel("time in dump window (s)")
    ax.set_ylabel("|normal_z|")
    ax.grid(alpha=0.25)

    ax = axes[2, 1]
    ax.plot(trel, normal_dot_p10)
    ax.set_title("Common Observation Normal Dot P10")
    ax.set_xlabel("time in dump window (s)")
    ax.set_ylabel("|dot(n_a, n_b)|")
    ax.grid(alpha=0.25)

    for ax in axes.ravel():
        ax2 = ax.twiny()
        ax2.set_xlim(frame[0], frame[-1])
        ax2.set_xlabel("frame")
        ax2.tick_params(axis="x", labelsize=8)

    fig.tight_layout(rect=[0, 0.03, 1, 0.96])
    fig.savefig(path, bbox_inches="tight")
    plt.close(fig)


def print_report(rows: list[dict[str, float]], label_a: str, label_b: str, top_extra: int) -> None:
    only_b = np.array([row["only_b"] for row in rows])
    only_b_ratio = np.array([row["only_b_ratio"] for row in rows])
    no_a = np.array([row["traj_a_no_assoc_ratio"] for row in rows])
    no_b = np.array([row["traj_b_no_assoc_ratio"] for row in rows])
    residual_p95 = np.array([row["only_b_residual_p95"] for row in rows])
    normal_dot_p10 = np.array([row["common_normal_dot_abs_p10"] for row in rows])

    print(f"frames: {int(rows[0]['frame_index'])}..{int(rows[-1]['frame_index'])}")
    print(f"{label_b}-only count mean/p50/p95/max: {only_b.mean():.1f} / {np.percentile(only_b, 50):.1f} / {np.percentile(only_b, 95):.1f} / {only_b.max():.0f}")
    print(f"{label_b}-only ratio mean/p50/p95/max: {only_b_ratio.mean():.3f} / {np.percentile(only_b_ratio, 50):.3f} / {np.percentile(only_b_ratio, 95):.3f} / {only_b_ratio.max():.3f}")
    print(f"no-association ratio {label_a} mean/p95: {no_a.mean():.3f} / {np.percentile(no_a, 95):.3f}")
    print(f"no-association ratio {label_b} mean/p95: {no_b.mean():.3f} / {np.percentile(no_b, 95):.3f}")
    print(f"{label_b}-only residual p95 mean/p95/max: {residual_p95.mean():.3f} / {np.percentile(residual_p95, 95):.3f} / {residual_p95.max():.3f}")
    print(f"common normal dot abs p10 mean/min: {normal_dot_p10.mean():.3f} / {normal_dot_p10.min():.3f}")

    ranked = sorted(rows, key=lambda row: row["only_b"], reverse=True)[:top_extra]
    print(f"\ntop {len(ranked)} frames by {label_b}-only observations:")
    for row in ranked:
        print(
            f"frame={int(row['frame_index'])} "
            f"t={row['timestamp_sec']:.9f} "
            f"{label_b}_only={row['only_b']:.0f} "
            f"ratio={row['only_b_ratio']:.3f} "
            f"no_{label_a}={row['traj_a_no_assoc_ratio']:.3f} "
            f"no_{label_b}={row['traj_b_no_assoc_ratio']:.3f} "
            f"res_p95={row['only_b_residual_p95']:.3f} "
            f"cent_p95={row['only_b_centroid_p95']:.3f} "
            f"nz_p10={row['only_b_normal_z_abs_p10']:.3f}"
        )


def main() -> None:
    args = parse_args()
    frames_a = read_dump(args.dump_a)
    frames_b = read_dump(args.dump_b)
    traj_a = read_trajectory(args.traj_a)
    traj_b = read_trajectory(args.traj_b)
    rows = frame_rows(frames_a, frames_b, traj_a, traj_b)

    summary_path = args.out_prefix.with_suffix(".csv")
    plot_path = args.out_prefix.with_suffix(".png")
    write_summary(summary_path, rows)
    plot_summary(plot_path, rows, args.label_a, args.label_b)
    print_report(rows, args.label_a, args.label_b, args.top_extra)
    print(f"wrote {summary_path}")
    print(f"wrote {plot_path}")


if __name__ == "__main__":
    main()
