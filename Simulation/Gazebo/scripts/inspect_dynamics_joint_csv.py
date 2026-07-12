#!/usr/bin/env python3

import argparse
import json
from pathlib import Path

import pandas as pd


DEFAULT_DT = 0.01


def main():
    parser = argparse.ArgumentParser(
        description="Inspect STEP Dynamics joint CSV and map RL/LL columns to URDF joint names."
    )
    parser.add_argument("csv_path", help="Path to Dynamics CSV, e.g. Dynamics/walk_forward_debug_slow.csv")
    parser.add_argument(
        "--mapping",
        default="Simulation/Gazebo/config/joint_mapping.json",
        help="Path to joint mapping JSON",
    )
    parser.add_argument("--start-frame", type=int, default=398)
    parser.add_argument("--end-frame", type=int, default=405)
    parser.add_argument("--dt", type=float, default=DEFAULT_DT)
    args = parser.parse_args()

    csv_path = Path(args.csv_path)
    mapping_path = Path(args.mapping)

    if not csv_path.exists():
        raise FileNotFoundError(f"CSV not found: {csv_path}")

    if not mapping_path.exists():
        raise FileNotFoundError(f"Mapping not found: {mapping_path}")

    df = pd.read_csv(csv_path)
    mapping = json.loads(mapping_path.read_text())

    if "frame" not in df.columns:
        raise ValueError("CSV must contain a 'frame' column")

    selected = df[(df["frame"] >= args.start_frame) & (df["frame"] <= args.end_frame)]

    if selected.empty:
        raise ValueError(f"No frames found in range {args.start_frame}~{args.end_frame}")

    print("[Dynamics CSV Joint Command Inspection]")
    print(f"csv_path: {csv_path}")
    print(f"frame range: {args.start_frame}~{args.end_frame}")
    print(f"dt: {args.dt}")
    print()

    for _, row in selected.iterrows():
        frame = int(row["frame"])
        time_sec = frame * args.dt
        print(f"frame={frame}, time={time_sec:.3f}s")

        for dynamics_col, info in mapping.items():
            if dynamics_col not in df.columns:
                print(f"  MISSING COLUMN: {dynamics_col}")
                continue

            urdf_joint = info["urdf_joint"]
            motor_id = info["motor_id"]
            angle_rad = float(row[dynamics_col])

            print(
                f"  {dynamics_col:8s} -> {urdf_joint:28s} "
                f"motor={motor_id:2d}, angle_rad={angle_rad:+.9f}"
            )

        print()


if __name__ == "__main__":
    main()
