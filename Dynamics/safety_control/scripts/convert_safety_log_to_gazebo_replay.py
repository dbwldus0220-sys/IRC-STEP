#!/usr/bin/env python3
"""Convert safety-control command logs to Gazebo leg-joint replay CSVs."""

import argparse
import sys
from pathlib import Path
from typing import Iterable

import pandas as pd


DEL_T_SECONDS = 0.01
SAFETY_CONTROL_DIR = Path(__file__).resolve().parents[1]
DEFAULT_OUTPUT_DIR = SAFETY_CONTROL_DIR / "gazebo_replay_logs"

REPLAY_GUIDANCE = """
Replay guidance:
  The generated joint values come from safety_control's final All_Theta
  (safe_*) commands. callback.cpp has already applied the real motor direction
  signs to All_Theta. Do NOT pass --mirror-left-pitch-chain when replaying this
  CSV, because that would apply an additional sign conversion.

Recommended command:
  python3 Simulation/Gazebo/scripts/replay_roll_joints_from_csv.py \\
    Dynamics/safety_control/gazebo_replay_logs/command_32_gazebo_safe.csv \\
    --mode legs \\
    --relative-to-frame 0 \\
    --use-gazebo-offsets \\
    --start-frame 0 \\
    --dt 0.05 \\
    --hold-start 0.0 \\
    --progress-every 50
"""

JOINT_MAPPING = {
    "RL0_wrap": "safe_0",
    "RL1_wrap": "safe_1",
    "RL2_wrap": "safe_2",
    "RL3_wrap": "safe_3",
    "RL4_wrap": "safe_4",
    "RL5_wrap": "safe_5",
    "LL0_wrap": "safe_6",
    "LL1_wrap": "safe_7",
    "LL2_wrap": "safe_8",
    "LL3_wrap": "safe_9",
    "LL4_wrap": "safe_10",
    "LL5_wrap": "safe_11",
}


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Convert safety_control safe joint commands into Gazebo replay CSVs."
        ),
        epilog=REPLAY_GUIDANCE,
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    input_group = parser.add_mutually_exclusive_group(required=True)
    input_group.add_argument("--input", type=Path, help="One safety log CSV")
    input_group.add_argument(
        "--input-dir",
        type=Path,
        help="Directory containing safety log CSV files",
    )
    parser.add_argument(
        "--output-dir",
        type=Path,
        default=DEFAULT_OUTPUT_DIR,
        help=f"Output directory (default: {DEFAULT_OUTPUT_DIR})",
    )
    return parser.parse_args()


def collect_input_files(args: argparse.Namespace) -> Iterable[Path]:
    if args.input is not None:
        if not args.input.is_file():
            raise FileNotFoundError(f"Input CSV does not exist: {args.input}")
        return [args.input]

    if not args.input_dir.is_dir():
        raise NotADirectoryError(
            f"Input directory does not exist: {args.input_dir}"
        )

    # summary.csv contains aggregate statistics rather than safe joint rows.
    input_files = sorted(
        path
        for path in args.input_dir.glob("*.csv")
        if path.name != "summary.csv"
    )
    if not input_files:
        raise FileNotFoundError(
            f"No safety log CSV files found in: {args.input_dir}"
        )
    return input_files


def output_path_for(input_path: Path, output_dir: Path) -> Path:
    return output_dir / f"{input_path.stem}_gazebo_safe.csv"


def convert_file(input_path: Path, output_dir: Path) -> Path:
    data = pd.read_csv(input_path)
    required_columns = {"frame", *JOINT_MAPPING.values()}
    missing_columns = sorted(required_columns.difference(data.columns))
    if missing_columns:
        raise ValueError(
            f"Required columns are missing from {input_path}: "
            f"{', '.join(missing_columns)}"
        )

    output = pd.DataFrame(index=data.index)
    output["frame"] = range(len(data))
    output["time"] = output["frame"] * DEL_T_SECONDS
    output["source_frame"] = data["frame"]
    for gazebo_joint, safety_column in JOINT_MAPPING.items():
        output[gazebo_joint] = data[safety_column]

    output_dir.mkdir(parents=True, exist_ok=True)
    output_path = output_path_for(input_path, output_dir)
    output.to_csv(output_path, index=False)
    print(f"[OK] rows={len(output)} path={output_path}")
    return output_path


def main() -> int:
    args = parse_args()
    input_files = collect_input_files(args)

    conversion_failed = False
    for input_path in input_files:
        try:
            convert_file(input_path, args.output_dir)
        except (OSError, pd.errors.ParserError, ValueError) as error:
            conversion_failed = True
            print(f"[ERROR] {error}", file=sys.stderr)

    return 1 if conversion_failed else 0


if __name__ == "__main__":
    try:
        sys.exit(main())
    except (FileNotFoundError, NotADirectoryError) as error:
        print(f"[ERROR] {error}", file=sys.stderr)
        sys.exit(1)
