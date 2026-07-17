#!/usr/bin/env python3
"""Publish motion commands to a separately running dry-run safety node."""

import argparse
import shutil
import subprocess
import sys
import time
from pathlib import Path

import pandas as pd


DEFAULT_COMMANDS = [1, 2, 3, 5, 6, 7, 8, 13, 14, 25, 32]
EXPECTED_GATE_BLOCKED_COMMANDS = {2, 3, 32}
ROLL_JOINTS = [1, 5, 7, 11]
SAFETY_CONTROL_DIR = Path(__file__).resolve().parents[1]
SOURCE_LOG = SAFETY_CONTROL_DIR / "safety_all_theta_command_log.csv"
OUTPUT_DIR = SAFETY_CONTROL_DIR / "dry_run_logs"


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Publish /motion_command messages and analyze the safety command "
            "CSV. Start safety_control_node with STEP_DRY_RUN_NO_DXL=ON first."
        )
    )
    parser.add_argument(
        "--commands",
        nargs="+",
        type=int,
        default=DEFAULT_COMMANDS,
        help="Motion commands to test (default: %(default)s)",
    )
    parser.add_argument(
        "--wait-seconds",
        type=float,
        default=10.0,
        help="Minimum seconds to collect rows after each publish (default: 10)",
    )
    parser.add_argument(
        "--motion-timeout",
        type=float,
        default=180.0,
        help="Maximum seconds to wait for CSV writing to stop (default: 180)",
    )
    parser.add_argument(
        "--expect-gate",
        action="store_true",
        help=(
            "Expect commands 2, 3, and 32 to be blocked by "
            "STEP_REAL_ROBOT_COMMAND_GATE"
        ),
    )
    return parser.parse_args()


def publish_command(command: int) -> None:
    message = f"{{command: {command}, angle: 0}}"
    subprocess.run(
        [
            "ros2",
            "topic",
            "pub",
            "--once",
            "/motion_command",
            "robot_msgs/msg/MotionCommand",
            message,
        ],
        check=True,
    )


def wait_for_log(timeout_seconds: float = 5.0) -> None:
    deadline = time.monotonic() + timeout_seconds
    while time.monotonic() < deadline:
        if SOURCE_LOG.is_file() and SOURCE_LOG.stat().st_size > 0:
            return
        time.sleep(0.1)
    raise TimeoutError(f"CSV log was not created: {SOURCE_LOG}")


def wait_until_log_is_idle(timeout_seconds: float, idle_seconds: float = 1.0) -> None:
    deadline = time.monotonic() + timeout_seconds
    previous_size = -1
    idle_since = time.monotonic()

    while time.monotonic() < deadline:
        current_size = SOURCE_LOG.stat().st_size
        if current_size != previous_size:
            previous_size = current_size
            idle_since = time.monotonic()
        elif time.monotonic() - idle_since >= idle_seconds:
            return
        time.sleep(0.25)

    raise TimeoutError(
        f"CSV was still changing after {timeout_seconds} seconds: {SOURCE_LOG}"
    )


def analyze_command(command: int, csv_path: Path) -> dict:
    data = pd.read_csv(csv_path)
    required_columns = {"go"}
    for joint in ROLL_JOINTS:
        required_columns.update(
            {
                f"raw_{joint}",
                f"safe_{joint}",
                f"delta_{joint}",
                f"roll_guard_used_{joint}",
            }
        )

    missing = sorted(required_columns.difference(data.columns))
    if missing:
        raise ValueError(f"Missing CSV columns in {csv_path}: {missing}")

    command_data = data[data["go"] == command].copy()
    if command_data.empty:
        raise ValueError(
            f"No rows for command {command} in {csv_path}. "
            "The previous motion may still be running; increase --wait-seconds."
        )

    summary = {"command": command, "rows": len(command_data)}
    for joint in ROLL_JOINTS:
        summary[f"max_raw_step_{joint}"] = (
            command_data[f"raw_{joint}"].diff().abs().max()
        )
        summary[f"max_safe_step_{joint}"] = (
            command_data[f"safe_{joint}"].diff().abs().max()
        )
        summary[f"max_delta_{joint}"] = (
            command_data[f"delta_{joint}"].abs().max()
        )
        summary[f"roll_guard_used_sum_{joint}"] = int(
            command_data[f"roll_guard_used_{joint}"].sum()
        )
    return summary


def main() -> int:
    args = parse_args()
    if args.wait_seconds <= 0:
        raise ValueError("--wait-seconds must be greater than zero")
    if args.motion_timeout <= 0:
        raise ValueError("--motion-timeout must be greater than zero")

    print("[SAFETY] This script must only be used with a node built using")
    print("         -DSTEP_DRY_RUN_NO_DXL=ON")
    print("[INFO] Keep safety_control_node running in another terminal.")

    OUTPUT_DIR.mkdir(parents=True, exist_ok=True)
    summaries = []

    for command in args.commands:
        destination = OUTPUT_DIR / f"command_{command}.csv"
        expected_blocked = (
            args.expect_gate and command in EXPECTED_GATE_BLOCKED_COMMANDS
        )
        if expected_blocked:
            destination.unlink(missing_ok=True)
        SOURCE_LOG.unlink(missing_ok=True)
        print(f"[INFO] Publishing command {command}")
        publish_command(command)
        time.sleep(args.wait_seconds)

        if expected_blocked and not (
            SOURCE_LOG.is_file() and SOURCE_LOG.stat().st_size > 0
        ):
            print(f"[PASS] command_{command}: BLOCKED_EXPECTED (no CSV created)")
            continue

        wait_for_log()
        if expected_blocked:
            raise ValueError(
                f"command_{command} was expected to be blocked, but CSV was created: "
                f"{SOURCE_LOG}"
            )
        wait_until_log_is_idle(args.motion_timeout)
        shutil.copy2(SOURCE_LOG, destination)
        summaries.append(analyze_command(command, destination))
        print(f"[PASS] command_{command}: ALLOWED (saved {destination})")

    summary_path = OUTPUT_DIR / "summary.csv"
    pd.DataFrame(summaries).to_csv(summary_path, index=False)
    print(f"[DONE] Summary saved to {summary_path}")
    return 0


if __name__ == "__main__":
    try:
        sys.exit(main())
    except (OSError, subprocess.CalledProcessError, TimeoutError, ValueError) as error:
        print(f"[ERROR] {error}", file=sys.stderr)
        sys.exit(1)
