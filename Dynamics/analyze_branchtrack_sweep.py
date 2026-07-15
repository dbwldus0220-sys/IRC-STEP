#!/usr/bin/env python3

import argparse
from pathlib import Path

import pandas as pd


ROLL_COLUMNS = ("RL1_wrap", "RL5_wrap", "LL1_wrap", "LL5_wrap")


def analyze(csv_path: Path, top_count: int) -> None:
    data = pd.read_csv(csv_path)
    print(f"\n[{csv_path.name}]")

    for column in ROLL_COLUMNS:
        absolute_delta = data[column].diff().abs()
        max_delta_index = absolute_delta.idxmax()
        print(
            f"  {column}: range=[{data[column].min():+.9f}, "
            f"{data[column].max():+.9f}], "
            f"max_delta={absolute_delta.loc[max_delta_index]:.9f} "
            f"at frame={int(data.loc[max_delta_index, 'frame'])}"
        )

    roll_abs_sum = data[["RL_roll_abs_sum", "LL_roll_abs_sum"]].max(axis=1)
    roll_max_index = roll_abs_sum.idxmax()
    print(
        f"  roll_abs_sum max={roll_abs_sum.loc[roll_max_index]:.9f} "
        f"at frame={int(data.loc[roll_max_index, 'frame'])}"
    )

    for side in ("RL", "LL"):
        convergence_column = f"{side}_converged"
        error_column = f"{side}_final_ERR"
        unconverged_frames = data.loc[
            data[convergence_column] == 0, "frame"
        ].astype(int)
        largest_errors = data.nlargest(top_count, error_column)[
            ["frame", error_column]
        ]
        top_error_text = ", ".join(
            f"{int(row.frame)}:{getattr(row, error_column):.9g}"
            for row in largest_errors.itertuples(index=False)
        )
        print(
            f"  {side}: converged={int(data[convergence_column].sum())}/"
            f"{len(data)}, final_ERR max={data[error_column].max():.9g}, "
            f"mean={data[error_column].mean():.9g}"
        )
        print(f"    unconverged_frames={unconverged_frames.tolist()}")
        print(f"    top_{top_count}_final_ERR={top_error_text}")


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Compare STEP small-roll branch-tracking sweep CSV files."
    )
    parser.add_argument("csv_paths", nargs="+", type=Path)
    parser.add_argument("--top", type=int, default=10)
    args = parser.parse_args()

    if args.top <= 0:
        parser.error("--top must be greater than 0")
    for csv_path in args.csv_paths:
        if not csv_path.is_file():
            parser.error(f"CSV file not found: {csv_path}")
        analyze(csv_path, args.top)


if __name__ == "__main__":
    main()
