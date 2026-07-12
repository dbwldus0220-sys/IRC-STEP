import math
from pathlib import Path

import numpy as np
import pandas as pd
import matplotlib.pyplot as plt

df = pd.read_csv("walk_forward_debug_slow.csv")

cols = [
    "RL0_wrap", "RL1_wrap", "RL2_wrap", "RL3_wrap", "RL4_wrap", "RL5_wrap",
    "LL0_wrap", "LL1_wrap", "LL2_wrap", "LL3_wrap", "LL4_wrap", "LL5_wrap",
]

warning_messages = []
check_status = {
    "Trajectory Check": "PASS",
    "IK Angle Check": "PASS",
    "IK Numerical Stability Check": "PASS",
    "IK Solution Continuity Check": "PASS",
    "Joint Velocity Check": "PASS",
    "Joint Acceleration Check": "PASS",
    "Dynamixel Position Check": "PASS",
    "Motor Command Delta Check": "PASS",
}


def record_warning(category, message):
    """Record and print one offline validation warning."""
    check_status[category] = "WARNING"
    warning_messages.append(message)
    print(f"WARNING: {message}")


for col in cols:
    if col in df.columns:
        plt.figure()
        plt.plot(df["frame"], df[col])
        plt.title(col)
        plt.xlabel("frame")
        plt.ylabel("angle [rad]")
        plt.grid(True)
        plt.savefig(f"{col}.png")
        plt.close()

print("Saved all wrapped joint plots as PNG files.")

delta_cols = [
    "RL2_wrap", "RL3_wrap", "RL4_wrap",
    "LL2_wrap", "LL3_wrap", "LL4_wrap",
]
del_t = 0.01
velocity_limit = 10.0

trajectory_cols = [
    "Ref_RL_x", "Ref_RL_y", "Ref_RL_z",
    "Ref_LL_x", "Ref_LL_y", "Ref_LL_z",
]
trajectory_delta_limit = 0.02
trajectory_velocity_limit = 1.0
trajectory_acceleration_limit = 20.0
trajectory_z_min = -0.005

if all(col in df.columns for col in trajectory_cols):
    for col in trajectory_cols:
        trajectory_delta = df[col].diff()
        trajectory_velocity = trajectory_delta / del_t
        trajectory_acceleration = trajectory_velocity.diff() / del_t

        print(
            f"{col} max abs(delta): "
            f"{trajectory_delta.abs().max():.9f} m/frame"
        )
        print(
            f"{col} max abs(velocity): "
            f"{trajectory_velocity.abs().max():.9f} m/s"
        )
        print(
            f"{col} max abs(acceleration): "
            f"{trajectory_acceleration.abs().max():.9f} m/s^2"
        )

        delta_violations = trajectory_delta.abs() > trajectory_delta_limit
        for row_index in trajectory_delta[delta_violations].index:
            warning_message = (
                f"Trajectory delta limit exceeded: reference={col}, "
                f"frame={int(df.loc[row_index, 'frame'])}, "
                f"delta={trajectory_delta.loc[row_index]:.9f} m/frame"
            )
            record_warning("Trajectory Check", warning_message)

        velocity_violations = trajectory_velocity.abs() > trajectory_velocity_limit
        for row_index in trajectory_velocity[velocity_violations].index:
            warning_message = (
                f"Trajectory velocity limit exceeded: reference={col}, "
                f"frame={int(df.loc[row_index, 'frame'])}, "
                f"velocity={trajectory_velocity.loc[row_index]:.9f} m/s"
            )
            record_warning("Trajectory Check", warning_message)

        acceleration_violations = (
            trajectory_acceleration.abs() > trajectory_acceleration_limit
        )
        for row_index in trajectory_acceleration[acceleration_violations].index:
            warning_message = (
                f"Trajectory acceleration limit exceeded: reference={col}, "
                f"frame={int(df.loc[row_index, 'frame'])}, "
                f"acceleration={trajectory_acceleration.loc[row_index]:.9f} m/s^2"
            )
            record_warning("Trajectory Check", warning_message)

        if col in ("Ref_RL_z", "Ref_LL_z"):
            z_violations = df[col] < trajectory_z_min
            for row_index in df.index[z_violations]:
                warning_message = (
                    f"Trajectory z below minimum: reference={col}, "
                    f"frame={int(df.loc[row_index, 'frame'])}, "
                    f"z={df.loc[row_index, col]:.9f} m"
                )
                record_warning("Trajectory Check", warning_message)
else:
    print("trajectory reference columns not found, skipped")

for col in delta_cols:
    if col not in df.columns:
        print(f"Skipped {col}: column not found.")
        continue

    delta = df[col].diff()
    max_abs_delta = delta.abs().max()
    print(f"{col} max abs(delta): {max_abs_delta:.9f} rad/frame")

    plt.figure()
    plt.plot(df["frame"].iloc[1:], delta.iloc[1:])
    plt.title(f"{col} frame-to-frame delta")
    plt.xlabel("frame")
    plt.ylabel("delta angle [rad/frame]")
    plt.grid(True)
    plt.savefig(f"{col}_delta.png")
    plt.close()

print("Saved selected wrapped joint delta plots as PNG files.")

acceleration_limit = 100.0
joint_motion_data = {}
joint_acceleration_warning_representatives = []
joint_velocity_maxima = []
joint_acceleration_maxima = []

for col in cols:
    if col not in df.columns:
        print(f"Skipped unwrapped velocity/acceleration check for {col}: column not found.")
        continue

    unwrapped_angle = pd.Series(
        np.unwrap(df[col].to_numpy()),
        index=df.index,
    )
    velocity = unwrapped_angle.diff() / del_t
    valid_velocity = velocity.dropna()
    acceleration = velocity.diff() / del_t
    valid_acceleration = acceleration.dropna()
    joint_motion_data[col] = {
        "angle": unwrapped_angle,
        "velocity": velocity,
        "acceleration": acceleration,
    }
    max_abs_velocity = valid_velocity.abs().max()
    max_abs_acceleration = valid_acceleration.abs().max()

    if not valid_velocity.empty:
        max_velocity_row_index = valid_velocity.abs().idxmax()
        joint_velocity_maxima.append({
            "joint": col,
            "frame": int(df.loc[max_velocity_row_index, "frame"]),
            "max_abs_velocity": max_abs_velocity,
        })
    if not valid_acceleration.empty:
        max_acceleration_row_index = valid_acceleration.abs().idxmax()
        joint_acceleration_maxima.append({
            "joint": col,
            "frame": int(df.loc[max_acceleration_row_index, "frame"]),
            "max_abs_acceleration": max_abs_acceleration,
        })

    velocity_violations = valid_velocity.abs() > velocity_limit
    acceleration_violations = valid_acceleration.abs() > acceleration_limit
    acceleration_warning_count = int(acceleration_violations.sum())

    print(
        f"{col} max abs velocity using unwrapped angle: "
        f"{max_abs_velocity:.9f} rad/s"
    )
    print(
        f"{col} max abs acceleration using unwrapped angle: "
        f"{max_abs_acceleration:.9f} rad/s^2"
    )
    print(f"{col} acceleration warning count: {acceleration_warning_count}")

    if velocity_violations.any():
        violation_velocities = valid_velocity[velocity_violations]
        row_index = violation_velocities.abs().idxmax()
        warning_message = (
            f"Joint velocity limit exceeded using unwrapped angle: joint={col}, "
            f"frame={int(df.loc[row_index, 'frame'])}, "
            f"velocity={valid_velocity.loc[row_index]:.9f} rad/s, "
            f"count={int(velocity_violations.sum())}"
        )
        record_warning("Joint Velocity Check", warning_message)

    if acceleration_violations.any():
        violation_accelerations = valid_acceleration[acceleration_violations]
        row_index = violation_accelerations.abs().idxmax()
        joint_acceleration_warning_representatives.append({
            "joint": col,
            "row_index": row_index,
            "frame": int(df.loc[row_index, "frame"]),
            "acceleration": valid_acceleration.loc[row_index],
        })
        warning_message = (
            f"Joint acceleration limit exceeded using unwrapped angle: joint={col}, "
            f"frame={int(df.loc[row_index, 'frame'])}, "
            f"acceleration={valid_acceleration.loc[row_index]:.9f} rad/s^2, "
            f"count={acceleration_warning_count}"
        )
        record_warning("Joint Acceleration Check", warning_message)

    plt.figure()
    plt.plot(df.loc[valid_velocity.index, "frame"], valid_velocity)
    plt.axhline(velocity_limit, color="red", linestyle="--", label="velocity limit")
    plt.axhline(-velocity_limit, color="red", linestyle="--")
    plt.title(f"{col} unwrapped velocity")
    plt.xlabel("frame")
    plt.ylabel("angular velocity [rad/s]")
    plt.grid(True)
    plt.legend()
    plt.savefig(f"{col}_unwrapped_velocity.png")
    plt.close()

    plt.figure()
    plt.plot(df.loc[valid_acceleration.index, "frame"], valid_acceleration)
    plt.axhline(
        acceleration_limit,
        color="red",
        linestyle="--",
        label="acceleration limit",
    )
    plt.axhline(-acceleration_limit, color="red", linestyle="--")
    plt.title(f"{col} unwrapped acceleration")
    plt.xlabel("frame")
    plt.ylabel("angular acceleration [rad/s^2]")
    plt.grid(True)
    plt.legend()
    plt.savefig(f"{col}_unwrapped_acceleration.png")
    plt.close()

print("Saved all unwrapped joint velocity plots as PNG files.")
print("Saved all unwrapped joint acceleration plots as PNG files.")

joint_angle_min = -3.0
joint_angle_max = 3.0
limit_violation_found = False

for col in cols:
    if col not in df.columns:
        print(f"Skipped limit check for {col}: column not found.")
        continue

    violations = df[(df[col] < joint_angle_min) | (df[col] > joint_angle_max)]
    for _, row in violations.iterrows():
        warning_message = (
            f"Joint angle limit violation: joint={col}, "
            f"frame={int(row['frame'])}, value={row[col]:.9f} rad"
        )
        record_warning("IK Angle Check", warning_message)
        limit_violation_found = True

if not limit_violation_found:
    print("Joint angle limit check passed")

# TODO: Replace all temporary motor assignments with STEP's verified wiring table,
# encoder zero positions, joint directions, and mechanical joint limits.
motor_models = {
    "MX-28": {"ticks_per_revolution": 4096, "position_min": 0, "position_max": 4095},
    "MX-64": {"ticks_per_revolution": 4096, "position_min": 0, "position_max": 4095},
    "MX-106": {"ticks_per_revolution": 4096, "position_min": 0, "position_max": 4095},
}

# STEP leg motor mapping based on callback.cpp All_Theta direction.
# TODO: callback.cpp의 All_Theta 계산식에 들어 있는 관절별 calibration offset도 나중에 반영해야 함.
# 현재 zero_offset=2048은 임시 기준이며, 실제 하드웨어 zero position 확인 전까지 모터 명령에 사용하면 안 됨.
joint_motor_map = {
    "RL0_wrap": {"motor_id": 10, "model": "MX-106", "direction": -1, "zero_offset": 2048, "min_rad": -3.0, "max_rad": 3.0},
    "RL1_wrap": {"motor_id": 13, "model": "MX-106", "direction":  1, "zero_offset": 2048, "min_rad": -3.0, "max_rad": 3.0},
    "RL2_wrap": {"motor_id": 15, "model": "MX-106", "direction":  1, "zero_offset": 2048, "min_rad": -3.0, "max_rad": 3.0},
    "RL3_wrap": {"motor_id": 17, "model": "MX-106", "direction": -1, "zero_offset": 2048, "min_rad": -3.0, "max_rad": 3.0},
    "RL4_wrap": {"motor_id": 19, "model": "MX-106", "direction": -1, "zero_offset": 2048, "min_rad": -3.0, "max_rad": 3.0},
    "RL5_wrap": {"motor_id": 21, "model": "MX-106", "direction": -1, "zero_offset": 2048, "min_rad": -3.0, "max_rad": 3.0},

    "LL0_wrap": {"motor_id": 12, "model": "MX-106", "direction": -1, "zero_offset": 2048, "min_rad": -3.0, "max_rad": 3.0},
    "LL1_wrap": {"motor_id": 14, "model": "MX-106", "direction":  1, "zero_offset": 2048, "min_rad": -3.0, "max_rad": 3.0},
    "LL2_wrap": {"motor_id": 16, "model": "MX-106", "direction": -1, "zero_offset": 2048, "min_rad": -3.0, "max_rad": 3.0},
    "LL3_wrap": {"motor_id": 18, "model": "MX-106", "direction":  1, "zero_offset": 2048, "min_rad": -3.0, "max_rad": 3.0},
    "LL4_wrap": {"motor_id": 20, "model": "MX-106", "direction":  1, "zero_offset": 2048, "min_rad": -3.0, "max_rad": 3.0},
    "LL5_wrap": {"motor_id": 22, "model": "MX-106", "direction": -1, "zero_offset": 2048, "min_rad": -3.0, "max_rad": 3.0}
}

DEG2RAD = math.pi / 180.0

start_offset = {
    "RL0_wrap": 0.0,
    "RL1_wrap": 0.001941,
    "RL2_wrap": 0.122416,
    "RL3_wrap": 0.196013,
    "RL4_wrap": -0.073595,
    "RL5_wrap": 0.001941,

    "LL0_wrap": 0.0,
    "LL1_wrap": 0.001941,
    "LL2_wrap": -0.122416,
    "LL3_wrap": -0.196013,
    "LL4_wrap": 0.073595,
    "LL5_wrap": 0.001941,
}

calibration_offset = {
    "RL0_wrap": 0.0,
    "RL1_wrap": -3.0 * DEG2RAD,
    "RL2_wrap": -17.0 * DEG2RAD,
    "RL3_wrap": 40.0 * DEG2RAD,
    "RL4_wrap": 24.22 * DEG2RAD,
    "RL5_wrap": -2.0 * DEG2RAD,

    "LL0_wrap": 0.0,
    "LL1_wrap": 2.0 * DEG2RAD,
    "LL2_wrap": 17.0 * DEG2RAD,
    "LL3_wrap": -40.0 * DEG2RAD,
    "LL4_wrap": -21.22 * DEG2RAD,
    "LL5_wrap": -2.0 * DEG2RAD,
}


def calculate_all_theta(angle_rad, joint_name, joint_config):
    """Calculate a callback.cpp-style temporary All_Theta candidate."""
    limited_angle = min(
        max(angle_rad, joint_config["min_rad"]),
        joint_config["max_rad"],
    )

    return (
        joint_config["direction"] * limited_angle
        + start_offset[joint_name]
        + calibration_offset[joint_name]
    )


def angle_rad_to_position(angle_rad, joint_name, joint_config):
    """Convert radians to a temporary Dynamixel position value without communication."""
    model_config = motor_models[joint_config["model"]]
    ticks_per_radian = model_config["ticks_per_revolution"] / (2.0 * math.pi)
    all_theta = calculate_all_theta(angle_rad, joint_name, joint_config)

    position = (all_theta + math.pi) * ticks_per_radian

    return min(
        max(position, model_config["position_min"]),
        model_config["position_max"],
    )


print("Temporary offline Dynamixel position ranges (no commands are sent):")
position_warning_min = 300
position_warning_max = 3795

for col in cols:
    if col not in df.columns:
        print(f"Skipped position conversion for {col}: column not found.")
        continue

    config = joint_motor_map[col]
    positions = df[col].apply(lambda angle: angle_rad_to_position(angle, col, config))
    position_min = positions.min()
    position_max = positions.max()
    print(
        f"{col}: motor_id={config['motor_id']}, model={config['model']}, "
        f"position min/max={position_min} / {position_max}"
    )

    # This is not an error; it flags candidates that need extra checking before hardware use.
    if position_min < position_warning_min or position_max > position_warning_max:
        warning_message = (
            f"Dynamixel position near range end: joint={col}, "
            f"motor_id={config['motor_id']}, "
            f"position min={position_min}, position max={position_max}"
        )
        record_warning("Dynamixel Position Check", warning_message)

# This CSV contains offline candidates for review only; it is not a motor command file.
candidate_data = {"frame": df["frame"]}

for all_theta_index, col in enumerate(cols):
    config = joint_motor_map[col]
    candidate_data[f"All_Theta{all_theta_index}"] = df[col].apply(
        lambda angle, joint_name=col, joint_config=config: calculate_all_theta(
            angle,
            joint_name,
            joint_config,
        )
    )

for col in cols:
    config = joint_motor_map[col]
    candidate_data[f"Motor{config['motor_id']}_pos"] = df[col].apply(
        lambda angle, joint_name=col, joint_config=config: angle_rad_to_position(
            angle,
            joint_name,
            joint_config,
        )
    )

candidate_df = pd.DataFrame(candidate_data)
candidate_output_file = "motor_command_candidate.csv"
candidate_df.to_csv(candidate_output_file, index=False)

print(
    f"Saved offline candidate CSV: {candidate_output_file} "
    "(review only; no motor commands are sent)."
)
print("Offline candidate motor position ranges and frame deltas:")
motor_position_delta_limit = 50.0
motor_delta_warning_representatives = []
motor_delta_maxima = []

for col in cols:
    motor_id = joint_motor_map[col]["motor_id"]
    motor_position_col = f"Motor{motor_id}_pos"
    motor_positions = candidate_df[motor_position_col]
    motor_position_delta = motor_positions.diff()
    valid_motor_position_delta = motor_position_delta.dropna()
    max_abs_position_delta = valid_motor_position_delta.abs().max()
    if not valid_motor_position_delta.empty:
        max_delta_row_index = valid_motor_position_delta.abs().idxmax()
        motor_delta_maxima.append({
            "motor": f"Motor{motor_id}",
            "frame": int(candidate_df.loc[max_delta_row_index, "frame"]),
            "max_abs_delta_tick_per_frame": max_abs_position_delta,
        })
    print(
        f"{motor_position_col}: position min={motor_positions.min()}, "
        f"position max={motor_positions.max()}, "
        f"max abs position delta per frame={max_abs_position_delta} tick/frame"
    )

    plt.figure()
    plt.plot(
        candidate_df.loc[valid_motor_position_delta.index, "frame"],
        valid_motor_position_delta,
    )
    plt.axhline(
        motor_position_delta_limit,
        color="red",
        linestyle="--",
        label="position delta limit",
    )
    plt.axhline(-motor_position_delta_limit, color="red", linestyle="--")
    plt.title(f"{motor_position_col} frame-to-frame delta")
    plt.xlabel("frame")
    plt.ylabel("position delta [tick/frame]")
    plt.grid(True)
    plt.legend()
    plt.savefig(f"{motor_position_col}_delta.png")
    plt.close()

    delta_violations = motor_position_delta.abs() > motor_position_delta_limit
    if delta_violations.any():
        violation_deltas = motor_position_delta[delta_violations]
        row_index = violation_deltas.abs().idxmax()
        motor_delta_warning_representatives.append({
            "motor_id": motor_id,
            "position_col": motor_position_col,
            "row_index": row_index,
            "frame": int(candidate_df.loc[row_index, "frame"]),
            "delta": motor_position_delta.loc[row_index],
        })
        warning_message = (
            f"Motor command position delta limit exceeded: motor_id={motor_id}, "
            f"frame={int(candidate_df.loc[row_index, 'frame'])}, "
            f"delta={motor_position_delta.loc[row_index]:.9f} tick"
        )
        record_warning("Motor Command Delta Check", warning_message)

print("Saved all motor position delta plots as PNG files.")

# Offline comparison only. These rate-limited candidates are never sent to motors.
roll_rate_limit_joints = ["RL1_wrap", "RL5_wrap", "LL1_wrap", "LL5_wrap"]
roll_rate_limit = 0.05
roll_limited_data = {"frame": df["frame"]}
roll_limited_comparison = []

for joint in roll_rate_limit_joints:
    limited_col = f"{joint}_limited"
    original_angles = df[joint]
    limited_angles = []

    if not original_angles.empty:
        previous_candidate = original_angles.iloc[0]
        limited_angles.append(previous_candidate)
        for original_current in original_angles.iloc[1:]:
            candidate_delta = np.clip(
                original_current - previous_candidate,
                -roll_rate_limit,
                roll_rate_limit,
            )
            previous_candidate += candidate_delta
            limited_angles.append(previous_candidate)

    limited_angle_series = pd.Series(limited_angles, index=original_angles.index)
    limited_velocity = limited_angle_series.diff() / del_t
    limited_acceleration = limited_velocity.diff() / del_t

    roll_limited_data[limited_col] = limited_angle_series
    roll_limited_data[f"{limited_col}_velocity"] = limited_velocity
    roll_limited_data[f"{limited_col}_acceleration"] = limited_acceleration

    config = joint_motor_map[joint]
    motor_position_col = f"Motor{config['motor_id']}_pos"
    limited_motor_positions = limited_angle_series.apply(
        lambda angle, joint_name=joint, joint_config=config: angle_rad_to_position(
            angle,
            joint_name,
            joint_config,
        )
    )
    limited_motor_delta = limited_motor_positions.diff()
    roll_limited_data[motor_position_col] = limited_motor_positions
    roll_limited_data[f"{motor_position_col}_delta_per_frame"] = limited_motor_delta

    original_velocity = joint_motion_data[joint]["velocity"]
    original_acceleration = joint_motion_data[joint]["acceleration"]
    original_motor_delta = candidate_df[motor_position_col].diff()
    roll_limited_comparison.append({
        "joint": joint,
        "motor_id": config["motor_id"],
        "original_max_abs_velocity": original_velocity.abs().max(),
        "limited_max_abs_velocity": limited_velocity.abs().max(),
        "original_max_abs_acceleration": original_acceleration.abs().max(),
        "limited_max_abs_acceleration": limited_acceleration.abs().max(),
        "original_max_abs_motor_delta": original_motor_delta.abs().max(),
        "limited_max_abs_motor_delta": limited_motor_delta.abs().max(),
    })

roll_limited_candidate_df = pd.DataFrame(roll_limited_data)
roll_limited_candidate_output_file = "motor_command_candidate_roll_limited.csv"
roll_limited_candidate_df.to_csv(roll_limited_candidate_output_file, index=False)

print("\n[Offline Roll Rate Limit Candidate]")
print(f"Temporary roll rate limit: {roll_rate_limit:.3f} rad/frame")
print(
    "joint, motor_id, original_max_abs_velocity, limited_max_abs_velocity, "
    "original_max_abs_acceleration, limited_max_abs_acceleration, "
    "original_max_abs_motor_delta_tick_per_frame, "
    "limited_max_abs_motor_delta_tick_per_frame"
)
for result in roll_limited_comparison:
    print(
        f"{result['joint']}, {result['motor_id']}, "
        f"{result['original_max_abs_velocity']:.9f}, "
        f"{result['limited_max_abs_velocity']:.9f}, "
        f"{result['original_max_abs_acceleration']:.9f}, "
        f"{result['limited_max_abs_acceleration']:.9f}, "
        f"{result['original_max_abs_motor_delta']:.9f}, "
        f"{result['limited_max_abs_motor_delta']:.9f}"
    )
print(
    f"Saved offline roll rate-limited candidate CSV: "
    f"{roll_limited_candidate_output_file} (no motor commands are sent)."
)

roll_rate_limit_candidates = [0.03, 0.05, 0.08, 0.10]
roll_rate_limit_sweep_results = []

for rate_limit_candidate in roll_rate_limit_candidates:
    max_abs_velocity = 0.0
    max_abs_acceleration = 0.0
    max_abs_motor_delta = 0.0
    max_abs_angle_difference = 0.0

    for joint in roll_rate_limit_joints:
        original_angles = df[joint]
        limited_angles = []

        if not original_angles.empty:
            previous_candidate = original_angles.iloc[0]
            limited_angles.append(previous_candidate)
            for original_current in original_angles.iloc[1:]:
                candidate_delta = np.clip(
                    original_current - previous_candidate,
                    -rate_limit_candidate,
                    rate_limit_candidate,
                )
                previous_candidate += candidate_delta
                limited_angles.append(previous_candidate)

        limited_angle_series = pd.Series(
            limited_angles,
            index=original_angles.index,
        )
        limited_velocity = limited_angle_series.diff() / del_t
        limited_acceleration = limited_velocity.diff() / del_t

        config = joint_motor_map[joint]
        limited_motor_positions = limited_angle_series.apply(
            lambda angle, joint_name=joint, joint_config=config: (
                angle_rad_to_position(angle, joint_name, joint_config)
            )
        )
        limited_motor_delta = limited_motor_positions.diff()

        max_abs_velocity = max(
            max_abs_velocity,
            limited_velocity.abs().max(),
        )
        max_abs_acceleration = max(
            max_abs_acceleration,
            limited_acceleration.abs().max(),
        )
        max_abs_motor_delta = max(
            max_abs_motor_delta,
            limited_motor_delta.abs().max(),
        )
        max_abs_angle_difference = max(
            max_abs_angle_difference,
            (original_angles - limited_angle_series).abs().max(),
        )

    roll_rate_limit_sweep_results.append({
        "rate_limit": rate_limit_candidate,
        "max_abs_velocity": max_abs_velocity,
        "max_abs_acceleration": max_abs_acceleration,
        "max_abs_motor_delta": max_abs_motor_delta,
        "max_abs_angle_difference": max_abs_angle_difference,
    })

print("\n[Roll Rate Limit Sweep]")
print(
    "rate_limit_rad_per_frame, max_abs_roll_joint_velocity, "
    "max_abs_roll_joint_acceleration, "
    "max_abs_roll_motor_delta_tick_per_frame, "
    "max_abs_original_candidate_angle_difference"
)
for result in roll_rate_limit_sweep_results:
    print(
        f"{result['rate_limit']:.3f}, "
        f"{result['max_abs_velocity']:.9f}, "
        f"{result['max_abs_acceleration']:.9f}, "
        f"{result['max_abs_motor_delta']:.9f}, "
        f"{result['max_abs_angle_difference']:.9f}"
    )

roll_lowpass_alphas = [0.10, 0.20, 0.30, 0.50]
roll_lowpass_sweep_results = []
roll_lowpass_alpha020_data = {"frame": df["frame"]}

for alpha in roll_lowpass_alphas:
    max_abs_velocity = 0.0
    max_abs_acceleration = 0.0
    max_abs_motor_delta = 0.0
    max_abs_angle_difference = 0.0

    for joint in roll_rate_limit_joints:
        original_angles = df[joint]
        filtered_angles = []

        if not original_angles.empty:
            filtered_previous = original_angles.iloc[0]
            filtered_angles.append(filtered_previous)
            for original_current in original_angles.iloc[1:]:
                filtered_previous = (
                    alpha * original_current
                    + (1.0 - alpha) * filtered_previous
                )
                filtered_angles.append(filtered_previous)

        filtered_angle_series = pd.Series(
            filtered_angles,
            index=original_angles.index,
        )
        filtered_velocity = filtered_angle_series.diff() / del_t
        filtered_acceleration = filtered_velocity.diff() / del_t

        config = joint_motor_map[joint]
        motor_position_col = f"Motor{config['motor_id']}_pos"
        filtered_motor_positions = filtered_angle_series.apply(
            lambda angle, joint_name=joint, joint_config=config: (
                angle_rad_to_position(angle, joint_name, joint_config)
            )
        )
        filtered_motor_delta = filtered_motor_positions.diff()

        max_abs_velocity = max(
            max_abs_velocity,
            filtered_velocity.abs().max(),
        )
        max_abs_acceleration = max(
            max_abs_acceleration,
            filtered_acceleration.abs().max(),
        )
        max_abs_motor_delta = max(
            max_abs_motor_delta,
            filtered_motor_delta.abs().max(),
        )
        max_abs_angle_difference = max(
            max_abs_angle_difference,
            (original_angles - filtered_angle_series).abs().max(),
        )

        if alpha == 0.20:
            filtered_col = f"{joint}_lowpass_alpha020"
            roll_lowpass_alpha020_data[filtered_col] = filtered_angle_series
            roll_lowpass_alpha020_data[f"{filtered_col}_velocity"] = (
                filtered_velocity
            )
            roll_lowpass_alpha020_data[f"{filtered_col}_acceleration"] = (
                filtered_acceleration
            )
            roll_lowpass_alpha020_data[motor_position_col] = (
                filtered_motor_positions
            )
            roll_lowpass_alpha020_data[
                f"{motor_position_col}_delta_per_frame"
            ] = filtered_motor_delta

    roll_lowpass_sweep_results.append({
        "alpha": alpha,
        "max_abs_velocity": max_abs_velocity,
        "max_abs_acceleration": max_abs_acceleration,
        "max_abs_motor_delta": max_abs_motor_delta,
        "max_abs_angle_difference": max_abs_angle_difference,
    })

print("\n[Roll Low Pass Filter Sweep]")
print(
    "alpha, max_abs_roll_joint_velocity, "
    "max_abs_roll_joint_acceleration, "
    "max_abs_roll_motor_delta_tick_per_frame, "
    "max_abs_original_filtered_angle_difference"
)
for result in roll_lowpass_sweep_results:
    print(
        f"{result['alpha']:.2f}, "
        f"{result['max_abs_velocity']:.9f}, "
        f"{result['max_abs_acceleration']:.9f}, "
        f"{result['max_abs_motor_delta']:.9f}, "
        f"{result['max_abs_angle_difference']:.9f}"
    )

roll_lowpass_alpha020_candidate_df = pd.DataFrame(roll_lowpass_alpha020_data)
roll_lowpass_alpha020_output_file = (
    "motor_command_candidate_roll_lowpass_alpha020.csv"
)
roll_lowpass_alpha020_candidate_df.to_csv(
    roll_lowpass_alpha020_output_file,
    index=False,
)
print(
    f"Saved offline roll low-pass candidate CSV: "
    f"{roll_lowpass_alpha020_output_file} (no motor commands are sent)."
)

roll_rate_lowpass_combinations = [
    (0.05, 0.20),
    (0.05, 0.30),
    (0.08, 0.20),
    (0.08, 0.30),
]
roll_rate_lowpass_sweep_results = []

for rate_limit_candidate, alpha in roll_rate_lowpass_combinations:
    max_abs_velocity = 0.0
    max_abs_acceleration = 0.0
    max_abs_motor_delta = 0.0
    max_abs_angle_difference = 0.0

    for joint in roll_rate_limit_joints:
        original_angles = df[joint]
        rate_limited_angles = []

        if not original_angles.empty:
            previous_rate_limited = original_angles.iloc[0]
            rate_limited_angles.append(previous_rate_limited)
            for original_current in original_angles.iloc[1:]:
                rate_limited_delta = np.clip(
                    original_current - previous_rate_limited,
                    -rate_limit_candidate,
                    rate_limit_candidate,
                )
                previous_rate_limited += rate_limited_delta
                rate_limited_angles.append(previous_rate_limited)

        rate_limited_series = pd.Series(
            rate_limited_angles,
            index=original_angles.index,
        )
        filtered_angles = []
        if not rate_limited_series.empty:
            filtered_previous = rate_limited_series.iloc[0]
            filtered_angles.append(filtered_previous)
            for rate_limited_current in rate_limited_series.iloc[1:]:
                filtered_previous = (
                    alpha * rate_limited_current
                    + (1.0 - alpha) * filtered_previous
                )
                filtered_angles.append(filtered_previous)

        candidate_angles = pd.Series(
            filtered_angles,
            index=original_angles.index,
        )
        candidate_velocity = candidate_angles.diff() / del_t
        candidate_acceleration = candidate_velocity.diff() / del_t

        config = joint_motor_map[joint]
        candidate_motor_positions = candidate_angles.apply(
            lambda angle, joint_name=joint, joint_config=config: (
                angle_rad_to_position(angle, joint_name, joint_config)
            )
        )
        candidate_motor_delta = candidate_motor_positions.diff()

        max_abs_velocity = max(
            max_abs_velocity,
            candidate_velocity.abs().max(),
        )
        max_abs_acceleration = max(
            max_abs_acceleration,
            candidate_acceleration.abs().max(),
        )
        max_abs_motor_delta = max(
            max_abs_motor_delta,
            candidate_motor_delta.abs().max(),
        )
        max_abs_angle_difference = max(
            max_abs_angle_difference,
            (original_angles - candidate_angles).abs().max(),
        )

    roll_rate_lowpass_sweep_results.append({
        "rate_limit": rate_limit_candidate,
        "alpha": alpha,
        "max_abs_velocity": max_abs_velocity,
        "max_abs_acceleration": max_abs_acceleration,
        "max_abs_motor_delta": max_abs_motor_delta,
        "max_abs_angle_difference": max_abs_angle_difference,
    })

print("\n[Roll Rate Limit Plus Low Pass Sweep]")
print(
    "rate_limit_rad_per_frame, alpha, max_abs_roll_joint_velocity, "
    "max_abs_roll_joint_acceleration, "
    "max_abs_roll_motor_delta_tick_per_frame, "
    "max_abs_original_candidate_angle_difference"
)
for result in roll_rate_lowpass_sweep_results:
    print(
        f"{result['rate_limit']:.3f}, "
        f"{result['alpha']:.2f}, "
        f"{result['max_abs_velocity']:.9f}, "
        f"{result['max_abs_acceleration']:.9f}, "
        f"{result['max_abs_motor_delta']:.9f}, "
        f"{result['max_abs_angle_difference']:.9f}"
    )

original_ik_csv = Path("walk_forward_debug_slow_original.csv")
damped_ik_csv = Path("walk_forward_debug_slow_damped.csv")

if original_ik_csv.exists() and damped_ik_csv.exists():
    comparison_datasets = {
        "original": pd.read_csv(original_ik_csv),
        "damped": pd.read_csv(damped_ik_csv),
    }
    comparison_results = {}

    def max_across_columns(dataset, column_names):
        available_columns = [
            column for column in column_names if column in dataset.columns
        ]
        if not available_columns:
            return float("nan")
        return dataset[available_columns].max().max()

    def min_across_columns(dataset, column_names):
        available_columns = [
            column for column in column_names if column in dataset.columns
        ]
        if not available_columns:
            return float("nan")
        return dataset[available_columns].min().min()

    for dataset_name, comparison_df in comparison_datasets.items():
        max_roll_velocity = 0.0
        max_roll_acceleration = 0.0
        max_roll_motor_delta = 0.0

        for joint in roll_rate_limit_joints:
            if joint not in comparison_df.columns:
                continue

            unwrapped_angle = pd.Series(
                np.unwrap(comparison_df[joint].to_numpy()),
                index=comparison_df.index,
            )
            joint_velocity = unwrapped_angle.diff() / del_t
            joint_acceleration = joint_velocity.diff() / del_t
            max_roll_velocity = max(
                max_roll_velocity,
                joint_velocity.abs().max(),
            )
            max_roll_acceleration = max(
                max_roll_acceleration,
                joint_acceleration.abs().max(),
            )

            config = joint_motor_map[joint]
            motor_positions = comparison_df[joint].apply(
                lambda angle, joint_name=joint, joint_config=config: (
                    angle_rad_to_position(angle, joint_name, joint_config)
                )
            )
            max_roll_motor_delta = max(
                max_roll_motor_delta,
                motor_positions.diff().abs().max(),
            )

        converged_columns = [
            column
            for column in ("RL_converged", "LL_converged")
            if column in comparison_df.columns
        ]
        converged_false_count = (
            int((comparison_df[converged_columns] == 0).sum().sum())
            if converged_columns
            else 0
        )

        comparison_results[dataset_name] = {
            "max_ik_condition_number": max_across_columns(
                comparison_df,
                ["RL_condition_number", "LL_condition_number"],
            ),
            "min_singular_value": min_across_columns(
                comparison_df,
                ["RL_min_singular_value", "LL_min_singular_value"],
            ),
            "max_ik_frame_delta": max_across_columns(
                comparison_df,
                ["RL_max_abs_frame_delta", "LL_max_abs_frame_delta"],
            ),
            "max_roll_joint_velocity": max_roll_velocity,
            "max_roll_joint_acceleration": max_roll_acceleration,
            "max_roll_motor_delta_tick_per_frame": max_roll_motor_delta,
            "ik_final_err_max": max_across_columns(
                comparison_df,
                ["RL_final_ERR", "LL_final_ERR"],
            ),
            "converged_false_count": converged_false_count,
        }

    print("\n[Original vs Damped IK Comparison]")
    print("metric, original, damped")
    comparison_metric_order = [
        "max_ik_condition_number",
        "min_singular_value",
        "max_ik_frame_delta",
        "max_roll_joint_velocity",
        "max_roll_joint_acceleration",
        "max_roll_motor_delta_tick_per_frame",
        "ik_final_err_max",
        "converged_false_count",
    ]
    for metric in comparison_metric_order:
        original_value = comparison_results["original"][metric]
        damped_value = comparison_results["damped"][metric]
        if metric == "converged_false_count":
            print(f"{metric}, {original_value}, {damped_value}")
        else:
            print(f"{metric}, {original_value:.9e}, {damped_value:.9e}")

# Offline comparison only. This guard does not modify the C++ IK result.
continuity_limit = 0.05
continuity_limited_data = {"frame": df["frame"]}
original_max_abs_joint_velocity = 0.0
candidate_max_abs_joint_velocity = 0.0
original_max_abs_joint_acceleration = 0.0
candidate_max_abs_joint_acceleration = 0.0
original_max_abs_motor_delta = 0.0
candidate_max_abs_motor_delta = 0.0
max_abs_continuity_angle_difference = 0.0

for joint in cols:
    candidate_col = f"{joint}_continuity_limited"
    original_angles = df[joint]
    continuity_limited_angles = []

    if not original_angles.empty:
        previous_candidate = original_angles.iloc[0]
        continuity_limited_angles.append(previous_candidate)
        for original_current in original_angles.iloc[1:]:
            candidate_delta = np.clip(
                original_current - previous_candidate,
                -continuity_limit,
                continuity_limit,
            )
            previous_candidate += candidate_delta
            continuity_limited_angles.append(previous_candidate)

    candidate_angles = pd.Series(
        continuity_limited_angles,
        index=original_angles.index,
    )
    candidate_velocity = candidate_angles.diff() / del_t
    candidate_acceleration = candidate_velocity.diff() / del_t
    continuity_limited_data[candidate_col] = candidate_angles
    continuity_limited_data[f"{candidate_col}_velocity"] = candidate_velocity
    continuity_limited_data[f"{candidate_col}_acceleration"] = (
        candidate_acceleration
    )

    original_velocity = joint_motion_data[joint]["velocity"]
    original_acceleration = joint_motion_data[joint]["acceleration"]
    original_max_abs_joint_velocity = max(
        original_max_abs_joint_velocity,
        original_velocity.abs().max(),
    )
    candidate_max_abs_joint_velocity = max(
        candidate_max_abs_joint_velocity,
        candidate_velocity.abs().max(),
    )
    original_max_abs_joint_acceleration = max(
        original_max_abs_joint_acceleration,
        original_acceleration.abs().max(),
    )
    candidate_max_abs_joint_acceleration = max(
        candidate_max_abs_joint_acceleration,
        candidate_acceleration.abs().max(),
    )
    max_abs_continuity_angle_difference = max(
        max_abs_continuity_angle_difference,
        (original_angles - candidate_angles).abs().max(),
    )

    config = joint_motor_map[joint]
    motor_position_col = f"Motor{config['motor_id']}_pos"
    candidate_motor_positions = candidate_angles.apply(
        lambda angle, joint_name=joint, joint_config=config: (
            angle_rad_to_position(angle, joint_name, joint_config)
        )
    )
    candidate_motor_delta = candidate_motor_positions.diff()
    continuity_limited_data[motor_position_col] = candidate_motor_positions
    continuity_limited_data[f"{motor_position_col}_delta_per_frame"] = (
        candidate_motor_delta
    )

    original_motor_delta = candidate_df[motor_position_col].diff()
    original_max_abs_motor_delta = max(
        original_max_abs_motor_delta,
        original_motor_delta.abs().max(),
    )
    candidate_max_abs_motor_delta = max(
        candidate_max_abs_motor_delta,
        candidate_motor_delta.abs().max(),
    )

continuity_limited_candidate_df = pd.DataFrame(continuity_limited_data)
continuity_limited_output_file = (
    "motor_command_candidate_continuity_limited.csv"
)
continuity_limited_candidate_df.to_csv(
    continuity_limited_output_file,
    index=False,
)

print("\n[Offline IK Solution Continuity Candidate]")
print(f"Temporary continuity limit: {continuity_limit:.3f} rad/frame")
print("metric, original, continuity_candidate")
print(
    f"max_abs_joint_velocity, {original_max_abs_joint_velocity:.9f}, "
    f"{candidate_max_abs_joint_velocity:.9f}"
)
print(
    f"max_abs_joint_acceleration, "
    f"{original_max_abs_joint_acceleration:.9f}, "
    f"{candidate_max_abs_joint_acceleration:.9f}"
)
print(
    f"max_abs_motor_delta_tick_per_frame, "
    f"{original_max_abs_motor_delta:.9f}, "
    f"{candidate_max_abs_motor_delta:.9f}"
)
print(
    f"max_abs_original_candidate_angle_difference, 0.000000000, "
    f"{max_abs_continuity_angle_difference:.9f}"
)
print(
    f"Saved offline continuity-limited candidate CSV: "
    f"{continuity_limited_output_file} (no motor commands are sent)."
)

spike_debug_joints = ["RL1_wrap", "RL5_wrap", "LL1_wrap", "LL5_wrap"]
spike_debug_motor_columns = [
    "Motor13_pos", "Motor21_pos", "Motor14_pos", "Motor22_pos",
]
spike_debug_data = {"frame": df["frame"]}

for joint in spike_debug_joints:
    unwrapped_angle = joint_motion_data[joint]["angle"]
    spike_debug_data[f"{joint}_angle"] = df[joint]
    spike_debug_data[f"{joint}_unwrapped_angle"] = unwrapped_angle
    spike_debug_data[f"{joint}_angle_delta_per_frame"] = unwrapped_angle.diff()
    spike_debug_data[f"{joint}_velocity"] = joint_motion_data[joint]["velocity"]
    spike_debug_data[f"{joint}_acceleration"] = joint_motion_data[joint][
        "acceleration"
    ]

for motor_position_col in spike_debug_motor_columns:
    motor_positions = candidate_df[motor_position_col]
    spike_debug_data[motor_position_col] = motor_positions
    spike_debug_data[f"{motor_position_col}_delta_per_frame"] = (
        motor_positions.diff()
    )

spike_debug_full_df = pd.DataFrame(spike_debug_data)
spike_debug_df = spike_debug_full_df[
    spike_debug_full_df["frame"].between(390, 410)
].copy()
spike_debug_output_file = "spike_debug_frame390_410.csv"
spike_debug_df.to_csv(spike_debug_output_file, index=False)
print(f"Saved spike detail CSV: {spike_debug_output_file}")

spike_console_df = spike_debug_df[
    spike_debug_df["frame"].between(398, 405)
]
print("\n[Spike Debug Frames 398-405]")
print("[Spike Joint Detail]")
for joint in spike_debug_joints:
    joint_console_columns = [
        "frame",
        f"{joint}_angle",
        f"{joint}_unwrapped_angle",
        f"{joint}_angle_delta_per_frame",
        f"{joint}_velocity",
        f"{joint}_acceleration",
    ]
    print(f"\n[{joint}]")
    print(
        spike_console_df[joint_console_columns].to_string(
            index=False,
            float_format=lambda value: f"{value:.9f}",
        )
    )

motor_console_columns = ["frame"]
for motor_position_col in spike_debug_motor_columns:
    motor_console_columns.extend([
        motor_position_col,
        f"{motor_position_col}_delta_per_frame",
    ])
print("\n[Spike Motor Position Detail]")
print(
    spike_console_df[motor_console_columns].to_string(
        index=False,
        float_format=lambda value: f"{value:.9f}",
    )
)

if all(col in df.columns for col in trajectory_cols):
    spike_trajectory_data = {"frame": df["frame"]}
    for trajectory_col in trajectory_cols:
        spike_trajectory_data[trajectory_col] = df[trajectory_col]
        spike_trajectory_data[f"{trajectory_col}_delta_per_frame"] = (
            df[trajectory_col].diff()
        )

    spike_trajectory_df = pd.DataFrame(spike_trajectory_data)
    spike_trajectory_console_df = spike_trajectory_df[
        spike_trajectory_df["frame"].between(398, 405)
    ]
    print("\n[Spike Trajectory Detail]")
    print(
        spike_trajectory_console_df.to_string(
            index=False,
            float_format=lambda value: f"{value:.9f}",
        )
    )
else:
    print("Spike trajectory detail skipped: reference columns not found")

ik_debug_cols = [
    "RL_condition_number", "LL_condition_number",
    "RL_min_singular_value", "LL_min_singular_value",
    "RL_iteration_count", "LL_iteration_count",
    "RL_final_ERR", "LL_final_ERR",
    "RL_converged", "LL_converged",
    "RL_max_abs_delta_theta", "LL_max_abs_delta_theta",
    "RL_delta_theta_1", "RL_delta_theta_5",
    "LL_delta_theta_1", "LL_delta_theta_5",
]
ik_condition_number_records = []
ik_min_singular_value_records = []
ik_final_err_records = []
ik_max_delta_theta_records = []
ik_condition_number_limit = 1.0e4
ik_min_singular_value_limit = 1.0e-6

if all(col in df.columns for col in ik_debug_cols):
    spike_ik_columns = ["frame"] + ik_debug_cols
    spike_ik_debug_df = df.loc[
        df["frame"].between(398, 405),
        spike_ik_columns,
    ]
    print("\n[Spike IK Debug Detail]")
    print(
        spike_ik_debug_df.to_string(
            index=False,
            float_format=lambda value: f"{value:.9e}",
        )
    )

    for leg in ("RL", "LL"):
        condition_col = f"{leg}_condition_number"
        min_singular_col = f"{leg}_min_singular_value"
        final_err_col = f"{leg}_final_ERR"
        max_delta_col = f"{leg}_max_abs_delta_theta"
        converged_col = f"{leg}_converged"

        for row_index in df.index:
            frame = int(df.loc[row_index, "frame"])
            ik_condition_number_records.append({
                "leg": leg,
                "frame": frame,
                "condition_number": df.loc[row_index, condition_col],
            })
            ik_min_singular_value_records.append({
                "leg": leg,
                "frame": frame,
                "min_singular_value": df.loc[row_index, min_singular_col],
            })
            ik_final_err_records.append({
                "leg": leg,
                "frame": frame,
                "final_ERR": df.loc[row_index, final_err_col],
            })
            ik_max_delta_theta_records.append({
                "leg": leg,
                "frame": frame,
                "max_abs_delta_theta": df.loc[row_index, max_delta_col],
            })

        condition_violations = df[condition_col] > ik_condition_number_limit
        if condition_violations.any():
            violation_values = df.loc[condition_violations, condition_col]
            row_index = violation_values.idxmax()
            record_warning(
                "IK Numerical Stability Check",
                f"IK condition number limit exceeded: leg={leg}, "
                f"frame={int(df.loc[row_index, 'frame'])}, "
                f"condition_number={df.loc[row_index, condition_col]:.9e}, "
                f"count={int(condition_violations.sum())}",
            )

        singular_value_violations = (
            df[min_singular_col] < ik_min_singular_value_limit
        )
        if singular_value_violations.any():
            violation_values = df.loc[
                singular_value_violations,
                min_singular_col,
            ]
            row_index = violation_values.idxmin()
            record_warning(
                "IK Numerical Stability Check",
                f"IK minimum singular value below limit: leg={leg}, "
                f"frame={int(df.loc[row_index, 'frame'])}, "
                f"min_singular_value={df.loc[row_index, min_singular_col]:.9e}, "
                f"count={int(singular_value_violations.sum())}",
            )

        convergence_failures = df[converged_col] == 0
        if convergence_failures.any():
            failed_err_values = df.loc[convergence_failures, final_err_col]
            row_index = failed_err_values.idxmax()
            record_warning(
                "IK Numerical Stability Check",
                f"IK convergence failure: leg={leg}, "
                f"frame={int(df.loc[row_index, 'frame'])}, "
                f"final_ERR={df.loc[row_index, final_err_col]:.9e}, "
                f"count={int(convergence_failures.sum())}",
            )
else:
    print("IK debug columns not found, numerical stability check skipped")

ik_frame_delta_cols = [
    *[f"RL_frame_delta_{joint_index}" for joint_index in range(6)],
    *[f"LL_frame_delta_{joint_index}" for joint_index in range(6)],
    "RL_max_abs_frame_delta",
    "LL_max_abs_frame_delta",
]
ik_frame_delta_records = []
ik_frame_delta_limit = 0.10

if all(col in df.columns for col in ik_frame_delta_cols):
    ik_frame_delta_detail_cols = [
        "frame",
        "RL_frame_delta_1",
        "RL_frame_delta_5",
        "LL_frame_delta_1",
        "LL_frame_delta_5",
    ]
    ik_frame_delta_detail_df = df.loc[
        df["frame"].between(398, 405),
        ik_frame_delta_detail_cols,
    ]
    print("\n[IK Frame Delta Detail]")
    print(
        ik_frame_delta_detail_df.to_string(
            index=False,
            float_format=lambda value: f"{value:.9f}",
        )
    )

    for leg in ("RL", "LL"):
        max_frame_delta_col = f"{leg}_max_abs_frame_delta"
        for row_index in df.index:
            ik_frame_delta_records.append({
                "leg": leg,
                "frame": int(df.loc[row_index, "frame"]),
                "max_abs_frame_delta": df.loc[row_index, max_frame_delta_col],
            })

        frame_delta_violations = df[max_frame_delta_col] > ik_frame_delta_limit
        if frame_delta_violations.any():
            violation_values = df.loc[
                frame_delta_violations,
                max_frame_delta_col,
            ]
            row_index = violation_values.idxmax()
            record_warning(
                "IK Solution Continuity Check",
                f"IK frame delta limit exceeded: leg={leg}, "
                f"frame={int(df.loc[row_index, 'frame'])}, "
                f"max_abs_frame_delta="
                f"{df.loc[row_index, max_frame_delta_col]:.9f} rad/frame, "
                f"count={int(frame_delta_violations.sum())}",
            )
else:
    print("IK frame delta columns not found, solution continuity check skipped")

zoom_frame_radius = 20
max_zoom_warning_count = 5

top_joint_warnings = sorted(
    joint_acceleration_warning_representatives,
    key=lambda warning: abs(warning["acceleration"]),
    reverse=True,
)[:max_zoom_warning_count]

for warning in top_joint_warnings:
    joint = warning["joint"]
    warning_frame = warning["frame"]
    frame_mask = df["frame"].between(
        warning_frame - zoom_frame_radius,
        warning_frame + zoom_frame_radius,
    )

    joint_zoom_plots = [
        ("angle", "angle [rad]"),
        ("velocity", "angular velocity [rad/s]"),
        ("acceleration", "angular acceleration [rad/s^2]"),
    ]
    for plot_name, y_label in joint_zoom_plots:
        values = joint_motion_data[joint][plot_name]
        plt.figure()
        plt.plot(df.loc[frame_mask, "frame"], values.loc[frame_mask])
        plt.axvline(warning_frame, color="red", linestyle="--", label="warning frame")
        plt.title(f"{joint} {plot_name} near frame {warning_frame}")
        plt.xlabel("frame")
        plt.ylabel(y_label)
        plt.grid(True)
        plt.legend()
        plt.savefig(f"{joint}_zoom_{plot_name}_frame{warning_frame}.png")
        plt.close()

top_motor_warnings = sorted(
    motor_delta_warning_representatives,
    key=lambda warning: abs(warning["delta"]),
    reverse=True,
)[:max_zoom_warning_count]

for warning in top_motor_warnings:
    motor_position_col = warning["position_col"]
    warning_frame = warning["frame"]
    frame_mask = candidate_df["frame"].between(
        warning_frame - zoom_frame_radius,
        warning_frame + zoom_frame_radius,
    )
    motor_positions = candidate_df[motor_position_col]
    motor_position_delta = motor_positions.diff()

    motor_zoom_plots = [
        ("position", motor_positions, "position [tick]"),
        ("delta", motor_position_delta, "position delta [tick/frame]"),
    ]
    for plot_name, values, y_label in motor_zoom_plots:
        plt.figure()
        plt.plot(candidate_df.loc[frame_mask, "frame"], values.loc[frame_mask])
        plt.axvline(warning_frame, color="red", linestyle="--", label="warning frame")
        plt.title(f"{motor_position_col} {plot_name} near frame {warning_frame}")
        plt.xlabel("frame")
        plt.ylabel(y_label)
        plt.grid(True)
        plt.legend()
        plt.savefig(
            f"{motor_position_col}_zoom_{plot_name}_frame{warning_frame}.png"
        )
        plt.close()

if top_joint_warnings or top_motor_warnings:
    print(
        "Saved warning zoom plots for up to 5 joint acceleration warnings "
        "and 5 motor delta warnings."
    )

print("\n[Top IK Condition Number]")
print("leg, frame, condition_number")
for result in sorted(
    ik_condition_number_records,
    key=lambda item: item["condition_number"],
    reverse=True,
)[:10]:
    print(
        f"{result['leg']}, {result['frame']}, "
        f"{result['condition_number']:.9e}"
    )

print("\n[Top IK Min Singular Value]")
print("leg, frame, min_singular_value")
for result in sorted(
    ik_min_singular_value_records,
    key=lambda item: item["min_singular_value"],
)[:10]:
    print(
        f"{result['leg']}, {result['frame']}, "
        f"{result['min_singular_value']:.9e}"
    )

print("\n[Top IK Final ERR]")
print("leg, frame, final_ERR")
for result in sorted(
    ik_final_err_records,
    key=lambda item: item["final_ERR"],
    reverse=True,
)[:10]:
    print(
        f"{result['leg']}, {result['frame']}, "
        f"{result['final_ERR']:.9e}"
    )

print("\n[Top IK Max Abs Delta Theta]")
print("leg, frame, max_abs_delta_theta")
for result in sorted(
    ik_max_delta_theta_records,
    key=lambda item: item["max_abs_delta_theta"],
    reverse=True,
)[:10]:
    print(
        f"{result['leg']}, {result['frame']}, "
        f"{result['max_abs_delta_theta']:.9e}"
    )

print("\n[Top IK Frame Delta]")
print("leg, frame, max_abs_frame_delta")
for result in sorted(
    ik_frame_delta_records,
    key=lambda item: item["max_abs_frame_delta"],
    reverse=True,
)[:10]:
    print(
        f"{result['leg']}, {result['frame']}, "
        f"{result['max_abs_frame_delta']:.9f}"
    )

print("\n[Top Joint Velocity]")
print("joint, frame, max_abs_velocity")
for result in sorted(
    joint_velocity_maxima,
    key=lambda item: item["max_abs_velocity"],
    reverse=True,
)[:10]:
    print(
        f"{result['joint']}, {result['frame']}, "
        f"{result['max_abs_velocity']:.9f}"
    )

print("\n[Top Joint Acceleration]")
print("joint, frame, max_abs_acceleration")
for result in sorted(
    joint_acceleration_maxima,
    key=lambda item: item["max_abs_acceleration"],
    reverse=True,
)[:10]:
    print(
        f"{result['joint']}, {result['frame']}, "
        f"{result['max_abs_acceleration']:.9f}"
    )

print("\n[Top Motor Command Delta]")
print("motor, frame, max_abs_delta_tick_per_frame")
for result in sorted(
    motor_delta_maxima,
    key=lambda item: item["max_abs_delta_tick_per_frame"],
    reverse=True,
)[:10]:
    print(
        f"{result['motor']}, {result['frame']}, "
        f"{result['max_abs_delta_tick_per_frame']:.9f}"
    )

print("\n[Offline Debug Summary]")
for category, status in check_status.items():
    print(f"{category}: {status}")

if not warning_messages:
    print("No major offline issue detected.")
else:
    print("Warnings:")
    for warning_message in warning_messages:
        print(f"- {warning_message}")
