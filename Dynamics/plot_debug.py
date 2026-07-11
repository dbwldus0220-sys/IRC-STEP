import math

import pandas as pd
import matplotlib.pyplot as plt

df = pd.read_csv("walk_forward_debug_slow.csv")

cols = [
    "RL0_wrap", "RL1_wrap", "RL2_wrap", "RL3_wrap", "RL4_wrap", "RL5_wrap",
    "LL0_wrap", "LL1_wrap", "LL2_wrap", "LL3_wrap", "LL4_wrap", "LL5_wrap",
]

warning_messages = []
check_status = {
    "IK Angle Check": "PASS",
    "Joint Velocity Check": "PASS",
    "Joint Acceleration Check": "PASS",
    "Dynamixel Position Check": "PASS",
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

    velocity = delta / del_t
    max_abs_velocity = velocity.abs().max()
    print(f"{col} max abs(velocity): {max_abs_velocity:.9f} rad/s")

    velocity_violations = velocity.abs() > velocity_limit
    for row_index in velocity[velocity_violations].index:
        warning_message = (
            f"Joint velocity limit violation: joint={col}, "
            f"frame={int(df.loc[row_index, 'frame'])}, "
            f"velocity={velocity.loc[row_index]:.9f} rad/s"
        )
        record_warning("Joint Velocity Check", warning_message)

    plt.figure()
    plt.plot(df["frame"].iloc[1:], velocity.iloc[1:])
    plt.axhline(velocity_limit, color="red", linestyle="--", label="velocity limit")
    plt.axhline(-velocity_limit, color="red", linestyle="--")
    plt.title(f"{col} velocity")
    plt.xlabel("frame")
    plt.ylabel("angular velocity [rad/s]")
    plt.grid(True)
    plt.legend()
    plt.savefig(f"{col}_velocity.png")
    plt.close()

print("Saved selected wrapped joint delta plots as PNG files.")
print("Saved selected wrapped joint velocity plots as PNG files.")

acceleration_limit = 100.0

for col in cols:
    if col not in df.columns:
        print(f"Skipped acceleration check for {col}: column not found.")
        continue

    velocity = df[col].diff() / del_t
    acceleration = velocity.diff() / del_t
    valid_acceleration = acceleration.dropna()
    max_abs_acceleration = valid_acceleration.abs().max()
    print(
        f"{col} max abs(acceleration): "
        f"{max_abs_acceleration:.9f} rad/s^2"
    )

    acceleration_violations = valid_acceleration.abs() > acceleration_limit
    for row_index in valid_acceleration[acceleration_violations].index:
        warning_message = (
            f"Joint acceleration limit exceeded: joint={col}, "
            f"frame={int(df.loc[row_index, 'frame'])}, "
            f"acceleration={valid_acceleration.loc[row_index]:.9f} rad/s^2"
        )
        record_warning("Joint Acceleration Check", warning_message)

    plt.figure()
    plt.plot(df.loc[valid_acceleration.index, "frame"], valid_acceleration)
    plt.axhline(
        acceleration_limit,
        color="red",
        linestyle="--",
        label="acceleration limit",
    )
    plt.axhline(-acceleration_limit, color="red", linestyle="--")
    plt.title(f"{col} acceleration")
    plt.xlabel("frame")
    plt.ylabel("angular acceleration [rad/s^2]")
    plt.grid(True)
    plt.legend()
    plt.savefig(f"{col}_acceleration.png")
    plt.close()

print("Saved all wrapped joint acceleration plots as PNG files.")

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
print("Offline candidate motor position ranges:")
for col in cols:
    motor_id = joint_motor_map[col]["motor_id"]
    motor_position_col = f"Motor{motor_id}_pos"
    print(
        f"{motor_position_col}: min={candidate_df[motor_position_col].min()}, "
        f"max={candidate_df[motor_position_col].max()}"
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
