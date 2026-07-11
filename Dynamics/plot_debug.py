import math

import pandas as pd
import matplotlib.pyplot as plt

df = pd.read_csv("walk_forward_debug_slow.csv")

cols = [
    "RL0_wrap", "RL1_wrap", "RL2_wrap", "RL3_wrap", "RL4_wrap", "RL5_wrap",
    "LL0_wrap", "LL1_wrap", "LL2_wrap", "LL3_wrap", "LL4_wrap", "LL5_wrap",
]

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
        print(
            f"Joint velocity limit violation: joint={col}, "
            f"frame={int(df.loc[row_index, 'frame'])}, "
            f"velocity={velocity.loc[row_index]:.9f} rad/s"
        )

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

joint_angle_min = -3.0
joint_angle_max = 3.0
limit_violation_found = False

for col in cols:
    if col not in df.columns:
        print(f"Skipped limit check for {col}: column not found.")
        continue

    violations = df[(df[col] < joint_angle_min) | (df[col] > joint_angle_max)]
    for _, row in violations.iterrows():
        print(
            f"Joint angle limit violation: joint={col}, "
            f"frame={int(row['frame'])}, value={row[col]:.9f} rad"
        )
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

# TODO: motor_id, model, direction, zero_offset, min_rad, and max_rad are placeholders.
joint_motor_map = {
    joint_name: {
        "motor_id": motor_id,
        "model": "MX-64",
        "direction": 1,
        "zero_offset": 2048,
        "min_rad": -3.0,
        "max_rad": 3.0,
    }
    for motor_id, joint_name in enumerate(cols, start=1)
}


def angle_rad_to_position(angle_rad, joint_config):
    """Convert radians to a temporary Dynamixel position value without communication."""
    model_config = motor_models[joint_config["model"]]
    limited_angle = min(
        max(angle_rad, joint_config["min_rad"]),
        joint_config["max_rad"],
    )
    ticks_per_radian = model_config["ticks_per_revolution"] / (2.0 * math.pi)
    position = round(
        joint_config["zero_offset"]
        + joint_config["direction"] * limited_angle * ticks_per_radian
    )
    return min(
        max(position, model_config["position_min"]),
        model_config["position_max"],
    )


print("Temporary offline Dynamixel position ranges (no commands are sent):")
for col in cols:
    if col not in df.columns:
        print(f"Skipped position conversion for {col}: column not found.")
        continue

    config = joint_motor_map[col]
    positions = df[col].map(lambda angle: angle_rad_to_position(angle, config))
    print(
        f"{col}: motor_id={config['motor_id']}, model={config['model']}, "
        f"position min/max={positions.min()} / {positions.max()}"
    )
