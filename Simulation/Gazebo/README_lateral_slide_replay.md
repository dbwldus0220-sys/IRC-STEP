# STEP lateral-slide replay diagnosis

`step_lateral_slide_ctrl.sdf` is a diagnostic model for visualizing the pelvis
lateral motion that is assumed by the STEP walking IK. It replaces the fixed
world joint with a single prismatic joint along Gazebo X. All other base
translations and rotations remain constrained.

This is not a floating-base dynamics or walking-stability test. The base motion
is commanded from the CSV reference while the leg joints use position
controllers.

## Start Gazebo

From the repository root:

```bash
export GZ_SIM_RESOURCE_PATH=$GZ_SIM_RESOURCE_PATH:$(pwd)/Simulation/Gazebo/models
gz sim Simulation/Gazebo/worlds/step_lateral_slide_ctrl_test.world.sdf
```

The additional controller topic is:

```text
/step/base_lateral_joint/cmd_pos
```

## Replay legs and pelvis lateral motion

```bash
python3 Simulation/Gazebo/scripts/replay_roll_joints_from_csv.py \
  Dynamics/walk_forward_debug_slow_ik_post_limit012_long.csv \
  --mode legs \
  --relative-to-frame 0 \
  --use-gazebo-offsets \
  --mirror-left-pitch-chain \
  --replay-base-lateral-from-com-y \
  --base-lateral-axis-sign 1 \
  --base-lateral-scale 0.25 \
  --start-frame 0 \
  --end-frame 1349 \
  --dt 0.05 \
  --progress-every 50
```

Start with a small scale such as `0.25`, verify that the pelvis moves toward the
intended support side, and then increase toward `1.0`. If the lateral direction
is reversed, use `--base-lateral-axis-sign -1`.

The command is calculated from `COM_y` (or `Ycom` when `COM_y` is absent):

```text
base_lateral_cmd = scale * axis_sign * (value - value_at_base_frame)
```

`--relative-to-frame` selects the base frame. If it is omitted, the first
selected replay frame is used. STEP debug CSV files currently store `COM_y` in
meters; values that clearly look like millimeters are converted to meters by the
script and the detected unit is printed at startup.

## Lock the diagnostic base lateral joint

To replay all 12 leg joints while commanding the diagnostic base lateral joint
to remain at zero on every frame:

```bash
python3 Simulation/Gazebo/scripts/replay_roll_joints_from_csv.py \
  Dynamics/safety_control/safety_all_theta_command_log.csv \
  --mode legs \
  --lock-base-lateral \
  --start-frame 0 \
  --end-frame 500 \
  --dt 0.01
```

`--lock-base-lateral` cannot be combined with
`--replay-base-lateral-from-com-y`. If the optional lateral controller topic is
missing or publishing fails, the script prints a warning and continues replaying
the selected leg joints.
