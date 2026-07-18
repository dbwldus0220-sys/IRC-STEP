# STEP foot orientation logging

`replay_roll_joints_from_csv.py` can record the world pose of
`right_foot_link` and `left_foot_link` while replaying a Dynamics CSV. Use the
original fixed-base joint-state world as the comparison baseline.

## Start the baseline world

From the repository root:

```bash
export GZ_SIM_RESOURCE_PATH="$PWD/Simulation/Gazebo/models${GZ_SIM_RESOURCE_PATH:+:$GZ_SIM_RESOURCE_PATH}"

gz sim -r \
  Simulation/Gazebo/worlds/step_fixed_base_ctrl_joint_state_test.world.sdf
```

## Replay Command 1 and log foot orientation

Use the same CSV path, frame range, offsets, and other replay options as the
existing Command 1 baseline. Add `--log-foot-orientation`:

```bash
python3 Simulation/Gazebo/scripts/replay_roll_joints_from_csv.py \
  PATH/TO/command_1.csv \
  --mode legs \
  --start-frame 0 \
  --end-frame 500 \
  --dt 0.01 \
  --log-foot-orientation
```

Joint tracking and foot orientation logging can run together:

```bash
python3 Simulation/Gazebo/scripts/replay_roll_joints_from_csv.py \
  PATH/TO/command_1.csv \
  --mode legs \
  --start-frame 0 \
  --end-frame 500 \
  --dt 0.01 \
  --log-joint-tracking \
  --log-foot-orientation
```

## Diagnostic ankle-roll command scaling

Use `--ankle-roll-command-scale` to scale only the commands published to
`right_ankle_roll_joint` and `left_ankle_roll_joint`. The default is `1.0`, so
existing replay behavior is unchanged. This option does not modify the source
CSV or safety-control output.

For example, compare the original Command 1 baseline against a `1.5` ankle-roll
command scale while recording both joint tracking and foot orientation:

```bash
python3 Simulation/Gazebo/scripts/replay_roll_joints_from_csv.py \
  PATH/TO/command_1.csv \
  --mode legs \
  --start-frame 0 \
  --end-frame 500 \
  --dt 0.01 \
  --ankle-roll-command-scale 1.5 \
  --log-joint-tracking \
  --log-foot-orientation
```

The `command_position` column in `gazebo_joint_tracking_log.csv` contains the
scaled value actually sent to Gazebo. Start close to `1.0` and increase in
small steps. A larger command can increase overshoot, oscillation, foot tilt,
or contact force even when the mean tracking error appears smaller.

This is a diagnostic option, not a walking-control recommendation. A Command 1
test with scale `2.0` was rejected: it produced little visible improvement in
sole leveling, while foot pitch/yaw orientation and ankle-roll tracking error
remained worse. Keep the default at `1.0` for baseline replay.

The foot log is written to `gazebo_foot_orientation_log.csv` with these
columns:

```text
frame,time,link_name,x,y,z,roll,pitch,yaw
```

Angles are world-frame roll, pitch, and yaw in radians. A level sole should
have nearly constant world roll and pitch after accounting for the mesh/link
frame's zero-orientation offset; do not assume that both values must be zero
before checking that offset.

## Quick analysis

Print the roll/pitch range and maximum absolute values for each foot:

```bash
python3 -c 'import pandas as pd; d=pd.read_csv("gazebo_foot_orientation_log.csv"); print(d.groupby("link_name")[["roll", "pitch"]].agg(["min", "max"])); print(d.groupby("link_name")[["roll", "pitch"]].apply(lambda x: x.abs().max()))'
```

Inspect the first logged pose for each foot to identify the initial world-frame
orientation offset:

```bash
python3 -c 'import pandas as pd; d=pd.read_csv("gazebo_foot_orientation_log.csv"); print(d.groupby("link_name", sort=False).first()[["frame", "roll", "pitch", "yaw"]])'
```

## Ankle-roll command sign test

With the original fixed-base joint-state world running, execute all four step
commands (`+0.10`, `-0.10`, `+0.20`, `-0.20 rad`) for both ankle-roll joints:

```bash
python3 Simulation/Gazebo/scripts/test_ankle_roll_sign_orientation.py
```

The output is `gazebo_ankle_roll_sign_orientation_log.csv`. Each command case
is preceded by a zero-command reset period. This is a diagnostic test only; the
script does not modify an SDF, replay CSV, or safety-control output.

Summarize the initial-to-final foot roll and ankle actual-position changes:

```bash
python3 -c 'import pandas as pd; d=pd.read_csv("gazebo_ankle_roll_sign_orientation_log.csv"); g=d.groupby(["joint_name", "command_value"], sort=False); s=g.agg(foot_roll_first=("foot_roll", "first"), foot_roll_last=("foot_roll", "last"), ankle_actual_first=("ankle_actual", "first"), ankle_actual_last=("ankle_actual", "last")); s["foot_roll_delta"]=s.foot_roll_last-s.foot_roll_first; s["ankle_actual_delta"]=s.ankle_actual_last-s.ankle_actual_first; s["command_to_foot_roll_sign"]=s.foot_roll_delta.apply(lambda x: "positive" if x>0 else "negative" if x<0 else "zero"); print(s)'
```

For each side, compare the sign of `command_value` with `foot_roll_delta`.
Also verify `ankle_actual_delta`: a foot-roll sign conclusion is unreliable if
the ankle itself did not move enough. Because the link frame can have a nonzero
initial orientation, use the roll delta rather than the absolute world roll.
