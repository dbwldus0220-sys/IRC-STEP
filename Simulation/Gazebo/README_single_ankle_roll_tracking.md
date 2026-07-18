# STEP single ankle-roll Gazebo tracking test

This diagnostic sends commands to only one ankle-roll position-controller
topic. It does not modify the other leg joint commands.

Start the fixed-base joint-state test world from the repository root:

```bash
export GZ_SIM_RESOURCE_PATH=$GZ_SIM_RESOURCE_PATH:$(pwd)/Simulation/Gazebo/models
gz sim Simulation/Gazebo/worlds/step_fixed_base_ctrl_joint_state_test.world.sdf
```

Test the right ankle roll with a 0.20 rad, 0.5 Hz sine command for 8 seconds:

```bash
python3 Simulation/Gazebo/scripts/test_single_ankle_roll_tracking.py \
  --joint right \
  --waveform sine \
  --amplitude 0.20 \
  --frequency 0.5 \
  --duration 8 \
  --dt 0.05
```

Test the left ankle roll with a 0.20 rad step:

```bash
python3 Simulation/Gazebo/scripts/test_single_ankle_roll_tracking.py \
  --joint left \
  --waveform step \
  --amplitude 0.20 \
  --duration 8 \
  --dt 0.05
```

Any Gazebo joint exposed through the STEP position-controller topics can be
selected with `--joint-name`. The command topic is constructed automatically as
`/step/<joint_name>/cmd_pos`; this option takes precedence over `--joint`.

Test the right hip roll:

```bash
python3 Simulation/Gazebo/scripts/test_single_ankle_roll_tracking.py \
  --joint-name right_hip_roll_joint \
  --waveform sine \
  --amplitude 0.20 \
  --frequency 0.5 \
  --duration 8 \
  --dt 0.05
```

Test the right ankle pitch with a step command:

```bash
python3 Simulation/Gazebo/scripts/test_single_ankle_roll_tracking.py \
  --joint-name right_ankle_pitch_joint \
  --waveform step \
  --amplitude 0.20 \
  --duration 8 \
  --dt 0.05
```

The default output is `gazebo_single_joint_tracking_log.csv`. Each row contains
`frame,time,joint_name,command_position,actual_position,error`, where error is
`command_position - actual_position`. The script reads actual positions from
`/step/leg_joint_states` as `gz.msgs.Model.joint[].axis1.position` and sends a
final 0.0 command when the test exits.
