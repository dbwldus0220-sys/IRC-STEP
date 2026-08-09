# Gravity free-base constant-pose initialization test

This diagnostic world resets all 12 leg joints to the test-only standing
candidate with at least 20 mm static COM margin before the first physics
update. It does not change the existing fixed-base or free-base models,
controller gains, command scaling, or walking code.

SDFormat's old `<axis><initial_position>` tag is not used: the installed
SDFormat 14 schema marks it as unimplemented and removes it after SDF 1.8.
Instead, the test-only system calls Gazebo's `Joint::ResetPosition` during
system configuration. Each existing `JointPositionController` also receives
the same `<initial_position>` as its initial hold target.

## Build the test-only initializer

From the repository root:

```bash
cmake \
  -S Simulation/Gazebo/plugins/constant_pose_initializer \
  -B /tmp/step_constant_pose_initializer_build \
  -DCMAKE_BUILD_TYPE=Release
cmake --build /tmp/step_constant_pose_initializer_build --parallel
```

## Start the world

```bash
export GZ_SIM_RESOURCE_PATH="$PWD/Simulation/Gazebo/models${GZ_SIM_RESOURCE_PATH:+:$GZ_SIM_RESOURCE_PATH}"
export GZ_SIM_SYSTEM_PLUGIN_PATH="/tmp/step_constant_pose_initializer_build${GZ_SIM_SYSTEM_PLUGIN_PATH:+:$GZ_SIM_SYSTEM_PLUGIN_PATH}"

gz sim \
  Simulation/Gazebo/worlds/step_free_base_constant_pose_initial_test.world.sdf
```

The world opens paused. Before pressing play, the model is already configured
at the standing-candidate joint pose. The sole boxes have about 0.008 rad of
roll and nearly zero pitch; the world spawn height places the lowest right
sole corner at ground height.

The controllers already use the constant pose as their initial target. To
also publish the target continuously from the diagnostic script, run this
before pressing play:

```bash
python3 Simulation/Gazebo/scripts/publish_leg_constant_pose.py \
  --duration 10 \
  --dt 0.05 \
  --standing-candidate \
  --hold-initial-pose
```

`--hold-initial-pose` starts with interpolation factor `1.0`; it does not ramp
from zero. Without this option, the existing smoothstep ramp remains the
default behavior.

## Check initial joint position and sole contact

Gazebo's Contact system creates the sensor publishers during the first
simulation update. If the GUI has opened paused and has never advanced, first
press play briefly, or execute exactly one step while keeping the world
paused:

```bash
gz service \
  -s /world/step_constant_pose_initial_test/control \
  --reqtype gz.msgs.WorldControl \
  --reptype gz.msgs.Boolean \
  --timeout 3000 \
  --req 'step: true'
```

The publishers can then be verified independently of whether contact is
currently occurring:

```bash
gz topic -i -t /step/left_sole_contacts
gz topic -i -t /step/right_sole_contacts
```

Both commands should report a `gz.msgs.Contacts` publisher. `No publishers`
before the first step does not mean that the collision reference is invalid;
it means the Contact system has not executed its first update yet. If it still
appears after a step, confirm that the model and initializer plugin were loaded
without errors and that `GZ_SIM_RESOURCE_PATH`, `GZ_SIM_SYSTEM_PLUGIN_PATH`,
and `GZ_PARTITION` match between the Gazebo and `gz topic` terminals.

The model publishes all leg joint states on:

```bash
gz topic -e -t /step/leg_joint_states
```

The test world loads Gazebo's Contact system. Each flat sole collision has an
independent contact sensor:

```bash
gz topic -e -t /step/left_sole_contacts
gz topic -e -t /step/right_sole_contacts
```

A valid initial contact message names both the corresponding
`*_sole_box_collision` and `ground_plane::link::collision`. Its contact normal
should be close to `0 0 1`, and penetration `depth` should remain small. In the
GUI, enable collision visualization to inspect the two sole boxes directly.

This is an initialization and contact diagnostic, not a balance controller.
A free-base humanoid may still fall later if its center of mass leaves the
support polygon or the unchanged gains cannot reject the disturbance. Test in
simulation only; do not transfer this initializer directly to hardware.
