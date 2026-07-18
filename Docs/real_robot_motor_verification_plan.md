# Real Robot Motor Verification Plan

## Purpose

Before sending any walking command to the real STEP robot, verify motor ID mapping, joint direction signs, zero offsets, and safe joint limits one joint at a time.

This verification must be completed before running command 1 on the real robot.

## Current candidate leg mapping

Right leg:

- RL0 / RHY: motor ID 10
- RL1 / RHRx: motor ID 13
- RL2 / RHP: motor ID 15
- RL3 / RKN: motor ID 17
- RL4 / RAP: motor ID 19
- RL5 / RAR: motor ID 21

Left leg:

- LL0 / LHY: motor ID 12
- LL1 / LHR: motor ID 14
- LL2 / LHP: motor ID 16
- LL3 / LKN: motor ID 18
- LL4 / LAP: motor ID 20
- LL5 / LAR: motor ID 22

## Final command sign map from callback.cpp

The final All_Theta command sign map currently used before sending to Dynamixel is:

- All_Theta[0] / motor 10: -
- All_Theta[1] / motor 13: +
- All_Theta[2] / motor 15: +
- All_Theta[3] / motor 17: -
- All_Theta[4] / motor 19: -
- All_Theta[5] / motor 21: -
- All_Theta[6] / motor 12: -
- All_Theta[7] / motor 14: +
- All_Theta[8] / motor 16: -
- All_Theta[9] / motor 18: +
- All_Theta[10] / motor 20: +
- All_Theta[11] / motor 22: -

## Verification order

### 1. Read-only ID scan

- Connect Dynamixel bus.
- Do not send walking commands.
- Do not run command 1, 2, 3, or 32.
- Confirm that all expected motor IDs respond.
- Record missing, duplicated, or unexpected IDs.

Expected leg motor IDs:

- Right leg: 10, 13, 15, 17, 19, 21
- Left leg: 12, 14, 16, 18, 20, 22

### 2. Present position read check

- Read present position from each motor.
- Confirm values are stable.
- Confirm no motor reports abnormal angle or communication error.
- Do not apply large motion.

### 3. One-joint direction test

Use only a small command per joint.

Recommended first test magnitude:

- ±0.03 rad to ±0.05 rad only

For each joint:

- Move only one motor.
- Observe the physical joint direction.
- Confirm whether positive command matches expected joint positive direction.
- Stop immediately if the wrong joint moves.
- Stop immediately if the direction is reversed unexpectedly.
- Stop immediately if the joint approaches a mechanical limit.

### 4. Zero offset check

For each leg joint:

- Place the robot in a known safe reference posture.
- Read present position.
- Compare with expected zero or initial offset.
- Record required offset correction.
- Do not use walking commands until offsets are confirmed.

### 5. Joint limit check

For each joint:

- Confirm software limit.
- Confirm mechanical limit.
- Confirm that commanded test range is inside both limits.
- Use conservative limits before the first walking test.

### 6. Real walking command approval

Walking command 1 is allowed only after:

- Motor ID mapping verified
- Direction signs verified
- Zero offsets verified
- Joint limits verified
- Startup safe sequence verified
- Emergency stop procedure prepared
- Robot physically supported during the first test

Command 2, command 3, and command 32 must remain blocked for real-robot testing.

## Current Gazebo candidates

Gazebo validation found that excessive hip roll variation caused diagonal leg twisting.

Current Gazebo candidates:

- command 1: hip roll scale 0.0
- command 32: hip roll scale 0.0

These are Gazebo candidates only and must not be treated as final real-robot parameters.