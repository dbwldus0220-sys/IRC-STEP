# Single Motor Test Usage

This document describes how to use `Dynamics/single_motor_test.cpp` for real-robot motor verification.

This program must be used before any walking command is sent to the real robot.

## Safety rules

- Do not run walking commands during this test.
- Test only one motor at a time.
- Start with read-only mode.
- Use very small movement only.
- Keep `MOVE_TICK` within the configured safety limit.
- Stop immediately if the wrong joint moves, direction is unexpected, or the joint approaches a mechanical limit.

## Expected STEP leg motor IDs

Right leg:

- 10: RL0 / RHY
- 13: RL1 / RHRx
- 15: RL2 / RHP
- 17: RL3 / RKN
- 19: RL4 / RAP
- 21: RL5 / RAR

Left leg:

- 12: LL0 / LHY
- 14: LL1 / LHR
- 16: LL2 / LHP
- 18: LL3 / LKN
- 20: LL4 / LAP
- 22: LL5 / LAR

## 1. ID scan mode

Use this first after connecting the Dynamixel bus.

Expected behavior:

- Opens the port.
- Checks expected leg motor IDs.
- Prints which IDs respond.
- Does not enable torque.
- Does not write goal position.
- Does not move any motor.

Example build:

```bash
cd ~/IRC/IRC-STEP/Dynamics

g++ -std=c++17 single_motor_test.cpp -o single_motor_id_scan \
  -DID_SCAN=1 \
  -ldxl_x64_cpp