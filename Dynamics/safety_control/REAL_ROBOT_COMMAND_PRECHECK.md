# Real Robot Command Pre-check

This document summarizes which motion commands are currently safe candidates for real-robot testing and which commands should remain blocked or require further Gazebo/debug validation.

## Current safety principle

Before sending any command to the real robot:

1. The command must be generated through dry-run mode first.
2. The final `All_Theta` command must be logged as `safe_*`.
3. Roll guard or command-specific scale behavior must be checked.
4. Gazebo replay must show no sudden bending, full rotation, or large discontinuity.
5. Commands with unresolved lateral kick, twisting, or strong side bending should not be sent to the real robot as normal walking motions.

---

## Command status table

| Command | Current status | Applied correction | Gazebo result | Real robot decision |
|---|---|---|---|---|
| command_1 | Candidate | command_1-only roll50 under `STEP_ROLL_SCALE_TEST` | Initial right-leg lateral kick disappeared. Slight twisting remains. No sudden bending or full rotation. | Test candidate with caution |
| command_2 | Needs improvement | None | Small motion. Side bending remains. No sudden bending or full rotation. | Hold / do not use as normal motion yet |
| command_3 | Needs improvement | None | Small motion and side twisting remain. Yaw-only isolation did not fix it. No-roll reduced twisting slightly but did not remove it. | Hold / do not use as normal motion yet |
| command_32 | Needs improvement | None after scope update | Initial right-leg lateral kick remains. Roll50 did not solve it. Soft-start helped but reduced motion size. | Hold / analyze start transition |

## command_1 real-robot test conditions

`command_1` is the only current command-specific roll scale candidate.

Expected settings:

- `STEP_DRY_RUN_NO_DXL=OFF` only when actual robot testing is intentionally started.
- `STEP_ROLL_SCALE_TEST=ON` if testing the command_1 roll50 candidate.
- command_1:
  - right roll scale = 0.5
  - left roll scale = 0.5
- command_2, command_3, command_32:
  - right roll scale = 1.0
  - left roll scale = 1.0

Before real-robot execution, confirm in dry-run logs:

- command_1 `roll_scale_applied sum = rows`
- command_2/3/32 `roll_scale_applied sum = 0`
- no unexpected roll guard spikes
- no sudden jump in `safe_*`

## Commands not ready for normal real-robot use

### command_2

Reason:

- Motion appears small in Gazebo.
- Side bending remains.
- roll50 made side bending slightly smaller but also made the motion clearly smaller.
- Therefore roll50 is not a good correction for command_2.

Decision:

- Do not use as a normal real-robot motion yet.
- Requires trajectory/posture quality improvement.

### command_3

Reason:

- Motion appears small in Gazebo.
- Side twisting remains.
- Removing yaw did not reduce twisting.
- Removing roll reduced twisting slightly, but did not remove it.
- Therefore command_3 twisting is not a yaw-only or roll-only problem.

Decision:

- Do not use as a normal real-robot motion yet.
- Requires combined roll/pitch trajectory review.

### command_32

Reason:

- Initial right-leg lateral kick remains.
- roll50 and right40_left50 did not meaningfully remove the kick.
- pitch_step025 did not remove the kick.
- lateral-slide world reduced the kick only slightly.
- soft-start reduced the kick more, but also reduced motion size.

Decision:

- Do not use as a normal real-robot motion yet.
- Treat as a start-transition / trajectory-structure issue.

## Real-robot pre-test checklist

### Startup-safe staged prepare limitation

With `STEP_REAL_ROBOT_STARTUP_SAFE=ON`, run startup commands strictly in this
order: command_90 (configure), command_91 (present-position preload and torque
enable), command_92 (CENTER), and command_93 (WALK_READY). Normal motion must
remain blocked until command_93 completes.

`Dxl::read_rad()` currently reads each present-position register without
checking every SDK communication result or confirming that data is available.
Before real-robot use, add per-ID response validation and prevent torque enable
if any joint position is missing or invalid. Until then, command_91 must be
tested with the robot physically supported and emergency power-off reachable.

Before enabling Dynamixel communication:

- [ ] Confirm the robot is physically supported or held safely.
- [ ] Confirm emergency power-off is reachable.
- [ ] Confirm only the intended command is being tested.
- [ ] Confirm dry-run CSV was generated immediately before the test.
- [ ] Confirm `safe_*` max step is within expected range.
- [ ] Confirm no unexpected `roll_guard_used_*` spike.
- [ ] Confirm command-specific roll scale is applied only to command_1.
- [ ] Confirm command_2, command_3, and command_32 are not accidentally selected for normal motion.
- [ ] Start with low-risk posture or single-motion test, not continuous walking.
- [ ] Stop immediately if one leg kicks laterally, twists, or bends unexpectedly.

## Command gate dry-run verification result

`STEP_REAL_ROBOT_COMMAND_GATE` was verified with dry-run mode.

Build options:

- `STEP_DRY_RUN_NO_DXL=ON`
- `STEP_ROLL_SCALE_TEST=ON`
- `STEP_REAL_ROBOT_COMMAND_GATE=ON`

Verification command:

```bash
python3 safety_control/scripts/verify_dry_run.py --expect-gate --commands 1 2 3 32

Result:

command_1: ALLOWED
rows: 540
roll guard used sum for roll joints 1, 5, 7, 11: 0
max_safe_step was reduced compared to max_raw_step, as expected from command_1 roll50.
command_2: BLOCKED_EXPECTED
command_3: BLOCKED_EXPECTED
command_32: BLOCKED_EXPECTED

Conclusion:

The command gate correctly allows only command_1.
command_2, command_3, and command_32 are blocked when STEP_REAL_ROBOT_COMMAND_GATE=ON.
command_1 roll50 works without triggering the roll guard.

## Startup safe dry-run verification

`STEP_REAL_ROBOT_STARTUP_SAFE` was verified together with:

- `STEP_DRY_RUN_NO_DXL=ON`
- `STEP_ROLL_SCALE_TEST=ON`
- `STEP_REAL_ROBOT_COMMAND_GATE=ON`
- `STEP_REAL_ROBOT_STARTUP_SAFE=ON`

Verified sequence:

1. Node startup did not run automatic CENTER or WALK_READY motion.
2. DOF monitor waited for command 90 and did not send Dynamixel read packets before hardware prepare.
3. command_1 before hardware prepare was blocked with `hardware_not_prepared`.
4. command_90 performed hardware prepare in dry-run mode.
5. After command_90, command_1 was allowed.
6. command_1 generated `safety_all_theta_command_log.csv`.
7. command_1 roll50 was applied:
   - rows: 540
   - right roll scale: 0.5
   - left roll scale: 0.5
   - roll_scale_applied sum: 540
8. command_2, command_3, and command_32 remained blocked by the command gate.

Important real-robot note:

- `command_90` is the hardware activation command.
- On the real robot, command_90 may enable torque, write operating mode/PID settings, preload present position, and move through CENTER and WALK_READY.
- command_90 must only be tested with the robot physically supported and emergency power-off available.


## Startup safe staged verification

`STEP_REAL_ROBOT_STARTUP_SAFE` was updated so that real-robot startup preparation is split across commands 90 through 93.

Staged startup commands:

| Command | Meaning | Resulting stage |
|---|---|---|
| command_90 | Hardware configure: torque OFF, operating mode, LED, PID setup | Configured |
| command_91 | Present-position preload and torque enable | TorqueEnabled |
| command_92 | Move to CENTER | Centered |
| command_93 | Move to WALK_READY | WalkReady |

Dry-run verification confirmed:

- command_1 before command_93 was blocked with `walk_ready_not_completed`.
- command_92 before command_91 was blocked with `torque_not_enabled`.
- command_90 completed and advanced to `Configured`.
- command_91 completed and advanced to `TorqueEnabled`.
- command_92 completed and advanced to `Centered`.
- command_93 completed and advanced to `WalkReady`.
- command_1 was allowed only after `WalkReady`.
- command_2, command_3, and command_32 remained blocked by `STEP_REAL_ROBOT_COMMAND_GATE`.

Command_1 verification after staged startup:

- `safety_all_theta_command_log.csv` was generated.
- rows: 540
- right roll scale: 0.5
- left roll scale: 0.5
- roll scale was applied to all command_1 rows.

Important real-robot note:

- command_90 writes configuration registers.
- command_91 enables torque.
- command_92 is currently the riskiest stage because it moves the whole robot to CENTER.
- command_93 moves from CENTER to WALK_READY.
- Actual robot testing must use physical support and emergency power-off.


### Dry-run validation of real-robot command gate and startup-safe flow

A dry-run validation was performed with:

- `STEP_DRY_RUN_NO_DXL=ON`
- `STEP_REAL_ROBOT_COMMAND_GATE=ON`
- `STEP_REAL_ROBOT_STARTUP_SAFE=ON`

Result:

- On node startup, automatic torque enable, PID write, CENTER, and WALK_READY were disabled.
- DOF monitor waited for command 90 and did not send Dynamixel read packets in dry-run mode.
- command 90 was accepted and advanced the startup stage to `Configured`.
- command 91 was accepted and advanced the startup stage to `TorqueEnabled`.
- command 92 was accepted and advanced the startup stage to `Centered`.
- command 93 was accepted and advanced the startup stage to `WalkReady`.
- command 1 was allowed and generated a safety CSV.
- The CSV contained only `go=1`.
- command 2, command 3, and command 32 were blocked by `COMMAND_GATE` with reason `not_approved_for_real_robot_test`.

Judgment:

- The real-robot safety build correctly blocks non-approved walking commands.
- The startup-safe sequence correctly requires staged commands 90 → 91 → 92 → 93 before normal command execution.
- The dry-run mode confirms this flow without sending Dynamixel packets.


Important real-robot note:

- command_90 writes configuration registers.
- command_91 enables torque.
- command_92 is currently the riskiest stage because it moves the whole robot to CENTER.
- command_93 moves the robot to WALK_READY.
- command_1 is only the first low-risk walking command allowed by the current gate; it is not a full approval for unrestricted walking.


### Hip roll scale candidates before real-robot testing

Gazebo validation found that excessive hip roll variation was the main cause of diagonal leg twisting in command 1 and command 32.

Current Gazebo candidates:

- command 1: hip roll scale 0.0
- command 32: hip roll scale 0.0

Important safety notes:

- These are Gazebo candidates only.
- They must not be treated as final real-robot parameters.
- Command 32 should remain blocked by the real-robot command gate until separate approval.
- Command 1 should only be tested after motor direction signs, zero offsets, and joint limits are confirmed.
- The first real-robot test should use a short, low-risk command 1 trial with conservative torque/speed settings.
- If the robot leans, twists, or loads the ankle/hip unexpectedly, stop immediately.

### Real-robot command gate dry-run verification

The real-robot command gate was checked in dry-run mode with `STEP_REAL_ROBOT_COMMAND_GATE=ON` and `STEP_REAL_ROBOT_STARTUP_SAFE=ON`.

Command 32 single-command test:

- command sent: 32
- safety_all_theta_command_log.csv was not generated
- judgment: command 32 did not enter trajectory logging/execution

Command 2 single-command test:

- command sent: 2
- safety_all_theta_command_log.csv was not generated
- judgment: command 2 did not enter trajectory logging/execution

Command 3 single-command test:

- command sent: 3
- safety_all_theta_command_log.csv was not generated
- judgment: command 3 did not enter trajectory logging/execution

Conclusion:

- command 2, command 3, and command 32 remain blocked for real-robot testing.
- command 32 is still only a Gazebo candidate and is not approved for real-robot execution.