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