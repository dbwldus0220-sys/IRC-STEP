# Command Evaluation Summary

## Current result

### command_1
- Status: candidate for STEP_ROLL_SCALE_TEST roll50.
- Result: initial right-leg lateral kick disappeared in Gazebo replay.
- Remaining issue: very slight twisting remains.
- Safety: no sudden bending or full rotation observed.

### command_2
- Status: not a roll50 candidate.
- Result: roll50 slightly reduced side bending but made the motion clearly smaller.
- Remaining issue: small motion and side bending.
- Safety: no sudden bending or full rotation observed.

### command_3
- Status: not a roll50 candidate.
- Result: yaw isolation did not reduce twisting; no-roll replay reduced twisting slightly.
- Remaining issue: twisting is caused by mixed roll/pitch or replay structure, not yaw alone.
- Safety: no sudden bending or full rotation observed.

### command_32
- Status: do not force roll50.
- Result: roll50 did not remove initial lateral kick.
- Remaining issue: initial right-leg lateral kick likely from start transition, trajectory structure, support-leg compensation, or fixed-base replay.
- Safety: no sudden bending or full rotation observed after roll stabilization tests.