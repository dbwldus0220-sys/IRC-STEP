# STEP Dynamics Offline Debug Candidates

## Current Best Candidate

Date: 2026-07-16

Current best:
- y_scale = 0.02
- baseline continuation
- roll branch jumpguard threshold = 0.05 rad/frame
- roll LPF alpha = 0.50
- pitch-chain LPF alpha = 0.80
- roll acceleration limiter: not used

Build command:

g++ -std=c++17 -O2 \
  -DSTEP_DEBUG_SIMULATION \
  -DSTEP_DEBUG_SIMULATION_MAIN \
  -DSTEP_DEBUG_LONG_WALK \
  -DSTEP_DEBUG_IK_TARGET_Y_SCALE_LONG \
  -DSTEP_DEBUG_IK_TARGET_Y_SCALE=0.02 \
  -DSTEP_DEBUG_IK_TARGET_Y_SCALE_LABEL=002 \
  -DSTEP_DEBUG_IK_Y_SCALE_CONTINUATION \
  -DSTEP_DEBUG_IK_Y_SCALE_CONTINUATION_FROM_BASELINE \
  -DSTEP_DEBUG_IK_Y_CONTINUATION_STEPS=5 \
  -DSTEP_DEBUG_IK_ROLL_BRANCH_JUMP_REJECT \
  -DSTEP_DEBUG_IK_ROLL_BRANCH_JUMP_THRESHOLD=0.05 \
  -DSTEP_DEBUG_IK_ROLL_POST_LOW_PASS \
  -DSTEP_DEBUG_IK_ROLL_POST_LOW_PASS_ALPHA=0.50 \
  -DSTEP_DEBUG_IK_PITCH_CHAIN_POST_LOW_PASS \
  -DSTEP_DEBUG_IK_PITCH_CHAIN_POST_LOW_PASS_ALPHA=0.80 \
  Dynamics/STEP_Dynamics.cpp \
  -o Dynamics/step_debug_current_best

## Result summary

- One-turn roll branch issue mostly removed.
- Large backward leg extension reduced.
- Remaining weak lateral jerk exists in fixed-base Gazebo replay.
- Current best is not final real-robot walking control. It is an offline replay stabilization candidate.

Key numbers:
- RL_roll_abs_sum max ≈ 0.623
- LL_roll_abs_sum max ≈ 0.471
- RL_post_guard_fk_pos_err max ≈ 2.09 mm
- LL_post_guard_fk_pos_err max ≈ 1.20 mm
