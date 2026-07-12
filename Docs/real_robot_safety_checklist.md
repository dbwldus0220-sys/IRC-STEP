# STEP Real Robot Safety Checklist

## 1. Current status

- Dynamics debug simulation completed.
- Gazebo fixed-base replay completed.
- Roll spike detected around frames 398~405.
- Selected safety guard candidate:
  - roll rate limit = 0.05 rad/frame
  - target joints: RL1, RL5, LL1, LL5
- Debug macro:
  - STEP_DEBUG_ROLL_RATE_LIMIT
- Control-loop candidate macro:
  - STEP_ROLL_RATE_LIMIT_SAFETY

This guard is not a final IK solution.
It is a motor protection safety guard before real robot testing.

---

## 2. Do not test full walking first

Before any walking command:

- Do not run full gait.
- Do not enable all motors immediately.
- Do not send original unguarded roll commands to motors.
- Do not test without emergency stop ready.

---

## 3. Required checks before motor power

### Motor ID mapping

Leg joint mapping:

| Joint | Meaning | Motor ID | All_Theta index |
|---|---|---:|---:|
| RL0 | RHY | 10 | 0 |
| RL1 | RHRx | 13 | 1 |
| RL2 | RHP | 15 | 2 |
| RL3 | RKN | 17 | 3 |
| RL4 | RAP | 19 | 4 |
| RL5 | RAR | 21 | 5 |
| LL0 | LHY | 12 | 6 |
| LL1 | LHR | 14 | 7 |
| LL2 | LHP | 16 | 8 |
| LL3 | LKN | 18 | 9 |
| LL4 | LAP | 20 | 10 |
| LL5 | LAR | 22 | 11 |

Safety guard target:

| Joint | Motor ID | All_Theta index |
|---|---:|---:|
| RL1 | 13 | 1 |
| RL5 | 21 | 5 |
| LL1 | 14 | 7 |
| LL5 | 22 | 11 |

---

## 4. Direction sign verification

Before walking, verify each target motor direction manually.

Minimum test:
- Use only one motor at a time.
- Use very small movement.
- Confirm physical direction.
- Stop immediately if direction is wrong.

Priority motors:
1. Motor 13 / RL1
2. Motor 21 / RL5
3. Motor 14 / LL1
4. Motor 22 / LL5

---

## 5. Zero offset and joint limit check

Before walking:
- Confirm zero posture.
- Confirm each joint mechanical safe range.
- Compare expected 0 rad with actual posture.
- Do not trust software limits until hardware is verified.

---

## 6. Single motor test first

Use `single_motor_test.cpp`.

Rules:
- Test only one motor.
- Start with small tick movement.
- Confirm torque off after test.
- Keep robot supported.
- Do not allow legs to carry full load.

Recommended first test:
- Motor ID 17 or 18 only if mechanically safe.
- Then test roll motors one by one after direction is confirmed.

---

## 7. Control-loop safety guard

Candidate folder:
- `Dynamics/safety_control`

Original folder:
- `Dynamics/original_control`

Safety macro:
- `STEP_ROLL_RATE_LIMIT_SAFETY`

Guard location:
- `Callback::Write_All_Theta()`
- After All_Theta[0]~All_Theta[22] are calculated
- Before `dxl_->SetThetaRef(callback_->All_Theta)`

Guard target:
- All_Theta[1]
- All_Theta[5]
- All_Theta[7]
- All_Theta[11]

Limit:
- 0.05 rad/frame

---

## 8. First real robot test order

1. Check emergency stop.
2. Check power supply.
3. Check torque off command.
4. Run single motor read-only check.
5. Run single motor tiny movement.
6. Verify motor direction.
7. Verify zero offset.
8. Test fixed posture.
9. Test one-frame command.
10. Test slow guarded motion.
11. Only then consider slow walking.

---

## 9. Do not use yet

Do not run:
- original full walking command
- unguarded original CSV
- fast walking
- turn motion
- full-body motion with arms/neck uncontrolled

---

## 10. Current conclusion

The current best safety candidate is:

- roll rate limit = 0.05 rad/frame

Reason:
- 0.03 was smoother but posture distortion was too large.
- 0.05 had the best balance between reduced jerk and posture tracking.
- 0.08 had more jerk and did not track posture well.
- 0.10 tracked posture better but jerk increased again.

This is a safety guard, not the final IK continuity solution.
