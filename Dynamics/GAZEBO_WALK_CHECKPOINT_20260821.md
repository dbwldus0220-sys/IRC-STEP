# Gazebo walking checkpoint — 2026-08-21

## Current best baseline

Current best Gazebo walking configuration:

- trajectory:
  `Dynamics/walk_forward_gazebo_sdf6dik_candidateB_widestance1616_softreturn.csv`
- planner reference:
  `Dynamics/walk_forward_debug_widestance1616_softreturn.csv`
- RIGHT world-flat:
  - start = 0.76 s
  - end = 1.06 s
  - ramp-in = 0.10 s
  - ramp-out = 0.10 s
- RIGHT swing-world-Z:
  - start = 0.76 s
  - end = 1.14 s
  - ramp-in = 0.10 s
  - ramp-out = 0.15 s
  - correction release limit = 0.20 m/s
- RIGHT swing lateral scale = 0.75
- RIGHT lateral reference time = 0.83 s
- lateral balance:
  - Kp = 0.10
  - Kd = 0.015
  - max offset = 0.015 m
  - start/end = 0.80 / 1.30 s
- pitch-balance:
  - Kp = 0.10
  - Kd = 0.010
  - max = 2.0 deg
  - start/end = 0.80 / 1.44 s
- RIGHT touchdown flatten = -0.5 deg
- touchdown Z arrest:
  - threshold = 20 N
  - confirm = 0.02 s
- support handoff ramp = 0.20 s
- RIGHT support sign = +1
- RIGHT support max angle = 0.5 deg
- feedback joint speed limiter = 240 deg/s

## RIGHT world-flat finding

Keeping RIGHT world-flat active through touchdown caused extreme 6D IK
joint-rate requests.

Previous confirmed-Z-latch run:

- max requested RIGHT rate 1.14~1.30 s = 2105.3 deg/s
- all 17/17 frames exceeded 240 deg/s
- baseRoll @1.44 = -18.11 deg

Ending world-flat earlier substantially fixed this.

With world-flat end = 1.06 s:

- max requested rate around touchdown fell to approximately 200~210 deg/s
- robot no longer immediately fell
- RIGHT support became visibly better

Therefore world-flat end = 1.06 s is the current baseline.
Do not restore the old long world-flat window.

## Support handoff finding

Changing pitch-balance support handoff ramp:

- 0.08 s -> 0.20 s

visually reduced LEFT foot lifting/edge-lift behavior and improved
RIGHT support.

The 0.20 s ramp also reduced the maximum RIGHT requested joint rate
versus the 0.08 s handoff run.

Current baseline handoff ramp = 0.20 s.

## Confirmed touchdown Z settle

The previous confirmed Z latch prevented RIGHT world-Z target from
following later base motion upward, but it also froze an edge-contact
touchdown at a high sole-center Z.

Measured geometry showed:

- RIGHT sole center Z approximately 12~14 mm
- sole roll/pitch approximately 2~4 deg
- rotated sole lowest point approximately ground level
- flat sole center height would be approximately 7.5 mm

Thus the RIGHT sole was contacting through an edge/corner while the
center stayed several millimeters too high.

Publisher was modified to add optional confirmed-touchdown downward
settle while preserving the upward re-lift prevention.

New CLI:

- `--touchdown-z-arrest-settle-depth-m`
- `--touchdown-z-arrest-settle-speed-mps`

Current best:

- settle depth = 0.003 m
- settle speed = 0.020 m/s

Behavior:

- settle floor is latched once from confirmed touchdown Z
- downward-only
- air_world_z rising cannot lift the target
- hold is advanced only after successful IK
- default depth/speed 0 preserves the previous behavior

3 mm settle result versus no-settle:

- RIGHT contact >=1 N: 16/25 -> 21/25
- RIGHT contact >=10 N: 9/25 -> 12/25
- RIGHT contact >=20 N: 4/25 -> 6/25
- BOTH >=20 N: 1/25 -> 2/25
- baseRoll @1.30: -4.60 -> -4.23 deg
- baseRoll @1.44: -2.70 -> -2.10 deg
- yaw delta @1.44: +4.46 -> +3.94 deg
- max RIGHT requested rate remained below 240 deg/s
- settle reached exactly 3.0 mm

This is the best result so far.

## Replay extension to 1.70 s

Default replay-end was 1.44 s.

Current best configuration was replayed to 1.70 s without changing any
controller parameter.

Visual result:

- RIGHT foot successfully supports the robot when LEFT swing begins
- LEFT swing can start
- body tilt and yaw increase again during LEFT swing
- RIGHT step visually appears wide

Key measured geometry:

At t = 1.42 s:
- R-L lateral X = -202.6 mm
- R-L forward = +40.3 mm
- COM-R lateral X = +93.8 mm
- COM-R forward = -35.6 mm

At t = 1.44 s:
- R-L lateral X = -203.5 mm
- R-L forward = +39.8 mm
- COM-R lateral X = +88.4 mm

At t = 1.60 s:
- R-L lateral X = -212.8 mm
- R-L forward = +28.5 mm
- COM-R lateral X = +61.3 mm
- baseRoll = -4.11 deg
- yaw delta from 1.10 = +3.59 deg

At t = 1.70 s:
- R-L lateral X = -217.9 mm
- R-L forward = +21.1 mm
- COM-R lateral X = +64.5 mm
- baseRoll = -6.43 deg
- yaw delta from 1.10 = +4.97 deg

Actual RIGHT movement from t=0.83:

- to 1.18: lateral +3.1 mm, forward +31.2 mm
- to 1.42: lateral -8.4 mm, forward +33.7 mm
- to 1.60: lateral -10.3 mm, forward +32.6 mm
- to 1.70: lateral -7.9 mm, forward +31.1 mm

Therefore the visually wide RIGHT step is NOT primarily excessive
sagittal step length.

The stronger issue is lateral support geometry:

- R/L foot separation is already approximately 203 mm when LEFT swing begins
- it grows above 210 mm during swing
- RIGHT sole lateral half-width is approximately 45 mm
- COM remains approximately 60~90 mm laterally away from RIGHT sole center

Thus when LEFT becomes unloaded, COM is not sufficiently transferred
over the RIGHT support foot.

## Next resume point

Do NOT tune RIGHT sagittal step length first.

Keep the current best touchdown configuration fixed:

- world-flat end 1.06
- handoff 0.20
- settle 3 mm @ 20 mm/s
- flatten -0.5 deg
- limiter 240 deg/s

Next investigation:

1. RIGHT single-support lateral geometry during LEFT liftoff.
2. Why COM remains 60~90 mm away from RIGHT sole center.
3. Distinguish wide-stance planner/reference geometry from insufficient
   COM transfer before LEFT swing.
4. Only after this diagnosis decide whether to modify the wide lateral
   trajectory or extend/change lateral support feedback.

Do not revert to the old long world-flat window.
Do not remove confirmed Z latch/settle.
Do not modify sagittal RIGHT step length based only on visual width.
