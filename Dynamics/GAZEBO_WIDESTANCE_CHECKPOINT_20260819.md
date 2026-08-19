# Gazebo Wide-Stance Checkpoint — 2026-08-19

## Root-cause finding

Original STEP_Dynamics planner:
- LEFT foot = +60 mm
- RIGHT foot = -60 mm
- stance width = 120 mm
- start ZMP ~= +57 / -60 mm

Candidate-B Gazebo:
- LEFT sole ~= +96 mm
- RIGHT sole ~= -97 mm
- stance width ~= 194 mm

generate_walk_csv_sdf_6d_ik.py discards absolute source position:
    delta = source_positions - source_positions[0]
and applies the delta on Candidate B.

Therefore a 120-mm-stance lateral COM trajectory was being replayed
on the ~194-mm Candidate-B stance.

## Baseline evidence

At RIGHT liftoff around t=0.85:
- BASELINE COM relative LEFT sole ~= -62.0 mm
- WIDE1616 COM relative LEFT sole ~= -38.4 mm

WIDE1616 significantly reduced early LEFT-single-support collapse.

At t=1.20:
BASELINE:
- basePitch ~= -12.92 deg
- COMrelLEFT ~= -121.8 mm
- COM Vx ~= -0.366 m/s

WIDE1616:
- basePitch ~= -3.41 deg
- COMrelLEFT ~= -66.8 mm
- COM Vx ~= -0.181 m/s

Conclusion:
wide-stance lateral COM transfer is a major correction.

## WIDE1616 failure

Full 1.616 lateral scaling caused excessive RIGHT lateral velocity
at touchdown and severe slip.

## RIGHTLANDSOFT / HYBRID

RIGHT landing lateral motion was softened while WIDE LEFT/COM stayed.

Result:
- liftoff remained good
- touchdown improved
- large post-touchdown yaw twist developed
- LEFT sole yaw reached about +17 deg
- base yaw reached about +13.5 deg

## SOFTRETURN

COM_y, Ref_RL_y, Ref_LL_y, CP_y were returned together using
baseline-rate after the wide pre-liftoff transfer.

IK:
- RIGHT 250/250
- LEFT 250/250
- RIGHT max 191.34 deg/s
- LEFT max 160.99 deg/s
- no joint-limit violation
- Candidate B frame 0 exact

Compared with HYBRID, SOFTRETURN reduced post-touchdown slip/yaw:
at t=1.44:
- RIGHT yaw: +14.02 -> +8.61 deg
- LEFT yaw: +17.36 -> +11.07 deg
- base yaw: +13.51 -> +8.78 deg

But support transfer still fails.

SOFTRETURN contact example:
- t=1.20 RFz=72.1 N, LFz=32.5 N
- t=1.22 RFz=0.0 N, LFz=25.7 N
- t=1.24 RFz=57.2 N, LFz=14.2 N
- t=1.26 RFz=55.3 N, LFz=1.4 N
- t=1.32 RFz=0.0 N, LFz=68.6 N

Late baseRoll is also worse:
- HYBRID t=1.44: -16.25 deg
- SOFTRETURN t=1.44: -19.17 deg

## Current baseline settings

- Candidate B
- joint P=80, I=0, D=0
- lateral Kp=0.10, Kd=0.015, max=0.015 m
- pitch Kp=0.10, Kd=0.010, max=2 deg
- RIGHT swing lateral scale=0.75
- touchdown flatten=-0.5 deg
- touchdown Z arrest enabled
- pitch support handoff +1, max=0.5 deg

## Next investigation

Do NOT continue global lateral gain / stance sweeps first.

Use SOFTRETURN as the current experiment reference and inspect
t ~= 1.18 to 1.34 s:

1. RIGHT sole Z and vertical velocity immediately before touchdown
2. RIGHT/LEFT Fz and contact bounce
3. touchdown-arrest state
4. pitch support-handoff beta/state
5. baseRoll and COM Y / COM Vy
6. RIGHT foot fore-aft motion and contact geometry

Main remaining hypothesis:
RIGHT touchdown impact / rebound and support-transfer timing are causing
the post-touchdown failure after the wide-stance single-support fix.
