# IRC-STEP

## STEP Dynamics Offline Debug

### 현재 오프라인 디버그 목적

- 실제 로봇에 적용하기 전에 `STEP_Dynamics.cpp`에서 계산한 관절각이 발산하지 않는지 확인합니다.
- CSV와 `plot_debug.py`를 이용해 wrapped angle, velocity, Dynamixel position 후보값을 확인합니다.
- 발 reference 궤적부터 관절각, 속도, 가속도, Dynamixel 변환까지 단계별로 검사해 문제 발생 구간을 구분합니다.

### 현재 안전 후보 보행 파라미터

```cpp
Go_Straight_start(0.03, 0.20, 0.02)
```

### 생성/검증 파일

- `Dynamics/walk_forward_debug_slow.csv`
- `Dynamics/plot_debug.py`
- `Dynamics/motor_command_candidate.csv` (실제 모터 명령 파일이 아닌 offline candidate)

### 검증 기능

- Wrapped joint angle의 joint angle limit을 검사합니다.
- Wrapped joint velocity와 joint acceleration을 계산하고 임시 limit 초과 여부를 검사합니다.
- `walk_forward_debug_slow.csv`에 좌·우 발 기준 궤적인 `Ref_RL_x/y/z`, `Ref_LL_x/y/z` 컬럼을 저장합니다.
- 발 reference의 frame 간 delta, velocity, acceleration 및 z 최솟값을 이용해 trajectory continuity를 검사합니다.
- callback.cpp 방식의 `All_Theta`와 Dynamixel position 후보값을 `motor_command_candidate.csv`로 내보냅니다.
- `motor_command_candidate.csv`는 검토를 위한 offline candidate이며 실제 모터에 전송하는 명령 파일이 아닙니다.
- 마지막에 Trajectory, IK Angle, Joint Velocity, Joint Acceleration, Dynamixel Position 카테고리별 PASS/WARNING 요약을 출력합니다.

### 확인 결과

- Joint angle limit check passed
- Wrapped joint velocity check passed under temporary 10 rad/s limit
- Joint acceleration check applied under temporary 100 rad/s² limit
- Foot reference trajectory continuity check applied
- Offline debug PASS/WARNING summary applied
- `motor_command_candidate.csv` offline candidate export applied
- Dynamixel leg motor ID mapping applied
- Direction mapping applied from `callback.cpp`
- Temporary Dynamixel position candidates are within 0~4095

### 다리 모터 매핑

| Joint | Description | Motor ID |
|---|---|---:|
| RL0 | RHY | 10 |
| RL1 | RHRx | 13 |
| RL2 | RHP | 15 |
| RL3 | RKN | 17 |
| RL4 | RAP | 19 |
| RL5 | RAR | 21 |
| LL0 | LHY | 12 |
| LL1 | LHR | 14 |
| LL2 | LHP | 16 |
| LL3 | LKN | 18 |
| LL4 | LAP | 20 |
| LL5 | LAR | 22 |

### callback.cpp 기준 direction

- RL = `[-1, +1, +1, -1, -1, -1]`
- LL = `[-1, +1, -1, +1, +1, -1]`

### 주의사항

- 현재 결과는 오프라인 검증용입니다.
- 실제 하드웨어 zero offset과 최종 관절 limit은 단일 모터 테스트로 확인해야 합니다.
- 전체 보행 실행 전 motor 17 또는 motor 18 단일 모터 테스트부터 진행해야 합니다.
- 실물 로봇 없이 full walking을 실행하지 마십시오.
