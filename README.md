# IRC-STEP

## STEP Dynamics Offline Debug

### 현재 오프라인 디버그 목적

- 실제 로봇에 적용하기 전에 `STEP_Dynamics.cpp`에서 계산한 관절각이 발산하지 않는지 확인합니다.
- CSV와 `plot_debug.py`를 이용해 wrapped angle, velocity, Dynamixel position 후보값을 확인합니다.

### 현재 안전 후보 보행 파라미터

```cpp
Go_Straight_start(0.03, 0.20, 0.02)
```

### 생성/검증 파일

- `Dynamics/walk_forward_debug_slow.csv`
- `Dynamics/plot_debug.py`

### 확인 결과

- Joint angle limit check passed
- Wrapped joint velocity check passed under temporary 10 rad/s limit
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
