# Gazebo Dry-run Replay Findings

## Replay 데이터 해석 시 주의사항

- `safety_control` CSV는 최종 `All_Theta`, 즉 `safe_*` 명령을 기반으로 한다.
- `callback.cpp`에서 실제 모터 방향에 맞춘 부호가 이미 `All_Theta`에 반영되어 있다.
- 따라서 Gazebo replay 시 `--mirror-left-pitch-chain` 옵션을 사용하지 않는다. 이 옵션을 사용하면 왼쪽 pitch chain에 부호 변환이 중복 적용된다.
- `safe_0`부터 `safe_22`까지의 번호는 Dynamixel motor ID가 아니라 `All_Theta` 배열 인덱스다.

Gazebo leg joint 매핑은 다음과 같다.

| Gazebo joint | Safety CSV column |
|---|---|
| `RL0_wrap` | `safe_0` |
| `RL1_wrap` | `safe_1` |
| `RL2_wrap` | `safe_2` |
| `RL3_wrap` | `safe_3` |
| `RL4_wrap` | `safe_4` |
| `RL5_wrap` | `safe_5` |
| `LL0_wrap` | `safe_6` |
| `LL1_wrap` | `safe_7` |
| `LL2_wrap` | `safe_8` |
| `LL3_wrap` | `safe_9` |
| `LL4_wrap` | `safe_10` |
| `LL5_wrap` | `safe_11` |

## Command별 관찰 결과

### Command 1

- Original replay에서 초반 오른다리의 lateral 튐이 관찰되었다.
- `roll50` 후처리 CSV에서는 초반 튐이 사라졌다.
- 따라서 command 1은 command별 roll scale 적용 후보로 볼 수 있다.

### Command 2

- Original replay부터 전체 motion range가 작다.
- 관찰된 최대 step은 약 `0.0216 rad/frame`이다.
- Roll scale을 적용하면 동작이 더 작게 보이므로, 현재는 원본 scale 유지 후보로 본다.

### Command 3

- Command 2와 유사하게 original replay부터 motion range가 작다.
- 한쪽 yaw가 들어가면서 약간 비틀리는 느낌이 있지만 max step 자체는 작다.
- 현재 관찰 결과로는 roll scale로 해결할 문제는 아닌 것으로 판단한다.

### Command 32

- Original replay에서는 전체적으로 옆으로 휘는 느낌이 있다.
- Lateral-slide world에서는 fixed-base 조건보다 휘는 느낌이 조금 줄어든다.
- `roll50` 또는 `right40_left50` 후처리 결과가 상대적으로 가장 덜 휘어 보였다.
- 초반 오른다리 튐은 일부 남아 있다.
- 특정 단일 관절의 step 조정만으로는 초반 튐이 해결되지 않았다.

## 결론 및 적용 원칙

- 모든 motion에 전역 `roll50`을 적용하는 방식은 부적절하다.
- Command별 roll scale 또는 테스트 전용 선택 옵션이 필요하다.
- 실제 로봇 제어에 바로 반영하지 않는다.
- 실제 로봇 적용 전에는 `STEP_ROLL_SCALE_TEST`와 같은 compile-time 또는 runtime 테스트 옵션으로만 검증한다.
- Scale 변경은 먼저 dry-run CSV와 Gazebo에서 확인하고, 관절별 step, 전체 자세, 지면 접촉 및 균형 영향을 함께 검토한다.

