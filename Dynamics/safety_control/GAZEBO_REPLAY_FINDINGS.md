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

#### command_1 final check

- STEP_ROLL_SCALE_TEST 적용 범위를 command_1 전용으로 줄인 뒤 다시 Gazebo replay를 확인했다.
- command_1에는 right=0.5, left=0.5 roll scale이 적용되었다.
- 초반 오른다리 lateral kick은 사라졌다.
- 다리가 아주 살짝 비틀리는 느낌은 남아 있었지만, 움직임이 크게 죽지는 않았다.
- 급격한 꺾임이나 한 바퀴 회전은 보이지 않았다.
- command_1 전용 roll50은 현재 적용 후보로 유지한다.



#### command_32 follow-up conclusion

- command_32에 roll50을 적용한 코드 적용본을 확인했다.
- roll scale은 정상 적용되었고, roll guard는 개입하지 않았다.
- roll 계열 max_safe_step은 충분히 작았다.
- 하지만 초반 오른다리 lateral kick은 여전히 남았다.
- right40_left50 후처리도 육안상 큰 개선이 없었다.
- pitch_step025 후처리도 초반 lateral kick을 제거하지 못했다.
- lateral-slide world에서는 fixed-base보다 약간 줄어드는 것처럼 보였다.
- soft-start에서는 lateral kick이 더 줄었지만, 전체 움직임도 작아졌다.
- 따라서 command_32 초반 lateral kick은 roll scale 단독 문제가 아니라 시작 전환부, 궤적 구조, support-leg 보정, 또는 fixed-base replay 조건의 영향으로 판단한다.
- command_32에는 roll50을 강제 적용하기보다 테스트 옵션/후보로 유지한다.


#### command_2 / command_3 roll50 follow-up

- command_2와 command_3에는 STEP_ROLL_SCALE_TEST가 적용되지 않는 것을 확인했다.
  - roll_scale_right=1
  - roll_scale_left=1
  - roll_scale_applied sum=0
  - raw_range와 safe_range가 동일했다.
- 따라서 command_2/3에서 보이는 움직임 작음, 다리 옆 휨, command_3의 비틀림은 roll scale test 때문에 새로 생긴 문제가 아니다.

- command_2 roll50 후처리 replay를 확인했다.
- 급격한 꺾임은 없었고, 옆으로 휘는 현상은 약간 줄었다.
- 하지만 움직임이 확실히 더 작아졌다.
- 따라서 command_2는 roll50 적용 후보로 보지 않는다.

- command_3 roll50 후처리 replay를 확인했다.
- 다리가 옆으로 비틀리는 현상이 여전히 남아 있었다.
- 따라서 command_3의 비틀림은 roll scale 단독 문제가 아니며, roll50 적용 후보로 보지 않는다.

- command_2/3는 큰 안전 발산 문제는 없지만, 자세 품질 개선이 필요한 명령으로 분류한다.


#### command_3 yaw/roll isolation check

- command_3의 옆 비틀림 원인을 분리하기 위해 후처리 replay를 확인했다.
- RL0 yaw를 첫 프레임 값으로 고정한 no_yaw replay에서는 비틀림이 줄어들지 않았다.
- 따라서 command_3 비틀림은 yaw 단독 문제가 아니다.
- roll 관절(RL1, RL5, LL1, LL5)을 첫 프레임 값으로 고정한 no_roll replay에서는 비틀림이 약간 줄었다.
- 하지만 비틀림이 완전히 사라지지는 않았다.
- 따라서 command_3 비틀림은 roll 계열 영향이 일부 있으나, roll scale 단독으로 해결되는 문제는 아니다.
- command_3는 추후 전체 roll/pitch 궤적 구조와 fixed-base replay 조건을 함께 고려해 개선해야 한다.


### command_32 compensation hold+ramp test

`STEP_COMMAND32_COMPENSATION_TEST` was tested for command_32.

Final tested candidate:

- compensation hold frames: 80
- compensation ramp frames: 135
- frame 0~79: compensation scale = 0.0
- frame 80~214: compensation scale ramps from 0.0 to 1.0 using smoothstep
- frame 215 onward: compensation scale = 1.0

Gazebo replay result:

- The initial right-leg lateral kick disappeared.
- Overall command_32 motion remained acceptable.
- The legs were not perfectly straight throughout the motion; slight bending remained during some sections.
- No sharp initial side kick, sudden rotation, or severe leg snap was observed.

Judgment:

- This is the best command_32 candidate so far.
- The remaining slight leg bending is acceptable for the current Gazebo-level check.
- Further tuning should focus on real-robot validation or detailed posture tuning, not more aggressive suppression of the command.


### command_3 twist follow-up

Command_3 compensation-off replay did not clearly reduce the lateral twist.

Visual observation suggested that the pelvis/hip chain was not being held rigidly enough during replay, rather than a single trajectory discontinuity.

Gazebo controller inspection showed:
- hip and ankle joints use the same PID gains: P=20, I=0, D=1
- effort limit is 8.4 N·m
- passive damping/friction are not explicitly configured
- hip yaw/roll/pitch use the same gains as ankle joints despite larger downstream inertia

Current judgment:
- command_3 twist should not yet be treated as a trajectory bug.
- Gazebo hip controller stiffness / damping / tracking error must be checked first.
- Next checks:
  1. log command vs actual joint positions
  2. test higher hip PID gain model
  3. test small passive damping model


#### command_3 ankle roll stiff test

An ankle-roll-only PID gain test was performed to check whether the remaining command_3 leg shaking was caused by weak ankle roll position control.

Test setup:

- Base model: joint-state replay test model
- Target joints:
  - right_ankle_roll_joint
  - left_ankle_roll_joint
- Original gain:
  - P=20, I=0, D=1
- Test gain:
  - P=60, I=0, D=3
- Replay:
  - command_3
  - hold-start = 2.0 s
  - joint tracking topic: `/step/leg_joint_states`

Baseline tracking result:

- right_ankle_roll_joint
  - max error ≈ 0.145 rad
  - mean error ≈ 0.078 rad
- left_ankle_roll_joint
  - max error ≈ 0.146 rad
  - mean error ≈ 0.081 rad

Ankle-roll-stiff result:

- right_ankle_roll_joint
  - max error ≈ 0.144 rad
  - mean error ≈ 0.065 rad
- left_ankle_roll_joint
  - max error ≈ 0.134 rad
  - mean error ≈ 0.064 rad

Observation:

- The ankle roll mean tracking error decreased slightly.
- The ankle roll max tracking error remained almost unchanged.
- The visible leg shaking / twisting was not clearly resolved.

Judgment:

- Ankle roll PID gain alone is not the main cause of the command_3 shaking.
- The result suggests that damping, contact/friction, foot-ground interaction, or trajectory/contact timing should be checked next.
- The next recommended test is an ankle roll damping test, using small passive damping on the ankle roll joints while keeping the original PID gains.



#### command_3 ankle roll damping test

An ankle-roll damping test was performed after the ankle-roll-only PID gain test did not clearly resolve the visible leg shaking.

Test setup:

- Base model: joint-state replay test model
- Target joints:
  - right_ankle_roll_joint
  - left_ankle_roll_joint
- PID gain:
  - P=20, I=0, D=1
- Added passive dynamics:
  - damping=0.2
  - friction=0.0
- Replay:
  - command_3
  - hold-start = 2.0 s
  - joint tracking topic: `/step/leg_joint_states`

Result:

- right_ankle_roll_joint
  - max error ≈ 0.145 rad
  - mean error ≈ 0.073 rad
- left_ankle_roll_joint
  - max error ≈ 0.134 rad
  - mean error ≈ 0.076 rad

Observation:

- The visible shaking/twisting did not clearly improve.
- Max ankle roll tracking error remained almost unchanged.
- Mean ankle roll error was not clearly improved compared with the ankle-roll-stiff test.

Judgment:

- Ankle roll passive damping alone is not the main cause of the command_3 shaking.
- Together with the ankle-roll-stiff test, this suggests that the remaining command_3 shaking is likely related to foot-ground contact, collision/friction modeling, trajectory/contact timing, or Gazebo base constraint effects rather than simple ankle roll controller weakness.


#### command_2 joint tracking comparison

A command_2 replay tracking test was performed using the same joint-state logging setup as command_3.

Result:

- right_ankle_roll_joint
  - max error ≈ 0.163 rad
  - mean error ≈ 0.072 rad
- left_ankle_roll_joint
  - max error ≈ 0.162 rad
  - mean error ≈ 0.073 rad
- hip yaw / hip roll tracking errors remained small.

Observation:

- command_2 showed a similar ankle roll tracking error pattern to command_3.
- The ankle roll max error was even slightly larger than command_3 in this replay.
- Hip yaw and hip roll tracking errors were still small.

Judgment:

- The remaining shaking is not unique to command_3.
- The issue is more likely related to the step-in-place / turn-type motion family, foot-ground contact, collision/friction modeling, or Gazebo base constraint effects.
- Further tuning of command_3-only compensation is unlikely to be efficient at this stage.