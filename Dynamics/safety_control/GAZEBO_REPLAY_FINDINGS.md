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




#### command_1 fixed-base joint tracking

Command_1 was replayed in the fixed-base joint-state Gazebo world.

Visual observation:

- The initial right-leg lateral kick was not observed.
- No large rotation or snap occurred through the replay.
- Both legs still bent sideways, with the right leg showing stronger lateral bending.
- The ankles appeared to under-rotate rather than over-rotate, so the soles did not stay level with the ground.

Tracking result:

- right_ankle_roll_joint:
  - max error ≈ 0.192 rad
  - mean error ≈ 0.085 rad
- left_ankle_roll_joint:
  - max error ≈ 0.166 rad
  - mean error ≈ 0.087 rad
- right_ankle_pitch_joint:
  - max error ≈ 0.173 rad
  - mean error ≈ 0.072 rad
- left_ankle_pitch_joint:
  - max error ≈ 0.017 rad
  - mean error ≈ 0.007 rad

Observation:

- The largest errors were concentrated in ankle roll, especially the right ankle roll.
- For right_ankle_roll_joint, command_position reached about 0.18~0.20 rad while actual_position stayed near 0 rad.
- This suggests that the right ankle roll joint may not be following the command under Gazebo replay, or that foot-ground contact is preventing the commanded ankle roll motion.

Next check:

- Run a single-joint Gazebo tracking test for right_ankle_roll_joint and left_ankle_roll_joint.
- If the joint fails to track even without walking contact, inspect the Gazebo controller/SDF/joint axis/effort limit.
- If the joint tracks well alone, focus on foot-ground contact, sole collision, and command_1 landing/support timing.


## Ankle axis 원인 분리 결과

Command 1에서 발바닥이 지면과 수평을 이루지 못하고 ankle tracking error가 크게 나타나는 원인을 분리하기 위해 fixed-base 모델의 foot collision, controller gain/effort, ankle joint axis를 각각 변경해 비교했다.

### Original SDF 기준 결과

- 기준 모델은 `step_fixed_base_ctrl_joint_state.sdf`이다.
- Command 1 replay에서 발바닥 수평이 잘 맞지 않았고 ankle tracking error가 크게 나타났다.
- Single-joint test에서도 ankle pitch/roll의 actual range가 command range에 비해 매우 작았다.
- 반면 같은 script와 controller 구조를 사용하는 `right_hip_roll_joint`는 정상적으로 추종했다.
- 이후 비교에서도 Command 1 replay의 기준은 original SDF로 유지한다.

### Foot collision 제거 테스트

- `step_fixed_base_ctrl_joint_state_no_foot_collision.sdf`에서 양쪽 foot link의 collision만 제거했다.
- Foot visual, controller plugin, joint-state publisher는 유지했다.
- Foot collision을 제거해도 ankle single-joint tracking은 개선되지 않았다.
- 따라서 foot collision만으로 ankle motion이 제한되는 문제는 아니다.

### Ankle gain/effort 강화 테스트

- `step_fixed_base_ctrl_joint_state_ankle_strong.sdf`에서 네 ankle joint만 다음과 같이 강화했다.
  - effort limit: `8.4 -> 100`
  - P gain: `20 -> 100`
  - I gain: `0` 유지
  - D gain: `1 -> 5`
- Gain과 effort를 크게 올려도 ankle tracking은 개선되지 않았다.
- 따라서 controller gain 또는 effort 부족만으로 현재 현상을 설명할 수 없다.

### Axis-only 테스트

네 ankle joint의 axis를 원본 `0 0 1`에서 동일한 후보 축으로 바꾸는 single-joint test를 수행했다.

| Test model | Ankle axis | 주요 결과 |
|---|---|---|
| Original / axis Z | `0 0 1` | Ankle pitch/roll actual range가 매우 작음 |
| Axis Y | `0 1 0` | Ankle roll actual range가 약 `0.10~0.12 rad`로 증가 |
| Axis X | `1 0 0` | Ankle roll actual range가 약 `0.265 rad`까지 증가 |

Axis X single-joint tracking 결과:

- `right_ankle_roll_joint`
  - command range: `0.4 rad`
  - actual range: 약 `0.266 rad`
  - max error: 약 `0.115 rad`
  - mean error: 약 `0.062 rad`
  - correlation: 약 `0.897`
- `left_ankle_roll_joint`
  - command range: `0.4 rad`
  - actual range: 약 `0.265 rad`
  - max error: 약 `0.154 rad`
  - mean error: 약 `0.062 rad`
  - correlation: 약 `0.898`
- `right_ankle_pitch_joint`
  - command range: `0.4 rad`
  - actual range: 약 `0.117 rad`
  - max error: 약 `0.170 rad`
  - mean error: 약 `0.114 rad`
  - correlation: 약 `0.517`
- 비교 기준인 `right_hip_roll_joint`는 actual range 약 `0.380 rad`, correlation 약 `0.996`으로 정상 추종했다.

Axis X는 ankle roll의 single-joint tracking 수치를 가장 크게 개선했지만 Command 1 replay의 자세 품질은 오히려 나빠졌다.

- 발이 사선 방향으로 비틀렸다.
- 왼발도 바닥과 수평을 이루지 못했다.
- 전체 보행 자세가 original SDF보다 나빠졌다.

### Roll X / Pitch Y 분리 테스트

- 양쪽 ankle roll axis는 `1 0 0`으로 설정했다.
- 양쪽 ankle pitch axis는 `0 1 0`으로 설정했다.
- Ankle roll의 single-joint tracking 수치는 현재 테스트 중 가장 좋아졌다.
- 그러나 시작 자세부터 양발이 사선으로 비틀렸다.
- 왼발은 일부 구간에서 수평에 가까워졌지만 오른발은 계속 수평을 이루지 못했다.
- Tracking 수치 개선이 올바른 foot orientation이나 보행 자세를 의미하지 않는다는 점을 확인했다.

### Ankle-roll command scale 2.0 테스트

원본 `step_fixed_base_ctrl_joint_state.sdf`와 원본 fixed-base world에서 Command 1을 replay하고, `replay_roll_joints_from_csv.py`의 `--ankle-roll-command-scale 2.0` 옵션으로 양쪽 ankle-roll command만 2배 적용했다.

Foot orientation summary:

| Link | Axis | Mean absolute orientation | Maximum absolute orientation |
|---|---|---:|---:|
| `right_foot_link` | roll | 약 `0.339 rad` | 약 `1.133 rad` |
| `left_foot_link` | roll | 약 `0.251 rad` | 약 `0.911 rad` |
| `right_foot_link` | pitch | 약 `0.150 rad` | - |
| `left_foot_link` | pitch | 약 `0.120 rad` | - |

- 양발 yaw mean absolute orientation은 약 `0.116~0.117 rad`로 증가했다.

Joint tracking result:

- `left_ankle_roll_joint`
  - max error: 약 `0.224 rad`
  - mean error: 약 `0.146 rad`
- `right_ankle_roll_joint`
  - max error: 약 `0.194 rad`
  - mean error: 약 `0.136 rad`

Observation and judgment:

- Ankle-roll command를 2배로 키웠을 때 foot roll 평균은 아주 조금 감소했다.
- 그러나 육안상 발바닥 수평 문제는 크게 개선되지 않았다.
- Foot pitch와 yaw orientation은 오히려 악화되었다.
- Ankle-roll tracking error도 보행 개선을 뒷받침할 정도로 줄지 않았다.
- 따라서 scale `2.0`은 Command 1 보행 개선 후보에서 제외한다.
- `--ankle-roll-command-scale`은 원인 분리용 diagnostic option으로만 유지하며 기본값은 `1.0`을 유지한다.

### 종합 판단

- 단순히 ankle axis만 X 또는 Y로 교체하는 방식은 최종 해결책으로 사용하면 안 된다.
- Axis-only 변경은 single-joint tracking actual range를 증가시키지만 initial pose와 foot orientation을 망가뜨린다.
- 따라서 현재 문제는 joint axis 하나의 오류라기보다 다음 요소 사이의 불일치 가능성이 높다.
  - ankle joint pose와 joint frame
  - joint axis가 표현되는 frame
  - foot link의 초기 orientation
  - Gazebo zero-position offset
  - replay command의 부호와 joint positive direction
- `ankle_axis_x`, `ankle_axis_y`, `ankle_axis_z`, `roll_x_pitch_y`, `roll_x_pitch_z` 모델과 world는 원인 분리용 diagnostic files로만 유지한다.
- Axis-only modified SDF는 Command 1 보행 replay나 실제 로봇 제어 모델로 사용하지 않는다.
- Ankle-roll command scaling도 현재 보행 개선안으로 채택하지 않는다. 특히 scale `2.0`은 rejected test로 기록한다.

### 다음 진단 단계

1. Original `step_fixed_base_ctrl_joint_state.sdf`와 original Command 1 replay를 기준선으로 유지한다.
2. Command 1 첫 프레임에서 다음 orientation을 original 모델과 diagnostic 모델 사이에서 비교한다.
   - `right_ankle_link`
   - `left_ankle_link`
   - `right_foot_link`
   - `left_foot_link`
3. 각 ankle joint에 대해 joint pose 회전과 axis를 같은 reference frame으로 변환하여 실제 회전축을 비교한다.
4. CSV의 첫 command 값, Gazebo joint zero position, visual/collision의 초기 orientation 사이에 offset이 있는지 확인한다.
5. 작은 양수 명령을 한 관절씩 입력하고 link가 회전하는 실제 방향을 기록하여 command sign과 joint positive direction을 확인한다.
6. 좌우 ankle의 pose, axis, offset, command sign이 올바른 대칭 관계인지 비교한다.

다음 수정은 axis만 교체하기 전에 foot link orientation, joint frame, Gazebo offset, command sign을 하나의 좌표계에서 함께 검증한 뒤 결정한다.


#### ankle roll command scale follow-up

The ankle roll command scale test was extended after confirming that large single-joint ankle roll commands can move the Gazebo ankle roll joints.

Results:

- scale 2.0:
  - Visual improvement was minimal.
  - Foot roll mean_abs decreased only slightly.
  - Foot pitch and yaw increased noticeably.
  - Both feet showed worse yaw orientation.

- scale 3.0:
  - Visual result became worse.
  - The whole leg posture appeared diagonally twisted.
  - right_foot_link:
    - roll mean_abs: ~0.303
    - pitch mean_abs: ~0.191
    - yaw mean_abs: ~0.220
  - left_foot_link:
    - roll mean_abs: ~0.232
    - pitch mean_abs: ~0.148
    - yaw mean_abs: ~0.227
  - ankle roll tracking error remained large:
    - left_ankle_roll max error: ~0.222
    - right_ankle_roll max error: ~0.193

Judgment:

- Increasing ankle roll command scale is not a valid fix.
- It may slightly reduce roll mean_abs, but it significantly worsens pitch/yaw orientation and twists the leg posture.
- scale 4.0 was not tested because scale 3.0 already showed worse visual behavior.
- The ankle roll issue should not be solved by command scaling alone.



#### Command 1 hip roll scale diagnostic

The largest visual improvement for command 1 came from reducing hip roll command variation.

Tested variants:

- hip roll scale 0.50
- hip roll scale 0.25
- hip roll scale 0.10
- hip roll scale 0.00

Observations:

- 0.50 reduced the diagonal twisting compared with the original command.
- 0.25 improved the forward leg extension further.
- 0.10 looked straighter than 0.25.
- 0.00 looked similar to 0.10 and produced the cleanest forward extension in the visual check.

Numerical trend:

- Foot roll mean_abs did not significantly improve.
- Foot pitch mean_abs decreased strongly as hip roll scale was reduced.
- This means the visible diagonal twisting was mainly caused by hip roll / lateral posture, not by ankle roll alone.

Representative hiproll0 result:

- right_foot_link pitch mean_abs: ~0.0146
- left_foot_link pitch mean_abs: ~0.0052
- right_foot_link target frame 100 pitch: ~-0.0739
- right_foot_link target frame 102 pitch: ~-0.0785
- left_foot_link target frame 177 pitch: ~0.0156
- left_foot_link target frame 304 pitch: ~0.0195

Current judgment:

- Best visual candidate: hip roll scale 0.00.
- Safer fallback candidate: hip roll scale 0.10.
- The final implementation should keep this as a command-1-specific test option first, instead of applying it globally.



#### Command 1 code-level hip roll scale verification

The command-1 hip roll scale option was implemented in safety_control and verified through dry-run logging and Gazebo replay.

Dry-run verification:

- `command1_hip_roll_scale = 0`
- `command1_hip_roll_scale_applied = 1`
- `safe_1` range: 0.0
- `safe_7` range: 0.0

This confirms that the right and left hip roll commands are held at their command-1 reference values when the option is enabled.

Gazebo replay result using the code-generated dry-run CSV:

- The legs extended forward much more cleanly.
- The diagonal twisting was almost removed visually.
- The result was clearly better than the original command 1.
- It did not look weaker than the previous 10% hip-roll candidate.

Representative foot orientation result:

- right_foot_link pitch mean_abs: ~0.0141
- left_foot_link pitch mean_abs: ~0.0046
- right_foot_link frame 100 pitch: ~-0.0702
- right_foot_link frame 102 pitch: ~-0.0708
- left_foot_link frame 177 pitch: ~0.0117
- left_foot_link frame 304 pitch: ~0.0145

Judgment:

- The safety_control implementation matches the previous manual CSV hiproll0 diagnostic result.
- Current best Gazebo candidate for command 1: command-1 hip roll scale 0.0.
- Keep this as a command-1-specific test option for now, not a global walking change.


#### Command 32 hip roll scale diagnostic

After verifying that the command-1 hip roll scale option does not affect command 32, command 32 was replayed as a regression check.

Observation with command 32 baseline after the existing hold/ramp compensation:

- The initial side kick did not return.
- However, both legs still showed diagonal twisting during forward extension.

A manual Gazebo replay CSV was then generated with the command 32 hip roll variation removed:

- `RL1_wrap` and `LL1_wrap` were held at their first-frame reference values.
- This corresponds to hip roll scale 0.0 for command 32.

Visual result:

- The initial side kick still did not appear.
- The diagonal leg twisting was clearly reduced.
- The legs extended more cleanly forward.
- The motion did not look weak or awkward.

Judgment:

- Command 32 appears to have the same hip-roll-overvariation issue as command 1.
- The existing command32 hold/ramp compensation should remain.
- A separate command-32-specific hip roll scale test option should be added later.
- This should not reuse the command-1 option globally; command 1 and command 32 should have independent scale controls.





#### Command 32 code-level hip roll scale verification

A command-32-specific hip roll scale option was implemented and verified through dry-run logging and Gazebo replay.

Dry-run verification:

- go unique: [32]
- command1_hip_roll_scale: [1]
- command1_hip_roll_scale_applied: [0]
- command32_hip_roll_scale: [0]
- command32_hip_roll_scale_applied: [1]
- safe_1 range: 0.0
- safe_7 range: 0.0

This confirms that the command-32 hip roll scale option is independent from the command-1 option and only applies during command 32.

Gazebo replay result using the code-generated dry-run CSV:

- The initial side kick did not appear.
- The diagonal leg twisting was removed visually.
- The legs did not look weak or awkward.
- The existing command32 hold/ramp compensation remained effective.

Representative foot orientation result:

- right_foot_link pitch mean_abs: ~0.0077
- left_foot_link pitch mean_abs: ~0.0058
- right_foot_link pitch max_abs: ~0.0457
- left_foot_link pitch max_abs: ~0.0335

Joint tracking result:

- right_ankle_pitch_joint max_abs_error: ~0.1768
- left_ankle_roll_joint max_abs_error: ~0.1414
- right_ankle_roll_joint max_abs_error: ~0.1331
- hip roll tracking errors remained very small.

Judgment:

- Command 32 had the same hip-roll-overvariation issue as command 1.
- A command-32-specific hip roll scale of 0.0 is currently the best Gazebo candidate.
- This should remain command-specific and should not be applied globally to all motions.
