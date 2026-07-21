# Dynamics Bridge Action Map

## 1. 목적

`/navigation/motion_command`의 추상 action을 STEP Dynamics 또는 SDK가 실행할 실제 동작으로 변환한다.

---

## 2. 정지 계열

다음 action은 우선 모두 정지 또는 대기 명령으로 분류한다.

- `STOP`
- `WAIT`
- `WAIT_GO_CONFIRMATION`
- `WAIT_SCORE_CONFIRMATION`
- `BALL_LOST_STOP`
- `GOAL_LOST_STOP`

예상 Dynamics 동작:

- 새로운 보행 모션을 시작하지 않음
- 현재 실행 중인 모션이 있다면 완료까지 대기
- 기본 자세 유지
- 새로운 action 대기

주의:

현재 Dynamics에는 일반적인 의미의 즉시 정지 명령이 명확히 분리되어 있지 않다.

- `command=97`은 실제 정지가 아니라 `/motion_end`만 발행한다.
- `command=98`은 WALK_READY 자세 전환 및 recovery 관련 동작을 포함한다.

따라서 일반 `STOP` 또는 `WAIT`에 `97`, `98`을 직접 연결하지 않는다.

---

## 3. 전진 계열

- `STRAIGHT`
- `APPROACH`
- `SLOW_APPROACH`
- `FINE_FORWARD_STEP`
- `APPROACH_GOAL`
- `APPROACH_HURDLE`

예상 구분:

- `STRAIGHT`: 일반 직진
- `APPROACH`: 공 접근
- `SLOW_APPROACH`: 느린 공 접근
- `FINE_FORWARD_STEP`: 미세 전진
- `APPROACH_GOAL`: 골대 접근
- `APPROACH_HURDLE`: 허들 접근

현재 Dynamics의 이산 모션 command를 사용하여 각각 다른 보폭으로 매핑한다.

---

## 4. 좌측 이동 및 좌회전 계열

- `TURN_LEFT`
- `ALIGN_LEFT`
- `RECOVER_LEFT`
- `RECOVER_TURN_LEFT`
- `RECOVER_GOAL_TURN_LEFT`

구분:

- `TURN_LEFT`: 각도 기반 일반 좌회전
- `ALIGN_LEFT`: 고정된 짧은 좌회전
- `RECOVER_LEFT`: 회전이 아닌 왼쪽 측면 이동
- `RECOVER_TURN_LEFT`: 라인 복구용 좌회전
- `RECOVER_GOAL_TURN_LEFT`: 골대 복구용 좌회전

---

## 5. 우측 이동 및 우회전 계열

- `TURN_RIGHT`
- `ALIGN_RIGHT`
- `RECOVER_RIGHT`
- `RECOVER_TURN_RIGHT`
- `RECOVER_GOAL_TURN_RIGHT`

구분:

- `TURN_RIGHT`: 각도 기반 일반 우회전
- `ALIGN_RIGHT`: 고정된 짧은 우회전
- `RECOVER_RIGHT`: 회전이 아닌 오른쪽 측면 이동
- `RECOVER_TURN_RIGHT`: 라인 복구용 우회전
- `RECOVER_GOAL_TURN_RIGHT`: 골대 복구용 우회전

---

## 6. 후진 계열

- `RETREAT_GOAL`

예상 Dynamics 동작:

- 골대가 너무 가까울 때 짧게 후진
- 큰 후진보다 반걸음 후진을 우선 사용

---

## 7. 특수 모션 계열

### 7.1 공 집기

추상 action:

- `PICKUP_NOW`

초기 Dynamics command:

- `9`

실제 동작:

```text
command 9
→ Picking_Motion
→ /motion_end
```

재집기 관련 command:

| Command | 의미 |
|---:|---|
| `10` | Recatch |
| `31` | Re_Catch2 |
| `11` | Catch_Finish |
| `22` | 집기 실패 후 후진 시퀀스 |

현재 `10 → 31 → 11` 자동 재집기 시퀀스는 `main.cpp`에서 주석 처리되어 있다.

따라서 초기 bridge에서는 `PICKUP_NOW`를 `command=9`에 연결한다.

### 7.2 슛

추상 action:

- `SHOT`

초기 Dynamics command:

- `17`

실제 내부 시퀀스:

```text
command 17: shoot_ready
→ command 18: shoot
→ command 19: shoot_finish
→ 후속 회전
→ /motion_end
```

`command=17`을 받으면 Dynamics 내부에서 `shoot_mode=true`가 설정되어 전체 슛 시퀀스가 자동 진행된다.

따라서 bridge는 `SHOT`에 대해 최초 한 번만 `command=17`을 발행한다.

### 7.3 허들 넘기

추상 action:

- `GO`

초기 Dynamics command:

- `14`

실제 내부 시퀀스:

```text
command 14: forward_short
→ command 14: forward_short
→ command 20: hurdle
→ /motion_end
```

`command=14`를 받으면 Dynamics 내부에서 `hurdle_mode=true`가 설정된다.

`command=20`만 직접 보내면 허들 동작 하나만 실행되고, 접근 동작을 포함한 전체 시퀀스는 시작되지 않는다.

따라서 초기 bridge에서는 `GO`를 `command=14`에 연결한다.

주의:

`GO`는 일반 전진이 아니라 hurdle planner의 최종 특수 실행 action이다.

---

## 8. Dynamics 담당자와 합의할 항목

- 일반 정지 명령의 실제 구현 방식
- 실행 중인 모션의 즉시 중단 방식
- 일반 전진 보폭
- 느린 접근 보폭
- 미세 전진 보폭
- 일반 회전과 정렬용 미세 회전 구분
- 후진 동작 지원 방식
- `PICKUP_NOW` 성공 판정
- `SHOT` 성공 판정
- `GO` 허들 통과 판정
- 모션 실행 중 새 명령 처리 방식
- 모션 완료 응답 형식
- 모션 실패 응답 형식
- timeout 판정 주체
- mission phase 전환 주체

---

## 9. `/navigation/motion_command` 입력 규격

메시지 타입:

`std_msgs/msg/String`

`data` 내부 형식:

JSON

기본 필드:

- `phase`
- `source`
- `action`
- `valid`
- `reason`
- `sdk_motion_requested`
- `requires_ack`
- `source_command`

`motion_decision_node` 추가 필드:

- `command_id`
- `event_id`
- `request_latched`
- `sdk_motion_id`
- `input_age_sec`
- `ball_tracking`
- `goal_tracking`
- `source_node`

주의:

`decision.to_dict()`에도 `sdk_motion_requested`가 포함되어 있지만, 최종 publish 직전 `payload.update()`에서 다시 설정된다.

따라서 최종 JSON에서는 `payload.update()`에서 설정된 값이 사용된다.

### 9.1 Dynamics bridge 필수 사용 필드

- `action`
- `valid`
- `command_id`
- `event_id`
- `sdk_motion_requested`
- `request_latched`
- `sdk_motion_id`
- `source_command`

### 9.2 보조 필드

- `phase`
- `source`
- `reason`
- `requires_ack`

### 9.3 초기 bridge 처리 원칙

1. `valid == false`이면 새로운 모션을 실행하지 않는다.
2. 동일한 `command_id`는 중복 실행하지 않는다.
3. `request_latched == true`이면 완료 응답 전 같은 특수 모션을 재실행하지 않는다.
4. `event_id`가 있는 특수 모션은 완료 또는 실패 응답과 연결한다.
5. `sdk_motion_id`는 현재 `null`이므로 bridge에서 Dynamics command로 매핑한다.
6. 일반 action도 Dynamics가 이전 모션을 완료하기 전에는 새로 발행하지 않는다.

---

## 10. 특수 모션 ACK 및 중복 실행 방지 규칙

### 10.1 ACK가 필요한 terminal action

다음 세 action만 완료 또는 실패 응답이 필요한 특수 모션이다.

- `PICKUP_NOW`
- `SHOT`
- `GO`

이 action이 선택되면:

- `requires_ack = true`
- `request_latched = true`
- `event_id`가 부여됨

### 10.2 최초 실행 요청

같은 terminal action이 새로 등장한 첫 주기에는:

- `sdk_motion_requested = true`
- `event_id`가 1 증가함

Dynamics bridge는 이때에만 실제 특수 모션을 시작한다.

### 10.3 반복 publish 처리

같은 terminal action이 이후 주기에도 계속 유지되면:

- `sdk_motion_requested = false`
- `request_latched = true`
- `event_id`는 동일하게 유지됨

따라서 Dynamics bridge는 동일한 `event_id`를 다시 실행하면 안 된다.

### 10.4 일반 action

다음과 같은 일반 보행 및 정렬 action은 기본적으로 ACK가 필요하지 않다.

- `STRAIGHT`
- `TURN_LEFT`
- `TURN_RIGHT`
- `ALIGN_LEFT`
- `ALIGN_RIGHT`
- `APPROACH`
- `SLOW_APPROACH`
- `FINE_FORWARD_STEP`
- `APPROACH_GOAL`
- `APPROACH_HURDLE`
- `RETREAT_GOAL`
- `RECOVER_LEFT`
- `RECOVER_RIGHT`
- `STOP`
- `WAIT`

기본 값:

- `requires_ack = false`
- `request_latched = false`
- `event_id = null`

다만 현재 Dynamics는 일반 보행 command도 완료될 때까지 다른 명령을 무시한다.

따라서 bridge 내부에서는 일반 action도 `/motion_end` 수신 전까지 실행 중 상태로 관리해야 한다.

### 10.5 Dynamics bridge 처리 원칙

1. `valid == false`이면 실행하지 않는다.
2. 일반 action은 최신 `command_id`를 기준으로 처리한다.
3. 특수 모션은 `sdk_motion_requested == true`일 때만 시작한다.
4. 이미 처리한 `event_id`는 다시 실행하지 않는다.
5. 특수 모션 완료 또는 실패 결과는 동일한 `event_id`와 연결한다.
6. 특수 모션 실행 중에는 일반 보행 명령을 실행하지 않는다.
7. 일반 모션 실행 중 들어온 새 명령은 최신 값만 보관하고 완료 후 다시 판단한다.

---

## 11. 특수 모션 상태 응답 인터페이스 초안

### 11.1 제안 토픽

Dynamics bridge 또는 SDK 실행 노드가 다음 토픽을 발행한다.

`/motion/status`

메시지 타입:

`std_msgs/msg/String`

`data` 내부 형식:

JSON

### 11.2 제안 JSON 구조

```json
{
  "command_id": 123,
  "event_id": 4,
  "action": "PICKUP_NOW",
  "status": "SUCCEEDED",
  "motion_id": 9,
  "fail_count": 0,
  "reason": "motion_completed",
  "source_node": "dynamics_bridge"
}
```

### 11.3 필수 필드

- `command_id`
- `event_id`
- `action`
- `status`

### 11.4 보조 필드

- `motion_id`
- `fail_count`
- `reason`
- `source_node`

### 11.5 status 후보

- `ACCEPTED`
- `RUNNING`
- `SUCCEEDED`
- `FAILED`
- `TIMEOUT`
- `CANCELED`

### 11.6 status 의미

#### `ACCEPTED`

Dynamics bridge가 요청을 정상적으로 수신한 상태다.

#### `RUNNING`

실제 모션이 현재 실행 중인 상태다.

#### `SUCCEEDED`

모션이 정상적으로 완료된 상태다.

#### `FAILED`

모션 실행에 실패했거나 안전 조건에 의해 실패한 상태다.

#### `TIMEOUT`

정해진 시간 안에 모션 완료 신호가 오지 않은 상태다.

#### `CANCELED`

긴급 정지 또는 상위 취소로 모션이 중단된 상태다.

### 11.7 event ID 연결 규칙

`/navigation/motion_command`의 `event_id`와 `/motion/status`의 `event_id`는 반드시 동일해야 한다.

예:

```text
요청:
event_id = 4
action = PICKUP_NOW

응답:
event_id = 4
action = PICKUP_NOW
status = SUCCEEDED
```

다른 `event_id`의 응답은 현재 요청 완료로 처리하면 안 된다.

---

## 12. 기존 Motion Decision 명세와의 대조

`src/mission_control/docs/MOTION_DECISION_SPEC.md` 확인 결과, 특수 모션 응답 구조의 필요성은 기존 공식 문서에도 명시되어 있다.

### 12.1 기존 문서에서 확정된 내용

SDK 실행기는 다음 식별자 중 하나를 응답에 포함해야 한다.

- `command_id`
- `event_id`

SDK 실행기 상태 후보:

- `RUNNING`
- `SUCCEEDED`
- `FAILED`
- `TIMEOUT`

실패 관련 보조 정보:

- 실패 횟수
- 선택적 실패 원인

`mission_control`은 완료 또는 실패 응답을 받은 뒤에만 다음 처리를 수행해야 한다.

- terminal lock 해제
- 다음 mission phase 전환
- 재시도 또는 실패 처리

### 12.2 아직 확정되지 않은 내용

기존 문서에는 다음 항목이 구체적으로 정해져 있지 않다.

- 응답 토픽 이름
- ROS 메시지 타입
- JSON 전체 필드명
- `command_id`와 `event_id` 중 어떤 값을 필수로 사용할지
- `ACCEPTED` 상태 사용 여부
- `CANCELED` 상태 사용 여부
- 상태를 한 번만 발행할지 반복 발행할지
- timeout 판정 주체
- lock phase 전환 주체
- 성공 후 다음 phase 결정 규칙
- 실패 후 재시도 횟수

### 12.3 현재 제안안

응답 토픽 초안:

`/motion/status`

메시지 타입 초안:

`std_msgs/msg/String`

필수 JSON 필드 초안:

```json
{
  "command_id": 123,
  "event_id": 4,
  "action": "PICKUP_NOW",
  "status": "SUCCEEDED",
  "fail_count": 0,
  "reason": "motion_completed",
  "source_node": "dynamics_bridge"
}
```

최소 필수 상태:

- `RUNNING`
- `SUCCEEDED`
- `FAILED`
- `TIMEOUT`

이 구조는 Dynamics 담당자와 합의 전까지 초안으로 유지한다.

---

## 13. Dynamics 실제 ROS 인터페이스

### 13.1 입력 토픽

토픽:

`/motion_command`

메시지 타입:

`robot_msgs/msg/MotionCommand`

필드:

```text
int32 command
int32 angle
```

### 13.2 완료 토픽

토픽:

`/motion_end`

메시지 타입:

`robot_msgs/msg/MotionEnd`

필드:

```text
bool finished
int32 command
bool motion_end_detect
```

현재 Dynamics는 완료 발행 시 다음 값만 설정한다.

```cpp
end_msg.motion_end_detect = true;
```

따라서 현재 `/motion_end`만으로는 다음을 구분할 수 없다.

- 완료된 command 번호
- 성공 또는 실패 여부
- 대응하는 mission `event_id`
- timeout 여부
- 취소 여부

현재 예상 발행 값:

```text
finished = false
command = 0
motion_end_detect = true
```

### 13.3 기존 Dynamics command 목록

| Command | 의미 |
|---:|---|
| `0` | start pose |
| `1` | 기본 직진 |
| `2` | 좌회전 |
| `3` | 우회전 |
| `4` | 큰 후진 |
| `5` | 반걸음 후진 |
| `6` | 반걸음 전진 |
| `7` | 왼쪽 측면 이동 |
| `8` | 오른쪽 측면 이동 |
| `9` | PickMotion |
| `10` | Recatch |
| `11` | Catch_Finish |
| `12` | 1 step 전진 |
| `13` | 허들 접근용 전진 |
| `14` | 짧은 전진 |
| `15` | 짧은 좌회전 |
| `16` | 짧은 우회전 |
| `17` | shoot_ready |
| `18` | shoot |
| `19` | shoot_finish |
| `20` | hurdle |
| `21` | 빠른 직진 |
| `22` | 집기 실패 후 후진 |
| `23` | 목 올리기 |
| `24` | 목 내리기 |
| `25` | forward_40 |
| `26` | forward_15 |
| `27` | forward_2 |
| `28` | 허들 시퀀스용 후진 |
| `29` | 좌측 사선 전진 |
| `30` | 우측 사선 전진 |
| `31` | Re_Catch2 |
| `32` | 빠른 40 step |
| `33` | 제자리걸음 |
| `77` | recovery |
| `97` | 실제 모션 없이 `/motion_end` 발행 |
| `98` | stop 및 WALK_READY 전환 |
| `90` | 하드웨어 configure |
| `91` | preload 및 torque enable |
| `92` | CENTER 자세 |
| `93` | WALK_READY 자세 |

---

## 14. 일반 action → Dynamics command 1차 변환표

이 표는 현재 Dynamics에 존재하는 이산 모션을 기준으로 작성한 초기 bridge 매핑이다.

실물 테스트와 Gazebo 검증 전까지는 확정 파라미터가 아닌 1차 후보로 관리한다.

### 14.1 직진 및 접근

| 추상 action | Dynamics command | Angle | 의미 |
|---|---:|---:|---|
| `STRAIGHT` | `1` | `0` | 기본 직진 |
| `APPROACH` | `12` | `0` | 1 step 접근 |
| `SLOW_APPROACH` | `6` | `0` | 반걸음 전진 |
| `FINE_FORWARD_STEP` | `27` | `0` | 미세 전진 |
| `APPROACH_GOAL` | `6` | `0` | 골대 방향 반걸음 접근 |
| `APPROACH_HURDLE` | `13` | `0` | 허들 접근용 전진 |

### 14.2 회전 및 정렬

| 추상 action | Dynamics command | Angle 입력 |
|---|---:|---|
| `TURN_LEFT` | `2` | `abs(target_heading_change_deg)` |
| `TURN_RIGHT` | `3` | `abs(target_heading_change_deg)` |
| `ALIGN_LEFT` | `15` | `0` |
| `ALIGN_RIGHT` | `16` | `0` |
| `RECOVER_TURN_LEFT` | `2` | `abs(target_heading_change_deg)` |
| `RECOVER_TURN_RIGHT` | `3` | `abs(target_heading_change_deg)` |
| `RECOVER_GOAL_TURN_LEFT` | `2` | `abs(target_heading_change_deg)` |
| `RECOVER_GOAL_TURN_RIGHT` | `3` | `abs(target_heading_change_deg)` |

Dynamics의 `MotionCommand.angle`은 `int32`이므로 bridge에서는 다음과 같이 변환한다.

```text
angle = round(abs(target_heading_change_deg))
```

초기 안전 제한 후보:

```text
1도 <= angle <= 55도
```

수치가 없거나 유효하지 않으면 고정 짧은 회전 명령을 사용한다.

- 왼쪽: `command=15`
- 오른쪽: `command=16`

Goal planner의 `ALIGN_LEFT`, `ALIGN_RIGHT`는 `bearing_error_deg`와 `offset_x_norm`을 제공한다.

Hurdle planner의 `ALIGN_LEFT`, `ALIGN_RIGHT`는 `hurdle_angle_deg`를 제공한다.

초기 bridge에서는 이 값을 직접 `angle`에 사용하지 않고 고정 짧은 회전 command를 사용한다.

### 14.3 측면 라인 복귀

| 추상 action | Dynamics command | Angle |
|---|---:|---:|
| `RECOVER_LEFT` | `7` | `0` |
| `RECOVER_RIGHT` | `8` | `0` |

`RECOVER_LEFT`, `RECOVER_RIGHT`는 회전 명령이 아니라 라인 중심으로 돌아가기 위한 측면 이동 명령이다.

### 14.4 후진

| 추상 action | Dynamics command | Angle |
|---|---:|---:|
| `RETREAT_GOAL` | `5` | `0` |

골대가 너무 가까울 때 큰 후진 `command=4`보다 반걸음 후진 `command=5`를 우선 사용한다.

### 14.5 정지 및 대기 action

다음 action은 새로운 보행 모션을 시작하지 않는다.

- `STOP`
- `WAIT`
- `BALL_LOST_STOP`
- `GOAL_LOST_STOP`
- `WAIT_GO_CONFIRMATION`
- `WAIT_SCORE_CONFIRMATION`

초기 bridge 처리:

```text
Dynamics command를 새로 발행하지 않음
```

주의:

`valid == false`이면서 `STOP` 또는 `WAIT`가 발행되는 경우에도 `command=97`이나 `command=98`을 자동으로 보내지 않는다.

### 14.6 특수 모션

| 추상 action | Dynamics command | Angle |
|---|---:|---:|
| `PICKUP_NOW` | `9` | `0` |
| `SHOT` | `17` | `0` |
| `GO` | `14` | `0` |

특수 모션은 `sdk_motion_requested=true`인 최초 event에서만 발행한다.

---

## 15. Dynamics 내부 특수 시퀀스 주의사항

### 15.1 PICKUP 시퀀스

현재 자동 재집기 시퀀스는 주석 처리되어 있다.

주석 처리된 원래 흐름:

```text
10
→ 31
→ 11
→ 후진
→ 회전
```

초기 bridge에서는 `PICKUP_NOW → command=9`만 사용한다.

재집기와 성공 판정은 추후 FSM 항목으로 분리한다.

### 15.2 SHOT 시퀀스

현재 활성화된 흐름:

```text
17
→ 18
→ 19
→ 좌회전 또는 우회전
→ /motion_end
```

`SHOT`을 실행할 때 bridge는 중간 command인 `18`, `19`를 별도로 발행하지 않는다.

### 15.3 HURDLE 시퀀스

현재 활성화된 흐름:

```text
14
→ 14
→ 20
→ /motion_end
```

`APPROACH_HURDLE → command=13`은 `hurdle_mode=true`를 설정한다.

그러나 기존의 `13 → 14` 자동 전환 코드는 현재 주석 처리되어 있다.

따라서 `command=13` 사용 후 의도하지 않은 hurdle mode 상태가 남는지 확인이 필요하다.

### 15.4 도달 불가능한 허들 분기

현재 허들 후속 코드에는 다음과 같은 구조가 있다.

```cpp
if (current_go_ == 20)
{
    if (current_go_ == 20)
    {
        current_go_ = 3;
    }
    else if (current_go_ == 3)
    {
        current_go_ = 25;
    }
}
```

바깥 조건이 이미 `current_go_ == 20`이므로 내부의 `current_go_ == 3` 분기는 실행될 수 없다.

이 문제는 bridge 구현과 분리하여 Dynamics 시퀀스 수정 항목으로 관리한다.

---

## 16. 일반 모션 실행 상태 관리 규칙

현재 Dynamics는 모션 실행 중 대부분의 새 명령을 무시한다.

따라서 planner의 10Hz 출력을 그대로 `/motion_command`로 전달하면 안 된다.

### 16.1 기본 상태

Bridge 내부 상태 초안:

```text
IDLE
RUNNING_GENERAL
RUNNING_SPECIAL
```

### 16.2 IDLE 상태

새로운 유효 action을 받으면:

1. 추상 action을 Dynamics command로 변환한다.
2. `/motion_command`에 한 번만 발행한다.
3. 일반 action이면 `RUNNING_GENERAL`로 전환한다.
4. 특수 action이면 `RUNNING_SPECIAL`로 전환한다.

### 16.3 RUNNING_GENERAL 상태

일반 Dynamics 모션이 실행 중인 상태다.

처리 규칙:

- 새로운 `/navigation/motion_command`를 즉시 Dynamics에 전달하지 않는다.
- 최신 유효 action 하나만 pending 값으로 저장한다.
- `/motion_end`를 받으면 `IDLE`로 전환한다.
- 이후 최신 pending action을 다시 검토한다.

### 16.4 RUNNING_SPECIAL 상태

`PICKUP_NOW`, `SHOT`, `GO`가 실행 중인 상태다.

처리 규칙:

- 동일 `event_id`를 다시 실행하지 않는다.
- 일반 action을 실행하지 않는다.
- 새 특수 event도 현재 모션 완료 전에는 실행하지 않는다.
- `/motion_end`를 받으면 해당 `event_id`와 연결한다.
- 성공 또는 실패 상태를 `/motion/status`로 변환한다.
- 완료 처리 후 `IDLE`로 전환한다.

### 16.5 pending command 처리

Bridge는 실행 중 들어온 모든 command를 큐에 쌓지 않는다.

최신 action 하나만 유지한다.

이유:

- 비전 결과는 빠르게 변함
- 오래된 action을 순서대로 실행하면 현재 상황과 맞지 않음
- 모션 종료 후 가장 최근 판단만 실행하는 것이 안전함

### 16.6 중복 방지

일반 action:

- 동일 `command_id` 재실행 금지
- Dynamics 실행 중에는 발행 금지

특수 action:

- 동일 `event_id` 재실행 금지
- `sdk_motion_requested=false`이면 발행 금지
- `request_latched=true` 상태에서는 완료 전 재실행 금지

---

## 17. 현재 남은 문제

1. `/motion_end`가 완료된 command 번호를 포함하지 않는다.
2. `/motion_end`가 성공과 실패를 구분하지 않는다.
3. `/motion_end`가 `event_id`를 포함하지 않는다.
4. 일반 `STOP`의 실제 Dynamics command가 없다.
5. `APPROACH_HURDLE → command=13`의 내부 상태 영향을 확인해야 한다.
6. PICKUP 자동 재집기 시퀀스가 비활성화되어 있다.
7. 허들 후속 분기에 도달 불가능한 조건이 있다.
8. 실물 command gate가 활성화되면 대부분의 command가 차단될 수 있다.
9. startup safe가 활성화되면 `90 → 91 → 92 → 93` 순서를 완료해야 한다.
10. 성공, 실패, timeout에 따른 mission phase 전환 규칙이 미정이다.

---

## 18. 초기 bridge 구현 우선순위

1. `/navigation/motion_command` JSON subscriber 구현
2. JSON 필수 필드 검증
3. 추상 action → Dynamics command 변환
4. `/motion_command` publisher 구현
5. 일반 모션 실행 상태 관리
6. 특수 모션 `event_id` 중복 방지
7. `/motion_end` subscriber 구현
8. pending 최신 action 처리
9. `/motion/status` 초안 publisher 구현
10. Dynamics 완료 메시지 확장 여부 결정