# Dynamics Bridge Action Map

## 1. 목적

`/navigation/motion_command`의 추상 action을
STEP Dynamics 또는 SDK가 실행할 실제 동작으로 변환한다.

---

## 2. 정지 계열

다음 action은 우선 모두 정지 명령으로 분류한다.

- `STOP`
- `WAIT`
- `WAIT_GO_CONFIRMATION`
- `WAIT_SCORE_CONFIRMATION`
- `BALL_LOST_STOP`
- `GOAL_LOST_STOP`

예상 Dynamics 동작:

- 현재 보행 종료
- 기본 자세 유지
- 새로운 action 대기

실제 motion ID 또는 함수:

- 미정

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

실제 Dynamics 함수와 속도 차이는 추후 결정한다.

---

## 4. 좌회전 계열

- `TURN_LEFT`
- `ALIGN_LEFT`
- `RECOVER_LEFT`
- `RECOVER_TURN_LEFT`
- `RECOVER_GOAL_TURN_LEFT`

예상 Dynamics 동작:

- 좌회전 보행
- 필요에 따라 일반 회전과 미세 회전으로 구분

실제 motion ID 또는 함수:

- 미정

---

## 5. 우회전 계열

- `TURN_RIGHT`
- `ALIGN_RIGHT`
- `RECOVER_RIGHT`
- `RECOVER_TURN_RIGHT`
- `RECOVER_GOAL_TURN_RIGHT`

예상 Dynamics 동작:

- 우회전 보행
- 필요에 따라 일반 회전과 미세 회전으로 구분

실제 motion ID 또는 함수:

- 미정

---

## 6. 후진 계열

- `RETREAT_GOAL`

예상 Dynamics 동작:

- 후진 보행 또는 뒤로 한 걸음

실제 motion ID 또는 함수:

- 미정

---

## 7. 특수 모션 계열

### 공 집기

- `PICKUP_NOW`

예상 동작:

- 공 집기 모션

실제 motion ID:

- 미정

### 슛

- `SHOT`

예상 동작:

- 공 던지기 또는 슛 모션

실제 motion ID:

- 미정

### 허들 넘기

- `GO`

예상 동작:

- 허들 넘기 모션

주의:

`GO`는 일반 전진이 아니라 허들 planner에서 최종 실행 action으로 사용된다.

실제 motion ID:

- 미정

---

## 8. Dynamics 담당자와 합의할 항목

- 정지 명령의 실제 구현 방식
- 일반 전진 속도
- 느린 접근 속도
- 미세 전진 보폭
- 일반 회전과 정렬용 미세 회전 구분
- 후진 동작 지원 여부
- `PICKUP_NOW` motion ID
- `SHOT` motion ID
- `GO` 허들 모션 ID
- 모션 실행 중 새 명령 처리 방식
- 모션 완료 응답 토픽
- 모션 실패 응답 토픽
