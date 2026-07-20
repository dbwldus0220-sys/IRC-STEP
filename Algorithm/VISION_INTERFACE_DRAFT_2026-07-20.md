# STEP Vision–Algorithm Interface Draft

작성일: 2026-07-20  
상태: 영상처리 담당자 확인 전 초안

## 1. 현재 확인된 전체 흐름

```text
카메라·RealSense
    ↓
YOLO 검출
    ↓
/vision/detections
    ↓
공·라인·허들 분석
    ↓
/vision/ball_info
/vision/line_info
/vision/hurdle_info
    ↓
알고리즘 판단
    ↓
Dynamics /motion_command

## 2. 현재 확인된 Vision 토픽

영상 분석 결과
/vision/detections
/vision/ball_info
/vision/line_info
/vision/hurdle_info
/vision/goal_info
/vision/mission_state
/vision/robot_pose
/vision/visual_odom
영상처리 저장소의 판단 결과
/navigation/ball_command
/navigation/line_command
/navigation/hurdle_command
/navigation/goal_command
/navigation/motion_command
미션 입력
/mission/phase

3. Dynamics 인터페이스

MotionCommand
int32 command
int32 angle
MotionEnd
bool finished
int32 command
bool motion_end_detect

4. 현재 확인된 연결 차이

Vision 쪽 출력 형식:

std_msgs/msg/String
JSON

Dynamics 쪽 입력 형식:

robot_msgs/msg/MotionCommand

따라서 추후 Vision JSON을 읽어 MotionCommand로 변환하는 연결 구조가 필요하다.

5. 현재 권장 역할 분담 초안

Vision
- 카메라 및 RealSense 처리
- 객체 검출
- 공·라인·허들의 위치, 각도, 거리, 신뢰도 계산
- /vision/*_info 발행

Algorithm
- 최신 Vision 정보 저장
- 미션 상태 관리
- 행동 우선순위 결정
- 모션 중 다음 행동 미리 판단
- 최종 command, angle 생성

Dynamics
- MotionCommand 수신
- 100Hz 모터 제어
- 모션 실행
- MotionEnd 발행

6. 아직 확인이 필요한 사항

실제 사용하는 라인 분석기 버전
motion_decision_node.py가 최종용인지 테스트용인지
각 navigation planner의 담당 범위
/mission/phase 발행 주체
최종 launch 파일
대회에서 실제 사용할 토픽 목록
JSON 출력 필드가 앞으로 바뀔 가능성

## 8. 알고리즘 입력값 1차 분류

### 공 Ball

#### 필수 입력

- `detected`
- `confidence`
- `state`
- `offset_x_norm`
- `bearing_deg`
- `distance_m`
- `depth_valid`
- `depth_age_sec`
- `pickup_ready`
- `pickup_now`

#### 보조 입력

- `offset_y_norm`
- `horizontal_distance_m`
- `lateral_offset_m`
- `is_centered`
- `is_close`
- `approach_ready`
- `candidate_count`

#### 디버그용

- `center_x`
- `center_y`
- `bbox`
- `width_px`
- `height_px`
- `area_px`
- `image_width`
- `image_height`
- `note`
- `candidates`

#### 사용 보류

- `target_priority_score`
- `vertical_offset_m`
- `elevation_deg`

이유:
초기 알고리즘은 공의 검출 여부, 좌우 오차, 실제 거리, 집기 가능 상태만으로 충분하다. 후보 전체 목록과 세부 영상 좌표는 디버깅에만 사용한다.

---

### 라인 Line

#### 필수 입력

- `detected`
- `filtered_heading_error_deg`
- `filtered_lateral_offset_norm`
- `filter_ready`
- `missed_line_frames`
- `detection_quality`
- `geometry_quality`

#### 보조 입력

- `heading_error_deg`
- `lateral_offset_norm`
- `near_heading_deg`
- `far_heading_deg`
- `turn_angle_deg`
- `turn_consistency`
- `heading_stability_deg`
- `heading_quality`
- `mean_confidence`

#### 디버그용

- `line_count`
- `center_points_px`
- `segment_angles_deg`
- `segment_vertical_spans_px`
- `usable_segment_count`
- `lateral_offset_px`
- `fit_rmse_px`
- `filter_history_size`
- `processing_ms`
- `image_width`
- `image_height`

#### 사용 보류

- `near_fit_heading_deg`
- `heading_segment_count_used`
- `angle_change_mean_deg`
- `angle_change_max_deg`
- `median_heading_error_deg`
- `median_lateral_offset_norm`

이유:
실제 주행 판단은 시간 필터가 적용된 방향 오차와 좌우 오차를 우선 사용한다. 원본값과 세그먼트 정보는 필터 실패 시 대체값 또는 디버깅용으로 둔다.

---

### 허들 Hurdle

#### 필수 입력

- `detected`
- `confidence`
- `state`
- `offset_x_norm`
- `bearing_deg`
- `distance_m`
- `hurdle_angle_deg`
- `depth_valid`
- `depth_age_sec`
- `is_centered`
- `go_now`

#### 보조 입력

- `horizontal_distance_m`
- `lateral_offset_m`
- `left_depth_m`
- `right_depth_m`
- `depth_in_go_range`
- `go_depth_error_m`
- `estimated_width_m`
- `depth_sample_count`

#### 디버그용

- `center_x`
- `center_y`
- `bbox`
- `width_px`
- `height_px`
- `area_px`
- `candidate_count`
- `candidates`
- `image_width`
- `image_height`
- `note`

#### 사용 보류

- `target_priority_score`
- `offset_y_norm`
- `elevation_deg`

이유:
허들 접근은 좌우 정렬, 허들 기울기, 실제 거리, 넘기 가능 여부가 핵심이다. 후보 목록과 화면상 세부 좌표는 디버깅에만 사용한다.