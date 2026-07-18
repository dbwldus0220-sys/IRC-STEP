# Single Motor Test Usage

This document describes how to use `Dynamics/single_motor_test.cpp` for real-robot motor verification.

This program must be used before any walking command is sent to the real robot.

## Safety rules

- Do not run walking commands during this test.
- Test only one motor at a time.
- Start with read-only mode.
- Use very small movement only.
- Keep `MOVE_TICK` within the configured safety limit.
- Stop immediately if the wrong joint moves, direction is unexpected, or the joint approaches a mechanical limit.

## Expected STEP leg motor IDs

Right leg:

- 10: RL0 / RHY
- 13: RL1 / RHRx
- 15: RL2 / RHP
- 17: RL3 / RKN
- 19: RL4 / RAP
- 21: RL5 / RAR

Left leg:

- 12: LL0 / LHY
- 14: LL1 / LHR
- 16: LL2 / LHP
- 18: LL3 / LKN
- 20: LL4 / LAP
- 22: LL5 / LAR

## 1. ID scan mode

Use this first after connecting the Dynamixel bus.

Expected behavior:

- Opens the port.
- Checks expected leg motor IDs.
- Prints which IDs respond.
- Does not enable torque.
- Does not write goal position.
- Does not move any motor.

Example build:

```bash
cd ~/IRC/IRC-STEP/Dynamics

g++ -std=c++17 single_motor_test.cpp -o single_motor_id_scan \
  -DID_SCAN=1 \
  -ldxl_x64_cpp


 ## 실제 로봇 연결 전 시리얼 포트 확인

ID 스캔 모드나 단일 모터 테스트를 실행하기 전에 Dynamixel USB 장치가 연결되어 있는지 확인해야 한다.

사용 가능한 USB 시리얼 장치 확인:

```bash
ls /dev/ttyUSB*
ls /dev/ttyACM*

/dev/ttyUSB0가 없으면 프로그램에서 다음과 같은 메시지가 뜰 수 있다.

[ERROR] Failed to open port: /dev/ttyUSB0

이 메시지는 로봇 또는 USB2Dynamixel 어댑터가 연결되어 있지 않을 때 정상적으로 발생할 수 있다.
이 경우 모터 토크가 켜지지 않고, 목표 위치도 전송되지 않으며, 모터는 움직이지 않아야 한다.

포트 권한 확인

포트가 존재하는데도 열리지 않는 경우 권한 문제일 수 있다.

ls -l /dev/ttyUSB0
groups

현재 사용자가 dialout 그룹에 포함되어 있지 않다면 다음 명령으로 추가할 수 있다.

sudo usermod -aG dialout $USER

그룹을 추가한 뒤에는 로그아웃 후 다시 로그인해야 적용된다.

실제 로봇 첫 연결 순서

실제 로봇을 연결할 때는 다음 순서를 따른다.

로봇을 안전하게 고정하거나 지지한다.
로봇 전원을 안전하게 켠다.
Dynamixel USB 어댑터를 연결한다.
ls /dev/ttyUSB*로 포트가 존재하는지 확인한다.
가장 먼저 ID 스캔 모드를 실행한다.
그다음 읽기 전용 모드로 모터 하나의 현재 위치를 확인한다.
읽기 전용 모드로 모든 예상 다리 모터 ID를 확인한다.
읽기 전용 확인이 모두 끝난 뒤에만 무릎 모터 하나를 아주 작게 움직이는 테스트를 한다.
이 과정 중에는 보행 명령을 절대 보내지 않는다.
하드웨어가 연결되지 않았을 때의 정상 동작

Dynamixel 어댑터가 연결되어 있지 않은 상태에서 ID 스캔 모드를 실행하면 포트 열기에 실패할 수 있다.

이때 정상적으로 안전 종료되면 다음 조건을 만족해야 한다.

포트 열기 실패 메시지가 출력된다.
토크가 켜지지 않는다.
목표 위치가 전송되지 않는다.
모터 이동 명령이 전송되지 않는다.
프로그램이 안전하게 종료된다.

하드웨어가 연결되지 않은 상태에서 확인된 결과:

[MODE] ID_SCAN
[ERROR] Failed to open port: /dev/ttyUSB0
[SUMMARY] torque_was_enabled=no
[SUMMARY] return_to_start=not_required
[SUMMARY] torque_disable=not_required

이 결과는 포트가 없을 때도 프로그램이 위험한 명령을 보내지 않았다는 의미다.

