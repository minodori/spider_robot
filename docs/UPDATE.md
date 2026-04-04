# Update

## 2026.04.05

### walk_sim2.py — 논리각(Logical Angle) 좌표계 기반 시뮬레이션 재구현

#### 배경
- 기존 walk_sim.py는 물리각(서보각) 기준으로 파라미터를 관리하여 PyBullet과 firmware 간 좌표계 불일치 문제 존재
- 논리각(월드각) 기준으로 통일하여 시뮬레이션과 firmware 일관성 확보

#### 좌표계 정의 확정
- 논리각: 0°=전방, 90°=수평, 180°=후방, CCW=양수
- 물리각: 서보 하드웨어 각도, toServo() 변환 함수로만 접근
- ARM 변환식: A(+45), B(+135), C(-45), D(+225)
- FOOT 변환식: phy = 210 - logical

#### 수평선 기준 정립
- 앞다리(A/B) 수평 = 물리각 135°/45°
- 뒷다리(C/D) 수평 = 물리각 45°/135°
- logContact = 75° (수평 전방 15°), logToeoff = 125° (수평 후방 35°)
- logStl: A/B = 45°, C/D = 135°

#### ARM_LIMIT / FOOT_LIMIT 추가
- ARM_LIMIT = 15°: 기구 안전을 위해 논리각 범위 양 끝단 여유각
- FOOT_LIMIT = 10°: foot 서보 한계 여유각
- 시뮬레이션에는 offset 미적용 (offset은 실제 하드웨어 공차 보정용)

#### 주요 수정 이력

| 항목 | 내용 | 결과 |
|------|------|------|
| 파라미터 교체 | PHY_CONTACT/MOUNT_YAW/ARM_DIR → LOG_CONTACT/LOG_STL/LOG_TOEOFF | 논리각 통일 |
| to_servo_arm() 추가 | 논리각→물리각 변환 함수 | A:+45, B:+135, C:-45, D:+225 |
| to_servo_foot() 추가 | phy = 210 - logical | foot 변환 통일 |
| arm_sim() 수정 | logical - MOUNT_YAW → PyBullet rad | URDF joint 방향 일치 |
| foot_sim() 부호 | +/-math.radians(phy-90) 반복 시도 | URDF axis 방향 확인 중 |
| stall() 개선 | p.createConstraint()로 초기화 중 몸체 고정 | 중력에 의한 쓰러짐 방지 |
| foot joint force | 60 → 200 | 중력 버팀 강화 |
| 키보드 이벤트 | 메인루프 중복 getKeyboardEvents() 제거 | Space키 정상 작동 |
| update_camera() | 수동 뷰 각도 유지 (getDebugVisualizerCamera 활용) | 사용자 뷰 유지 |
| angularDamping | 0.9 → 5.0 | 회전 억제 (임시) |

#### 현재 상태 및 미해결 문제
- 초기 stall 자세: 정상 (몸체 수평, 다리 대각선 방향)
- Space/S키: 정상 작동
- 보행 중 몸체 회전 문제: foot_sim() 부호 및 LOG_FDW 값 튜닝 진행 중
- foot joint 방향: URDF axis xyz="0 1 0" 기준 양수/음수 방향 확인 필요
- CoG pre-shift 미구현: 구현 예정

#### 다음 작업
- foot_sim() 방향 및 LOG_FDW/LOG_FUP 값 확정
- CoG pre-shift 구현
- main.cpp 논리각 기반으로 동일하게 수정

---

## 2026.03.08

### 동작 연속 반복 기능 추가 (웹 + 시리얼)
- **요청**: 동작 버튼을 누르면 해당 동작이 무한 반복되고, 정지 버튼/명령으로 중단
- `loop_running` (volatile bool) 플래그 추가
- `startLoopAction(code)` 추가: 기존 태스크 강제 종료 후 새 태스크를 루프 모드로 시작
- `startOnceAction(code)` 추가: 기존 태스크 강제 종료 후 새 태스크를 1회 실행 모드로 시작
- `stopAction()` 추가: `loop_running` 해제 → FreeRTOS 태스크 강제 종료(`vTaskDelete`) → `stall()` 복귀
- `actionTask` 내부를 `do-while(loop_running)` 구조로 변경하여 반복/1회 통합 처리
- 웹: walk, round, left, right → 연속 반복 / up, down → 1회 실행
- 웹 정지 버튼(■) → `stopAction()` 즉시 호출 (동작 중 여부 무관)

### 시리얼 통신 입력 방식 개선
- **요청**: 명령은 엔터로 실행, 스페이스바는 엔터 없이 즉시 정지
- `Serial.readStringUntil('\n')` (블로킹) → `Serial.read()` 문자 단위 루프로 변경
- `static String serialBuf` 로 문자를 누적, `\n` 수신 시 명령 처리
- `' '`(스페이스) 수신 시 즉시 `stopAction()` 호출 후 버퍼 초기화
- `\r` (CR) 무시 처리 추가
- 시리얼 신규 명령: `stop`(+엔터) → 연속 동작 정지

### stall() 함수 점진적 이동 개선
- **요청**: stall 명령 실행 시 현재 위치에서 초기자세로 spd 속도에 맞춰 부드럽게 이동
- 8개 서보를 매 루프마다 1도씩 목표 방향으로 동시 이동, `delay(spd)` 적용
- 이동 거리가 다른 서보들이 동시에 출발하고 각자 도착 시 정지

### up/dn 1회 실행 + updn 연속 반복 분리
- **요청**: up/dn은 1회만 수행, 대신 up-dn 교대 무한 반복 명령 추가
- `up` / `dn` (웹+시리얼) → `startOnceAction` 으로 변경 (1회 실행)
- `updn` 명령 신규 추가 (웹 버튼 + 시리얼): `flat_up()` → `flat_dw()` 교대 무한 반복
- `actionTask` case 5 추가: `flat_up()` + `flat_dw()` 한 세트

## 2026.03.04
### Walk
- AA,BA,CA,DA만 움직이고 AF~DF는 움직이지 않는다.
- AA~DA는 동시에 반시계 방향으로 45도- 시계방향 90도- AA 반시계 방향 90도 - CA 반시계 방향 90도 - CA 시계 방향 90도 - BA 반시계 방향 90도 - BA 시계 방향 90도 - DA 반시계 방향 90도