# Update

## 2026.04.10

### main.cpp — CoG pre-shift 및 FOOT 좌표계 재설계

#### FOOT 논리각 좌표계 재정의
- A,C: 양수 논리각, 변환식 `phy = 210 - logical`
- B,D: 음수 논리각, 변환식 `phy = 210 + logical`
- `toServoFoot(leg, logical)`: A,C / B,D 분기 처리로 수정
- `legFdw(leg)`, `legStlF(leg)`, `legFup(leg)` 헬퍼 함수 추가 (A,C=양수, B,D=음수 자동 처리)
- `angle[]` 초기값 수정: B,D foot = 음수 논리각

#### CoG pre-shift 구현
- `cogShift(swingLeg)`: swing 전 지지 발 foot 각도 이동으로 무게중심 조정
- `inward[4][4]` 테이블: `+1`=이동, `0`=skip
  - A swing: B,D 이동 / A,C skip (같은 쪽 왼발)
  - D swing: A,C 이동 / B,D skip (같은 쪽 오른발)
  - B swing: A,C 이동 / B,D skip (같은 쪽 오른발)
  - C swing: B,D 이동 / A,C skip (같은 쪽 왼발)
- 목표값: A,C = `logFdw + cogAlpha`, B,D = `-(logFdw + cogAlpha)` (물리각 대칭)
- `cogShift` 점진적 이동: 1도씩 `spdDelay()` 적용
- `cogRestore(bool skip[4])`: swing 후 지지 발 `legFdw`로 점진적 복귀
  - 다음 cogShift 대상 다리는 skip 배열로 제외
  - skipA/skipD: `{true, false, true, false}` (A,C 제외, B,D 복귀)
  - skipB/skipD: `{false, true, false, true}` (B,D 제외, A,C 복귀)
- `cogAlpha`: 시리얼 명령 `cog40`으로 런타임 튜닝, 범위 0~80°

#### stall() 개선
- `stall(bool smooth=false)` 파라미터 추가
  - `smooth=false` (전원인가): 목표 물리각으로 즉시 이동
  - `smooth=true` (동작중 정지): 현재 논리각에서 1도씩 점진적 이동, `delay(spd)` 고정 사용 (step_mode 무관)
- `stopAction()`에서 `stall(true)` 호출

#### flat_up / flat_dw 개선
- 각 다리 독립적으로 현재 위치 → 목표값까지 점진적 이동 (`while(!done)` 루프)
- `legFdw(leg)`, `legFup(leg)` 기준으로 B,D 음수 논리각 처리

#### fodw() 개선
- `fodw(int leg, int tgt=-999)`: 목표 논리각 파라미터 추가, 기본값 `legFdw(leg)`

#### 시리얼 명령 버그 수정
- `cog` 명령이 `c`로 시작해 개별 서보 명령(`ca`, `cf`)으로 파싱되던 문제 수정
- 개별 서보 명령 조건에 두 번째 글자 `a` 또는 `f` 체크 추가

#### sts 출력 개선
- 마지막 두 열 `Arm(phy)/Foot(phy)` → `Arm(log)/Foot(log)` 로 수정 (angle[] 논리각 표시)
- `cogAlpha` 값 추가 출력

---

## 2026.04.05

### main.cpp — 논리각(Logical Angle) 좌표계 기반 전면 재구현

#### 좌표계 정의 확정
- 논리각: 0°=전방, 90°=수평, 180°=후방, CCW=양수 (전체 코드 공통 기준)
- 물리각: 서보 하드웨어 각도 (0~180°), `toServoArm()` / `toServoFoot()` 통해서만 접근
- ARM 변환식: A(phy=log+45), B(phy=log+135), C(phy=log-45), D(phy=log+225)
- FOOT 변환식: phy = 210 - logical

#### 파라미터 논리각 기준으로 교체
- `phyContact[]`, `phyStl[]`, `armDir[]` 제거
- `logStl[4] = {45, -45, 135, -135}` 추가 (대각선 중립)
- `logContact[4] = {45, -45, 45, -45}` 추가 (수평 전방 15°)
- `logToeoff(leg)` 함수 추가: `logContact ± stride` 자동 계산
- `stride` 변수로 보폭 동적 조정 가능
- `ARM_LIMIT=15`, `FOOT_LIMIT=10` 추가 (기구 안전 여유각)
- `armOffset[4]`, `footOffset[4]` 추가 (다리별 조립 공차 보정)

#### angle[] 논리각으로 변경
- 기존: `angle[8]` = 물리각 기준 현재 서보 각도
- 변경: `angle[8]` = 논리각 기준 현재 각도
- 초기값: arm `{90, -90, 90, -90}`, foot `{90, 90, 90, 90}` (수평 논리각)
- 파워온 시 서보가 어떤 위치에 있든 수평 논리각 기준으로 stall 이동

#### 서보 제어 함수 재설계
- `writeServo(idx, phy)`: 물리각으로 서보 직접 제어, `angle[]` 갱신 없음
- `writeArmLog(leg, logical)`: 논리각 → `angle[]` 저장 + 물리각 변환 후 write
- `writeFootLog(leg, logical)`: 논리각 → `angle[]` 저장 + 물리각 변환 후 write
- 기존 `writeAA/writeAF...` 개별 함수 제거

#### 동작 함수 논리각 기준으로 수정
- `stall()`: `angle[]` 논리각 ↔ 목표 논리각 비교, 1도씩 이동
- `armTo(leg, logTarget)`: `angle[]` 직접 논리각 기준 이동 (물리각 변환 제거)
- `foup(leg)` / `fodw(leg)`: `210-angle[]` 역변환 제거, `angle[]` 직접 사용
- `WALK()`: A→D→B→C 순서, 논리각 기준 contact/toeoff 이동
- `ROUND()`: 논리각 기준 전방→후방 회전

#### 시리얼 명령 추가
- `la75`, `lb-75`: 논리각으로 arm 직접 설정
- `oa5`, `ob-3`: 다리별 arm offset 설정
- `foa5`, `fob-3`: 다리별 foot offset 설정
- `str50`: stride 설정 (5~90°), 변경 시 toeoff 자동 재계산 출력
- `sts` 출력 개선: stride, armOffset, footOffset, 논리각/물리각 동시 표시

---

## 2026.03.08

### 동작 연속 반복 기능 추가 (웹 + 시리얼)
- `loop_running` (volatile bool) 플래그 추가
- `startLoopAction(code)` 추가: 기존 태스크 강제 종료 후 새 태스크를 루프 모드로 시작
- `startOnceAction(code)` 추가: 기존 태스크 강제 종료 후 새 태스크를 1회 실행 모드로 시작
- `stopAction()` 추가: `loop_running` 해제 → FreeRTOS 태스크 강제 종료(`vTaskDelete`) → `stall()` 복귀
- `actionTask` 내부를 `do-while(loop_running)` 구조로 변경하여 반복/1회 통합 처리
- 웹: walk, round, left, right → 연속 반복 / up, down → 1회 실행

### 시리얼 통신 입력 방식 개선
- `Serial.readStringUntil('\n')` (블로킹) → `Serial.read()` 문자 단위 루프로 변경
- `' '`(스페이스) 수신 시 즉시 `stopAction()` 호출
- `\r` (CR) 무시 처리 추가

### stall() 함수 점진적 이동
- 8개 서보를 매 루프마다 1도씩 목표 방향으로 동시 이동, `delay(spd)` 적용

### up/dn 1회 실행 + updn 연속 반복 분리
- `up` / `dn` → `startOnceAction` 으로 변경 (1회 실행)
- `updn` 명령 신규 추가: `flat_up()` → `flat_dw()` 교대 무한 반복

## 2026.03.04
### Walk
- AA,BA,CA,DA만 움직이고 AF~DF는 움직이지 않는다.
- AA~DA는 동시에 반시계 방향으로 45도- 시계방향 90도- AA 반시계 방향 90도 - CA 반시계 방향 90도 - CA 시계 방향 90도 - BA 반시계 방향 90도 - BA 시계 방향 90도 - DA 반시계 방향 90도