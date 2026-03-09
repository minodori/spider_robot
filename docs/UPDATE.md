# Update

 
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

