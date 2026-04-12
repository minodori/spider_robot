/* -----------------------------------------------------------------------------
  Spider Robot - ESP32 Web Control
  4 Legs x 2 Servos = 8 Servos
  다리: A(앞왼), B(앞오), C(뒤왼), D(뒤오)
  서보: aa/af, ba/bf, ca/cf, da/df (arm/foot)

  좌표계:
    논리각(Logical): 0=전방, 90=수평, 180=후방, CCW=양수
    물리각(Physical): 서보 하드웨어 각도 (0~180°)
    변환: toServoArm(leg, logical), toServoFoot(logical)
   -----------------------------------------------------------------------------*/

#include <Arduino.h>
#include <WiFi.h>
#include <WebServer.h>
#include <ESP32Servo.h>

/* WiFi AP 설정 --------------------------------------------------------------*/
#ifndef ROBOT_ID
  #define ROBOT_ID 1
#endif
#define _STR(x) #x
#define _XSTR(x) _STR(x)
const char* ap_ssid = "SpiderRobot" _XSTR(ROBOT_ID);
const char* ap_password = "12345678";
WebServer server(80);

/* 서보 모터 -----------------------------------------------------------------*/
Servo servo_aa, servo_af;  // Leg A (앞왼)
Servo servo_ba, servo_bf;  // Leg B (앞오)
Servo servo_ca, servo_cf;  // Leg C (뒤왼)
Servo servo_da, servo_df;  // Leg D (뒤오)

const int pin_aa = 26, pin_af = 12;  // Leg A
const int pin_ba = 32, pin_bf = 33;  // Leg B
const int pin_ca = 17, pin_cf = 2;   // Leg C
const int pin_da = 18, pin_df = 5;   // Leg D

Servo* servos[8] = {&servo_aa, &servo_af, &servo_ba, &servo_bf,
                    &servo_ca, &servo_cf, &servo_da, &servo_df};
const int pins[8] = {pin_aa, pin_af, pin_ba, pin_bf, pin_ca, pin_cf, pin_da, pin_df};
const char* names[8] = {"aa", "af", "ba", "bf", "ca", "cf", "da", "df"};

/* 논리각 파라미터 -----------------------------------------------------------*/
// 논리각: 0=전방, 90=수평, 180=후방, CCW=양수
// 범위: A:-45~135  B:-135~45  C:45~225  D:-225~-45
//                          A    B    C    D
const int logStl[4]     = { 45, -45,  135, -135}; // 중립(대각선) 논리각
const int logContact[4] = { 55, -55,   75,  -75}; // contact 논리각 (수평 전방 15°)
int stride              = 35;                      // stride (°), toeoff-contact
// logToeoff: logContact 기준 stride만큼 후방으로 이동
// A/C(양수): contact+stride  B/D(음수): contact-stride
inline int logToeoff(int leg) {
  return logContact[leg] > 0 ? logContact[leg] + stride
                             : logContact[leg] - stride;
}

// ARM 안전 여유각 (양 끝단에서 clamp)
const int ARM_LIMIT = 15;

// ARM 조립 공차 보정 offset (다리별 개별 튜닝, 논리각 기준)
int armOffset[4] = {0, 0, 0, 8};

/* FOOT 논리각 파라미터 ------------------------------------------------------*/
// A,C: phy = 210 - logical  (논리각 양수)
// B,D: phy = 210 + logical  (논리각 음수)
const int logFdw  =  160;  // 서있는 자세 A,C (phy=40)
const int logStlF =  135;  // 대기 자세   A,C (phy=75)
const int logFup  =  130;  // 몸 낮게     A,C (phy=90)
// B,D는 음수 논리각 사용
// -logFdw=-170 → phy=210+(-170)=40, -logStlF=-135 → phy=75, -logFup=-120 → phy=90
const int FOOT_LIMIT = 5;  // 양 끝단 여유각
int cogAlpha         = 40; // CoG pre-shift 이동량 (논리각, cog 명령으로 튜닝)

// 다리별 foot 논리각 기준값 (A,C 양수 / B,D 음수)
inline int legFdw (int leg) { return (leg==1||leg==3) ? -logFdw  : logFdw;  }
inline int legStlF(int leg) { return (leg==1||leg==3) ? -logStlF : logStlF; }
inline int legFup (int leg) { return (leg==1||leg==3) ? -logFup  : logFup;  }

// FOOT 조립 공차 보정 offset (다리별 개별 튜닝, 논리각 기준)
int footOffset[4] = {0, 0, 0, 0};

int spd    = 3;   // 속도 (delay ms, 1~10)
int ledPin = 23;

volatile bool step_mode = false;  // 스텝 모드 (스페이스=한스텝)
volatile bool step_go   = false;  // 스텝 신호 (loop→actionTask)

/* 현재 각도 (논리각) ---------------------------------------------------------*/
// angle[] = 논리각 기준 현재 각도 (arm: 수평=90/-90/90/-90, foot: 수평=90)
// foot 초기값: A,C=+90(수평), B,D=-90(수평)
int angle[8] = {90, 90, -90, -90, 90, 90, -90, -90}; // aa,af,ba,bf,ca,cf,da,df

/* 비동기 동작 ---------------------------------------------------------------*/
volatile bool action_running = false;
volatile bool loop_running   = false;
TaskHandle_t ActionTask = NULL;

/* 논리각 → 물리각 변환 함수 -------------------------------------------------*/
int clampInt(int val, int lo, int hi) {
  if (val < lo) return lo;
  if (val > hi) return hi;
  return val;
}

// ARM 논리각 → 물리각
// A: phy = log + 45   B: phy = log + 135
// C: phy = log - 45   D: phy = log + 225
int toServoArm(int leg, int logical) {
  // 1. ARM_LIMIT clamp
  int ranges[4][2] = {
    {-45 + ARM_LIMIT, 135 - ARM_LIMIT},   // A
    {-135 + ARM_LIMIT,  45 - ARM_LIMIT},  // B
    { 45 + ARM_LIMIT, 225 - ARM_LIMIT},   // C
    {-225 + ARM_LIMIT, -45 - ARM_LIMIT},  // D
  };
  logical = clampInt(logical, ranges[leg][0], ranges[leg][1]);
  // 2. offset 적용
  logical += armOffset[leg];
  // 3. 물리각 변환
  switch(leg) {
    case 0: return logical + 45;   // A
    case 1: return logical + 135;  // B
    case 2: return logical - 45;   // C
    case 3: return logical + 225;  // D
  }
  return 90;
}

// FOOT 논리각 → 물리각
// A,C: phy = 210 - logical  B,D: phy = 210 + logical
int toServoFoot(int leg, int logical) {
  bool bd = (leg == 1 || leg == 3);
  if (!bd) logical = clampInt(logical,  30 + FOOT_LIMIT,  210 - FOOT_LIMIT);
  else     logical = clampInt(logical, -210 + FOOT_LIMIT, -30 - FOOT_LIMIT);
  logical += footOffset[leg];
  return bd ? 210 + logical : 210 - logical;
}

/* delay 헬퍼 ----------------------------------------------------------------*/
// delay()는 vTaskDelay 기반 → WDT 피드 O
// delayMicroseconds()는 busy-wait → 1ms 이상 사용 시 WDT 리부트 위험
inline void spdDelay() {
  if (step_mode) {
    step_go = false;
    while (!step_go && action_running) vTaskDelay(1);  // 스페이스 대기
  } else {
    delay(spd);
  }
}

/* 함수 선언 -----------------------------------------------------------------*/
void writeServo(int idx, int ang);
void writeAA(int ang); void writeAF(int ang);
void writeBA(int ang); void writeBF(int ang);
void writeCA(int ang); void writeCF(int ang);
void writeDA(int ang); void writeDF(int ang);
void writeArmLog(int leg, int logical);
void writeFootLog(int leg, int logical);

void stall(bool smooth = false);
void WALK(void);
void ROUND(void);
void flat_up(void);
void flat_dw(void);
void BLINK(void);
void foup(int leg); void fodw(int leg, int tgt = -999);
void armTo(int leg, int logTarget);
void cogShift(int swingLeg);
void cogRestore(bool skip[4]);
void handleAction(void);
void stopAction(void);
void startOnceAction(int code);
void startLoopAction(int code);
void actionTask(void* param);

/* 서보 제어 -----------------------------------------------------------------*/
// writeServo: 물리각으로 서보 직접 제어 (angle[] 갱신 없음)
void writeServo(int idx, int phy) {
  switch(idx) {
    case 0: servo_aa.write(phy); break; case 1: servo_af.write(phy); break;
    case 2: servo_ba.write(phy); break; case 3: servo_bf.write(phy); break;
    case 4: servo_ca.write(phy); break; case 5: servo_cf.write(phy); break;
    case 6: servo_da.write(phy); break; case 7: servo_df.write(phy); break;
  }
}

// 논리각으로 arm 서보 제어 + angle[] 논리각 저장
void writeArmLog(int leg, int logical) {
  angle[leg * 2] = logical;
  writeServo(leg * 2, toServoArm(leg, logical));
}

// 논리각으로 foot 서보 제어 + angle[] 논리각 저장
void writeFootLog(int leg, int logical) {
  angle[leg * 2 + 1] = logical;
  writeServo(leg * 2 + 1, toServoFoot(leg, logical));
}

/* 웹 페이지 -----------------------------------------------------------------*/
const char index_html[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html>
<head>
  <meta charset="UTF-8">
  <meta name="viewport" content="width=device-width, initial-scale=1.0, user-scalable=no">
  <title>%TITLE%</title>
  <style>
    *{box-sizing:border-box;margin:0;padding:0}
    body{font-family:Arial;background:linear-gradient(135deg,#1a1a2e,#16213e);min-height:100vh;display:flex;flex-direction:column;align-items:center;padding:20px;color:#fff}
    h1{margin-bottom:20px;font-size:24px;text-shadow:0 0 10px #00ff88}
    .status{background:#0f3460;padding:10px 20px;border-radius:10px;margin-bottom:20px}
    .pad{display:grid;grid-template-columns:repeat(3,70px);gap:10px;margin-bottom:20px}
    .btn{width:70px;height:70px;border:none;border-radius:15px;font-size:24px;cursor:pointer}
    .btn:active{transform:scale(0.95)}
    .mv{background:linear-gradient(145deg,#00ff88,#00cc6a);color:#1a1a2e}
    .st{background:linear-gradient(145deg,#ff4757,#ff3344);color:#fff}
    .act{background:linear-gradient(145deg,#5f27cd,#341f97);color:#fff;width:100%;height:50px;font-size:16px}
    .empty{background:transparent}
    .grid{display:grid;grid-template-columns:repeat(2,1fr);gap:10px;width:230px;margin-bottom:20px}
    .spd{width:230px;background:#0f3460;padding:15px;border-radius:15px}
    .spd label{display:block;margin-bottom:10px}
    .spd input{width:100%;height:20px}
    #st{color:#00ff88}
  </style>
</head>
<body>
  <h1>🕷️ %TITLE%</h1>
  <div class="status">상태: <span id="st">대기</span></div>
  <div class="pad">
    <div class="btn empty"></div>
    <button class="btn mv" onclick="cmd('walk')">▲</button>
    <div class="btn empty"></div>
    <button class="btn mv" onclick="cmd('left')">◀</button>
    <button class="btn st" onclick="cmd('stop')">■</button>
    <button class="btn mv" onclick="cmd('right')">▶</button>
    <div class="btn empty"></div>
    <button class="btn mv" onclick="cmd('walk')">▼</button>
    <div class="btn empty"></div>
  </div>
  <div class="grid">
    <button class="btn act" onclick="cmd('up')">⬆️ Up</button>
    <button class="btn act" onclick="cmd('down')">⬇️ Down</button>
    <button class="btn act" onclick="cmd('updn')">↕️ UpDn</button>
    <button class="btn act" onclick="cmd('round')">🔄 Round</button>
    <button class="btn act" onclick="cmd('init')">🔧 Init</button>
  </div>
  <div class="spd">
    <label>속도: <span id="sv">3</span></label>
    <input type="range" min="1" max="10" value="3" onchange="document.getElementById('sv').innerText=this.value;cmd('speed&val='+this.value)">
  </div>
  <script>
    function cmd(c){fetch('/action?cmd='+c).then(r=>r.text()).then(d=>document.getElementById('st').innerText=d).catch(()=>document.getElementById('st').innerText='오류')}
  </script>
</body>
</html>
)rawliteral";

/* 웹 액션 처리 --------------------------------------------------------------*/
void handleAction() {
  String cmd = server.arg("cmd");
  String res = "OK";
  if (cmd == "stop" || cmd == "init") { stopAction(); res = "정지"; }
  else if (cmd == "speed") { spd = 11 - server.arg("val").toInt(); res = "속도설정"; }
  else {
    int code = 0;
    if      (cmd == "walk")  { res = "전진(연속)"; code = 1; }
    else if (cmd == "left" || cmd == "right" || cmd == "round") { res = "회전(연속)"; code = 2; }
    else if (cmd == "up")    { startOnceAction(3); res = "일어서기"; }
    else if (cmd == "down")  { startOnceAction(4); res = "앉기"; }
    else if (cmd == "updn")  { res = "Up-Dn(연속)"; code = 5; }
    if (code) startLoopAction(code);
  }
  server.send(200, "text/plain", res);
}

/* 비동기 액션 ---------------------------------------------------------------*/
void actionTask(void* param) {
  int act = *((int*)param);
  delete (int*)param;
  action_running = true;
  digitalWrite(ledPin, HIGH);
  do {
    switch(act) {
      case 1: WALK(); break;
      case 2: ROUND(); break;
      case 3: flat_up(); break;
      case 4: flat_dw(); break;
      case 5: flat_up(); flat_dw(); break;
    }
  } while (loop_running);
  digitalWrite(ledPin, LOW);
  action_running = false;
  ActionTask = NULL;
  vTaskDelete(NULL);
}

void startOnceAction(int code) {
  if (ActionTask) { loop_running = false; vTaskDelete(ActionTask); ActionTask = NULL; action_running = false; }
  loop_running = false;
  int* p = new int(code);
  xTaskCreatePinnedToCore(actionTask, "Act", 4096, p, 1, &ActionTask, 0);
}

void startLoopAction(int code) {
  if (ActionTask) { loop_running = false; vTaskDelete(ActionTask); ActionTask = NULL; action_running = false; }
  loop_running = true;
  int* p = new int(code);
  xTaskCreatePinnedToCore(actionTask, "Act", 4096, p, 1, &ActionTask, 0);
}

void stopAction() {
  loop_running = false;
  if (ActionTask) { vTaskDelete(ActionTask); ActionTask = NULL; }
  action_running = false;
  stall(true);  // 동작중 정지: 점진적 이동
}

/* 동작 함수 -----------------------------------------------------------------*/
void BLINK() {
  for (int i = 0; i < 3; i++) {
    digitalWrite(ledPin, HIGH); delay(100);
    digitalWrite(ledPin, LOW);  delay(100);
  }
}

// stall: 초기자세로 이동
// smooth=false (전원인가): 목표 물리각으로 즉시 이동
// smooth=true  (동작중):   현재 논리각에서 1도씩 점진적 이동
void stall(bool smooth) {
  BLINK();
  if (!smooth) {
    // 전원인가: 목표 물리각으로 즉시 write
    for (int i = 0; i < 4; i++) {
      writeServo(i*2,   toServoArm(i,  logStl[i]));
      writeServo(i*2+1, toServoFoot(i, legStlF(i)));
    }
  } else {
    // 동작중: 현재 논리각 → 목표 논리각으로 1도씩 점진적 이동
    int targets[8];
    for (int i = 0; i < 4; i++) {
      targets[i*2]   = logStl[i];
      targets[i*2+1] = legStlF(i);
    }
    bool done = false;
    while (!done) {
      done = true;
      for (int i = 0; i < 4; i++) {
        if (angle[i*2] < targets[i*2])        { writeArmLog(i,  angle[i*2]   + 1); done = false; }
        else if (angle[i*2] > targets[i*2])   { writeArmLog(i,  angle[i*2]   - 1); done = false; }
        if (angle[i*2+1] < targets[i*2+1])    { writeFootLog(i, angle[i*2+1] + 1); done = false; }
        else if (angle[i*2+1] > targets[i*2+1]){ writeFootLog(i, angle[i*2+1] - 1); done = false; }
      }
      delay(spd);  // step_mode 무관하게 항상 delay
    }
  }
  // angle[] 논리각 최종 업데이트
  for (int i = 0; i < 4; i++) {
    angle[i*2]   = logStl[i];
    angle[i*2+1] = legStlF(i);
  }
}

void flat_dw() {
  // 현재 위치 → legFup(leg) 까지 점진적으로 (각 다리 독립)
  bool done = false;
  while (!done) {
    done = true;
    for (int leg = 0; leg < 4; leg++) {
      int cur = angle[leg*2+1];
      int tgt = legFup(leg);
      if (cur != tgt) {
        writeFootLog(leg, cur + (cur < tgt ? 1 : -1));
        done = false;
      }
    }
    spdDelay();
  }
  delay(500);
}

void flat_up() {
  // 현재 위치 → legFdw(leg) 까지 점진적으로 (각 다리 독립)
  bool done = false;
  while (!done) {
    done = true;
    for (int leg = 0; leg < 4; leg++) {
      int cur = angle[leg*2+1];
      int tgt = legFdw(leg);
      if (cur != tgt) {
        writeFootLog(leg, cur + (cur < tgt ? 1 : -1));
        done = false;
      }
    }
    spdDelay();
  }
  delay(500);
}

/* 발 up/down (논리각 기준) --------------------------------------------------*/
void foup(int leg) {
  int cur = angle[leg*2+1];
  int tgt = legFup(leg);
  if (cur < tgt) { for (int l = cur; l <= tgt; l++) { writeFootLog(leg, l); spdDelay(); } }
  else           { for (int l = cur; l >= tgt; l--) { writeFootLog(leg, l); spdDelay(); } }
}

void fodw(int leg, int tgt) {
  int cur = angle[leg*2+1];
  if (tgt == -999) tgt = legFdw(leg);  // 기본값: legFdw
  if (cur < tgt) { for (int l = cur; l <= tgt; l++) { writeFootLog(leg, l); spdDelay(); } }
  else           { for (int l = cur; l >= tgt; l--) { writeFootLog(leg, l); spdDelay(); } }
}

/* 팔 이동 (논리각 기준) -----------------------------------------------------*/
// armTo: 현재 논리각 → 목표 논리각 (1도씩 이동)
void armTo(int leg, int logTarget) {
  int cur = angle[leg*2];  // 이미 논리각
  if (cur < logTarget) {
    for (int l = cur; l <= logTarget; l++) { writeArmLog(leg, l); spdDelay(); }
  } else {
    for (int l = cur; l >= logTarget; l--) { writeArmLog(leg, l); spdDelay(); }
  }
}

// CoG pre-shift: swing 다리 반대쪽으로 무게중심 이동
// inward 테이블: +1=안쪽, -1=바깥쪽, 0=skip
// 물리각 기준으로 대칭 계산:
//   안쪽 목표 phy = 210 - (logFdw + cogAlpha)
//   바깥쪽 목표 phy = 210 - (logFdw - cogAlpha)
//   A,C: logical = 210 - phy
//   B,D: logical = phy - 210
void cogShift(int swingLeg) {
  const int inward[4][4] = {
    { 0, +1, 0, +1},  // A swing: B,D 안쪽, C 바깥쪽
    {+1,  0, +1, 0},  // B swing: A,C 안쪽, D 바깥쪽
    {0, +1,  0, +1},  // C swing: A 바깥쪽, B,D 안쪽
    {+1, 0, +1,  0},  // D swing: A,C 안쪽, B 바깥쪽
  };
  int tgtAC =  logFdw + cogAlpha;  // A,C 목표 논리각
  int tgtBD = -(logFdw + cogAlpha); // B,D 목표 논리각 (대칭)

  int targets[4];
  for (int leg = 0; leg < 4; leg++) {
    if (inward[swingLeg][leg] == 0) {
      targets[leg] = angle[leg*2+1];  // skip
    } else {
      bool bd = (leg == 1 || leg == 3);
      targets[leg] = bd ? tgtBD : tgtAC;
    }
  }
  bool done = false;
  while (!done) {
    done = true;
    for (int leg = 0; leg < 4; leg++) {
      int cur = angle[leg*2+1];
      if (cur < targets[leg])      { writeFootLog(leg, cur+1); done = false; }
      else if (cur > targets[leg]) { writeFootLog(leg, cur-1); done = false; }
    }
    spdDelay();
  }
}

void cogRestore(bool skip[4]) {
  bool done = false;
  while (!done) {
    done = true;
    for (int leg = 0; leg < 4; leg++) {
      if (skip[leg]) continue;
      int cur = angle[leg*2+1];
      int tgt = legFdw(leg);
      if (cur < tgt)      { writeFootLog(leg, cur+1); done = false; }
      else if (cur > tgt) { writeFootLog(leg, cur-1); done = false; }
    }
    spdDelay();
  }
}

void WALK() {
  // 정적 크리프: A→D→B→C 순서
  // cogRestore: 다음 cogShift와 방향 같은 다리는 제외, 반전되는 다리는 복귀
  bool skipA[4] = {true,  false, true,  false};  // A,C 제외, B,D 복귀
  bool skipD[4] = {false, true,  false, true };   // B,D 제외, A,C 복귀
  bool skipB[4] = {true,  false, true,  false};   // A,C 제외, B,D 복귀
  bool skipN[4] = {false, false, false, false};   // 전체 복귀

  cogShift(0); foup(0); armTo(0, logContact[0]); cogRestore(skipA); fodw(0); armTo(0, logToeoff(0)); 
  cogShift(3); foup(3); armTo(3, logContact[3]); cogRestore(skipD); fodw(3); armTo(3, logToeoff(3)); 
  cogShift(1); foup(1); armTo(1, logContact[1]); cogRestore(skipB); fodw(1); armTo(1, logToeoff(1)); 
  cogShift(2); foup(2); armTo(2, logContact[2]); cogRestore(skipN); fodw(2); armTo(2, logToeoff(2)); 
  // cogShift(0); foup(0); armTo(0, logContact[0]); fodw(0); armTo(0, logToeoff(0)); cogRestore(skipA);
  // cogShift(3); foup(3); armTo(3, logContact[3]); fodw(3); armTo(3, logToeoff(3)); cogRestore(skipD);
  // cogShift(1); foup(1); armTo(1, logContact[1]); fodw(1); armTo(1, logToeoff(1)); cogRestore(skipB);
  // cogShift(2); foup(2); armTo(2, logContact[2]); fodw(2); armTo(2, logToeoff(2)); cogRestore(skipN);
}

void ROUND() {
  for (int leg = 0; leg < 4; leg++) armTo(leg, logContact[leg]);
  for (int leg = 0; leg < 4; leg++) {
    foup(leg); armTo(leg, logToeoff(leg)); fodw(leg);
  }
}

/* Setup ---------------------------------------------------------------------*/
void setup() {
  Serial.begin(115200);
  delay(500);
  Serial.println("\n=== Spider Robot ===");

  for (int i = 0; i < 8; i++) servos[i]->attach(pins[i]);
  pinMode(ledPin, OUTPUT);

  stall();
  Serial.println("Servos ready");

  WiFi.mode(WIFI_AP);
  WiFi.softAP(ap_ssid, ap_password);
  Serial.printf("WiFi: %s / %s\n", ap_ssid, ap_password);
  Serial.printf("IP: %s\n", WiFi.softAPIP().toString().c_str());

  server.on("/", []() {
    String page = index_html;
    page.replace("%TITLE%", ap_ssid);
    server.send(200, "text/html", page);
  });
  server.on("/action", handleAction);
  server.begin();

  Serial.println("Ready! (help for commands)\n");
}

/* Loop ----------------------------------------------------------------------*/
void loop() {
  server.handleClient();
  static String serialBuf = "";

  while (Serial.available()) {
    char c = (char)Serial.read();
    if (c == ' ') {
      if (step_mode) {
        step_go = true;  // 한 스텝 전진
        Serial.print(".");
      } else {
        stopAction();
        Serial.println("[SPACE] 정지");
      }
      serialBuf = "";
      continue;
    } else if (c == '\r') {
      continue;
    } else if (c != '\n') {
      serialBuf += c;
      continue;
    }

    serialBuf.trim();
    serialBuf.toLowerCase();
    if (serialBuf.length() == 0) { serialBuf = ""; continue; }
    String cmd = serialBuf;
    serialBuf = "";
    Serial.printf("> %s\n", cmd.c_str());

    // 개별 서보 직접 제어: aa90, bf60 등 (물리각 직접 입력, angle[] 미갱신)
    if (cmd.length() >= 3 && cmd[0] >= 'a' && cmd[0] <= 'd' && (cmd[1] == 'a' || cmd[1] == 'f')) {
      int leg  = cmd[0] - 'a';
      char part = cmd[1];
      int ang  = cmd.substring(2).toInt();
      if (ang >= 0 && ang <= 180) {
        int idx = leg * 2 + (part == 'f' ? 1 : 0);
        writeServo(idx, ang);  // 물리각 직접, angle[] 갱신 없음 (튜닝용)
        Serial.printf("%s → %d° (phy)\n", names[idx], ang);
      }
    }
    // 논리각 직접 입력: la75, lc-75 등 (l=logical, a/b/c/d + 논리각)
    else if (cmd.length() >= 3 && cmd[0] == 'l' && cmd[1] >= 'a' && cmd[1] <= 'd') {
      int leg = cmd[1] - 'a';
      int log = cmd.substring(2).toInt();
      writeArmLog(leg, log);
      Serial.printf("Leg%c arm → logical %d° (phy %d°)\n", 'A'+leg, log, toServoArm(leg, log));
    }
    // offset 설정: oa5, ob-3 등
    else if (cmd.length() >= 3 && cmd[0] == 'o' && cmd[1] >= 'a' && cmd[1] <= 'd') {
      int leg = cmd[1] - 'a';
      armOffset[leg] = cmd.substring(2).toInt();
      Serial.printf("Leg%c armOffset → %d°\n", 'A'+leg, armOffset[leg]);
    }
    else if (cmd == "off") {
      for (int i = 0; i < 8; i++) servos[i]->detach();
      Serial.println("서보 OFF");
    }
    else if (cmd == "on") {
      for (int i = 0; i < 8; i++) servos[i]->attach(pins[i]);
      stall();
      Serial.println("서보 ON");
    }
    else if (cmd == "stl" || cmd == "init") { stopAction(); Serial.println("초기자세"); }
    else if (cmd == "stop") { stopAction(); Serial.println("정지"); }
    else if (cmd == "wlk")  { startLoopAction(1); Serial.println("전진(연속) - stop으로 정지"); }
    else if (cmd == "rnd")  { startLoopAction(2); Serial.println("회전(연속) - stop으로 정지"); }
    else if (cmd == "up")   { startOnceAction(3); Serial.println("일어서기 (1회)"); }
    else if (cmd == "dn")   { startOnceAction(4); Serial.println("앉기 (1회)"); }
    else if (cmd == "updn") { startLoopAction(5); Serial.println("Up-Dn(연속) - stop으로 정지"); }
    else if (cmd.startsWith("spd")) {
      int val = cmd.substring(3).toInt();
      if (val >= 1 && val <= 10) {
        spd = 11 - val;
        Serial.printf("속도: %d (delay=%dms)\n", val, spd);
      } else {
        Serial.printf("속도 범위 오류 (1~10), 현재: %d\n", 11 - spd);
      }
    }
    else if (cmd == "step") {
      step_mode = !step_mode;
      step_go   = false;
      if (step_mode) Serial.println("스텝모드 ON  (스페이스=한스텝, 'step'=해제)");
      else           Serial.println("스텝모드 OFF");
    }
    // stride 설정: str30, str50 등
    else if (cmd.startsWith("str")) {
      int val = cmd.substring(3).toInt();
      if (val >= 5 && val <= 90) {
        stride = val;
        Serial.printf("stride: %d° (toeoff: A=%d B=%d C=%d D=%d)\n",
          stride, logToeoff(0), logToeoff(1), logToeoff(2), logToeoff(3));
      } else {
        Serial.printf("stride 범위 오류 (5~90), 현재: %d\n", stride);
      }
    }
    // CoG alpha 설정: cog30, cog45 등
    else if (cmd.startsWith("cog")) {
      int val = cmd.substring(3).toInt();
      if (val >= 0 && val <= 80) {
        cogAlpha = val;
        Serial.printf("cogAlpha: %d°\n", cogAlpha);
      } else {
        Serial.printf("cogAlpha 범위 오류 (0~80), 현재: %d\n", cogAlpha);
      }
    }
    // foot offset 설정: foa5, fob-3 등 (fo=footoffset + a/b/c/d + 값)
    else if (cmd.length() >= 4 && cmd.startsWith("fo") && cmd[2] >= 'a' && cmd[2] <= 'd') {
      int leg = cmd[2] - 'a';
      footOffset[leg] = cmd.substring(3).toInt();
      Serial.printf("Leg%c footOffset → %d°\n", 'A'+leg, footOffset[leg]);
    }
    else if (cmd == "sts") {
      Serial.println("\n[상태]");
      Serial.printf(" stride=%d  cogAlpha=%d\n", stride, cogAlpha);
      Serial.println(" Leg | logStl | contact | toeoff | armOfs | footOfs | Arm(log) | Foot(log)");
      Serial.println("-----|--------|---------|--------|--------|---------|----------|----------");
      for (int i = 0; i < 4; i++) {
        Serial.printf("  %c  |  %4d  |   %4d  |  %4d  |  %4d  |  %4d   |   %3d    |   %3d\n",
          'A'+i, logStl[i], logContact[i], logToeoff(i),
          armOffset[i], footOffset[i], angle[i*2], angle[i*2+1]);
      }
      Serial.println();
    }
    else if (cmd == "help") {
      Serial.println("\n[명령어]");
      Serial.println(" aa90, bf60      - 개별 서보 물리각 직접 설정");
      Serial.println(" la75, lb-75     - 논리각으로 arm 설정 (l=logical)");
      Serial.println(" oa5, ob-3       - 다리별 arm offset 설정");
      Serial.println(" foa5, fob-3     - 다리별 foot offset 설정");
      Serial.println(" str50           - stride 설정 (5~90)");
      Serial.println(" cog45           - CoG pre-shift 이동량 설정 (0~80)");
      Serial.println(" off/on          - 서보 끄기/켜기");
      Serial.println(" spd1~10         - 속도 설정 (1=느림, 10=빠름)");
      Serial.println(" step            - 스텝모드 토글 (스페이스=한스텝씩 이동)");
      Serial.println(" stl/init        - 초기 자세");
      Serial.println(" stop/스페이스    - 연속 동작 정지");
      Serial.println(" wlk/rnd         - 걷기/회전 (연속)");
      Serial.println(" up/dn           - 일어서기/앉기 (1회)");
      Serial.println(" updn            - Up-Dn 교대 (연속)");
      Serial.println(" sts             - 상태 확인\n");
    }
  }
  delay(1);
}