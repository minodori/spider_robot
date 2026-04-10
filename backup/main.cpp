/* -----------------------------------------------------------------------------
  Spider Robot - ESP32 Web Control
  4 Legs x 2 Servos = 8 Servos
  다리: A(앞왼), B(앞오), C(뒤왼), D(뒤오)
  서보: aa/af, ba/bf, ca/cf, da/df (arm/foot)
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

// 핀 배치
const int pin_aa = 26, pin_af = 12;  // Leg A
const int pin_ba = 32, pin_bf = 33;  // Leg B
const int pin_ca = 18, pin_cf = 2;   // Leg C
const int pin_da = 17, pin_df = 5;   // Leg D

// 서보 배열
Servo* servos[8] = {&servo_aa, &servo_af, &servo_ba, &servo_bf,
                    &servo_ca, &servo_cf, &servo_da, &servo_df};
const int pins[8] = {pin_aa, pin_af, pin_ba, pin_bf, pin_ca, pin_cf, pin_da, pin_df};
const char* names[8] = {"aa", "af", "ba", "bf", "ca", "cf", "da", "df"};

/* 팔 서보 보행 파라미터 ------------------------------------------------------*/
// 모든 서보는 물리 각도를 직접 기록 (변환 없음)
// contact: swing 후 발이 닿는 위치 (물리각)
// armDir:  stance 이동 방향 (+1=각도 증가, -1=감소)
// stride:  stance 이동량(°) → toeoff = contact + armDir * stride
// phyStl:  중립(대각선) 물리각
//                        A    B    C    D
int phyContact[4]  = {100,  80,  30, 150};  // swing-contact 물리각
int armDir[4]      = {  1,  -1,   1,  -1};  // +1: CW, -1: CCW
int stride         = 50;                     // stance 구간 (°)
int phyStl[4]      = { 90,  90,  90,  90};  // 중립 물리각

inline int phyToeoff(int leg) { return phyContact[leg] + armDir[leg] * stride; }

/* 발 서보 범위 --------------------------------------------------------------*/
int Fdw = 30;   // 서있는 자세 (발 아래)
int Fup = 60;   // 발 든 자세
int spd = 3;    // 속도 (delay ms)
int ledPin = 23;

/* 현재 각도 (물리각) ---------------------------------------------------------*/
int angle[8] = {90, 30, 90, 30, 90, 30, 90, 30};  // aa,af,ba,bf,ca,cf,da,df

/* 비동기 동작 ---------------------------------------------------------------*/
volatile bool action_running = false;
volatile bool loop_running = false;
TaskHandle_t ActionTask = NULL;

/* 함수 선언 -----------------------------------------------------------------*/
void writeServo(int idx, int ang);
void writeAA(int ang); void writeAF(int ang);
void writeBA(int ang); void writeBF(int ang);
void writeCA(int ang); void writeCF(int ang);
void writeDA(int ang); void writeDF(int ang);

void stall(void);
void WALK(void);
void ROUND(void);
void flat_up(void);
void flat_dw(void);
void BLINK(void);
void Afoup(void); void Afodw(void);
void Bfoup(void); void Bfodw(void);
void Cfoup(void); void Cfodw(void);
void Dfoup(void); void Dfodw(void);
void armAfw(void); void armAbw(void);
void armBfw(void); void armBbw(void);
void armCfw(void); void armCbw(void);
void armDfw(void); void armDbw(void);
void armTo(int idx, int target);
void handleAction(void);
void stopAction(void);
void startOnceAction(int code);
void startLoopAction(int code);
void actionTask(void* param);

/* 서보 제어 (물리각 직접 기록) -----------------------------------------------*/
void writeAA(int ang) { angle[0] = ang; servo_aa.write(ang); }
void writeAF(int ang) { angle[1] = ang; servo_af.write(ang); }
void writeBA(int ang) { angle[2] = ang; servo_ba.write(ang); }
void writeBF(int ang) { angle[3] = ang; servo_bf.write(ang); }
void writeCA(int ang) { angle[4] = ang; servo_ca.write(ang); }
void writeCF(int ang) { angle[5] = ang; servo_cf.write(ang); }
void writeDA(int ang) { angle[6] = ang; servo_da.write(ang); }
void writeDF(int ang) { angle[7] = ang; servo_df.write(ang); }

void writeServo(int idx, int ang) {
  switch(idx) {
    case 0: writeAA(ang); break;
    case 1: writeAF(ang); break;
    case 2: writeBA(ang); break;
    case 3: writeBF(ang); break;
    case 4: writeCA(ang); break;
    case 5: writeCF(ang); break;
    case 6: writeDA(ang); break;
    case 7: writeDF(ang); break;
  }
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
    if      (cmd == "walk")                          { res = "전진(연속)"; code = 1; }
    else if (cmd == "left" || cmd == "right" || cmd == "round") { res = "회전(연속)"; code = 2; }
    else if (cmd == "up")   { startOnceAction(3); res = "일어서기"; }
    else if (cmd == "down") { startOnceAction(4); res = "앉기"; }
    else if (cmd == "updn") { res = "Up-Dn(연속)"; code = 5; }
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
  if (ActionTask) {
    loop_running = false;
    vTaskDelete(ActionTask);
    ActionTask = NULL;
    action_running = false;
  }
  loop_running = false;
  int* p = new int(code);
  xTaskCreatePinnedToCore(actionTask, "Act", 4096, p, 1, &ActionTask, 0);
}

void startLoopAction(int code) {
  if (ActionTask) {
    loop_running = false;
    vTaskDelete(ActionTask);
    ActionTask = NULL;
    action_running = false;
  }
  loop_running = true;
  int* p = new int(code);
  xTaskCreatePinnedToCore(actionTask, "Act", 4096, p, 1, &ActionTask, 0);
}

void stopAction() {
  loop_running = false;
  if (ActionTask) {
    vTaskDelete(ActionTask);
    ActionTask = NULL;
  }
  action_running = false;
  stall();
}

/* 동작 함수 -----------------------------------------------------------------*/
void BLINK() {
  for (int i = 0; i < 3; i++) {
    digitalWrite(ledPin, HIGH); delay(100);
    digitalWrite(ledPin, LOW); delay(100);
  }
}

void stall() {
  BLINK();
  int targets[8] = {phyStl[0], Fdw, phyStl[1], Fdw, phyStl[2], Fdw, phyStl[3], Fdw};
  bool done = false;
  while (!done) {
    done = true;
    for (int i = 0; i < 8; i++) {
      if      (angle[i] < targets[i]) { writeServo(i, angle[i] + 1); done = false; }
      else if (angle[i] > targets[i]) { writeServo(i, angle[i] - 1); done = false; }
    }
    delay(spd);
  }
}

void flat_dw() {
  writeAF(Fdw); writeBF(Fdw); writeCF(Fdw); writeDF(Fdw);
  delay(500);
  for (int i = Fdw; i <= Fup + 50; i++) {
    writeAF(i); writeBF(i); writeCF(i); writeDF(i);
    delay(spd);
  }
  delay(500);
}

void flat_up() {
  writeAF(Fup); writeBF(Fup); writeCF(Fup); writeDF(Fup);
  delay(500);
  for (int i = Fup; i >= Fdw; i--) {
    writeAF(i); writeBF(i); writeCF(i); writeDF(i);
    delay(spd);
  }
  delay(500);
}

/* 발 up/down ----------------------------------------------------------------*/
void Afoup() { for (int i = Fdw; i <= Fup; i++) { writeAF(i); delay(spd); } }
void Afodw() { for (int i = Fup; i >= Fdw; i--) { writeAF(i); delay(spd); } }
void Bfoup() { for (int i = Fdw; i <= Fup; i++) { writeBF(i); delay(spd); } }
void Bfodw() { for (int i = Fup; i >= Fdw; i--) { writeBF(i); delay(spd); } }
void Cfoup() { for (int i = Fdw; i <= Fup; i++) { writeCF(i); delay(spd); } }
void Cfodw() { for (int i = Fup; i >= Fdw; i--) { writeCF(i); delay(spd); } }
void Dfoup() { for (int i = Fdw; i <= Fup; i++) { writeDF(i); delay(spd); } }
void Dfodw() { for (int i = Fup; i >= Fdw; i--) { writeDF(i); delay(spd); } }

/* 팔 이동 -------------------------------------------------------------------*/
// armTo: 현재 물리각 → 목표 물리각 (방향 자동 판별)
void armTo(int idx, int target) {
  int cur = angle[idx];
  if (cur < target) { for (int i = cur; i <= target; i++) { writeServo(idx, i); delay(spd); } }
  else              { for (int i = cur; i >= target; i--) { writeServo(idx, i); delay(spd); } }
}

// contact(전방)/toeoff(후방) 이동
void armAfw() { armTo(0, phyContact[0]); }  void armAbw() { armTo(0, phyToeoff(0)); }
void armBfw() { armTo(2, phyContact[1]); }  void armBbw() { armTo(2, phyToeoff(1)); }
void armCfw() { armTo(4, phyContact[2]); }  void armCbw() { armTo(4, phyToeoff(2)); }
void armDfw() { armTo(6, phyContact[3]); }  void armDbw() { armTo(6, phyToeoff(3)); }

void WALK() {
  // 정적 크리프: A→D→B→C 순서
  // 각 다리: swing(발들기) → contact → stand → toeoff
  Afoup(); armTo(0, phyContact[0]); Afodw(); armTo(0, phyToeoff(0));
  Dfoup(); armTo(6, phyContact[3]); Dfodw(); armTo(6, phyToeoff(3));
  Bfoup(); armTo(2, phyContact[1]); Bfodw(); armTo(2, phyToeoff(1));
  Cfoup(); armTo(4, phyContact[2]); Cfodw(); armTo(4, phyToeoff(2));
}

void ROUND() {
  armAfw(); armBfw(); armCfw(); armDfw();
  Afoup(); armAbw(); Afodw();
  Bfoup(); armBbw(); Bfodw();
  Cfoup(); armCbw(); Cfodw();
  Dfoup(); armDbw(); Dfodw();
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
      stopAction();
      Serial.println("[SPACE] 정지");
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

    // 개별 서보: aa90, bf60 등 (물리각 직접 입력)
    if (cmd.length() >= 3 && cmd[0] >= 'a' && cmd[0] <= 'd') {
      int leg = cmd[0] - 'a';
      char part = cmd[1];
      int ang = cmd.substring(2).toInt();
      if (ang >= 0 && ang <= 180) {
        int idx = leg * 2 + (part == 'f' ? 1 : 0);
        writeServo(idx, ang);
        Serial.printf("%s → %d°\n", names[idx], ang);
      }
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
    else if (cmd == "wlk") { startLoopAction(1); Serial.println("전진(연속) 시작 - stop으로 정지"); }
    else if (cmd == "rnd") { startLoopAction(2); Serial.println("회전(연속) 시작 - stop으로 정지"); }
    else if (cmd == "up")   { startOnceAction(3); Serial.println("일어서기 (1회)"); }
    else if (cmd == "dn")   { startOnceAction(4); Serial.println("앉기 (1회)"); }
    else if (cmd == "updn") { startLoopAction(5); Serial.println("Up-Dn(연속) 시작 - stop으로 정지"); }
    else if (cmd.startsWith("spd")) {
      int val = cmd.substring(3).toInt();
      if (val >= 1 && val <= 10) {
        spd = 11 - val;
        Serial.printf("속도: %d (delay=%dms)\n", val, spd);
      } else {
        Serial.printf("속도 범위 오류 (1~10), 현재: %d\n", 11 - spd);
      }
    }
    else if (cmd == "sts") {
      Serial.println("\n[상태]");
      Serial.println(" Leg | contact | toeoff | stl | Arm° | Foot°");
      Serial.println("-----|---------|--------|-----|------|------");
      for (int i = 0; i < 4; i++) {
        char leg = 'A' + i;
        Serial.printf("  %c  |   %3d   |  %3d   | %3d | %3d  | %3d\n",
          leg, phyContact[i], phyToeoff(i), phyStl[i], angle[i*2], angle[i*2+1]);
      }
      Serial.println();
    }
    else if (cmd == "help") {
      Serial.println("\n[명령어]");
      Serial.println(" aa90, bf60  - 개별 서보 물리각 직접 설정");
      Serial.println(" off/on      - 서보 끄기/켜기");
      Serial.println(" spd1~10     - 속도 설정 (1=느림, 10=빠름)");
      Serial.println(" stl/init    - 초기 자세");
      Serial.println(" stop/스페이스 - 연속 동작 정지");
      Serial.println(" wlk/rnd     - 걷기/회전 (연속)");
      Serial.println(" up/dn       - 일어서기/앉기 (1회)");
      Serial.println(" updn        - Up-Dn 교대 (연속)");
      Serial.println(" sts         - 상태 확인\n");
    }
  }
  delay(1);
}
