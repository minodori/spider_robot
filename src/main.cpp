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

/* 방향 반전 설정 ------------------------------------------------------------*/
// Arm: A,C 정방향, B,D 반전 (좌우 대칭)
// Foot: 전부 정방향 (120°=수평으로 조립)
bool reverseArm[4] = {false, false, false, false};  // A, B, C, D (혼 모두 위쪽)
bool reverseFoot[4] = {false, false, false, false};

/* 현재 각도 -----------------------------------------------------------------*/
int angle[8] = {90, 60, 90, 60, 90, 60, 90, 60};  // aa,af,ba,bf,ca,cf,da,df

/* 동작 범위 -----------------------------------------------------------------*/
int Fdw = 120, Fup = 40;           // Foot limit: 120(수평/슬립) ~ 40(완전 서기)
int footStand = 60;                // 서있는 자세 발 각도
int armFront = 45, armWide = 135;  // Arm limit: 45(앞으로 쭉) ~ 135(날개처럼 옆으로)
int armCenter = 90;                // 서있는 자세 팔 각도 (중앙)
int spd = 3;               // 속도 (delay ms)
int ledPin = 23;

/* 비동기 동작 ---------------------------------------------------------------*/
volatile bool action_running = false;
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
void handleAction(void);
void startAction(int code);
void actionTask(void* param);

/* 서보 제어 (방향 보정) -----------------------------------------------------*/
// A,B: 앞다리 - 각도 그대로
// C,D: 뒷다리 - 180-ang로 반전 (같은 각도 = 대칭 동작)
void writeAA(int ang) { angle[0] = ang; servo_aa.write(reverseArm[0] ? 180-ang : ang); }
void writeAF(int ang) { angle[1] = ang; servo_af.write(reverseFoot[0] ? 180-ang : ang); }
void writeBA(int ang) { angle[2] = ang; servo_ba.write(reverseArm[1] ? 180-ang : ang); }
void writeBF(int ang) { angle[3] = ang; servo_bf.write(reverseFoot[1] ? 180-ang : ang); }
void writeCA(int ang) {
  angle[4] = ang;
  int adj = 180 - ang;  // 뒷다리: 각도 반전
  servo_ca.write(reverseArm[2] ? 180-adj : adj);
}
void writeCF(int ang) { angle[5] = ang; servo_cf.write(reverseFoot[2] ? 180-ang : ang); }
void writeDA(int ang) {
  angle[6] = ang;
  int adj = 180 - ang;  // 뒷다리: 각도 반전
  servo_da.write(reverseArm[3] ? 180-adj : adj);
}
void writeDF(int ang) { angle[7] = ang; servo_df.write(reverseFoot[3] ? 180-ang : ang); }

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

  if (action_running) { res = "동작중"; }
  else if (cmd == "walk") { res = "전진"; startAction(1); }
  else if (cmd == "left" || cmd == "right") { res = "회전"; startAction(2); }
  else if (cmd == "up") { res = "일어서기"; startAction(3); }
  else if (cmd == "down") { res = "앉기"; startAction(4); }
  else if (cmd == "round") { res = "회전"; startAction(2); }
  else if (cmd == "stop" || cmd == "init") { res = "정지"; startAction(5); }
  else if (cmd == "speed") { spd = 11 - server.arg("val").toInt(); res = "속도설정"; }

  server.send(200, "text/plain", res);
}

/* 비동기 액션 ---------------------------------------------------------------*/
void actionTask(void* param) {
  int act = *((int*)param);
  delete (int*)param;
  action_running = true;
  digitalWrite(ledPin, HIGH);

  switch(act) {
    case 1: WALK(); break;
    case 2: ROUND(); break;
    case 3: flat_up(); break;
    case 4: flat_dw(); break;
    case 5: stall(); break;
  }

  digitalWrite(ledPin, LOW);
  action_running = false;
  ActionTask = NULL;
  vTaskDelete(NULL);
}

void startAction(int code) {
  if (action_running || ActionTask) return;
  int* p = new int(code);
  xTaskCreatePinnedToCore(actionTask, "Act", 4096, p, 1, &ActionTask, 0);
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
  writeAA(armCenter); writeBA(armCenter);
  writeCA(armCenter); writeDA(armCenter);
  writeAF(footStand); writeBF(footStand);
  writeCF(footStand); writeDF(footStand);
}

void flat_up() {
  // 일어서기: 현재 → 40° (다리 세움)
  int start = footStand;
  int end = Fup;  // 40
  writeAF(start); writeBF(start); writeCF(start); writeDF(start);
  delay(300);
  for (int i = start; i >= end; i--) {
    writeAF(i); writeBF(i); writeCF(i); writeDF(i);
    delay(spd);
  }
}

void flat_dw() {
  // 앉기: 현재 → 120° (다리 수평)
  int start = footStand;
  int end = Fdw;  // 120
  writeAF(start); writeBF(start); writeCF(start); writeDF(start);
  delay(300);
  for (int i = start; i <= end; i++) {
    writeAF(i); writeBF(i); writeCF(i); writeDF(i);
    delay(spd);
  }
}

void WALK() {
  // 전진: 4팔 동시 wide→front (몸 밀기)
  for (int i = armWide; i >= armFront; i--) {
    writeAA(i); writeBA(i); writeCA(i); writeDA(i);
    delay(spd);
  }
  // A: foot up - arm forward - foot down
  for (int i = footStand; i >= Fup; i--) { writeAF(i); delay(spd); }
  for (int i = armFront; i <= armWide; i++) { writeAA(i); delay(spd); }
  for (int i = Fup; i <= footStand; i++) { writeAF(i); delay(spd); }
  // D: foot up - arm forward - foot down
  for (int i = footStand; i >= Fup; i--) { writeDF(i); delay(spd); }
  for (int i = armFront; i <= armWide; i++) { writeDA(i); delay(spd); }
  for (int i = Fup; i <= footStand; i++) { writeDF(i); delay(spd); }
  // B: foot up - arm forward - foot down
  for (int i = footStand; i >= Fup; i--) { writeBF(i); delay(spd); }
  for (int i = armFront; i <= armWide; i++) { writeBA(i); delay(spd); }
  for (int i = Fup; i <= footStand; i++) { writeBF(i); delay(spd); }
  // C: foot up - arm forward - foot down
  for (int i = footStand; i >= Fup; i--) { writeCF(i); delay(spd); }
  for (int i = armFront; i <= armWide; i++) { writeCA(i); delay(spd); }
  for (int i = Fup; i <= footStand; i++) { writeCF(i); delay(spd); }
}

void ROUND() {
  // 모든 팔 벌리기
  for (int i = armCenter; i <= armWide; i++) {
    writeAA(i); writeBA(i); writeCA(i); writeDA(i);
    delay(spd);
  }
  // A: foot up - arm 좁히기 - foot down
  for (int i = footStand; i >= Fup; i--) { writeAF(i); delay(spd); }
  for (int i = armWide; i >= armFront; i--) { writeAA(i); delay(spd); }
  for (int i = Fup; i <= footStand; i++) { writeAF(i); delay(spd); }
  // B: foot up - arm 좁히기 - foot down
  for (int i = footStand; i >= Fup; i--) { writeBF(i); delay(spd); }
  for (int i = armWide; i >= armFront; i--) { writeBA(i); delay(spd); }
  for (int i = Fup; i <= footStand; i++) { writeBF(i); delay(spd); }
  // C: foot up - arm 좁히기 - foot down
  for (int i = footStand; i >= Fup; i--) { writeCF(i); delay(spd); }
  for (int i = armWide; i >= armFront; i--) { writeCA(i); delay(spd); }
  for (int i = Fup; i <= footStand; i++) { writeCF(i); delay(spd); }
  // D: foot up - arm 좁히기 - foot down
  for (int i = footStand; i >= Fup; i--) { writeDF(i); delay(spd); }
  for (int i = armWide; i >= armFront; i--) { writeDA(i); delay(spd); }
  for (int i = Fup; i <= footStand; i++) { writeDF(i); delay(spd); }
}

/* Setup ---------------------------------------------------------------------*/
void setup() {
  Serial.begin(115200);
  delay(500);
  Serial.println("\n=== Spider Robot ===");

  // 서보 연결
  for (int i = 0; i < 8; i++) servos[i]->attach(pins[i]);
  pinMode(ledPin, OUTPUT);

  stall();
  Serial.println("Servos ready");

  // WiFi
  WiFi.mode(WIFI_AP);
  WiFi.softAP(ap_ssid, ap_password);
  Serial.printf("WiFi: %s / %s\n", ap_ssid, ap_password);
  Serial.printf("IP: %s\n", WiFi.softAPIP().toString().c_str());

  // 웹서버
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

  if (Serial.available()) {
    String cmd = Serial.readStringUntil('\n');
    cmd.trim();
    cmd.toLowerCase();
    Serial.printf("> %s\n", cmd.c_str());

    // 개별 서보: aa90, bf60 등
    if (cmd.length() >= 3 && cmd[0] >= 'a' && cmd[0] <= 'd') {
      int leg = cmd[0] - 'a';  // 0~3
      char part = cmd[1];      // 'a' or 'f'
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
    else if (cmd == "stl" || cmd == "init") { stall(); Serial.println("초기자세"); }
    else if (cmd == "wlk") { WALK(); Serial.println("Walk 완료"); }
    else if (cmd == "rnd") { ROUND(); Serial.println("Round 완료"); }
    else if (cmd == "up") { flat_up(); Serial.println("Up 완료"); }
    else if (cmd == "dn") { flat_dw(); Serial.println("Down 완료"); }
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
      Serial.println(" Leg | Arm(pin) | Foot(pin) | Arm° | Foot°");
      Serial.println("-----|----------|-----------|------|------");
      Serial.printf("  A  |  %2d(%s)  |  %2d(%s)   | %3d  | %3d\n",
        pin_aa, reverseArm[0]?"R":"N", pin_af, reverseFoot[0]?"R":"N", angle[0], angle[1]);
      Serial.printf("  B  |  %2d(%s)  |  %2d(%s)   | %3d  | %3d\n",
        pin_ba, reverseArm[1]?"R":"N", pin_bf, reverseFoot[1]?"R":"N", angle[2], angle[3]);
      Serial.printf("  C  |  %2d(%s)  |  %2d(%s)   | %3d  | %3d\n",
        pin_ca, reverseArm[2]?"R":"N", pin_cf, reverseFoot[2]?"R":"N", angle[4], angle[5]);
      Serial.printf("  D  |  %2d(%s)  |  %2d(%s)   | %3d  | %3d\n",
        pin_da, reverseArm[3]?"R":"N", pin_df, reverseFoot[3]?"R":"N", angle[6], angle[7]);
      Serial.println();
    }
    else if (cmd == "help") {
      Serial.println("\n[명령어]");
      Serial.println(" aa90, bf60  - 개별 서보 (다리a~d, arm/foot, 각도)");
      Serial.println(" off/on      - 서보 끄기/켜기");
      Serial.println(" spd1~10     - 속도 설정 (1=느림, 10=빠름)");
      Serial.println(" stl(stall)       - 초기 자세");
      Serial.println(" wlk(walk)/rnd(round)  - 걷기/회전");
      Serial.println(" up/dn(down)     - 일어서기/앉기");
      Serial.println(" sts(status)      - 상태 확인\n");
    }
  }
  delay(1);
}
