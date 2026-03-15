/* -----------------------------------------------------------------------------
  Spider Robot - ESP32 Web Control (PCBWay Functions)
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

/* PCBWay 동작 파라미터 -------------------------------------------------------*/
// 모터 설정 60도, 조립 90도 경우, PCBWay
// int Fdw = 60;   // Foot down limit
// int Fup = 80;   // Foot up limit
// 모터 설정 120도, 조립 0도 경우, Yirugo
int Fdw = 30;   // Foot down limit 
int Fup = 50;   // Foot up limit
int Abw = 70;   // Arm backward limit
int Afw = 110;  // Arm forward limit
int spd = 3;    // Movement speed (~2-5)

int ledPin = 23;

/* 비동기 동작 ---------------------------------------------------------------*/
volatile bool action_running = false;
volatile bool loop_running = false;
// TaskHandle_t ActionTask = NULL;
TaskHandle_t ActionTask = NULL;

/* 함수 선언 -----------------------------------------------------------------*/
void WALK(void);
void ROUND(void);
void stall(void);
void BLINK(void);
void flat_up(void);
void flat_dw(void);
void go_ahead(void);
void Afoup(void); void Afodw(void);
void Bfoup(void); void Bfodw(void);
void Cfoup(void); void Cfodw(void);
void Dfoup(void); void Dfodw(void);
void armAfw(void); void armAbw(void);
void armBfw(void); void armBbw(void);
void armCfw(void); void armCbw(void);
void armDfw(void); void armDbw(void);
void handleAction(void);
void stopAction(void);
void startOnceAction(int code);
void startLoopAction(int code);
void actionTask(void* param);

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
    if      (cmd == "walk")                                        { res = "전진(연속)"; code = 1; }
    else if (cmd == "left" || cmd == "right" || cmd == "round")    { res = "회전(연속)"; code = 2; }
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

/* 동작 함수 (PCBWay) ---------------------------------------------------------*/
void BLINK() {
  for (int i = 0; i < 3; i++) {
    digitalWrite(ledPin, HIGH);
    delay(100);
    digitalWrite(ledPin, LOW);
    delay(100);
  }
}

void stall() {
  BLINK();
  servo_aa.write(90);
  servo_af.write(60);
  servo_ba.write(90);
  servo_bf.write(60);
  servo_ca.write(90);
  servo_cf.write(60);
  servo_da.write(90);
  servo_df.write(60);
  digitalWrite(ledPin, HIGH);
}

// Grouped movements for walking and rotation
void WALK() {
  go_ahead();
  Afoup();
  armAfw();
  Afodw();
  Cfoup();
  armCbw();
  Cfodw();
  Bfoup();
  armBbw();
  Bfodw();
  Dfoup();
  armDfw();
  Dfodw();
}

void ROUND() {
  armAfw();
  armBfw();
  armCfw();
  armDfw();
  Afoup();
  armAbw();
  Afodw();
  Bfoup();
  armBbw();
  Bfodw();
  Cfoup();
  armCbw();
  Cfodw();
  Dfoup();
  armDbw();
  Dfodw();
}

// Foot movements (up and down)
void Afoup() {
  int max = Fup + 5;
  int min = Fdw + 5;
  for (int i = min; i <= max; i++) {
    servo_af.write(i);
    delay(spd);
  }
}

void Bfoup() {
  int max = Fup + 5;
  int min = Fdw + 5;
  for (int i = min; i <= max; i++) {
    servo_bf.write(i);
    delay(spd);
  }
}

void Cfoup() {
  int max = Fup + 5;
  int min = Fdw + 5;
  for (int i = min; i <= max; i++) {
    servo_cf.write(i);
    delay(spd);
  }
}

void Dfoup() {
  int max = Fup + 5;
  int min = Fdw + 5;
  for (int i = min; i <= max; i++) {
    servo_df.write(i);
    delay(spd);
  }
}

void Afodw() {
  int max = Fup + 5;
  int min = Fdw + 5;
  for (int i = max; i >= min; i--) {
    servo_af.write(i);
    delay(spd);
  }
}

void Bfodw() {
  int max = Fup + 5;
  int min = Fdw + 5;
  for (int i = max; i >= min; i--) {
    servo_bf.write(i);
    delay(spd);
  }
}

void Cfodw() {
  int max = Fup + 5;
  int min = Fdw + 5;
  for (int i = max; i >= min; i--) {
    servo_cf.write(i);
    delay(spd);
  }
}

void Dfodw() {
  int max = Fup + 5;
  int min = Fdw + 5;
  for (int i = max; i >= min; i--) {
    servo_df.write(i);
    delay(spd);
  }
}

// Basic movements
void go_ahead() {
  // spd = 3;  // ~ 2-5
  for (int i = Afw; i >= Abw; i--) {
    servo_aa.write(i);
    servo_da.write(i);
    int y = Abw + Afw - i;
    servo_ca.write(y);
    servo_ba.write(y);
    delay(spd);
  }
}

void flat_up() {
  int max = Fup + 5;
  int min = Fdw + 5;
  servo_af.write(min);
  servo_bf.write(min);
  servo_cf.write(min);
  servo_df.write(min);
  delay(500);
  for (int i = min; i <= max + 50; i++) {
    servo_af.write(i);
    servo_bf.write(i);
    servo_cf.write(i);
    servo_df.write(i);
    delay(spd);
  }
  delay(500);
}

void flat_dw() {
  int max = Fup + 5;
  int min = Fdw + 5;
  servo_af.write(max);
  servo_bf.write(max);
  servo_cf.write(max);
  servo_df.write(max);
  delay(500);
  for (int i = max; i >= min; i--) {
    servo_af.write(i);
    servo_bf.write(i);
    servo_cf.write(i);
    servo_df.write(i);
    delay(spd);
  }
  delay(500);
}

// single arm mvt A
void armAfw() {
  int min = Abw + 0;
  int max = Afw + 0;
  for (int i = min; i <= max; i++) {
    servo_aa.write(i);
    delay(spd);
  }
}

void armAbw() {
  int min = Abw + 0;
  int max = Afw + 0;
  for (int i = max; i >= min; i--) {
    servo_aa.write(i);
    delay(spd);
  }
}

// single arm mvt B
void armBfw() {
  int min = Abw + 0;
  int max = Afw + 0;
  for (int i = min; i <= max; i++) {
    servo_ba.write(i);
    delay(spd);
  }
}

void armBbw() {
  int min = Abw + 0;
  int max = Afw + 0;
  for (int i = max; i >= min; i--) {
    servo_ba.write(i);
    delay(spd);
  }
}

// single arm mvt C
void armCfw() {
  int min = Abw + 0;
  int max = Afw + 0;
  for (int i = min; i <= max; i++) {
    servo_ca.write(i);
    delay(spd);
  }
}

void armCbw() {
  int min = Abw + 0;
  int max = Afw + 0;
  for (int i = max; i >= min; i--) {
    servo_ca.write(i);
    delay(spd);
  }
}

// single arm mvt D
void armDfw() {
  int min = Abw + 0;
  int max = Afw + 0;
  for (int i = min; i <= max; i++) {
    servo_da.write(i);
    delay(spd);
  }
}

void armDbw() {
  int min = Abw + 0;
  int max = Afw + 0;
  for (int i = max; i >= min; i--) {
    servo_da.write(i);
    delay(spd);
  }
}

/* Setup ---------------------------------------------------------------------*/
void setup() {
  for (int i = 0; i < 8; i++) servos[i]->attach(pins[i]);
  pinMode(ledPin, OUTPUT);
  Serial.begin(115200);
  delay(500);
  Serial.println("\n=== Spider Robot (PCBWay) ===");

  stall();
  delay(1000);
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

  Serial.println("Ready!\n");
}

/* Loop ----------------------------------------------------------------------*/
void loop() {
  server.handleClient();
  delay(1);
}
