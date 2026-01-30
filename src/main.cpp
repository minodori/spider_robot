/* -----------------------------------------------------------------------------
  - Project: Simple 3D Spider Robot (ESP32 Web Control Version)
  - Original: PCBWay Community - @maker101io
  - Modified for Web Control by: Claude
  - Date: 2025
  - 4 Legs x 2 Servos = 8 Servos total
   -----------------------------------------------------------------------------*/

#include <Arduino.h>
#include <WiFi.h>
#include <WebServer.h>
#include <ESP32Servo.h>

/* WiFi AP Mode 설정 --------------------------------------------------------*/
const char* ap_ssid = "SpiderRobot";
const char* ap_password = "12345678";

WebServer server(80);

/* Servos --------------------------------------------------------------------*/
Servo servo_footA, servo_armA;
Servo servo_footB, servo_armB;
Servo servo_footC, servo_armC;
Servo servo_footD, servo_armD;

// Servo motor pins
int srv_footA = 13;
int srv_armA = 12;
int srv_footB = 15;
int srv_armB = 2;
int srv_footC = 26;
int srv_armC = 25;
int srv_footD = 17;
int srv_armD = 5;

// Movement limits and speed settings
int Fdw = 60;   // Foot down limit 
int Fup = 80;   // Foot up limit
int Abw = 70;   // Arm backward limit
int Afw = 110;  // Arm forward limit
int spd = 3;    // Movement speed (~2-5)

int ledPin = 23;  // LED pin for status indication

/* 비동기 동작을 위한 변수 */
volatile bool action_running = false;
TaskHandle_t ActionTask = NULL;

/* Function declarations -----------------------------------------------------*/
void setupWiFi(void);
void setupWebServer(void);
void handleRoot(void);
void handleAction(void);

void WALK(void);
void ROUND(void);
void flat_up(void);
void flat_dw(void);
void stall(void);
void BLINK(void);

void Afoup(void); void Afodw(void);
void Bfoup(void); void Bfodw(void);
void Cfoup(void); void Cfodw(void);
void Dfoup(void); void Dfodw(void);

void armAfw(void); void armAbw(void);
void armBfw(void); void armBbw(void);
void armCfw(void); void armCbw(void);
void armDfw(void); void armDbw(void);
void go_ahead(void);

/* Web Page HTML -------------------------------------------------------------*/
const char index_html[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html>
<head>
  <meta charset="UTF-8">
  <meta name="viewport" content="width=device-width, initial-scale=1.0, user-scalable=no">
  <title>Spider Robot</title>
  <style>
    * { box-sizing: border-box; margin: 0; padding: 0; }
    body {
      font-family: Arial, sans-serif;
      background: linear-gradient(135deg, #1a1a2e 0%, #16213e 100%);
      min-height: 100vh;
      display: flex;
      flex-direction: column;
      align-items: center;
      padding: 20px;
      color: white;
    }
    h1 {
      margin-bottom: 20px;
      font-size: 24px;
      text-shadow: 0 0 10px #00ff88;
    }
    .status {
      background: #0f3460;
      padding: 10px 20px;
      border-radius: 10px;
      margin-bottom: 20px;
      font-size: 14px;
    }
    .control-pad {
      display: grid;
      grid-template-columns: repeat(3, 70px);
      grid-template-rows: repeat(3, 70px);
      gap: 10px;
      margin-bottom: 20px;
    }
    .btn {
      width: 70px;
      height: 70px;
      border: none;
      border-radius: 15px;
      font-size: 24px;
      cursor: pointer;
      transition: all 0.2s;
      display: flex;
      align-items: center;
      justify-content: center;
    }
    .btn:active {
      transform: scale(0.95);
    }
    .btn-move {
      background: linear-gradient(145deg, #00ff88, #00cc6a);
      color: #1a1a2e;
    }
    .btn-stop {
      background: linear-gradient(145deg, #ff4757, #ff3344);
      color: white;
    }
    .btn-action {
      background: linear-gradient(145deg, #5f27cd, #341f97);
      color: white;
      width: 100%;
      height: 50px;
      font-size: 16px;
    }
    .btn-empty {
      background: transparent;
      cursor: default;
    }
    .action-grid {
      display: grid;
      grid-template-columns: repeat(2, 1fr);
      gap: 10px;
      width: 230px;
      margin-bottom: 20px;
    }
    .speed-control {
      width: 230px;
      background: #0f3460;
      padding: 15px;
      border-radius: 15px;
    }
    .speed-control label {
      display: block;
      margin-bottom: 10px;
      font-size: 14px;
    }
    .speed-control input[type="range"] {
      width: 100%;
      height: 20px;
      -webkit-appearance: none;
      background: #1a1a2e;
      border-radius: 10px;
      outline: none;
    }
    .speed-control input[type="range"]::-webkit-slider-thumb {
      -webkit-appearance: none;
      width: 25px;
      height: 25px;
      background: #00ff88;
      border-radius: 50%;
      cursor: pointer;
    }
    #statusText {
      color: #00ff88;
    }
  </style>
</head>
<body>
  <h1>🕷️ Spider Robot</h1>
  
  <div class="status">
    상태: <span id="statusText">대기중</span>
  </div>
  
  <div class="control-pad">
    <div class="btn btn-empty"></div>
    <button class="btn btn-move" onclick="sendCmd('walk')">▲</button>
    <div class="btn btn-empty"></div>
    
    <button class="btn btn-move" onclick="sendCmd('left')">◀</button>
    <button class="btn btn-stop" onclick="sendCmd('stop')">■</button>
    <button class="btn btn-move" onclick="sendCmd('right')">▶</button>
    
    <div class="btn btn-empty"></div>
    <button class="btn btn-move" onclick="sendCmd('walk')">▼</button>
    <div class="btn btn-empty"></div>
  </div>
  
  <div class="action-grid">
    <button class="btn btn-action" onclick="sendCmd('up')">⬆️ Up</button>
    <button class="btn btn-action" onclick="sendCmd('down')">⬇️ Down</button>
    <button class="btn btn-action" onclick="sendCmd('round')">🔄 Round</button>
    <button class="btn btn-action" onclick="sendCmd('blink')">💡 Blink</button>
    <button class="btn btn-action" onclick="sendCmd('demo')">🎬 Demo</button>
    <button class="btn btn-action" onclick="sendCmd('init')">🔧 Init</button>
  </div>
  
  <div class="speed-control">
    <label>속도: <span id="speedVal">3</span></label>
    <input type="range" min="1" max="10" step="1" value="3" 
           onchange="setSpeed(this.value)">
  </div>

  <script>
    function sendCmd(cmd) {
      fetch('/action?cmd=' + cmd)
        .then(response => response.text())
        .then(data => {
          document.getElementById('statusText').innerText = data;
        })
        .catch(err => {
          document.getElementById('statusText').innerText = '연결 오류';
        });
    }
    
    function setSpeed(val) {
      document.getElementById('speedVal').innerText = val;
      fetch('/action?cmd=speed&val=' + val);
    }
  </script>
</body>
</html>
)rawliteral";

/* Setup ---------------------------------------------------------------------*/
void setup() {
  Serial.begin(115200);
  Serial.println("Spider Robot starting...");
  
  // Servo 연결
  servo_footA.attach(srv_footA);
  servo_armA.attach(srv_armA);
  servo_footB.attach(srv_footB);
  servo_armB.attach(srv_armB);
  servo_footC.attach(srv_footC);
  servo_armC.attach(srv_armC);
  servo_footD.attach(srv_footD);
  servo_armD.attach(srv_armD);
  
  pinMode(ledPin, OUTPUT);
  
  // 초기 위치 설정
  stall();
  Serial.println("Servos initialized");
  
  // WiFi AP 모드 시작
  setupWiFi();
  
  // 웹서버 시작
  setupWebServer();
  
  Serial.println("Spider Robot ready!");
}

void setupWiFi() {
  Serial.println("Setting up WiFi AP...");
  WiFi.mode(WIFI_AP);
  WiFi.softAP(ap_ssid, ap_password);
  
  IPAddress IP = WiFi.softAPIP();
  Serial.print("AP IP address: ");
  Serial.println(IP);
  Serial.print("WiFi Name: ");
  Serial.println(ap_ssid);
  Serial.print("Password: ");
  Serial.println(ap_password);
}

void setupWebServer() {
  server.on("/", handleRoot);
  server.on("/action", handleAction);
  server.begin();
  Serial.println("Web server started on port 80");
}

void handleRoot() {
  server.send(200, "text/html", index_html);
}

/* 비동기 동작 실행 태스크 */
void actionTask(void * parameter) {
  int action = *((int*)parameter);
  delete (int*)parameter;
  
  action_running = true;
  digitalWrite(ledPin, HIGH);
  
  switch(action) {
    case 1: WALK(); break;           // walk
    case 2: ROUND(); break;          // left (round)
    case 3: ROUND(); break;          // right (round)
    case 4: flat_up(); break;        // up
    case 5: flat_dw(); flat_up(); break;  // down
    case 6: ROUND(); break;          // round
    case 7: BLINK(); break;          // blink
    case 8: stall(); break;          // stop/init
    case 9:                          // demo
      BLINK();
      for (int i = 0; i < 5; i++) { WALK(); }
      stall();
      flat_up();
      delay(500);
      flat_dw();
      delay(500);
      for (int i = 0; i < 5; i++) { ROUND(); }
      stall();
      break;
  }
  
  digitalWrite(ledPin, LOW);
  action_running = false;
  ActionTask = NULL;
  vTaskDelete(NULL);
}

void startAction(int actionCode) {
  if (action_running || ActionTask != NULL) {
    return;
  }
  
  int* param = new int(actionCode);
  xTaskCreatePinnedToCore(
    actionTask,
    "ActionTask",
    4096,
    param,
    1,
    &ActionTask,
    0
  );
}

void handleAction() {
  String cmd = server.arg("cmd");
  String response = "OK";
  
  Serial.print("Command: ");
  Serial.println(cmd);
  
  if (action_running) {
    response = "동작 중...";
  }
  else if (cmd == "walk") {
    response = "전진";
    startAction(1);
  }
  else if (cmd == "left") {
    response = "좌회전";
    startAction(2);
  }
  else if (cmd == "right") {
    response = "우회전";
    startAction(3);
  }
  else if (cmd == "up") {
    response = "일어서기";
    startAction(4);
  }
  else if (cmd == "down") {
    response = "앉기";
    startAction(5);
  }
  else if (cmd == "round") {
    response = "회전";
    startAction(6);
  }
  else if (cmd == "blink") {
    response = "LED 깜빡";
    startAction(7);
  }
  else if (cmd == "stop" || cmd == "init") {
    response = "정지";
    startAction(8);
  }
  else if (cmd == "demo") {
    response = "데모 시작";
    startAction(9);
  }
  else if (cmd == "speed") {
    int val = server.arg("val").toInt();
    spd = 11 - val;  // 1~10 → 10~1 (값이 클수록 빠름)
    response = "속도: " + String(val);
  }
  
  server.send(200, "text/plain", response);
}

/* Loop ----------------------------------------------------------------------*/
void loop() {
  server.handleClient();
  
  // 시리얼 명령어 처리
  if (Serial.available()) {
    String cmd = Serial.readStringUntil('\n');
    cmd.trim();
    Serial.print("Serial cmd: ");
    Serial.println(cmd);
    
    if (cmd == "off") {
      servo_footA.detach();
      servo_armA.detach();
      servo_footB.detach();
      servo_armB.detach();
      servo_footC.detach();
      servo_armC.detach();
      servo_footD.detach();
      servo_armD.detach();
      Serial.println("All servos OFF");
    }
    else if (cmd == "on") {
      servo_footA.attach(srv_footA);
      servo_armA.attach(srv_armA);
      servo_footB.attach(srv_footB);
      servo_armB.attach(srv_armB);
      servo_footC.attach(srv_footC);
      servo_armC.attach(srv_armC);
      servo_footD.attach(srv_footD);
      servo_armD.attach(srv_armD);
      stall();
      Serial.println("All servos ON");
    }
    else if (cmd == "test") {
      Serial.println("Testing all servos...");
      stall();
      delay(1000);
      flat_up();
      delay(1000);
      flat_dw();
      delay(1000);
      stall();
      Serial.println("Test done");
    }
    else if (cmd == "walk") { WALK(); }
    else if (cmd == "round") { ROUND(); }
    else if (cmd == "up") { flat_up(); }
    else if (cmd == "down") { flat_dw(); }
    else if (cmd == "stop") { stall(); }
    else if (cmd == "help") {
      Serial.println("Commands: off, on, test, walk, round, up, down, stop, help");
    }
  }
  
  delay(1);
}

/* Movement functions --------------------------------------------------------*/

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

void go_ahead() {
  for (int i = Afw; i >= Abw; i--) {
    servo_armA.write(i);
    servo_armD.write(i);
    int y = Abw + Afw - i;
    servo_armC.write(y);
    servo_armB.write(y);
    delay(spd);
  }
}

void flat_up() {
  int max = Fup + 5;
  int min = Fdw + 5;
  servo_footA.write(min);
  servo_footB.write(min);
  servo_footC.write(min);
  servo_footD.write(min);
  delay(300);
  for (int i = min; i <= max + 50; i++) {
    servo_footA.write(i);
    servo_footB.write(i);
    servo_footC.write(i);
    servo_footD.write(i);
    delay(spd);
  }
  delay(300);
}

void flat_dw() {
  int max = Fup + 5;
  int min = Fdw + 5;
  servo_footA.write(max);
  servo_footB.write(max);
  servo_footC.write(max);
  servo_footD.write(max);
  delay(300);
  for (int i = max; i >= min; i--) {
    servo_footA.write(i);
    servo_footB.write(i);
    servo_footC.write(i);
    servo_footD.write(i);
    delay(spd);
  }
  delay(300);
}

void stall() {
  BLINK();
  servo_armA.write(90);
  servo_footA.write(60);
  servo_armB.write(90);
  servo_footB.write(60);
  servo_armC.write(90);
  servo_footC.write(60);
  servo_armD.write(90);
  servo_footD.write(60);
  digitalWrite(ledPin, HIGH);
}

void BLINK() {
  for (int i = 0; i < 3; i++) {
    digitalWrite(ledPin, HIGH);
    delay(100);
    digitalWrite(ledPin, LOW);
    delay(100);
  }
}

/* Foot movements ------------------------------------------------------------*/
void Afoup() {
  int max = Fup + 5;
  int min = Fdw + 5;
  for (int i = min; i <= max; i++) {
    servo_footA.write(i);
    delay(spd);
  }
}

void Bfoup() {
  int max = Fup + 5;
  int min = Fdw + 5;
  for (int i = min; i <= max; i++) {
    servo_footB.write(i);
    delay(spd);
  }
}

void Cfoup() {
  int max = Fup + 5;
  int min = Fdw + 5;
  for (int i = min; i <= max; i++) {
    servo_footC.write(i);
    delay(spd);
  }
}

void Dfoup() {
  int max = Fup + 5;
  int min = Fdw + 5;
  for (int i = min; i <= max; i++) {
    servo_footD.write(i);
    delay(spd);
  }
}

void Afodw() {
  int max = Fup + 5;
  int min = Fdw + 5;
  for (int i = max; i >= min; i--) {
    servo_footA.write(i);
    delay(spd);
  }
}

void Bfodw() {
  int max = Fup + 5;
  int min = Fdw + 5;
  for (int i = max; i >= min; i--) {
    servo_footB.write(i);
    delay(spd);
  }
}

void Cfodw() {
  int max = Fup + 5;
  int min = Fdw + 5;
  for (int i = max; i >= min; i--) {
    servo_footC.write(i);
    delay(spd);
  }
}

void Dfodw() {
  int max = Fup + 5;
  int min = Fdw + 5;
  for (int i = max; i >= min; i--) {
    servo_footD.write(i);
    delay(spd);
  }
}

/* Arm movements -------------------------------------------------------------*/
void armAfw() {
  int min = Abw;
  int max = Afw;
  for (int i = min; i <= max; i++) {
    servo_armA.write(i);
    delay(spd);
  }
}

void armAbw() {
  int min = Abw;
  int max = Afw;
  for (int i = max; i >= min; i--) {
    servo_armA.write(i);
    delay(spd);
  }
}

void armBfw() {
  int min = Abw;
  int max = Afw;
  for (int i = min; i <= max; i++) {
    servo_armB.write(i);
    delay(spd);
  }
}

void armBbw() {
  int min = Abw;
  int max = Afw;
  for (int i = max; i >= min; i--) {
    servo_armB.write(i);
    delay(spd);
  }
}

void armCfw() {
  int min = Abw;
  int max = Afw;
  for (int i = min; i <= max; i++) {
    servo_armC.write(i);
    delay(spd);
  }
}

void armCbw() {
  int min = Abw;
  int max = Afw;
  for (int i = max; i >= min; i--) {
    servo_armC.write(i);
    delay(spd);
  }
}

void armDfw() {
  int min = Abw;
  int max = Afw;
  for (int i = min; i <= max; i++) {
    servo_armD.write(i);
    delay(spd);
  }
}

void armDbw() {
  int min = Abw;
  int max = Afw;
  for (int i = max; i >= min; i--) {
    servo_armD.write(i);
    delay(spd);
  }
}