/* -----------------------------------------------------------------------------
  - Project: Remote control Crawling robot (ESP32 Web Control Version)
  - Original Author: panerqiang@sunfounder.com
  - Modified for ESP32 + Web GUI by: Claude
  - Date: 2025
   -----------------------------------------------------------------------------*/

#include <Arduino.h>
#include <WiFi.h>
#include <WebServer.h>
#include <ESP32Servo.h>
#include <Ticker.h>

/* WiFi AP Mode 설정 --------------------------------------------------------*/
const char* ap_ssid = "SpiderRobot";      // WiFi 이름
const char* ap_password = "12345678";      // 비밀번호 (8자 이상)

WebServer server(80);

/* Servos --------------------------------------------------------------------*/
Servo servo[4][3];

const int servo_pin[4][3] = { 
  {13, 12, 14},   // Leg 0
  {27, 26, 25},   // Leg 1
  {33, 32, 15},   // Leg 2
  {4,  16, 17}    // Leg 3
};

Ticker servoTimer;

/* Size of the robot ---------------------------------------------------------*/
const float length_a = 55;
const float length_b = 77.5;
const float length_c = 27.5;
const float length_side = 71;
const float z_absolute = -28;

/* Constants for movement ----------------------------------------------------*/
const float z_default = -50, z_up = -30, z_boot = z_absolute;
const float x_default = 62, x_offset = 0;
const float y_start = 0, y_step = 40;
const float y_default = x_default;

/* variables for movement ----------------------------------------------------*/
volatile float site_now[4][3];
volatile float site_expect[4][3];
float temp_speed[4][3];
float move_speed;
float speed_multiple = 1;
const float spot_turn_speed = 4;
const float leg_move_speed = 8;
const float body_move_speed = 3;
const float stand_seat_speed = 1;
volatile int rest_counter;

const float KEEP = 255;
const float pi = 3.1415926;

/* Constants for turn --------------------------------------------------------*/
const float temp_a = sqrt(pow(2 * x_default + length_side, 2) + pow(y_step, 2));
const float temp_b = 2 * (y_start + y_step) + length_side;
const float temp_c = sqrt(pow(2 * x_default + length_side, 2) + pow(2 * y_start + y_step + length_side, 2));
const float temp_alpha = acos((pow(temp_a, 2) + pow(temp_b, 2) - pow(temp_c, 2)) / 2 / temp_a / temp_b);

const float turn_x1 = (temp_a - length_side) / 2;
const float turn_y1 = y_start + y_step / 2;
const float turn_x0 = turn_x1 - temp_b * cos(temp_alpha);
const float turn_y0 = temp_b * sin(temp_alpha) - turn_y1 - length_side;

/* 비동기 동작을 위한 변수 */
volatile bool action_running = false;
TaskHandle_t ActionTask = NULL;

/* Function declarations -----------------------------------------------------*/
void servo_service(void);
void servo_attach(void);
void servo_detach(void);
void set_site(int leg, float x, float y, float z);
void wait_reach(int leg);
void wait_all_reach(void);
void sit(void);
void stand(void);
void turn_left(unsigned int step);
void turn_right(unsigned int step);
void step_forward(unsigned int step);
void step_back(unsigned int step);
void body_left(int i);
void body_right(int i);
void hand_wave(int i);
void hand_shake(int i);
void head_up(int i);
void head_down(int i);
void body_dance(int i);
void cartesian_to_polar(volatile float &alpha, volatile float &beta, volatile float &gamma, volatile float x, volatile float y, volatile float z);
void polar_to_servo(int leg, float alpha, float beta, float gamma);

void setupWiFi(void);
void setupWebServer(void);
void handleRoot(void);
void handleAction(void);

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
    <button class="btn btn-move" onclick="sendCmd('forward')">▲</button>
    <div class="btn btn-empty"></div>
    
    <button class="btn btn-move" onclick="sendCmd('left')">◀</button>
    <button class="btn btn-stop" onclick="sendCmd('stop')">■</button>
    <button class="btn btn-move" onclick="sendCmd('right')">▶</button>
    
    <div class="btn btn-empty"></div>
    <button class="btn btn-move" onclick="sendCmd('back')">▼</button>
    <div class="btn btn-empty"></div>
  </div>
  
  <div class="action-grid">
    <button class="btn btn-action" onclick="sendCmd('stand')">🧍 Stand</button>
    <button class="btn btn-action" onclick="sendCmd('sit')">🪑 Sit</button>
    <button class="btn btn-action" onclick="sendCmd('wave')">👋 Wave</button>
    <button class="btn btn-action" onclick="sendCmd('shake')">🤝 Shake</button>
    <button class="btn btn-action" onclick="sendCmd('dance')">💃 Dance</button>
    <button class="btn btn-action" onclick="sendCmd('headup')">⬆️ Head Up</button>
  </div>
  
  <div class="speed-control">
    <label>속도: <span id="speedVal">1.0</span>x</label>
    <input type="range" min="0.5" max="2.0" step="0.1" value="1.0" 
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
void setup()
{
  Serial.begin(115200);
  Serial.println("Robot starts initialization");
  
  // ESP32 서보 설정
  ESP32PWM::allocateTimer(0);
  ESP32PWM::allocateTimer(1);
  ESP32PWM::allocateTimer(2);
  ESP32PWM::allocateTimer(3);
  
  // initialize default parameter
  set_site(0, x_default - x_offset, y_start + y_step, z_boot);
  set_site(1, x_default - x_offset, y_start + y_step, z_boot);
  set_site(2, x_default + x_offset, y_start, z_boot);
  set_site(3, x_default + x_offset, y_start, z_boot);
  
  for (int i = 0; i < 4; i++)
  {
    for (int j = 0; j < 3; j++)
    {
      site_now[i][j] = site_expect[i][j];
    }
  }
  
  // start servo service (20ms = 50Hz)
  servoTimer.attach_ms(20, servo_service);
  Serial.println("Servo service started");
  
  // initialize servos
  servo_attach();
  Serial.println("Servos initialized");
  
  // WiFi AP 모드 시작
  setupWiFi();
  
  // 웹서버 시작
  setupWebServer();
  
  Serial.println("Robot initialization Complete");
}

void setupWiFi()
{
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

void setupWebServer()
{
  server.on("/", handleRoot);
  server.on("/action", handleAction);
  server.begin();
  Serial.println("Web server started on port 80");
}

void handleRoot()
{
  server.send(200, "text/html", index_html);
}

/* 비동기 동작 실행 태스크 */
void actionTask(void * parameter) {
  int action = *((int*)parameter);
  delete (int*)parameter;
  
  action_running = true;
  
  switch(action) {
    case 1: step_forward(2); break;
    case 2: step_back(2); break;
    case 3: turn_left(2); break;
    case 4: turn_right(2); break;
    case 5: stand(); break;
    case 6: sit(); break;
    case 7: hand_wave(2); break;
    case 8: hand_shake(2); break;
    case 9: body_dance(5); break;
    case 10: stand(); head_up(15); break;
  }
  
  action_running = false;
  ActionTask = NULL;
  vTaskDelete(NULL);
}

void startAction(int actionCode) {
  // 이미 동작 중이면 무시
  if (action_running || ActionTask != NULL) {
    return;
  }
  
  int* param = new int(actionCode);
  xTaskCreatePinnedToCore(
    actionTask,      // 함수
    "ActionTask",    // 이름
    4096,            // 스택 크기
    param,           // 파라미터
    1,               // 우선순위
    &ActionTask,     // 태스크 핸들
    0                // 코어 0에서 실행
  );
}

void handleAction()
{
  String cmd = server.arg("cmd");
  String response = "OK";
  
  Serial.print("Command received: ");
  Serial.println(cmd);
  
  if (action_running) {
    response = "동작 중...";
  }
  else if (cmd == "forward") {
    response = "전진";
    startAction(1);
  }
  else if (cmd == "back") {
    response = "후진";
    startAction(2);
  }
  else if (cmd == "left") {
    response = "좌회전";
    startAction(3);
  }
  else if (cmd == "right") {
    response = "우회전";
    startAction(4);
  }
  else if (cmd == "stop") {
    response = "정지";
    startAction(5);
  }
  else if (cmd == "stand") {
    response = "일어서기";
    startAction(5);
  }
  else if (cmd == "sit") {
    response = "앉기";
    startAction(6);
  }
  else if (cmd == "wave") {
    response = "손 흔들기";
    startAction(7);
  }
  else if (cmd == "shake") {
    response = "악수";
    startAction(8);
  }
  else if (cmd == "dance") {
    response = "춤추기";
    startAction(9);
  }
  else if (cmd == "headup") {
    response = "고개 들기";
    startAction(10);
  }
  else if (cmd == "speed") {
    float val = server.arg("val").toFloat();
    speed_multiple = val;
    response = "속도: " + String(val) + "x";
  }
  
  server.send(200, "text/plain", response);
}

void servo_attach(void)
{
  for (int i = 0; i < 4; i++)
  {
    for (int j = 0; j < 3; j++)
    {
      servo[i][j].setPeriodHertz(50);
      servo[i][j].attach(servo_pin[i][j], 500, 2400);
      delay(100);
    }
  }
}

void servo_detach(void)
{
  for (int i = 0; i < 4; i++)
  {
    for (int j = 0; j < 3; j++)
    {
      servo[i][j].detach();
      delay(100);
    }
  }
}

/* Loop ----------------------------------------------------------------------*/
void loop()
{
  server.handleClient();  // 웹 요청 처리
  
  // 시리얼 명령어 처리
  if (Serial.available()) {
    String cmd = Serial.readStringUntil('\n');
    cmd.trim();
    Serial.print("Serial cmd: ");
    Serial.println(cmd);
    
    if (cmd == "tstop") {
      // 타이머 정지 - 서보 개별 테스트용
      servoTimer.detach();
      Serial.println("Timer STOPPED - manual control enabled");
    }
    else if (cmd == "tstart") {
      // 타이머 재시작
      servoTimer.attach_ms(20, servo_service);
      Serial.println("Timer STARTED");
    }
    else if (cmd == "off") {
      // 모든 서보 신호 끄기
      servoTimer.detach();
      for (int i = 0; i < 4; i++) {
        for (int j = 0; j < 3; j++) {
          servo[i][j].detach();
        }
      }
      Serial.println("All servos OFF - no PWM signal");
    }
    else if (cmd == "on") {
      // 모든 서보 다시 연결
      servo_attach();
      servoTimer.attach_ms(20, servo_service);
      Serial.println("All servos ON");
    }
    else if (cmd == "test") {
      // GPIO13 서보 직접 테스트 (0~180도 왕복)
      servoTimer.detach();  // 먼저 타이머 정지
      Serial.println("Servo test on GPIO13 (servo[0][0])...");
      servo[0][0].write(0);
      Serial.println("0 deg");
      delay(2000);
      servo[0][0].write(90);
      Serial.println("90 deg");
      delay(2000);
      servo[0][0].write(180);
      Serial.println("180 deg");
      delay(2000);
      servo[0][0].write(90);
      Serial.println("Test done");
    }
    else if (cmd == "all") {
      // 모든 서보 테스트
      servoTimer.detach();
      Serial.println("Testing ALL servos 0->90->180->90...");
      for (int i = 0; i < 4; i++) {
        for (int j = 0; j < 3; j++) {
          servo[i][j].write(0);
        }
      }
      Serial.println("All 0 deg");
      delay(2000);
      for (int i = 0; i < 4; i++) {
        for (int j = 0; j < 3; j++) {
          servo[i][j].write(90);
        }
      }
      Serial.println("All 90 deg");
      delay(2000);
      for (int i = 0; i < 4; i++) {
        for (int j = 0; j < 3; j++) {
          servo[i][j].write(180);
        }
      }
      Serial.println("All 180 deg");
      delay(2000);
      for (int i = 0; i < 4; i++) {
        for (int j = 0; j < 3; j++) {
          servo[i][j].write(90);
        }
      }
      Serial.println("All test done");
    }
    // 개별 서보 제어: leg.joint.angle (예: 1.0.90 = leg1, joint0, 90도)
    else if (cmd.indexOf('.') > 0) {
      int leg = cmd.substring(0, 1).toInt();
      int joint = cmd.substring(2, 3).toInt();
      int angle = cmd.substring(4).toInt();
      if (leg >= 0 && leg < 4 && joint >= 0 && joint < 3 && angle >= 0 && angle <= 180) {
        servo[leg][joint].write(angle);
        Serial.printf("Servo[%d][%d] (GPIO%d) -> %d deg\n", leg, joint, servo_pin[leg][joint], angle);
      } else {
        Serial.println("Invalid: use leg.joint.angle (e.g., 1.0.90)");
      }
    }
    else if (cmd == "s0") { servo[0][0].write(0); Serial.println("Servo[0][0] -> 0"); }
    else if (cmd == "s90") { servo[0][0].write(90); Serial.println("Servo[0][0] -> 90"); }
    else if (cmd == "s180") { servo[0][0].write(180); Serial.println("Servo[0][0] -> 180"); }
    else if (cmd == "stand") { stand(); Serial.println("Stand done"); }
    else if (cmd == "sit") { sit(); Serial.println("Sit done"); }
    else if (cmd == "pins") {
      Serial.println("Servo pin mapping:");
      for (int i = 0; i < 4; i++) {
        Serial.printf("Leg %d: GPIO %d, %d, %d\n", i, servo_pin[i][0], servo_pin[i][1], servo_pin[i][2]);
      }
    }
    else if (cmd == "help") {
      Serial.println("Commands:");
      Serial.println("  tstop   - Stop timer (manual mode)");
      Serial.println("  tstart  - Start timer");
      Serial.println("  test    - Test GPIO13");
      Serial.println("  all     - Test all 12 servos");
      Serial.println("  pins    - Show pin mapping");
      Serial.println("  1.0.90  - Set leg.joint.angle");
      Serial.println("  s0/s90/s180 - Quick servo[0][0] control");
    }
  }
  
  delay(1);
}

/* Movement functions --------------------------------------------------------*/
void sit(void)
{
  move_speed = stand_seat_speed;
  for (int leg = 0; leg < 4; leg++)
  {
    set_site(leg, KEEP, KEEP, z_boot);
  }
  wait_all_reach();
}

void stand(void)
{
  move_speed = stand_seat_speed;
  for (int leg = 0; leg < 4; leg++)
  {
    set_site(leg, KEEP, KEEP, z_default);
  }
  wait_all_reach();
}

void turn_left(unsigned int step)
{
  move_speed = spot_turn_speed;
  while (step-- > 0)
  {
    if (site_now[3][1] == y_start)
    {
      set_site(3, x_default + x_offset, y_start, z_up);
      wait_all_reach();

      set_site(0, turn_x1 - x_offset, turn_y1, z_default);
      set_site(1, turn_x0 - x_offset, turn_y0, z_default);
      set_site(2, turn_x1 + x_offset, turn_y1, z_default);
      set_site(3, turn_x0 + x_offset, turn_y0, z_up);
      wait_all_reach();

      set_site(3, turn_x0 + x_offset, turn_y0, z_default);
      wait_all_reach();

      set_site(0, turn_x1 + x_offset, turn_y1, z_default);
      set_site(1, turn_x0 + x_offset, turn_y0, z_default);
      set_site(2, turn_x1 - x_offset, turn_y1, z_default);
      set_site(3, turn_x0 - x_offset, turn_y0, z_default);
      wait_all_reach();

      set_site(1, turn_x0 + x_offset, turn_y0, z_up);
      wait_all_reach();

      set_site(0, x_default + x_offset, y_start, z_default);
      set_site(1, x_default + x_offset, y_start, z_up);
      set_site(2, x_default - x_offset, y_start + y_step, z_default);
      set_site(3, x_default - x_offset, y_start + y_step, z_default);
      wait_all_reach();

      set_site(1, x_default + x_offset, y_start, z_default);
      wait_all_reach();
    }
    else
    {
      set_site(0, x_default + x_offset, y_start, z_up);
      wait_all_reach();

      set_site(0, turn_x0 + x_offset, turn_y0, z_up);
      set_site(1, turn_x1 + x_offset, turn_y1, z_default);
      set_site(2, turn_x0 - x_offset, turn_y0, z_default);
      set_site(3, turn_x1 - x_offset, turn_y1, z_default);
      wait_all_reach();

      set_site(0, turn_x0 + x_offset, turn_y0, z_default);
      wait_all_reach();

      set_site(0, turn_x0 - x_offset, turn_y0, z_default);
      set_site(1, turn_x1 - x_offset, turn_y1, z_default);
      set_site(2, turn_x0 + x_offset, turn_y0, z_default);
      set_site(3, turn_x1 + x_offset, turn_y1, z_default);
      wait_all_reach();

      set_site(2, turn_x0 + x_offset, turn_y0, z_up);
      wait_all_reach();

      set_site(0, x_default - x_offset, y_start + y_step, z_default);
      set_site(1, x_default - x_offset, y_start + y_step, z_default);
      set_site(2, x_default + x_offset, y_start, z_up);
      set_site(3, x_default + x_offset, y_start, z_default);
      wait_all_reach();

      set_site(2, x_default + x_offset, y_start, z_default);
      wait_all_reach();
    }
  }
}

void turn_right(unsigned int step)
{
  move_speed = spot_turn_speed;
  while (step-- > 0)
  {
    if (site_now[2][1] == y_start)
    {
      set_site(2, x_default + x_offset, y_start, z_up);
      wait_all_reach();

      set_site(0, turn_x0 - x_offset, turn_y0, z_default);
      set_site(1, turn_x1 - x_offset, turn_y1, z_default);
      set_site(2, turn_x0 + x_offset, turn_y0, z_up);
      set_site(3, turn_x1 + x_offset, turn_y1, z_default);
      wait_all_reach();

      set_site(2, turn_x0 + x_offset, turn_y0, z_default);
      wait_all_reach();

      set_site(0, turn_x0 + x_offset, turn_y0, z_default);
      set_site(1, turn_x1 + x_offset, turn_y1, z_default);
      set_site(2, turn_x0 - x_offset, turn_y0, z_default);
      set_site(3, turn_x1 - x_offset, turn_y1, z_default);
      wait_all_reach();

      set_site(0, turn_x0 + x_offset, turn_y0, z_up);
      wait_all_reach();

      set_site(0, x_default + x_offset, y_start, z_up);
      set_site(1, x_default - x_offset, y_start + y_step, z_default);
      set_site(2, x_default - x_offset, y_start + y_step, z_default);
      set_site(3, x_default + x_offset, y_start, z_default);
      wait_all_reach();

      set_site(0, x_default + x_offset, y_start, z_default);
      wait_all_reach();
    }
    else
    {
      set_site(1, x_default + x_offset, y_start, z_up);
      wait_all_reach();

      set_site(0, turn_x1 + x_offset, turn_y1, z_default);
      set_site(1, turn_x0 + x_offset, turn_y0, z_up);
      set_site(2, turn_x1 - x_offset, turn_y1, z_default);
      set_site(3, turn_x0 - x_offset, turn_y0, z_default);
      wait_all_reach();

      set_site(1, turn_x0 + x_offset, turn_y0, z_default);
      wait_all_reach();

      set_site(0, turn_x1 - x_offset, turn_y1, z_default);
      set_site(1, turn_x0 - x_offset, turn_y0, z_default);
      set_site(2, turn_x1 + x_offset, turn_y1, z_default);
      set_site(3, turn_x0 + x_offset, turn_y0, z_default);
      wait_all_reach();

      set_site(3, turn_x0 + x_offset, turn_y0, z_up);
      wait_all_reach();

      set_site(0, x_default - x_offset, y_start + y_step, z_default);
      set_site(1, x_default - x_offset, y_start + y_step, z_default);
      set_site(2, x_default + x_offset, y_start, z_default);
      set_site(3, x_default + x_offset, y_start, z_up);
      wait_all_reach();

      set_site(3, x_default + x_offset, y_start, z_default);
      wait_all_reach();
    }
  }
}

void step_forward(unsigned int step)
{
  move_speed = leg_move_speed;
  while (step-- > 0)
  {
    if (site_now[2][1] == y_start)
    {
      set_site(2, x_default + x_offset, y_start, z_up);
      wait_all_reach();
      set_site(2, x_default + x_offset, y_start + 2 * y_step, z_up);
      wait_all_reach();
      set_site(2, x_default + x_offset, y_start + 2 * y_step, z_default);
      wait_all_reach();

      move_speed = body_move_speed;

      set_site(0, x_default - x_offset, y_start, z_default);
      set_site(1, x_default - x_offset, y_start, z_default);
      set_site(2, x_default + x_offset, y_start + y_step, z_default);
      set_site(3, x_default + x_offset, y_start + y_step, z_default);
      wait_all_reach();

      move_speed = leg_move_speed;

      set_site(1, x_default - x_offset, y_start, z_up);
      wait_all_reach();
      set_site(1, x_default - x_offset, y_start + 2 * y_step, z_up);
      wait_all_reach();
      set_site(1, x_default - x_offset, y_start + 2 * y_step, z_default);
      wait_all_reach();

      move_speed = body_move_speed;

      set_site(0, x_default - x_offset, y_start + y_step, z_default);
      set_site(1, x_default - x_offset, y_start + y_step, z_default);
      set_site(2, x_default + x_offset, y_start, z_default);
      set_site(3, x_default + x_offset, y_start, z_default);
      wait_all_reach();

      move_speed = leg_move_speed;
    }
    else
    {
      set_site(0, x_default - x_offset, y_start + y_step, z_up);
      wait_all_reach();
      set_site(0, x_default - x_offset, y_start + 2 * y_step + y_step, z_up);
      wait_all_reach();
      set_site(0, x_default - x_offset, y_start + 2 * y_step + y_step, z_default);
      wait_all_reach();

      move_speed = body_move_speed;

      set_site(0, x_default - x_offset, y_start + 2 * y_step, z_default);
      set_site(1, x_default - x_offset, y_start + 2 * y_step, z_default);
      set_site(2, x_default + x_offset, y_start + y_step, z_default);
      set_site(3, x_default + x_offset, y_start + y_step, z_default);
      wait_all_reach();

      move_speed = leg_move_speed;

      set_site(3, x_default + x_offset, y_start + y_step, z_up);
      wait_all_reach();
      set_site(3, x_default + x_offset, y_start + 2 * y_step + y_step, z_up);
      wait_all_reach();
      set_site(3, x_default + x_offset, y_start + 2 * y_step + y_step, z_default);
      wait_all_reach();

      move_speed = body_move_speed;

      set_site(0, x_default - x_offset, y_start + y_step, z_default);
      set_site(1, x_default - x_offset, y_start + y_step, z_default);
      set_site(2, x_default + x_offset, y_start, z_default);
      set_site(3, x_default + x_offset, y_start, z_default);
      wait_all_reach();

      move_speed = leg_move_speed;
    }
  }
}

void step_back(unsigned int step)
{
  move_speed = leg_move_speed;
  while (step-- > 0)
  {
    if (site_now[3][1] == y_start)
    {
      set_site(3, x_default + x_offset, y_start, z_up);
      wait_all_reach();
      set_site(3, x_default + x_offset, y_start + 2 * y_step, z_up);
      wait_all_reach();
      set_site(3, x_default + x_offset, y_start + 2 * y_step, z_default);
      wait_all_reach();

      move_speed = body_move_speed;

      set_site(0, x_default - x_offset, y_start + 2 * y_step, z_default);
      set_site(1, x_default - x_offset, y_start + 2 * y_step, z_default);
      set_site(2, x_default + x_offset, y_start + y_step, z_default);
      set_site(3, x_default + x_offset, y_start + y_step, z_default);
      wait_all_reach();

      move_speed = leg_move_speed;

      set_site(0, x_default - x_offset, y_start + 2 * y_step, z_up);
      wait_all_reach();
      set_site(0, x_default - x_offset, y_start, z_up);
      wait_all_reach();
      set_site(0, x_default - x_offset, y_start, z_default);
      wait_all_reach();

      move_speed = body_move_speed;

      set_site(0, x_default - x_offset, y_start + y_step, z_default);
      set_site(1, x_default - x_offset, y_start + y_step, z_default);
      set_site(2, x_default + x_offset, y_start, z_default);
      set_site(3, x_default + x_offset, y_start, z_default);
      wait_all_reach();

      move_speed = leg_move_speed;
    }
    else
    {
      set_site(1, x_default - x_offset, y_start + y_step, z_up);
      wait_all_reach();
      set_site(1, x_default - x_offset, y_start - y_step, z_up);
      wait_all_reach();
      set_site(1, x_default - x_offset, y_start - y_step, z_default);
      wait_all_reach();

      move_speed = body_move_speed;

      set_site(0, x_default - x_offset, y_start, z_default);
      set_site(1, x_default - x_offset, y_start, z_default);
      set_site(2, x_default + x_offset, y_start + y_step, z_default);
      set_site(3, x_default + x_offset, y_start + y_step, z_default);
      wait_all_reach();

      move_speed = leg_move_speed;

      set_site(2, x_default + x_offset, y_start + y_step, z_up);
      wait_all_reach();
      set_site(2, x_default + x_offset, y_start - y_step, z_up);
      wait_all_reach();
      set_site(2, x_default + x_offset, y_start - y_step, z_default);
      wait_all_reach();

      move_speed = body_move_speed;

      set_site(0, x_default - x_offset, y_start + y_step, z_default);
      set_site(1, x_default - x_offset, y_start + y_step, z_default);
      set_site(2, x_default + x_offset, y_start, z_default);
      set_site(3, x_default + x_offset, y_start, z_default);
      wait_all_reach();

      move_speed = leg_move_speed;
    }
  }
}

void body_left(int i)
{
  set_site(0, site_now[0][0] + i, KEEP, KEEP);
  set_site(1, site_now[1][0] + i, KEEP, KEEP);
  set_site(2, site_now[2][0] - i, KEEP, KEEP);
  set_site(3, site_now[3][0] - i, KEEP, KEEP);
  wait_all_reach();
}

void body_right(int i)
{
  set_site(0, site_now[0][0] - i, KEEP, KEEP);
  set_site(1, site_now[1][0] - i, KEEP, KEEP);
  set_site(2, site_now[2][0] + i, KEEP, KEEP);
  set_site(3, site_now[3][0] + i, KEEP, KEEP);
  wait_all_reach();
}

void hand_wave(int i)
{
  float x_tmp;
  float y_tmp;
  float z_tmp;
  move_speed = 1;
  if (site_now[3][1] == y_start)
  {
    body_right(15);
    x_tmp = site_now[2][0];
    y_tmp = site_now[2][1];
    z_tmp = site_now[2][2];
    move_speed = body_move_speed;
    for (int j = 0; j < i; j++)
    {
      set_site(2, turn_x1, turn_y1, 50);
      wait_all_reach();
      set_site(2, turn_x0, turn_y0, 50);
      wait_all_reach();
    }
    set_site(2, x_tmp, y_tmp, z_tmp);
    wait_all_reach();
    move_speed = 1;
    body_left(15);
  }
  else
  {
    body_left(15);
    x_tmp = site_now[0][0];
    y_tmp = site_now[0][1];
    z_tmp = site_now[0][2];
    move_speed = body_move_speed;
    for (int j = 0; j < i; j++)
    {
      set_site(0, turn_x1, turn_y1, 50);
      wait_all_reach();
      set_site(0, turn_x0, turn_y0, 50);
      wait_all_reach();
    }
    set_site(0, x_tmp, y_tmp, z_tmp);
    wait_all_reach();
    move_speed = 1;
    body_right(15);
  }
}

void hand_shake(int i)
{
  float x_tmp;
  float y_tmp;
  float z_tmp;
  move_speed = 1;
  if (site_now[3][1] == y_start)
  {
    body_right(15);
    x_tmp = site_now[2][0];
    y_tmp = site_now[2][1];
    z_tmp = site_now[2][2];
    move_speed = body_move_speed;
    for (int j = 0; j < i; j++)
    {
      set_site(2, x_default - 30, y_start + 2 * y_step, 55);
      wait_all_reach();
      set_site(2, x_default - 30, y_start + 2 * y_step, 10);
      wait_all_reach();
    }
    set_site(2, x_tmp, y_tmp, z_tmp);
    wait_all_reach();
    move_speed = 1;
    body_left(15);
  }
  else
  {
    body_left(15);
    x_tmp = site_now[0][0];
    y_tmp = site_now[0][1];
    z_tmp = site_now[0][2];
    move_speed = body_move_speed;
    for (int j = 0; j < i; j++)
    {
      set_site(0, x_default - 30, y_start + 2 * y_step, 55);
      wait_all_reach();
      set_site(0, x_default - 30, y_start + 2 * y_step, 10);
      wait_all_reach();
    }
    set_site(0, x_tmp, y_tmp, z_tmp);
    wait_all_reach();
    move_speed = 1;
    body_right(15);
  }
}

void head_up(int i)
{
  set_site(0, KEEP, KEEP, site_now[0][2] - i);
  set_site(1, KEEP, KEEP, site_now[1][2] + i);
  set_site(2, KEEP, KEEP, site_now[2][2] - i);
  set_site(3, KEEP, KEEP, site_now[3][2] + i);
  wait_all_reach();
}

void head_down(int i)
{
  set_site(0, KEEP, KEEP, site_now[0][2] + i);
  set_site(1, KEEP, KEEP, site_now[1][2] - i);
  set_site(2, KEEP, KEEP, site_now[2][2] + i);
  set_site(3, KEEP, KEEP, site_now[3][2] - i);
  wait_all_reach();
}

void body_dance(int i)
{
  float body_dance_speed = 2;
  sit();
  move_speed = 1;
  set_site(0, x_default, y_default, KEEP);
  set_site(1, x_default, y_default, KEEP);
  set_site(2, x_default, y_default, KEEP);
  set_site(3, x_default, y_default, KEEP);
  wait_all_reach();
  
  set_site(0, x_default, y_default, z_default - 20);
  set_site(1, x_default, y_default, z_default - 20);
  set_site(2, x_default, y_default, z_default - 20);
  set_site(3, x_default, y_default, z_default - 20);
  wait_all_reach();
  move_speed = body_dance_speed;
  head_up(30);
  
  for (int j = 0; j < i; j++)
  {
    if (j > i / 4)
      move_speed = body_dance_speed * 2;
    if (j > i / 2)
      move_speed = body_dance_speed * 3;
    set_site(0, KEEP, y_default - 20, KEEP);
    set_site(1, KEEP, y_default + 20, KEEP);
    set_site(2, KEEP, y_default - 20, KEEP);
    set_site(3, KEEP, y_default + 20, KEEP);
    wait_all_reach();
    set_site(0, KEEP, y_default + 20, KEEP);
    set_site(1, KEEP, y_default - 20, KEEP);
    set_site(2, KEEP, y_default + 20, KEEP);
    set_site(3, KEEP, y_default - 20, KEEP);
    wait_all_reach();
  }
  move_speed = body_dance_speed;
  head_down(30);
}

/* Servo service -------------------------------------------------------------*/
#define E_DELTA 0.01
void servo_service(void)
{
  static float alpha, beta, gamma;

  for (int i = 0; i < 4; i++)
  {
    for (int j = 0; j < 3; j++)
    {
      if (abs(site_now[i][j] - site_expect[i][j]) < (abs(temp_speed[i][j]) + E_DELTA))
        site_now[i][j] = site_expect[i][j];
      else
        site_now[i][j] += temp_speed[i][j];
    }

    cartesian_to_polar(alpha, beta, gamma, site_now[i][0], site_now[i][1], site_now[i][2]);
    polar_to_servo(i, alpha, beta, gamma);
  }

  rest_counter++;
}

/* Set site ------------------------------------------------------------------*/
void set_site(int leg, float x, float y, float z)
{
  float length_x = 0, length_y = 0, length_z = 0;

  if (x != KEEP)
    length_x = x - site_now[leg][0];
  if (y != KEEP)
    length_y = y - site_now[leg][1];
  if (z != KEEP)
    length_z = z - site_now[leg][2];

  float length = sqrt(pow(length_x, 2) + pow(length_y, 2) + pow(length_z, 2));

  temp_speed[leg][0] = length_x / length * move_speed * speed_multiple;
  temp_speed[leg][1] = length_y / length * move_speed * speed_multiple;
  temp_speed[leg][2] = length_z / length * move_speed * speed_multiple;

  if (x != KEEP)
    site_expect[leg][0] = x;
  if (y != KEEP)
    site_expect[leg][1] = y;
  if (z != KEEP)
    site_expect[leg][2] = z;
}

/* Wait functions ------------------------------------------------------------*/
#define WAIT_TIMEOUT_MS 3000  // 3초 타임아웃

void wait_reach(int leg)
{
  unsigned long start = millis();
  while (millis() - start < WAIT_TIMEOUT_MS)
  {
    if (site_now[leg][0] == site_expect[leg][0])
      if (site_now[leg][1] == site_expect[leg][1])
        if (site_now[leg][2] == site_expect[leg][2])
          return;
    delay(1);  // 다른 태스크에 CPU 양보
  }
  // 타임아웃 - 그냥 진행
}

void wait_all_reach(void)
{
  unsigned long start = millis();
  for (int i = 0; i < 4; i++)
  {
    // 남은 시간 계산
    unsigned long elapsed = millis() - start;
    if (elapsed >= WAIT_TIMEOUT_MS) return;
    
    unsigned long remaining = WAIT_TIMEOUT_MS - elapsed;
    unsigned long legStart = millis();
    
    while (millis() - legStart < remaining)
    {
      if (site_now[i][0] == site_expect[i][0])
        if (site_now[i][1] == site_expect[i][1])
          if (site_now[i][2] == site_expect[i][2])
            break;
      delay(1);
    }
  }
}

/* Coordinate transformation -------------------------------------------------*/
void cartesian_to_polar(volatile float &alpha, volatile float &beta, volatile float &gamma, volatile float x, volatile float y, volatile float z)
{
  float v, w;
  w = (x >= 0 ? 1 : -1) * (sqrt(pow(x, 2) + pow(y, 2)));
  v = w - length_c;
  alpha = atan2(z, v) + acos((pow(length_a, 2) - pow(length_b, 2) + pow(v, 2) + pow(z, 2)) / 2 / length_a / sqrt(pow(v, 2) + pow(z, 2)));
  beta = acos((pow(length_a, 2) + pow(length_b, 2) - pow(v, 2) - pow(z, 2)) / 2 / length_a / length_b);
  gamma = (w >= 0) ? atan2(y, x) : atan2(-y, -x);
  
  alpha = alpha / pi * 180;
  beta = beta / pi * 180;
  gamma = gamma / pi * 180;
}

void polar_to_servo(int leg, float alpha, float beta, float gamma)
{
  if (leg == 0)
  {
    alpha = 90 - alpha;
    beta = beta;
    gamma += 90;
  }
  else if (leg == 1)
  {
    alpha += 90;
    beta = 180 - beta;
    gamma = 90 - gamma;
  }
  else if (leg == 2)
  {
    alpha += 90;
    beta = 180 - beta;
    gamma = 90 - gamma;
  }
  else if (leg == 3)
  {
    alpha = 90 - alpha;
    beta = beta;
    gamma += 90;
  }

  servo[leg][0].write(alpha);
  servo[leg][1].write(beta);
  servo[leg][2].write(gamma);
}