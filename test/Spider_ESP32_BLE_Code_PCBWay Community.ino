#include "BluetoothSerial.h"
#include <ESP32Servo.h>

BluetoothSerial SerialBT;
int BTData;

// Commands sent via the mobile app through Bluetooth
//UP = 50
//DOWN = 51
//WALK = 52
//ROUND = 53
//BLINK = 54
//STALL = 55

Servo servo_footA;
Servo servo_armA;
Servo servo_footB;
Servo servo_armB;
Servo servo_footC;
Servo servo_armC;
Servo servo_footD;
Servo servo_armD;

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

int p13 = 23;   // LED pin for status indication

/* Check if Bluetooth configurations are enabled in the SDK */
#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
#error Bluetooth is not enabled! Please run make menuconfig to and enable it
#endif

void setup() {
  servo_footA.attach(srv_footA);
  servo_armA.attach(srv_armA);
  servo_footB.attach(srv_footB);
  servo_armB.attach(srv_armB);
  servo_footC.attach(srv_footC);
  servo_armC.attach(srv_armC);
  servo_footD.attach(srv_footD);
  servo_armD.attach(srv_armD);
  pinMode(p13, OUTPUT);
  Serial.begin(115200);
  SerialBT.begin("@maker101io");
  Serial.println("Bluetooth Started! Ready to pair...");
  stall(); // Initialize robot to a stable position
  delay(1000);
}

void loop() {
  if(SerialBT.available()){
    BTData = SerialBT.read(); // Read data from the Bluetooth app
  }
  // Execute commands based on received data
  if (BTData == 53) {
    digitalWrite(p13, HIGH);
    ROUND();
    digitalWrite(p13, LOW);
  } else if (BTData == 52) {
    digitalWrite(p13, HIGH);
    WALK();
    digitalWrite(p13, LOW);
  } else if (BTData == 54) {
    BLINK();
    for (int i = 0; i < 7; i++) { WALK(); }
    stall();
    flat_up();
    delay(1000);
    flat_dw();
    delay(1000);
    for (int i = 0; i < 7; i++) { ROUND(); }
    delay(500);
    stall();
  } else if (BTData == 50) {
    flat_up();
  } else if (BTData == 51) {
    flat_dw();
    flat_up();
  } else if (BTData == 55) {
    stall();
    digitalWrite(p13, LOW);
  }
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

// Basic movements

void go_ahead() {
  spd = 3;  // ~ 2-5
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
  delay(500);
  for (int i = min; i <= max + 50; i++) {
    servo_footA.write(i);
    servo_footB.write(i);
    servo_footC.write(i);
    servo_footD.write(i);
    delay(spd);
  }
  delay(500);
}

void flat_dw() {
  int max = Fup + 5;
  int min = Fdw + 5;
  servo_footA.write(max);
  servo_footB.write(max);
  servo_footC.write(max);
  servo_footD.write(max);
  delay(500);
  for (int i = max; i >= min; i--) {
    servo_footA.write(i);
    servo_footB.write(i);
    servo_footC.write(i);
    servo_footD.write(i);
    delay(spd);
  }
  delay(500);
}

// single arm mvt A
void armAfw() {
  int min = Abw + 0;
  int max = Afw + 0;
  for (int i = min; i <= max; i++) {
    servo_armA.write(i);
    delay(spd);
  }
}

void armAbw() {
  int min = Abw + 0;
  int max = Afw + 0;
  for (int i = max; i >= min; i--) {
    servo_armA.write(i);
    delay(spd);
  }
}

// single arm mvt B
void armBfw() {
  int min = Abw + 0;
  int max = Afw + 0;
  for (int i = min; i <= max; i++) {
    servo_armB.write(i);
    delay(spd);
  }
}

void armBbw() {
  int min = Abw + 0;
  int max = Afw + 0;
  for (int i = max; i >= min; i--) {
    servo_armB.write(i);
    delay(spd);
  }
}

// single arm mvt C
void armCfw() {
  int min = Abw + 0;
  int max = Afw + 0;
  for (int i = min; i <= max; i++) {
    servo_armC.write(i);
    delay(spd);
  }
}

void armCbw() {
  int min = Abw + 0;
  int max = Afw + 0;
  for (int i = max; i >= min; i--) {
    servo_armC.write(i);
    delay(spd);
  }
}

// single arm mvt D
void armDfw() {
  int min = Abw + 0;
  int max = Afw + 0;
  for (int i = min; i <= max; i++) {
    servo_armD.write(i);
    delay(spd);
  }
}

void armDbw() {
  int min = Abw + 0;
  int max = Afw + 0;
  for (int i = max; i >= min; i--) {
    servo_armD.write(i);
    delay(spd);
  }
}

// Initialize robot to a stable state
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
  digitalWrite(p13, HIGH);
}

void BLINK() {
  for (int i = 0; i < 3; i++) {
    digitalWrite(p13, HIGH);
    delay(100);
    digitalWrite(p13, LOW);
    delay(100);
  }
}