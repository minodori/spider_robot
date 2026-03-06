#include <ESP32Servo.h>

Servo servo_footA;
Servo servo_armA;
Servo servo_footB;
Servo servo_armB;
Servo servo_footC;
Servo servo_armC;
Servo servo_footD;
Servo servo_armD;

// Connect servos to pins
int srv_footA = 13;
int srv_armA = 12;
int srv_footB = 15;
int srv_armB = 2;
int srv_footC = 26;
int srv_armC = 25;
int srv_footD = 17;
int srv_armD = 5;

void setInitialPositions() {
  // Leg servos: 60°
  servo_footA.write(60);
  servo_footB.write(60);
  servo_footC.write(60);
  servo_footD.write(60);

  // Arm servos: 90°
  servo_armA.write(90);
  servo_armB.write(90);
  servo_armC.write(90);
  servo_armD.write(90);
}

void setup() {
  servo_footA.attach(srv_footA);
  servo_armA.attach(srv_armA);
  servo_footB.attach(srv_footB);
  servo_armB.attach(srv_armB);
  servo_footC.attach(srv_footC);
  servo_armC.attach(srv_armC);
  servo_footD.attach(srv_footD);
  servo_armD.attach(srv_armD);

  // Set mounting positions
  setInitialPositions();
}

void loop() {

}

