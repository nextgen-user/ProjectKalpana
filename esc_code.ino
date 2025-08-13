#include <Servo.h>

Servo esc; // Create Servo object to control the ESC
int escPin = 9; // PWM pin connected to ESC signal

void setup() {
  Serial.begin(9600);
  esc.attach(escPin); 

  // // ESC Calibration Sequence
  // Serial.println("Calibrating ESC...");
  // esc.writeMicroseconds(2000); // Max throttle
  // delay(2000); // Wait 2 seconds
  // esc.writeMicroseconds(1000); // Min throttle
  // delay(2000); // Wait 2 seconds
  // Serial.println("Calibration complete. Starting motor control...");
}

void loop() {
  // Example: slowly ramp motor speed up and down
  for (int speed = 1000; speed <= 2000; speed += 10) {
    esc.writeMicroseconds(speed);
    delay(20);
  }
  for (int speed = 2000; speed >= 1000; speed -= 10) {
    esc.writeMicroseconds(speed);
    delay(20);
  }
  delay(10000);
}
