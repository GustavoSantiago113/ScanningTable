#include <Arduino.h>
#include <Stepper.h>

// 28BYJ-48 has 2048 steps per revolution (in half-step mode)
#define STEPS_PER_REV 2048  

// Define motor control pins
Stepper stepper(STEPS_PER_REV, 5, 0, 4, 2);

void setup() {
  stepper.setSpeed(10); // 10 RPM
}

void loop() {
  // 60 degrees = 1/6 of a revolution
  int steps_per_60deg = STEPS_PER_REV / 6;

  // Step 60 degrees clockwise
  stepper.step(steps_per_60deg);
  delay(3000); // 3 seconds

  // Repeat until full 360° turn
}