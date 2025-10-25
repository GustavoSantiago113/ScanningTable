#include <Stepper.h>
#define STEPS_PER_REV 2048

#define IN1 5   // D1
#define IN2 4   // D2
#define IN3 14  // D5
#define IN4 12  // D6

#define STEPS_PER_REV 2048
Stepper stepper(STEPS_PER_REV, IN1, IN3, IN2, IN4);

void setup() {
  Serial.begin(115200);
  stepper.setSpeed(8);
  Serial.println("Testing stepper...");
}

void loop() {
  Serial.println("Forward 360°");
  stepper.step(STEPS_PER_REV);
  delay(2000);
  Serial.println("Backward 360°");
  stepper.step(-STEPS_PER_REV);
  delay(2000);
}
