#include <AccelStepper.h>

// Define stepper motor connections and interface type
#define dirPin 13
#define stepPin 14
#define motorInterfaceType 1
#define microStep 32

// Create an instance of the AccelStepper class
AccelStepper stepper = AccelStepper(motorInterfaceType, stepPin, dirPin);

// Motor parameters
const float speedMax = 1000 * microStep;
int currentSpeed = 0; // Store current speed value

void setup() {
  Serial.begin(115200);

  // Initialize stepper motor
  stepper.setMaxSpeed(speedMax);
  stepper.setSpeed(0); // Start at 0 speed

  Serial.println("Stepper Serial Control Ready.");
  Serial.println("Send speed in steps/sec (e.g., 500):");
}

void loop() {
  // Check for serial input
  if (Serial.available() > 0) {
    String input = Serial.readStringUntil('\n');
    input.trim();  // Remove whitespace
    float speedInput = input.toFloat();

    currentSpeed = speedInput * microStep;  // Convert to microstepped speed
    stepper.setSpeed(currentSpeed);
    Serial.print("Updated speed: ");
    Serial.println(currentSpeed);
  }

  // Run the stepper at constant speed
  stepper.runSpeed();
}
