#include <avr/wdt.h>

// --- PIN DEFINITIONS ---
const int frontMotor1 = 8;
const int frontMotor2 = 9;
const int backMotor1 = 10;
const int backMotor2 = 11;
const int auxPin12 = 5;
const int auxPin13 = 6;

// Structure to hold each movement command: {leftSpeed, rightSpeed, duration_ms}
struct MoveCommand {
  int leftSpeed;
  int rightSpeed;
  unsigned long duration;
};

//{left, right, time}
//{+, -, time}-forward    {220, -240, 1000},   // FORWARD {-220, -220, 565},  // TURN_LEFT {220, 220, 490},  // TURN_RIGHT
MoveCommand path[] ={
  {220, -240, 1000},   // FORWARD
  {220, -240, 1000},   // FORWARD
  {-220, -220, 560},  // TURN_LEFT
  {220, -240, 1000},   // FORWARD
  {220, -240, 1000},   // FORWARD
  {220, -240, 1000},   // FORWARD
  {220, -240, 1000},   // FORWARD
  {220, 220, 470},  // TURN_RIGHT
  {220, -240, 1000},   // FORWARD
  {220, -240, 1000},   // FORWARD

  

};
int pathLength = sizeof(path) / sizeof(path[0]);
bool executed = false;

void setup() {
  // Configure motor control pins as outputs
  pinMode(frontMotor1, OUTPUT);
  pinMode(frontMotor2, OUTPUT);
  pinMode(backMotor1, OUTPUT);
  pinMode(backMotor2, OUTPUT);

  pinMode(auxPin12, OUTPUT);
  pinMode(auxPin13, OUTPUT);

  // Set default enable/speed PWM levels
  analogWrite(auxPin12, 200);
  analogWrite(auxPin13, 200);

  // Enable 2-second Watchdog Timer for safety
  wdt_enable(WDTO_2S);
}

// Custom non-blocking delay function that pets the watchdog timer
void safeDelay(unsigned long ms) {
  unsigned long start = millis();
  while (millis() - start < ms) {
    wdt_reset(); // Keep watchdog from resetting the MCU
  }
}

// --- MOTOR CONTROL FUNCTION ---
void setMotors(int leftSpeed, int rightSpeed) {
  // Left Motor Control
  if (leftSpeed > 0) {
    analogWrite(frontMotor1, abs(leftSpeed));
    analogWrite(frontMotor2, 0);
  } else if (leftSpeed < 0) {
    analogWrite(frontMotor1, 0);
    analogWrite(frontMotor2, abs(leftSpeed));
  } else {
    analogWrite(frontMotor1, 0);
    analogWrite(frontMotor2, 0);
  }

  // Right Motor Control
  if (rightSpeed > 0) {
    analogWrite(backMotor1, 0);
    analogWrite(backMotor2, abs(rightSpeed));
  } else if (rightSpeed < 0) {
    analogWrite(backMotor1, abs(rightSpeed));
    analogWrite(backMotor2, 0);
  } else {
    analogWrite(backMotor1, 0);
    analogWrite(backMotor2, 0);
  }
}

// Stop all motors
void stopMotors() {
  analogWrite(frontMotor1, 0);
  analogWrite(frontMotor2, 0);
  analogWrite(backMotor1, 0);
  analogWrite(backMotor2, 0);
}

void loop() {
  wdt_reset(); // Feed Watchdog at the start of loop

  // Run the path sequence only once upon startup/reset
  if (!executed) {
    // Initial delay before movement starts (3 seconds)
    safeDelay(10000);

    for (int i = 0; i < pathLength; i++) {
      wdt_reset();
      
      // Execute current move from the array
      setMotors(path[i].leftSpeed, path[i].rightSpeed);
      safeDelay(path[i].duration);
      
      // Stop briefly between movements
      stopMotors();
      safeDelay(500);
    }

    // Stop permanently after finishing the path
    stopMotors();
    executed = true; 
  }
}
