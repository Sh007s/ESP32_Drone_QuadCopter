#include "BLDCmotor.h"
#include "Receiver_FC.h" // Includes all constants: ESC_PINS, ARM_VALUE, etc.

// == ESC Object Definitions ==
Servo esc1, esc2, esc3, esc4;

// == Motor State Definitions ==
int m1_current = ARM_VALUE;
int m2_current = ARM_VALUE;
int m3_current = ARM_VALUE;
int m4_current = ARM_VALUE;

int8_t m1_rev = 0, m2_rev = 0, m3_rev = 0, m4_rev = 0; // **FIXED: Now int8_t**
// ===================== ESC INIT =====================
void ESC_init() {
  ESP32PWM::allocateTimer(0);
  ESP32PWM::allocateTimer(1);
  ESP32PWM::allocateTimer(2);
  ESP32PWM::allocateTimer(3);

  esc1.setPeriodHertz(50);
  esc2.setPeriodHertz(50);
  esc3.setPeriodHertz(50);
  esc4.setPeriodHertz(50);

  esc1.attach(ESC1_PIN, MIN_PULSE, MAX_PULSE);
  esc2.attach(ESC2_PIN, MIN_PULSE, MAX_PULSE);
  esc3.attach(ESC3_PIN, MIN_PULSE, MAX_PULSE);
  esc4.attach(ESC4_PIN, MIN_PULSE, MAX_PULSE);
}

bool Arm_BLDC_Motor() {

  static bool armingInProgress = false;
  static unsigned long armingStartTime = 0;

  // ✅ START ARMING ON MODE EDGE (ONLY ONCE)
  if (!armingInProgress) {
    armingInProgress = true;
    armingStartTime = millis();  // ✅ ALWAYS RESET HERE

    esc1.writeMicroseconds(ARM_VALUE);
    esc2.writeMicroseconds(ARM_VALUE);
    esc3.writeMicroseconds(ARM_VALUE);
    esc4.writeMicroseconds(ARM_VALUE);

    Serial.println("🟡 Arming Started...");
    return false;
  }

  unsigned long timedeg = millis() - armingStartTime;
  Serial.printf("⏳ Countdown for Arming: %lu ms\n", timedeg);

  if (timedeg >= 2000) {
    armingInProgress = false;
    Serial.println("✅ Arming Completed!");
    return true;
  }

  return false;
}

int applyDeadZone(int value) {
  if (value > DEADZONE_LOW && value < DEADZONE_HIGH)
    return ARM_VALUE;
  return value;
}

int8_t getDirection(int pwm) {
  if (pwm > ARM_VALUE) return 1;   // Forward
  if (pwm < ARM_VALUE) return -1;  // Reverse
  return 0;                        // Stop
}

void rampMotor(int *current, int target) {
  if (*current < target) {
    *current += RAMP_STEP;
    if (*current > target) *current = target;
  } else if (*current > target) {
    *current -= RAMP_STEP;
    if (*current < target) *current = target;
  }
}

// ======================= YOUR 4-MOTOR BLDC 3D DRIVER =======================

void setBLDCQuad(
  int m1, int m2, int m3, int m4,
  int8_t r1, int8_t r2, int8_t r3, int8_t r4) {
  // ------- MOTOR 1 -------
  if (r1 == -1) {
    esc1.writeMicroseconds(1475);
    delay(5);
    int out = map(m1, 0, 255, 1450, 1200);
    esc1.writeMicroseconds(out);
  } else {
    esc1.writeMicroseconds(1498);
    delay(5);
    int out = map(m1, 0, 255, 1500, 1800);
    esc1.writeMicroseconds(out);
  }

  // ------- MOTOR 2 -------
  if (r2 == -1) {
    esc2.writeMicroseconds(1475);
    delay(5);
    int out = map(m2, 0, 255, 1460, 1200);
    esc2.writeMicroseconds(out);
  } else {
    esc2.writeMicroseconds(1498);
    delay(5);
    int out = map(m2, 0, 255, 1500, 1800);
    esc2.writeMicroseconds(out);
  }

  // ------- MOTOR 3 -------
  if (r3 == -1) {
    esc3.writeMicroseconds(1475);
    delay(5);
    int out = map(m3, 0, 255, 1460, 1200);
    esc3.writeMicroseconds(out);
  } else {
    esc3.writeMicroseconds(1498);
    delay(5);
    int out = map(m3, 0, 255, 1500, 1800);
    esc3.writeMicroseconds(out);
  }

  // ------- MOTOR 4 -------
  if (r4 == -1) {
    esc4.writeMicroseconds(1475);
    delay(5);
    int out = map(m4, 0, 255, 1460, 1200);
    esc4.writeMicroseconds(out);
  } else {
    esc4.writeMicroseconds(1498);
    delay(5);
    int out = map(m4, 0, 255, 1500, 1800);
    esc4.writeMicroseconds(out);
  }
}
