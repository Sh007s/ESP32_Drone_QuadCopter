#include <Arduino.h>
#include "ESP32Servo.h"
#include "Receiver_FC.h"  // For ARM_VALUE, etc.

// ESC
extern Servo esc1, esc2, esc3, esc4;

// == Motor State Variables ==
// Current PWM values (1000-2000)
extern int m1_current;
extern int m2_current;
extern int m3_current;
extern int m4_current;

extern int8_t m1_rev, m2_rev, m3_rev, m4_rev;

void ESC_init();
bool Arm_BLDC_Motor();
void rampMotor(int *current, int target);
int applyDeadZone(int value);
void setBLDCQuad(
  int m1, int m2, int m3, int m4, 
  int8_t r1, int8_t r2, int8_t r3, int8_t r4);
  int8_t getDirection(int pwm);