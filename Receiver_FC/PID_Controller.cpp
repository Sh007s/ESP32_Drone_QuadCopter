#include "PID_Controller.h"
#include "BLDCmotor.h"
#include "Receiver_FC.h"
#include "esp_now.h"

// ====================== PID STATE VARIABLE DEFINITIONS ======================
// Define the state variables declared as extern in the header.
float rollI = 0, pitchI = 0, yawI = 0;
float rollPrevError = 0, pitchPrevError = 0, yawPrevError = 0;


// ====================== PID CONTROL IMPLEMENTATION ======================
void run_PID_control(float loopDt) {
  
  // ---------- USER INPUTS ----------
  // NOTE: Control struct must be externally defined and accessible (from esp_now.h)
  float rollStick = (int)Control.Roll - 127;
  float pitchStick = (int)Control.Pitch - 127;
  float yawStick = (int)Control.Yaw - 127;

  float rollSet = (rollStick / 127.0f) * MAX_ANGLE_CMD;
  float pitchSet = (pitchStick / 127.0f) * MAX_ANGLE_CMD;
  float yawRateSet = (yawStick / 127.0f) * 100.0f;

  // ---------- ERROR CALCULATION ----------
  float rollError = rollSet - g_roll;
  float pitchError = pitchSet - g_pitch;
  float yawError = yawRateSet - gyroZ;

  // ---------- INTEGRAL (I) TERM ----------
  rollI += rollError * loopDt;
  pitchI += pitchError * loopDt;
  yawI += yawError * loopDt;

  // ---------- DERIVATIVE (D) TERM ----------
  float rollD = (rollError - rollPrevError) / loopDt;
  float pitchD = (pitchError - pitchPrevError) / loopDt;
  float yawD = (yawError - yawPrevError) / loopDt;

  // **ANTI-WINDUP**
  rollI = constrain(rollI, -MAX_I_TERM, MAX_I_TERM);
  pitchI = constrain(pitchI, -MAX_I_TERM, MAX_I_TERM);
  yawI = constrain(yawI, -MAX_I_TERM, MAX_I_TERM);

  // **Update Previous Error for next loop's D-term**
  rollPrevError = rollError;
  pitchPrevError = pitchError;
  yawPrevError = yawError;

  // ---------- FINAL PID CORRECTION ----------
  float rollCorr = (KP_ROLL * rollError) + (KI_ROLL * rollI) + (KD_ROLL * rollD);
  float pitchCorr = (KP_PITCH * pitchError) + (KI_PITCH * pitchI) + (KD_PITCH * pitchD);
  float yawCorr = (KP_YAW * yawError) + (KI_YAW * yawI) + (KD_YAW * yawD);

  // ----------- 3D THROTTLE -----------
  float tStick = ((int)Control.Throttle - 127);
  // ✅ HARD DEAD ZONE
  if (abs(tStick) < THROTTLE_DEADZONE) {
    tStick = 0;
  }

  float thrust = tStick / 127.0f;
  int throttle = 1500 + (thrust * 300); // ±300 for safety

  if (tStick == 0) { // Keep motors alive at idle speed when armed, unless stick is at dead zone
    throttle = 1500 + IDLE_THROTTLE;
  }

  // ----------- 3D MIXER -----------
  float M1 = throttle + pitchCorr + rollCorr - yawCorr;  // Front-Right
  float M2 = throttle + pitchCorr - rollCorr + yawCorr;  // Front-Left
  float M3 = throttle - pitchCorr - rollCorr - yawCorr;  // Rear-Left
  float M4 = throttle - pitchCorr + rollCorr + yawCorr;  // Rear-Right
  
  // NOTE: Uncomment your debug line if needed, but remember to include
  // <stdio.h> or <cstdio> if using a pure C++ file outside of the .ino structure.
  Serial.printf(
     "ROLL TEST → rollCorr:%.2f pitchCorr:%.2f | M1:%.1f M2:%.1f M3:%.1f M4:%.1f\n",
     rollCorr, pitchCorr,
     M1, M2, M3, M4);


  // ----------- MOTOR RAMPING AND DIRECTION LOGIC -----------
  
  // Convert float mixer output to int PWM value
  int M1i = (int)M1;
  int M2i = (int)M2;
  int M3i = (int)M3;
  int M4i = (int)M4;

  // Ramping (using the external variables mX_current)
  rampMotor(&m1_current, M1i);
  rampMotor(&m2_current, M2i);
  rampMotor(&m3_current, M3i);
  rampMotor(&m4_current, M4i);

  // Auto Direction (mX_rev is extern from BLDCmotor.h, defined in BLDCmotor.cpp)
  m1_rev = getDirection(m1_current);
  m2_rev = getDirection(m2_current);
  m3_rev = getDirection(m3_current);
  m4_rev = getDirection(m4_current);

  // ----------- SEND TO MOTORS -----------
  // **CRITICAL:** Use mX_current for speed, not the raw mixer output M_i
  // The mX_rev variables must be int8_t (1, 0, or -1) for this to work with the BLDCmotor driver.
  setBLDCQuad(
    m1_current, m2_current, m3_current, m4_current,
    m1_rev, m2_rev, m3_rev, m4_rev);
}