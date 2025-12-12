#ifndef PID_CONTROLLER_H
#define PID_CONTROLLER_H

#include <Arduino.h>
#include "Receiver_FC.h"
#include "BLDCmotor.h"
// Include MPU6050 and esp_now headers if needed for structures
#include "esp_now.h"

// ====================== PID STATE VARIABLES (extern) ======================
// These variables store the previous error and integrated error across loops.
extern float rollI, pitchI, yawI;
extern float rollPrevError, pitchPrevError, yawPrevError;

// ====================== GLOBAL ANGLE VARIABLES (extern) ======================
// These are defined in the main .ino file but used in the PID calculation.
extern float g_roll, g_pitch, g_yaw;
extern float gyroZ; // The yaw rate is needed directly for the yaw rate PID loop

// ====================== CONTROL FUNCTION (prototype) ======================
/**
 * @brief Runs the main PID control loop, calculates motor corrections, 
 * and calls setBLDCQuad() to drive the motors.
 * * @param loopDt The time elapsed since the last loop iteration (in seconds).
 */
void run_PID_control(float loopDt);

#endif // PID_CONTROLLER_H