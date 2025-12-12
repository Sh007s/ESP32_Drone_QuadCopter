#ifndef ATTITUDE_FILTER_H
#define ATTITUDE_FILTER_H

#include <Arduino.h>
#include "Receiver_FC.h" // For FILTER_SIZE constant
#include <MPU6050.h>     // For the MPU6050 object definition
#include <Adafruit_BMP280.h> // For the BMP280 object definition

// ====================== EXTERNAL OBJECTS (Defined in Receiver_FC.ino) ======================
// The sensor objects must be defined in the main file but are used here.
extern MPU6050 mpu;
extern Adafruit_BMP280 bmp;

// ====================== RAW & FILTERED IMU Variables (extern) ======================
// These must be defined in Attitude_Filter.cpp (NOT Receiver_FC.ino anymore)
extern int16_t ax, ay, az;
extern int16_t gx, gy, gz;
extern float accelX, accelY, accelZ;
extern float gyroX, gyroY, gyroZ;

// ====================== ANGLE & ALTITUDE ESTIMATES (extern) ======================
extern float g_roll, g_pitch, g_yaw;
extern float g_kalAlt;

// ====================== KALMAN CLASS (Declaration) ======================
// State structure for Roll/Pitch Kalman Filter
class Kalman {
public:
  float Q_angle = 0.001;
  float Q_bias = 0.003;
  float R_measure = 0.03;

  float angle = 0;
  float bias = 0;
  float P[2][2] = { { 0, 0 }, { 0, 0 } };

  // Main calculation function
  float getAngle(float newAngle, float newRate, float dt);
};

extern Kalman kalRoll, kalPitch;

// ====================== FILTER STATE VARIABLES (extern) ======================
extern float kalman_alt;
extern unsigned long lastIMUTime;

// ====================== FUNCTION PROTOTYPES ======================
/**
 * @brief Initializes the filter states (e.g., setting initial altitude).
 * Must be called in setup() after MPU/BMP initialization.
 */
void init_AttitudeFilter();

/**
 * @brief Reads MPU/BMP, applies filters (MA, Kalman), and updates global angles.
 */
void updateIMU();

#endif // ATTITUDE_FILTER_H