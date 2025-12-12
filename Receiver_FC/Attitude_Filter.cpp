#include "Attitude_Filter.h"
#include <Wire.h> // Needed for I2C if sensors were initialized here (but better in .ino)
#include <math.h> // Necessary for PI and sqrt in updateIMU()

// ====================== RAW & FILTERED IMU Variable DEFINITIONS ======================
// These variables are declared extern in Attitude_Filter.h and are defined here.
int16_t ax = 0, ay = 0, az = 0;
int16_t gx = 0, gy = 0, gz = 0;
float accelX = 0, accelY = 0, accelZ = 0;
float gyroX = 0, gyroY = 0, gyroZ = 0;

// ====================== FILTER VARIABLE DEFINITIONS ======================
// IMU Angles
float g_roll = 0, g_pitch = 0, g_yaw = 0;
float g_kalAlt = 0;

// Angle Kalman Objects
Kalman kalRoll, kalPitch;

// Altitude Kalman Variables
float kalman_alt = 0;
float P_alt = 1.0, Q_alt = 0.15, R_alt = 5.0, K_alt = 0;

// Moving Average Variables (Needs to be explicitly sized here)
// NOTE: Assuming FILTER_SIZE is defined in Receiver_FC.h as a preprocessor macro
#define FILTER_SIZE 10

float axBuf[FILTER_SIZE], axSum = 0; int axIndex = 0;
float ayBuf[FILTER_SIZE], aySum = 0; int ayIndex = 0;
float azBuf[FILTER_SIZE], azSum = 0; int azIndex = 0;

float gxBuf[FILTER_SIZE], gxSum = 0; int gxIndex = 0;
float gyBuf[FILTER_SIZE], gySum = 0; int gyIndex = 0;
float gzBuf[FILTER_SIZE], gzSum = 0; int gzIndex = 0;

// Timing
unsigned long lastIMUTime = 0;


// ====================== Kalman Filter Class Implementation ======================
float Kalman::getAngle(float newAngle, float newRate, float dt) {
  // Kalman filter logic as provided in your original INO file
  float rate = newRate - bias;
  angle += dt * rate;

  P[0][0] += dt * (dt * P[1][1] - P[0][1] - P[1][0] + Q_angle);
  P[0][1] -= dt * P[1][1];
  P[1][0] -= dt * P[1][1];
  P[1][1] += Q_bias * dt;

  float S = P[0][0] + R_measure;
  float K0 = P[0][0] / S;
  float K1 = P[1][0] / S;

  float y = newAngle - angle;

  angle += K0 * y;
  bias += K1 * y;

  float P00_temp = P[0][0];
  float P01_temp = P[0][1];

  P[0][0] -= K0 * P00_temp;
  P[0][1] -= K0 * P01_temp;
  P[1][0] -= K1 * P00_temp;
  P[1][1] -= K1 * P01_temp;

  return angle;
}

// ====================== Moving Average Function Implementation ======================
float movingAverage(float newValue, float *buffer, int *index, float *sum) {
  *sum -= buffer[*index];
  buffer[*index] = newValue;
  *sum += newValue;
  *index = (*index + 1) % FILTER_SIZE;
  return *sum / FILTER_SIZE;
}

// ====================== Altitude Kalman Implementation ======================
float kalmanAltitude(float measurement) {
  P_alt += Q_alt;
  K_alt = P_alt / (P_alt + R_alt);
  kalman_alt += K_alt * (measurement - kalman_alt);
  P_alt = (1 - K_alt) * P_alt;
  return kalman_alt;
}

// ====================== INITIALIZATION FUNCTION ======================
void init_AttitudeFilter() {
    // Set initial altitude to the first reading
    kalman_alt = bmp.readAltitude(1013.25); 
    lastIMUTime = micros();
}


// ====================== IMU UPDATE FUNCTION ======================
void updateIMU() {
  
  // Read raw sensor data (updates extern variables ax, ay, az, etc.)
  mpu.getAcceleration(&ax, &ay, &az);
  mpu.getRotation(&gx, &gy, &gz);

  // Convert raw to real-world units (updates extern variables accelX, gyroX, etc.)
  accelX = ax / 16384.0;
  accelY = ay / 16384.0;
  accelZ = az / 16384.0;

  gyroX = gx / 131.0;
  gyroY = gy / 131.0;
  gyroZ = gz / 131.0; // gyroZ is the Yaw rate, used by the PID controller

  // Apply Moving Average Filter
  float fAX = movingAverage(accelX, axBuf, &axIndex, &axSum);
  float fAY = movingAverage(accelY, ayBuf, &ayIndex, &aySum);
  float fAZ = movingAverage(accelZ, azBuf, &azIndex, &azSum);

  float fGX = movingAverage(gyroX, gxBuf, &gxIndex, &gxSum);
  float fGY = movingAverage(gyroY, gyBuf, &gyIndex, &gySum);
  float fGZ = movingAverage(gyroZ, gzBuf, &gzIndex, &gzSum);

  // Calculate time delta (dt)
  unsigned long now = micros();
  float dt = (now - lastIMUTime) / 1000000.0f;
  lastIMUTime = now;
  if (dt <= 0 || dt > 0.05f) dt = 0.005f;

  // Angle from Accelerometer
  float accRoll = atan2(fAY, fAZ) * 180 / PI;
  float accPitch = atan2(-fAX, sqrt(fAY * fAY + fAZ * fAZ)) * 180 / PI;

  // Apply Kalman Filter for Angles
  g_roll = kalRoll.getAngle(accRoll, fGX, dt);
  g_pitch = kalPitch.getAngle(accPitch, fGY, dt);

  // Integrate Yaw Rate
  g_yaw += fGZ * dt;

  // Apply Kalman Filter for Altitude
  g_kalAlt = kalmanAltitude(bmp.readAltitude(1013.25));
}