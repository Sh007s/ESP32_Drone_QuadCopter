#include <WiFi.h>
#include <Wire.h>
#include <esp_now.h>
#include <Adafruit_BMP280.h>
#include "ESP32Servo.h"
#include "MPU6050.h"
#include "Receiver_FC.h"

// == PID STATE VARIABLES (NEW) ==
float rollI = 0, pitchI = 0, yawI = 0;
float rollPrevError = 0, pitchPrevError = 0, yawPrevError = 0;

MPU6050 mpu;
Adafruit_BMP280 bmp;

struct ControlData {
  uint8_t Throttle, Yaw, Pitch, Roll, Mode;
} Control;

struct MotorOutput {
  int m1, m2, m3, m4;
} bldcmotors;

typedef struct {
  int m1, m2, m3, m4;
  float pitch;
  float roll;
  float yaw;

} TelemetryPacket;

TelemetryPacket telemetry;

uint8_t transmitterMAC[6];
bool macStored = false;
bool isArmed = false;

// -------- RAW IMU --------
int16_t ax, ay, az;
int16_t gx, gy, gz;

// -------- FILTERED IMU --------
float accelX, accelY, accelZ;
float gyroX, gyroY, gyroZ;

// ANGLES
float g_roll = 0, g_pitch = 0, g_yaw = 0;
float g_kalAlt = 0;

// ===== MOVING AVERAGE FILTER =====
const int FILTER_SIZE = 10;

float axBuf[FILTER_SIZE], axSum = 0;
int axIndex = 0;
float ayBuf[FILTER_SIZE], aySum = 0;
int ayIndex = 0;
float azBuf[FILTER_SIZE], azSum = 0;
int azIndex = 0;

float gxBuf[FILTER_SIZE], gxSum = 0;
int gxIndex = 0;
float gyBuf[FILTER_SIZE], gySum = 0;
int gyIndex = 0;
float gzBuf[FILTER_SIZE], gzSum = 0;
int gzIndex = 0;

float movingAverage(float newValue, float *buffer, int *index, float *sum) {
  *sum -= buffer[*index];
  buffer[*index] = newValue;
  *sum += newValue;
  *index = (*index + 1) % FILTER_SIZE;
  return *sum / FILTER_SIZE;
}

// ===== KALMAN FOR ALTITUDE =====
float kalman_alt = 0;
float P_alt = 1.0, Q_alt = 0.15, R_alt = 5.0, K_alt = 0;

float kalmanAltitude(float measurement) {
  P_alt += Q_alt;
  K_alt = P_alt / (P_alt + R_alt);
  kalman_alt += K_alt * (measurement - kalman_alt);
  P_alt = (1 - K_alt) * P_alt;
  return kalman_alt;
}

// ===== KALMAN FOR ANGLES =====
class Kalman {
public:
  float Q_angle = 0.001;
  float Q_bias = 0.003;
  float R_measure = 0.03;

  float angle = 0;
  float bias = 0;
  float P[2][2] = { { 0, 0 }, { 0, 0 } };

  float getAngle(float newAngle, float newRate, float dt) {
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
};

Kalman kalRoll, kalPitch;

// TIMING
unsigned long lastIMUTime = 0;

// FAILSAFE
bool failsafe = false;
bool armed = false;
unsigned long lastPacketTime = 0;
//const unsigned long FAILSAFE_TIMEOUT = 300;

// ESC
Servo esc1, esc2, esc3, esc4;

int m1_current = ARM_VALUE;
int m2_current = ARM_VALUE;
int m3_current = ARM_VALUE;
int m4_current = ARM_VALUE;

int8_t m1_rev, m2_rev, m3_rev, m4_rev;

// ===================== ESP-NOW CALLBACK =====================
void OnDataRecv(const uint8_t *mac, const uint8_t *incomingData, int len) {

  if (!macStored) {
    memcpy(transmitterMAC, mac, 6);
    macStored = true;

    esp_now_peer_info_t peerInfo = {};
    memcpy(peerInfo.peer_addr, transmitterMAC, 6);
    peerInfo.channel = 0;
    peerInfo.encrypt = false;
    esp_now_add_peer(&peerInfo);
  }

  memcpy(&Control, incomingData, sizeof(Control));
  lastPacketTime = millis();
  failsafe = false;
  // -Serial.printf(
  //   "Throttle: %d | Yaw: %d | Pitch: %d | Roll: %d | Mode: %d\n",
  //   Control.Throttle,
  //   Control.Yaw,
  //   Control.Pitch,
  //   Control.Roll,
  //   Control.Mode);
}

// ===================== IMU UPDATE =====================
void updateIMU() {

  mpu.getAcceleration(&ax, &ay, &az);
  mpu.getRotation(&gx, &gy, &gz);

  accelX = ax / 16384.0;
  accelY = ay / 16384.0;
  accelZ = az / 16384.0;

  gyroX = gx / 131.0;
  gyroY = gy / 131.0;
  gyroZ = gz / 131.0;

  float fAX = movingAverage(accelX, axBuf, &axIndex, &axSum);
  float fAY = movingAverage(accelY, ayBuf, &ayIndex, &aySum);
  float fAZ = movingAverage(accelZ, azBuf, &azIndex, &azSum);

  float fGX = movingAverage(gyroX, gxBuf, &gxIndex, &gxSum);
  float fGY = movingAverage(gyroY, gyBuf, &gyIndex, &gySum);
  float fGZ = movingAverage(gyroZ, gzBuf, &gzIndex, &gzSum);

  unsigned long now = micros();
  float dt = (now - lastIMUTime) / 1000000.0f;
  lastIMUTime = now;
  if (dt <= 0 || dt > 0.05f) dt = 0.005f;

  float accRoll = atan2(fAY, fAZ) * 180 / PI;
  float accPitch = atan2(-fAX, sqrt(fAY * fAY + fAZ * fAZ)) * 180 / PI;

  g_roll = kalRoll.getAngle(accRoll, fGX, dt);
  g_pitch = kalPitch.getAngle(accPitch, fGY, dt);

  g_yaw += fGZ * dt;

  g_kalAlt = kalmanAltitude(bmp.readAltitude(1013.25));
}

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


// ===================== SETUP =====================
void setup() {
  Serial.begin(115200);

  WiFi.mode(WIFI_STA);
  esp_now_init();
  esp_now_register_recv_cb(OnDataRecv);

  Wire.begin(SDA_PIN, SCL_PIN);

  mpu.initialize();
  bmp.begin(0x76);

  ESC_init();

  kalman_alt = bmp.readAltitude(1013.25);
  lastIMUTime = micros();
}

// ===================== MAIN LOOP =====================
void loop() {

  updateIMU();

  if (millis() - lastPacketTime > FAILSAFE_TIMEOUT)
    failsafe = true;
  else
    failsafe = false;

  if (failsafe) {

    isArmed = false;
    esc1.writeMicroseconds(ARM_VALUE);
    esc2.writeMicroseconds(ARM_VALUE);
    esc3.writeMicroseconds(ARM_VALUE);
    esc4.writeMicroseconds(ARM_VALUE);
    return;
  }

  if (Control.Mode == 1 && !isArmed) {
    if (Arm_BLDC_Motor()) {
      isArmed = true;
      Serial.println("✅ Arming the 4 Motor");
    } else {
      Serial.println("⏳ Not Arming the 4 Motor");
    }
  }

  if (Control.Mode == 0 && isArmed) {
    isArmed = false;
    Serial.println("⛔ MOTORS DISARMED");
  }

  // ================= ✅ HARD ARM SAFETY LOCK =================
  // ❗ THIS IS THE MOST IMPORTANT SAFETY BLOCK
  if (!isArmed) {
    // Hold motors at neutral until armed
    esc1.writeMicroseconds(1500);
    esc2.writeMicroseconds(1500);
    esc3.writeMicroseconds(1500);
    esc4.writeMicroseconds(1500);
    return;  // ⛔ NOTHING BELOW THIS RUNS UNLESS ARMED
  }
  // --- CRITICAL LOOP TIME CALCULATION (loopDt) ---
  // This is the time delta for the control loop (PID)
  unsigned long now = micros();
  static unsigned long lastLoopTime = 0;
  float loopDt = (now - lastLoopTime) / 1000000.0f;  // in seconds
  lastLoopTime = now;

  // Safety check for dt
  if (loopDt <= 0.0f || loopDt > 0.1f) loopDt = 0.005f;

  // ---------- USER INPUTS ----------
  float rollStick = (int)Control.Roll - 127;
  float pitchStick = (int)Control.Pitch - 127;
  float yawStick = (int)Control.Yaw - 127;

  float rollSet = (rollStick / 127.0f) * MAX_ANGLE_CMD;
  float pitchSet = (pitchStick / 127.0f) * MAX_ANGLE_CMD;
  float yawRateSet = (yawStick / 127.0f) * 100.0f;

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

  // **ANTI-WINDUP** (Requires MAX_I_TERM constant)
  rollI = constrain(rollI, -MAX_I_TERM, MAX_I_TERM);
  pitchI = constrain(pitchI, -MAX_I_TERM, MAX_I_TERM);
  yawI = constrain(yawI, -MAX_I_TERM, MAX_I_TERM);

  // **Update Previous Error for next loop's D-term**
  rollPrevError = rollError;
  pitchPrevError = pitchError;
  yawPrevError = yawError;

  // ---------- FINAL PID CORRECTION ----------
  // Note: The new KP/KI/KD variables must be defined globally
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
  int throttle = 1500 + (thrust * 300);  // ±300 for safety

  if (isArmed && tStick == 0) {
    throttle = 1500 + IDLE_THROTTLE;
  }
  // ----------- 3D MIXER -----------
  // Mixer calculates the final PWM value (1000-2000) for each motor
  float M1 = throttle + pitchCorr + rollCorr - yawCorr;  // Front-Right
  float M2 = throttle + pitchCorr - rollCorr + yawCorr;  // Front-Left
  float M3 = throttle - pitchCorr - rollCorr - yawCorr;  // Rear-Left
  float M4 = throttle - pitchCorr + rollCorr + yawCorr;  // Rear-Right

  // Serial.printf(
  //   "TX P:%d R:%d Y:%d | PID PC:%.2f RC:%.2f YC:%.2f | MIX M1:%d M2:%d M3:%d M4:%d\n",
  //   Control.Pitch, Control.Roll, Control.Yaw,
  //   pitchCorr, rollCorr, yawCorr,
  //   M1, M2, M3, M4);
  Serial.printf(
    "ROLL TEST → rollCorr:%.2f pitchCorr:%.2f | M1:%.1f M2:%.1f M3:%.1f M4:%.1f\n",
    rollCorr, pitchCorr,
    M1, M2, M3, M4);

  //delay(1000);

  // ----------- CONVERT TO BLDC FORMAT -----------
  int m1_speed = abs(M1 - 1500);
  int m2_speed = abs(M2 - 1500);
  int m3_speed = abs(M3 - 1500);
  int m4_speed = abs(M4 - 1500);

  bool m1_rev = (M1 < 1500);
  bool m2_rev = (M2 < 1500);
  bool m3_rev = (M3 < 1500);
  bool m4_rev = (M4 < 1500);

  m1_speed = constrain(m1_speed, 0, 255);
  m2_speed = constrain(m2_speed, 0, 255);
  m3_speed = constrain(m3_speed, 0, 255);
  m4_speed = constrain(m4_speed, 0, 255);

  // ----------- SEND TO MOTORS -----------

  m1_speed = applyDeadZone(m1_speed);
  m2_speed = applyDeadZone(m2_speed);
  m3_speed = applyDeadZone(m3_speed);
  m4_speed = applyDeadZone(m4_speed);

  int M1i = (int)M1;
  int M2i = (int)M2;
  int M3i = (int)M3;
  int M4i = (int)M4;

  rampMotor(&m1_current, M1i);
  rampMotor(&m2_current, M2i);
  rampMotor(&m3_current, M3i);
  rampMotor(&m4_current, M4i);

  // ---------- SMOOTH RAMP ----------
  rampMotor(&m1_current, M1);
  rampMotor(&m2_current, M2);
  rampMotor(&m3_current, M3);
  rampMotor(&m4_current, M4);

  // ---------- AUTO DIRECTION ----------
  m1_rev = getDirection(m1_current);
  m2_rev = getDirection(m2_current);
  m3_rev = getDirection(m3_current);
  m4_rev = getDirection(m4_current);

  // ✅ ABSOLUTE MOTOR STOP WHEN THROTTLE IS CENTERED
  if (tStick == 0) {
    esc1.writeMicroseconds(1500);
    esc2.writeMicroseconds(1500);
    esc3.writeMicroseconds(1500);
    esc4.writeMicroseconds(1500);

    // Also force speeds to zero for safety
    m1_speed = 0;
    m2_speed = 0;
    m3_speed = 0;
    m4_speed = 0;

    return;  // ✅ Motors will NEVER move unless TX throttle moves
  }

  setBLDCQuad(
    m1_current, m2_current, m3_current, m4_current,
    m1_rev, m2_rev, m3_rev, m4_rev);

  // Serial.printf(
  //   "FINAL PWM → M1:%d M2:%d M3:%d M4:%d | PID P:%.2f R:%.2f Y:%.2f\n",
  //   m1_current, m2_current, m3_current, m4_current,
  //   pitchCorr, rollCorr, yawCorr);

  // ----------- TELEMETRY -----------
  if (macStored) {
    telemetry.pitch = g_pitch;
    telemetry.roll = g_roll;
    telemetry.yaw = g_yaw;

    telemetry.m1 = m1_current;
    telemetry.m2 = m2_current;
    telemetry.m3 = m3_current;
    telemetry.m4 = m4_current;

    esp_err_t result = esp_now_send(transmitterMAC, (uint8_t *)&telemetry, sizeof(telemetry));
    if (result == ESP_OK) {
      //    Serial.println("Receiver Telemetry data sent");
    } else {
      // Serial.print("Send error: ");
      // Serial.println(result);
    }
  }

  // // ---------- DEBUG ----------
  // Serial.printf("Pitch:%.2f Roll:%.2f Yaw:%.2f M1:%d M2:%d M3:%d M4:%d\nMode : %d\n",
  //               g_pitch, g_roll, g_yaw,
  //               (int)m1_speed, (int)m2_speed, (int)m3_speed, (int)m4_speed, Control.Mode);
}

// // ======================= YOUR 4-MOTOR BLDC 3D DRIVER =======================

void setBLDCQuad(
  int m1, int m2, int m3, int m4,
  bool r1, bool r2, bool r3, bool r4) {
  // ------- MOTOR 1 -------
  if (r1) {
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
  if (r2) {
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
  if (r3) {
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
  if (r4) {
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
