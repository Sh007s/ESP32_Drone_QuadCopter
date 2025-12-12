#include <WiFi.h>
#include <Wire.h>
#include "esp_now.h"
#include <Adafruit_BMP280.h>
#include "ESP32Servo.h"
#include "MPU6050.h"
#include "Receiver_FC.h"
#include "BLDCmotor.h"
#include "PID_Controller.h"   // New PID file
#include "Attitude_Filter.h"  // New Filter file

MPU6050 mpu;
Adafruit_BMP280 bmp;

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

  init_AttitudeFilter();

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
    esc1.writeMicroseconds(ARM_VALUE);
    esc2.writeMicroseconds(ARM_VALUE);
    esc3.writeMicroseconds(ARM_VALUE);
    esc4.writeMicroseconds(ARM_VALUE);
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
  // 6. PID Control Loop
  run_PID_control(loopDt);  // <--- CALLS the function now in PID_Controller.cpp

  // 7. CRITICAL MOTOR STOP SAFETY (Post-PID check)
  float tStick = ((int)Control.Throttle - 127);
  if (abs(tStick) < THROTTLE_DEADZONE) {
      tStick = 0;
  }
  
  if (isArmed && tStick == 0) {
    esc1.writeMicroseconds(ARM_VALUE);
    esc2.writeMicroseconds(ARM_VALUE);
    esc3.writeMicroseconds(ARM_VALUE);
    esc4.writeMicroseconds(ARM_VALUE);
    return; 
  }

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
          Serial.println("Receiver Telemetry data sent");
    } else {
       Serial.print("Send error: ");
       Serial.println(result);
    }
  }
}