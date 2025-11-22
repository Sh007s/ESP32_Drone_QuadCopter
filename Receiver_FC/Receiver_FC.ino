#include <WiFi.h>
#include <esp_now.h>
#include "ESP32Servo.h"

#define ESC1_PIN 13
#define ESC2_PIN 12
#define ESC3_PIN 14
#define ESC4_PIN 27
#define ARM_VALUE 1500  // Mid value (stopped / idle for many ESCs)
#define MIN_PULSE 1000  // Reserve Full Throttle
#define MAX_PULSE 2000  // Forward Full Throttle

struct ControlData {
  uint8_t Throttle;
  uint8_t Yaw;
  uint8_t Pitch;
  uint8_t Roll;
} Control;

struct MotorOutput {
  int m1;
  int m2;
  int m3;
  int m4;
} bldcmotors;

Servo esc1, esc2, esc3, esc4;
unsigned long lastPacketTime = 0;
const unsigned long FAILSAFE_TIMEOUT = 300;  // ms
bool failsafe = true;                        // start in failsafe
bool armed = false;

void ESC_init() {

  // OPTIONAL but recommended: allocate timers (up to 4)
  ESP32PWM::allocateTimer(0);
  ESP32PWM::allocateTimer(1);
  ESP32PWM::allocateTimer(2);
  ESP32PWM::allocateTimer(3);

  // Set PWM frequency for each ESC servo: 50 Hz for RC/ESCs
  esc1.setPeriodHertz(50);
  esc2.setPeriodHertz(50);
  esc3.setPeriodHertz(50);
  esc4.setPeriodHertz(50);

  // Attach ESCs: pin + min/max pulse in microseconds
  esc1.attach(ESC1_PIN, MIN_PULSE, MAX_PULSE);
  esc2.attach(ESC2_PIN, MIN_PULSE, MAX_PULSE);
  esc3.attach(ESC3_PIN, MIN_PULSE, MAX_PULSE);
  esc4.attach(ESC4_PIN, MIN_PULSE, MAX_PULSE);
}

void OnDataRecv(const uint8_t *mac, const uint8_t *data, int len) {
  memcpy(&Control, data, sizeof(Control));
  lastPacketTime = millis();
  failsafe = false;  // signal OK

  Serial.print("Throttle: ");
  Serial.print(Control.Throttle);
  Serial.print(" Roll: ");
  Serial.print(Control.Roll);
  Serial.print(" Pitch: ");
  Serial.print(Control.Pitch);
  Serial.print(" Yaw: ");
  Serial.println(Control.Yaw);
}

void setup() {
  Serial.begin(115200);
  WiFi.mode(WIFI_STA);

  if (esp_now_init() != ESP_OK) {
    Serial.println("ESP-NOW init failed!");
    return;
  }

  esp_now_register_recv_cb(OnDataRecv);

  ESC_init();

  // ARM ESCs (usually mid or minimum throttle depending on your ESC)
  esc1.writeMicroseconds(ARM_VALUE);
  delay(10);
  esc2.writeMicroseconds(ARM_VALUE);
  delay(20);
  esc3.writeMicroseconds(ARM_VALUE);
  delay(30);
  esc4.writeMicroseconds(ARM_VALUE);

  delay(2000);  // give ESCs time to arm
  //  Serial.println("ESC Armed");
}

void loop() {

  if (Serial.available()) {
    String cmd = Serial.readStringUntil('\n');
    cmd.trim();

    if (cmd.equalsIgnoreCase("ARM")) {
      armed = true;
      Serial.println(">>> ESC ARMED <<<");
    } else if (cmd.equalsIgnoreCase("DISARM")) {
      armed = false;
      Serial.println(">>> ESC DISARMED <<<");
    }
  }

  // ----------- FAILSAFE CHECK -----------
  if (millis() - lastPacketTime > FAILSAFE_TIMEOUT) {
    failsafe = true;
  }
  if (failsafe || !armed) {
    // STOP ALL MOTORS SAFELY
    esc1.writeMicroseconds(1500);
    esc2.writeMicroseconds(1500);
    esc3.writeMicroseconds(1500);
    esc4.writeMicroseconds(1500);

    if (failsafe){
      Serial.println("** FAILSAFE ACTIVE: NO SIGNAL **");
  //    delay(1000);
    }else
      Serial.println("** SYSTEM NOT ARMED **");
    delay(20);
    return;
  }

  // Convert throttle 0–255 → 1000–2000
  int throttle = map(Control.Throttle, 0, 255, MIN_PULSE, MAX_PULSE);

  // Convert stick inputs to ±128
  float roll = (int)Control.Roll - 127;
  float pitch = (int)Control.Pitch - 127;
  float yaw = (int)Control.Yaw - 127;

  // Adjust sensitivity (tuning)
  roll *= 3;
  pitch *= 3;
  yaw *= 3;

 // 3D MIXER centered at 1500
  int t = throttle - 1500;   // throttle offset for forward/back

  bldcmotors.m1 = 1500 + t + pitch + roll - yaw;
  bldcmotors.m2 = 1500 + t + pitch - roll + yaw;
  bldcmotors.m3 = 1500 + t - pitch - roll - yaw;
  bldcmotors.m4 = 1500 + t - pitch + roll + yaw;

  // Clamp to ESC range
  bldcmotors.m1 = constrain(bldcmotors.m1, MIN_PULSE, MAX_PULSE);
  bldcmotors.m2 = constrain(bldcmotors.m2, MIN_PULSE, MAX_PULSE);
  bldcmotors.m3 = constrain(bldcmotors.m3, MIN_PULSE, MAX_PULSE);
  bldcmotors.m4 = constrain(bldcmotors.m4, MIN_PULSE, MAX_PULSE);

  // Output to ESCs
  esc1.writeMicroseconds(bldcmotors.m1);
  esc2.writeMicroseconds(bldcmotors.m2);
  esc3.writeMicroseconds(bldcmotors.m3);
  esc4.writeMicroseconds(bldcmotors.m4);

  delay(10);
}
