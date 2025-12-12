#ifndef ESP_NOW_H // <-- Add this line at the very top
#define ESP_NOW_H // <-- Add this line right below the first one

#include <Arduino.h>
#include <esp_now.h>

typedef struct ControlData {
  uint8_t Throttle, Yaw, Pitch, Roll, Mode;
} Control_t;

typedef struct MotorOutput {
  int m1, m2, m3, m4;
} bldcmotors_t;

typedef struct {
  int m1, m2, m3, m4;
  float pitch;
  float roll;
  float yaw;

} TelemetryPacket_t;

extern TelemetryPacket_t telemetry;
extern bldcmotors_t bldcmotors;
extern Control_t Control;

extern uint8_t transmitterMAC[6];
extern bool macStored;
extern bool isArmed;

extern bool failsafe;
extern unsigned long lastPacketTime;

void OnDataRecv(const uint8_t *mac, const uint8_t *incomingData, int len);

#endif // ESP_NOW_H <-- Add this line at the very bottom