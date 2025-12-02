#ifndef TRANSMITTER_H
#define TRANSMITTER_H

#include <Arduino.h>

struct ControlData {
    uint8_t Throttle; // Vertical thrust control (0-255)
    uint8_t Yaw;      // Rotation around the Z-axis (0=Left, 127=Center, 255=Right)
    uint8_t Pitch;    // Forward/Backward movement (0=Forward/Backward, 127=Center)
    uint8_t Roll;     // Left/Right movement (0=Left, 127=Center, 255=Right)
};

typedef struct {
  int m1, m2, m3, m4;
  float pitch, roll, yaw;
} TelemetryPacket;

extern TelemetryPacket telemetry;

typedef struct {
  float P;
  float I;
  float D;
} PID_t;

extern PID_t PID;

// This is the variable you will use throughout the program (e.g., Control.Throttle)
extern ControlData Control;

void OLED_display_show();
void performBack();
void performSelect();
void performUp();
void performDown();
void drawMenu();
void drawRXData();
void drawPID();
void drawTXData();

#endif  // TRANSMITTER_H