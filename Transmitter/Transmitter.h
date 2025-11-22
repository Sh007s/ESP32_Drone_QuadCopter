#ifndef TRANSMITTER_H
#define TRANSMITTER_H

#include <Arduino.h>

struct ControlData {
    uint8_t Throttle; // Vertical thrust control (0-255)
    uint8_t Yaw;      // Rotation around the Z-axis (0=Left, 127=Center, 255=Right)
    uint8_t Pitch;    // Forward/Backward movement (0=Forward/Backward, 127=Center)
    uint8_t Roll;     // Left/Right movement (0=Left, 127=Center, 255=Right)
};

// This is the variable you will use throughout the program (e.g., Control.Throttle)
extern ControlData Control;

void OLED_display_show();

#endif  // TRANSMITTER_H