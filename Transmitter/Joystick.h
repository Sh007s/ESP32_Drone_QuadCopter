#ifndef JOYSTICK_H
#define JOYSTICK_H

 #include <Arduino.h>

// ===============================
// --- Joystick Pin Definitions
// ===============================
extern const int JOYSTICK1_X_PIN ; 
extern const int JOYSTICK1_Y_PIN ;
extern const int JOYSTICK2_X_PIN ;
extern const int JOYSTICK2_Y_PIN ;

// ===============================
// --- Calibration & Correction
// ===============================
extern int CENTER_POINT; 
extern const int Y_AXIS_BIAS_MAGNITUDE; 
extern const int DEAD_ZONE;
extern const int MAX_ADC_VALUE;

// ===============================
// --- Axis Inversion Flags
// ===============================
extern const bool INVERT_JOYSTICK1_X;
extern const bool INVERT_JOYSTICK1_Y; 
extern const bool INVERT_JOYSTICK2_X;
extern const bool INVERT_JOYSTICK2_Y; 

extern int xAxis1Raw, yAxis1Raw; 
extern int xAxis2Raw, yAxis2Raw;
extern int yAxis1TrueRaw;
extern int yAxis2TrueRaw; 

// Control variables are bytes (0-255)
extern byte xAxis1Control, yAxis1Control ;
extern byte xAxis2Control , yAxis2Control;

int readJoystickAxis(int pin);
byte mapAxisTo255(int rawValue, int center);

#endif  // JOYSTICK_H
