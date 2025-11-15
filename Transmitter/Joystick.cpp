#include "Joystick.h"

// ===============================
// --- Joystick Pin Definitions
// ===============================
const int JOYSTICK1_X_PIN = 33;
const int JOYSTICK1_Y_PIN = 32;
const int JOYSTICK2_X_PIN = 35;
const int JOYSTICK2_Y_PIN = 34;

// ===============================
// --- Calibration & Correction
// ===============================
int CENTER_POINT = 0;
const int Y_AXIS_BIAS_MAGNITUDE = 0;
const int DEAD_ZONE = 50;
const int MAX_ADC_VALUE = 4095;

// ===============================
// --- Axis Inversion Flags
// ===============================
const bool INVERT_JOYSTICK1_X = false;
const bool INVERT_JOYSTICK1_Y = false;
const bool INVERT_JOYSTICK2_X = false;
const bool INVERT_JOYSTICK2_Y = false;

// ===============================
// --- Joystick Variables
// ===============================
int xAxis1Raw = 0, yAxis1Raw = 0;
int xAxis2Raw = 0, yAxis2Raw = 0;
int yAxis1TrueRaw = 0, yAxis2TrueRaw = 0;

byte xAxis1Control = 0, yAxis1Control = 0;
byte xAxis2Control = 0, yAxis2Control = 0;


// ===============================
// --- Helper Functions (readJoystickAxis, printOLED, calibrateCenter are unchanged)
// ===============================
int readJoystickAxis(int pin) {
    const int numSamples = 10;
    unsigned long total = 0;
    for (int i = 0; i < numSamples; i++) {
        total += analogRead(pin);
    }
    return total / numSamples;
}

// ===============================
// --- MAPPING and DEADZONE Function
// ===============================
byte mapAxisTo255(int rawValue, int center) {
    // 1. Apply Deadzone
    if (rawValue > (center - DEAD_ZONE) && rawValue < (center + DEAD_ZONE)) {
        return 127; 
    }

    // 2. Map the value
    rawValue = constrain(rawValue, 0, MAX_ADC_VALUE);
    long mappedValue = map(rawValue, 0, MAX_ADC_VALUE, 0, 255);
    
    return (byte)mappedValue;
}

