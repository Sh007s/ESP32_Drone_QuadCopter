#define ESC1_PIN 13
#define ESC2_PIN 12
#define ESC3_PIN 14
#define ESC4_PIN 27

#define SDA_PIN 21
#define SCL_PIN 22

#define ARM_VALUE 1500
#define MIN_PULSE 1000
#define MAX_PULSE 2000

#define DEADZONE_LOW 1490
#define DEADZONE_HIGH 1510
#define THROTTLE_DEADZONE 10  // +/- 10 around center

#define RAMP_STEP 4  // Smoothness (2–6 recommended)
#define FAILSAFE_TIMEOUT 500

// PID / P-controller constants
#define MAX_ANGLE_CMD 20.0f
#define KP_ROLL 4.0f   //2.0
#define KP_PITCH 4.0f  //2.0
#define KP_YAW 1.5f
#define IDLE_THROTTLE 40  // keeps motors alive for control

// == PID CONSTANTS (NEW) ==
#define KI_ROLL 0.005f  // Integral gain
#define KD_ROLL 0.5f    // Derivative gain
#define KI_PITCH 0.005f
#define KD_PITCH 0.5f
#define KI_YAW 0.0f  // Integral may not be needed for rate-controlled Yaw
#define KD_YAW 0.0f  // Define KD_YAW to resolve the error

#define MAX_I_TERM 400.0f  // Anti-windup limit