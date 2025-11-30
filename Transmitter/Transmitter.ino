#include <Arduino.h>
#include <Wire.h>
#include <PCF8574.h>
#include "espnow.h"
#include "Joystick.h"
#include "OLED.h"
#include "Transmitter.h"

PCF8574 pcf(0x20);

void setup() {
  Serial.begin(115200);
  OLED_Display_init();
    // Button
  pcf.begin();  // default sets all pins HIGH (input mode)

  printOLED("OLED", "Initializing...");
  // Serial.println("\n--- ESP32 Dual Joystick Reader + OLED Display ---");
  analogReadResolution(12);
  analogSetAttenuation(ADC_11db);
  Dynamic_calibration();
  espnow_init();

  delay(2000);
}

void loop() {
  for (int i = 0; i < 8; i++) {
    // For buttons → use readButton (it handles inverted logic)
    int val = pcf.readButton(i);
    if (val == 0) {
      Serial.print("P");
      Serial.print(i);
      Serial.print(": ");
      Serial.print(val);
      Serial.print(" : Button is Pressed  ");
      Serial.println();
    }
  }
  // --- 1. Read Raw Values ---
  xAxis1Raw = readJoystickAxis(JOYSTICK1_X_PIN);
  yAxis1TrueRaw = readJoystickAxis(JOYSTICK1_Y_PIN);
  xAxis2Raw = readJoystickAxis(JOYSTICK2_X_PIN);
  yAxis2TrueRaw = readJoystickAxis(JOYSTICK2_Y_PIN);

  // --- 2. Apply Fixed Y-Axis Correction ---
  yAxis1Raw = yAxis1TrueRaw - Y_AXIS_BIAS_MAGNITUDE;
  yAxis2Raw = yAxis2TrueRaw - Y_AXIS_BIAS_MAGNITUDE;

  // --- 3. Apply Axis Inversion (Flags are FALSE, so this is skipped) ---
  if (INVERT_JOYSTICK1_X) xAxis1Raw = MAX_ADC_VALUE - xAxis1Raw;
  if (INVERT_JOYSTICK1_Y) yAxis1Raw = MAX_ADC_VALUE - yAxis1Raw;
  if (INVERT_JOYSTICK2_X) xAxis2Raw = MAX_ADC_VALUE - xAxis2Raw;
  if (INVERT_JOYSTICK2_Y) yAxis2Raw = MAX_ADC_VALUE - yAxis2Raw;

  // --- 4. Apply Deadzone and Map to 0-255 ---
  xAxis1Control = mapAxisTo255(xAxis1Raw, CENTER_POINT);
  yAxis1Control = mapAxisTo255(yAxis1Raw, CENTER_POINT);
  xAxis2Control = mapAxisTo255(xAxis2Raw, CENTER_POINT);
  yAxis2Control = mapAxisTo255(yAxis2Raw, CENTER_POINT);

  // ===============================
  // --- 5. Map Control Bytes to Drone Axes (Mode 2)
  // ===============================
  Control.Throttle = yAxis1Control;
  Control.Yaw = xAxis1Control;
  Control.Pitch = yAxis2Control;
  Control.Roll = xAxis2Control;

  // --- Serial Output (using the new struct) ---
  Serial.printf("T:%3d P:%3d Y:%3d R:%3d\n",
                Control.Throttle, Control.Pitch,
                Control.Yaw, Control.Roll);


  // --- Serial Output (Control values are 0-255) ---
  Serial.printf("J1X:%3d J1Y:%3d | J2X:%3d J2Y:%3d\n",
                xAxis1Control, yAxis1Control,
                xAxis2Control, yAxis2Control);
  Serial.printf("DEBUG: J1Y Corr: %4d | True Raw: %4d\n", yAxis1Raw, yAxis1TrueRaw);

  OLED_display_show();

  esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *)&Control, sizeof(Control));
  if (result == ESP_OK) {
    Serial.println("Broadcast sent");
  } else {
    Serial.print("Send error: ");
    Serial.println(result);
  }

  delay(100);
}