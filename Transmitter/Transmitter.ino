#include <Arduino.h>
#include <Wire.h>
#include <PCF8574.h>
#include "espnow.h"
#include "Joystick.h"
#include "OLED.h"
#include "Transmitter.h"

#define LONG_PRESS_TIME 600  // ms
#define Toggle_Switch1 0
#define Toggle_Switch2 1
#define Tactile_Button1 2
#define Tactile_Button2 3
#define PCF_ADDR 0x20

PCF8574 pcf(PCF_ADDR);

enum MenuState {
  MENU_MAIN,
  MENU_TX_DATA,
  MENU_RX_DATA,
  MENU_PID
};

TelemetryPacket telemetry;

uint8_t receiverMAC[6];
bool receiverFound = false;
bool macStored = false;

PID_t PID = { 1.5, 0.3, 0.05 };  // default values

MenuState currentMenu = MENU_MAIN;
int cursorIndex = 0;

const bool BUTTONS_ACTIVE_LOW = true;  // true if button pulls pin LOW when pressed
const uint8_t BUTTON_COUNT = 4;        // number of buttons used (<=8)
// Debounce
const unsigned long DEBOUNCE_MS = 30;

// Map your buttons to PCF8574 pins (P0..P7).
const uint8_t BUTTON_PINS[BUTTON_COUNT] = { 0, 1, 2, 3 };

uint8_t readPCF8574Buttons();
uint8_t buttons = 0;

// == == == == == == == == == == = ESP - NOW CALLBACK == == == == == == == == == == =
void OnDataRecv(const uint8_t *mac, const uint8_t *incomingData, int len) {

  if (!receiverFound && len == 6) {
    memcpy(receiverMAC, incomingData, 6);
    receiverFound = true;

    esp_now_peer_info_t peerInfo = {};
    memcpy(peerInfo.peer_addr, receiverMAC, 6);
    peerInfo.channel = 0;
    peerInfo.encrypt = false;
    esp_now_add_peer(&peerInfo);
    Serial.println("Receiver MAC stored:");
    for (int i = 0; i < 6; i++) { Serial.printf("%02X:", receiverMAC[i]); }
    return;
  }
  // If length matches telemetry struct → store telemetry
  if (len == sizeof(telemetry)) {
    memcpy(&telemetry, incomingData, sizeof(telemetry));
    Serial.println("Telemetry Received!");
  }
}

uint8_t readPCF8574Buttons() {

  uint8_t packed = 0;

  for (uint8_t i = 0; i < BUTTON_COUNT; i++) {
    int val = pcf.readButton(BUTTON_PINS[i]);
    bool pressed = (val == 0);
    if (pressed) packed |= (1 << i);
  }
  return packed;
}

void handleMenuButtons() {

  buttons = readPCF8574Buttons();  // <-- single variable

  bool UP = buttons & (1 << Tactile_Button1);
  bool DOWN = buttons & (1 << Tactile_Button2);

  static bool upPressed = false;
  static bool downPressed = false;

  static bool upLongDone = false;
  static bool downLongDone = false;

  static unsigned long upStart = 0;
  static unsigned long downStart = 0;

  unsigned long now = millis();

  // ---------------- UP BUTTON (Short = UP, Long = SELECT)
  if (UP) {
    if (!upPressed) {
      upPressed = true;
      upStart = now;
      upLongDone = false;
    } else if (!upLongDone && (now - upStart >= LONG_PRESS_TIME)) {
      Serial.println("DEBUG: Long Press SELECT");
      performSelect();
      upLongDone = true;
    }
  } else {
    if (upPressed && !upLongDone && (now - upStart < LONG_PRESS_TIME)) {
      Serial.println("DEBUG: Short Press UP");
      performUp();
    }
    upPressed = false;
    upLongDone = false;  // Reset
  }

  // ---------------- DOWN BUTTON (Short = DOWN, Long = BACK)
  if (DOWN) {
    if (!downPressed) {
      downPressed = true;
      downStart = now;
      downLongDone = false;
    } else if (!downLongDone && (now - downStart >= LONG_PRESS_TIME)) {
      Serial.println("DEBUG: Long Press BACK");
      performBack();
      downLongDone = true;
    }
  } else {
    if (downPressed && !downLongDone && (now - downStart < LONG_PRESS_TIME)) {
      Serial.println("DEBUG: Short Press DOWN");
      performDown();
    }
    downPressed = false;
    downLongDone = false;  // reset
  }
}

// ---------------- MENU ACTIONS ----------------
void performUp() {
  cursorIndex--;
  Serial.print("Buttons: ");
  Serial.println(buttons, BIN);
  if (cursorIndex < 0) cursorIndex = 2;
  Serial.println("ACTION: performUp()");
}

void performDown() {
  cursorIndex++;
  Serial.print("Buttons: ");
  Serial.println(buttons, BIN);
  if (cursorIndex > 2) cursorIndex = 0;
  Serial.println("ACTION: performDown()");
}

void performSelect() {
  if (currentMenu == MENU_MAIN) {
    if (cursorIndex == 0) currentMenu = MENU_TX_DATA;
    if (cursorIndex == 1) currentMenu = MENU_RX_DATA;
    if (cursorIndex == 2) currentMenu = MENU_PID;
  }
  Serial.print("Buttons: ");
  Serial.println(buttons, BIN);
  Serial.println("ACTION: performSelect()");
}
void performBack() {
  currentMenu = MENU_MAIN;
  Serial.print("Buttons: ");
  Serial.println(buttons, BIN);
  Serial.println("ACTION: performBack()");
}

// ------------------ DRAWING MENUS --------------------
void drawMainMenu() {
  OLED_Clear();
  OLED_PrintLine(0, "Main Menu");
  OLED_PrintLine(1, cursorIndex == 0 ? "> TX Data" : "  TX Data");
  OLED_PrintLine(2, cursorIndex == 1 ? "> RX Data" : "  RX Data");
  OLED_PrintLine(3, cursorIndex == 2 ? "> PID Page" : "  PID Page");
  OLED_Update();
}

void drawTXData() {
  OLED_Clear();
  OLED_PrintLine(0, "TX DATA");

  char line1[32], line2[32], line3[16];
  sprintf(line1, "T:%3d  P:%3d", Control.Throttle, Control.Pitch);
  sprintf(line2, "Y:%3d  R:%3d", Control.Yaw, Control.Roll);
  sprintf(line3, "Mode :%3d", Control.Mode);

  OLED_PrintLine(1, line1);
  OLED_PrintLine(2, line2);
  OLED_PrintLine(3, line3);
  OLED_Update();
}
void drawRXData() {
  OLED_Clear();
  OLED_PrintLine(0, "RX DATA");

  char line1[32], line2[32], line3[32];

  snprintf(line1, sizeof(line1), "m1:%d m2:%d", telemetry.m1, telemetry.m2);
  snprintf(line2, sizeof(line2), "m3:%d m4:%d", telemetry.m3, telemetry.m4);
  snprintf(line3, sizeof(line3), "Y:%d P:%d R:%d", telemetry.yaw, telemetry.pitch, telemetry.roll);


  OLED_PrintLine(1, line1);
  OLED_PrintLine(2, line2);
  OLED_PrintLine(3, line3);
  OLED_Update();
}


void drawPID() {
  OLED_Clear();
  OLED_PrintLine(0, "PID VALUES");

  char p[32], i[32], d[32];
  sprintf(p, "P: %.2f", PID.P);
  sprintf(i, "I: %.2f", PID.I);
  sprintf(d, "D: %.2f", PID.D);

  OLED_PrintLine(1, p);
  OLED_PrintLine(2, i);
  OLED_PrintLine(3, d);
  OLED_Update();
}

void drawMenu() {
  switch (currentMenu) {
    case MENU_MAIN: drawMainMenu(); break;
    case MENU_TX_DATA: drawTXData(); break;
    case MENU_RX_DATA: drawRXData(); break;
    case MENU_PID: drawPID(); break;
  }
}

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

  handleMenuButtons();  // <-- FIXED: Menu button processing here

  //   // --- 1. Read Raw Values ---
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
  Control.Mode = buttons;

  // // --- Serial Output (using the new struct) ---
  // Serial.printf("T:%3d P:%3d Y:%3d R:%3d\n",
  //               Control.Throttle, Control.Pitch,
  //               Control.Yaw, Control.Roll);

  // // --- Serial Output (Control values are 0-255) ---
  // Serial.printf("J1X:%3d J1Y:%3d | J2X:%3d J2Y:%3d\n",
  //               xAxis1Control, yAxis1Control,
  //               xAxis2Control, yAxis2Control);
  // Serial.printf("DEBUG: J1Y Corr: %4d | True Raw: %4d\n", yAxis1Raw, yAxis1TrueRaw);

  esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *)&Control, sizeof(Control));
  if (result == ESP_OK) {
    //  Serial.println("Broadcast sent");
  } else {
    // Serial.print("Send error: ");
    // Serial.println(result);
  }

  // -------- OLED MENU DRAW ----------
  drawMenu();

  delay(100);
}