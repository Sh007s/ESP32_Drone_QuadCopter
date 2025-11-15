#include "OLED.h"
#include "Joystick.h"

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

void OLED_Display_init() {
  Wire.begin(OLED_SDA, OLED_SCL);
  if (!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {
    Serial.println(F("SSD1306 allocation failed"));
    for (;;)
      ;
  }

  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
}

void Dynamic_calibration() {
  // --- Perform Dynamic Calibration ---
  Serial.println("--- Starting Calibration ---");
  CENTER_POINT = calibrateCenter(JOYSTICK1_X_PIN, "J1 X/All Axes");

  Serial.println("--- Calibration Complete ---");
  Serial.printf("Base Center Point: %d\n", CENTER_POINT);
  Serial.printf("Y-Axis Bias Magnitude (subtracted): %d\n", Y_AXIS_BIAS_MAGNITUDE);

  display.clearDisplay();
  display.setCursor(0, 0);
  display.println("Calibration Done!");
  display.setCursor(0, 15);
  display.printf("Base Center: %d", CENTER_POINT);
  display.setCursor(0, 25);
  display.printf("Y Subtract: %d", Y_AXIS_BIAS_MAGNITUDE);
  display.display();
}

void printOLED(const char* line1, const char* line2, int value) {
  display.clearDisplay();
  display.setCursor(0, 0);
  display.println(line1);
  display.setCursor(0, 16);
  display.println(line2);
  if (value >= 0) {
    display.setCursor(0, 32);
    display.printf("ADC: %d", value);
  }
  display.display();
}

int calibrateCenter(int pin, const char* label) {
  const int CALIBRATION_SAMPLES = 100;
  long total = 0;

  Serial.printf("Calibrating %s ...\n", label);
  printOLED("CALIBRATING", label);
  display.setTextSize(1);

  for (int i = 0; i < CALIBRATION_SAMPLES; i++) {
    int raw = analogRead(pin);
    total += raw;
    if (i % 10 == 0) {
      display.clearDisplay();
      display.setCursor(0, 0);
      display.printf("%s CENTER CAL", label);
      display.setCursor(0, 16);
      display.printf("Sample: %d/%d", i + 1, CALIBRATION_SAMPLES);
      display.setCursor(0, 32);
      display.printf("Raw ADC: %d", raw);
      display.display();
    }
    delay(10);
  }
  int centerValue = total / CALIBRATION_SAMPLES;
  Serial.printf("%s Center Calibrated to: %d\n", label, centerValue);
  display.clearDisplay();
  display.setCursor(0, 0);
  display.printf("%s Center", label);
  display.setCursor(0, 16);
  display.printf("Calibrated = %d", centerValue);
  display.display();
  delay(1000);
  return centerValue;
}

