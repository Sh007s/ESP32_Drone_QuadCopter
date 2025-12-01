#ifndef OLED_H
#define OLED_H

#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

// ===============================
// --- OLED Display Configuration
// ===============================
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET -1
#define OLED_SDA 21      //
#define OLED_SCL 22      // use the PCF8574 IC I2C interface 
#define SCREEN_ADDRESS 0x3C

extern Adafruit_SSD1306 display;

void OLED_Display_init();
void Dynamic_calibration();
void printOLED(const char* line1, const char* line2, int value = -1);
int calibrateCenter(int pin, const char* label);
void OLED_Update();
void OLED_PrintLine(int row, const char* text);
void OLED_Clear() ;


#endif  // OLED_H