// need to the function here
#include "Transmitter.h"
#include "Joystick.h"
#include "OLED.h"

ControlData Control;

void OLED_display_show() {

  display.clearDisplay();
  display.setTextSize(2);  // Increased size for control values

  // Row 1: Throttle (T) and Yaw (Y)
  display.setCursor(0, 0);
  display.printf("T:%3d", Control.Throttle);
  display.setCursor(64, 0);  // Move cursor to the middle for the second value
  display.printf("Y:%3d", Control.Yaw);

  // Row 2: Pitch (P) and Roll (R)
  display.setCursor(0, 20);  // Move down
  display.printf("P:%3d", Control.Pitch);
  display.setCursor(64, 20);  // Move to the middle
  display.printf("R:%3d", Control.Roll);

  display.setTextSize(1);                                     // Set size back to 1 for debug info
  display.drawFastHLine(0, 40, SCREEN_WIDTH, SSD1306_WHITE);  // Separator

  // Extra Debug/Status Line
  display.setCursor(0, 50);
  display.printf("Center: %4d | DZ: %d", CENTER_POINT, DEAD_ZONE);

  display.display();
}