#include <WiFi.h>
#include <esp_now.h>

struct ControlData {
  byte Throttle;  // Vertical thrust control (0-255)
  byte Yaw;       // Rotation around the Z-axis (0=Left, 127=Center, 255=Right)
  byte Pitch;     // Forward/Backward movement (0=Forward/Backward, 127=Center)
  byte Roll;      // Left/Right movement (0=Left, 127=Center, 255=Right)
} Control;

void OnDataRecv(const uint8_t *mac, const uint8_t *data, int len) {
  memcpy(&Control, data, sizeof(Control));
  Serial.print("Received form : ");
  for (int i = 0; i < 6; i++) {
    Serial.printf("%02X", mac[i]);
    if (i < 5)
      Serial.print(":");
  }

  Serial.print(" | Throttle : ");
  Serial.print(Control.Throttle);
  Serial.print(" | Yaw : ");
  Serial.print(Control.Yaw);
  Serial.print(" | Pitch : ");
  Serial.print(Control.Pitch);
  Serial.print(" | Roll : ");
  Serial.print(Control.Roll);
  Serial.println();
}
void setup() {
  Serial.begin(115200);
  WiFi.mode(WIFI_STA);
  Serial.println("ESP-NOW Receiver");

  if (esp_now_init() != ESP_OK) {
    Serial.println("ESP-NOW init failed");
    return;
  }

  esp_now_register_recv_cb(OnDataRecv);
}
void loop() {
}
