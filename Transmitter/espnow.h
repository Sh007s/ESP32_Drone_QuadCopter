#include <WiFi.h>
#include <esp_now.h>

extern uint8_t broadcastAddress[];

void espnow_init();
void onDataSent(const uint8_t *mac_addr, esp_now_send_status_t status);
