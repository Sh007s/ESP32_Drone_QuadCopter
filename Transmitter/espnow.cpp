#include "espnow.h"

uint8_t broadcastAddress[] = { 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF };
//unit8_t myMac[6];

void espnow_init() {
  WiFi.mode(WIFI_STA);
  Serial.println("ESP_NOW Transmitter");

  // Initialize ESP-NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("ESP_NOW init Failed");
    return;
  }

  esp_now_register_send_cb(onDataSent);

  // Add broadcast peer
  esp_now_peer_info_t peerInfo = {};
  memcpy(peerInfo.peer_addr, broadcastAddress, 6);
  peerInfo.channel = 0;
  peerInfo.encrypt = false;

  esp_err_t status = esp_now_add_peer(&peerInfo);
  if (status == ESP_OK) {
    Serial.println("Broadcast peer added");
  } else {
    Serial.printf("Failed to add peer. Error: %d\n", status);
  }

  esp_now_register_recv_cb(OnDataRecv);
}

void onDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  Serial.printf("Send Status: ");
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Success" : "Fail");
}
