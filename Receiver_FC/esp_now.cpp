#include "esp_now.h"

uint8_t transmitterMAC[6];
bool macStored = false;
bool isArmed = false;

bool failsafe = false;
bool armed = false;
unsigned long lastPacketTime = 0;

Control_t Control;
TelemetryPacket_t telemetry;  // <-- DEFINITION ADDED HERE!
bldcmotors_t bldcmotors;      // This definition was also missing.

// ===================== ESP-NOW CALLBACK =====================
void OnDataRecv(const uint8_t *mac, const uint8_t *incomingData, int len) {

  if (!macStored) {
    memcpy(transmitterMAC, mac, 6);
    macStored = true;

    esp_now_peer_info_t peerInfo = {};
    memcpy(peerInfo.peer_addr, transmitterMAC, 6);
    peerInfo.channel = 0;
    peerInfo.encrypt = false;
    esp_now_add_peer(&peerInfo);
  }

  memcpy(&Control, incomingData, sizeof(Control));
  lastPacketTime = millis();
  failsafe = false;
}
