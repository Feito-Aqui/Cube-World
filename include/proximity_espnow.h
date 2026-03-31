#pragma once

#include <Arduino.h>

#ifndef PROXIMITY_ESPNOW_ENABLED
#define PROXIMITY_ESPNOW_ENABLED 0
#endif

#ifndef PROXIMITY_ESPNOW_MAX_NEIGHBORS
#define PROXIMITY_ESPNOW_MAX_NEIGHBORS 12
#endif

#ifndef PROXIMITY_ESPNOW_BEACON_PERIOD_MS
#define PROXIMITY_ESPNOW_BEACON_PERIOD_MS 500U
#endif

#ifndef PROXIMITY_ESPNOW_NEIGHBOR_TIMEOUT_MS
#define PROXIMITY_ESPNOW_NEIGHBOR_TIMEOUT_MS 3000U
#endif

#ifndef PROXIMITY_ESPNOW_MAINTENANCE_PERIOD_MS
#define PROXIMITY_ESPNOW_MAINTENANCE_PERIOD_MS 200U
#endif

#ifndef PROXIMITY_ESPNOW_RSSI_NEAR_ENTER
#define PROXIMITY_ESPNOW_RSSI_NEAR_ENTER -29
#endif

#ifndef PROXIMITY_ESPNOW_RSSI_NEAR_EXIT
#define PROXIMITY_ESPNOW_RSSI_NEAR_EXIT -20
#endif

#ifndef PROXIMITY_ESPNOW_MIN_NEAR_CONFIRMATIONS
#define PROXIMITY_ESPNOW_MIN_NEAR_CONFIRMATIONS 3U
#endif

#ifndef PROXIMITY_ESPNOW_RSSI_ALPHA
#define PROXIMITY_ESPNOW_RSSI_ALPHA 0.35f
#endif

namespace proximity_espnow {

enum class NeighborState : uint8_t {
  Unknown = 0,
  Far = 1,
  Near = 2,
};

struct NeighborInfo {
  bool active = false;
  uint8_t mac[6] = {0};
  uint32_t nodeId = 0;
  int8_t lastRssi = -127;
  float averageRssi = -127.0f;
  uint32_t lastSeenMs = 0;
  NeighborState state = NeighborState::Unknown;
  uint8_t proximityConfirmations = 0;
  uint8_t lossCount = 0;
  uint32_t lastSeq = 0;
  uint16_t flags = 0;
};

struct Config {
  uint32_t beaconPeriodMs = PROXIMITY_ESPNOW_BEACON_PERIOD_MS;
  uint32_t neighborTimeoutMs = PROXIMITY_ESPNOW_NEIGHBOR_TIMEOUT_MS;
  uint32_t maintenancePeriodMs = PROXIMITY_ESPNOW_MAINTENANCE_PERIOD_MS;
  int8_t rssiNearEnter = PROXIMITY_ESPNOW_RSSI_NEAR_ENTER;
  int8_t rssiNearExit = PROXIMITY_ESPNOW_RSSI_NEAR_EXIT;
  uint8_t minNearConfirmations = PROXIMITY_ESPNOW_MIN_NEAR_CONFIRMATIONS;
  float rssiAlpha = PROXIMITY_ESPNOW_RSSI_ALPHA;
};

using NeighborCallback = void (*)(const NeighborInfo &neighbor);

bool init(const Config *config = nullptr);
void update();
void stop();

bool isBuildEnabled();
bool isRunning();

void setLocalFlags(uint16_t flags);
uint16_t getLocalFlags();
uint32_t getLocalNodeId();
bool getLocalMac(uint8_t mac[6]);

void setCallbacks(NeighborCallback onNear, NeighborCallback onFar);

size_t getNeighborCount();
size_t getNearNeighborCount();
bool hasAnyNearNeighbor();
size_t copyNeighbors(NeighborInfo *outNeighbors, size_t maxNeighbors);

}  // namespace proximity_espnow
