#include "proximity_espnow.h"

#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>

#include <math.h>
#include <string.h>

namespace proximity_espnow {
namespace {

constexpr char kLogPrefix[] = "[proximity] ";
constexpr uint8_t kBroadcastMac[6] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
constexpr uint32_t kBeaconSignature = 0x31584F52UL;  // "ROX1"
constexpr uint8_t kProtocolVersion = 1;
constexpr size_t kEventQueueSize = 16;
constexpr size_t kSniffCacheSize = 24;
constexpr uint32_t kSniffMatchWindowMs = 800U;
constexpr uint32_t kNeighborForgetMultiplier = 2U;

struct BeaconPacket {
  uint32_t signature;
  uint8_t protocolVersion;
  uint8_t reserved;
  uint16_t flags;
  uint32_t nodeId;
  uint32_t seq;
  uint32_t uptimeMs;
} __attribute__((packed));

struct PendingEvent {
  bool active = false;
  NeighborState state = NeighborState::Unknown;
  NeighborInfo neighbor;
};

struct SniffCacheEntry {
  bool active = false;
  uint8_t mac[6] = {0};
  uint32_t seq = 0;
  int8_t rssi = -127;
  uint32_t capturedAtMs = 0;
};

class ProximityManager {
 public:
  static ProximityManager &instance() {
    static ProximityManager manager;
    return manager;
  }

  bool init(const Config *config) {
    if (!isBuildEnabled()) {
      return false;
    }

    if (running_) {
      return true;
    }

    config_ = config ? *config : Config{};
    if (config_.minNearConfirmations == 0) {
      config_.minNearConfirmations = 1;
    }
    if (config_.rssiAlpha <= 0.0f || config_.rssiAlpha > 1.0f) {
      config_.rssiAlpha = PROXIMITY_ESPNOW_RSSI_ALPHA;
    }

    WiFi.mode(WIFI_STA);
    WiFi.setSleep(false);

    if (esp_wifi_get_mac(WIFI_IF_STA, localMac_) != ESP_OK) {
      Serial.printf("%sfailed to read local STA MAC\r\n", kLogPrefix);
      return false;
    }

    localNodeId_ = buildNodeId(localMac_);

    if (esp_now_init() != ESP_OK) {
      Serial.printf("%sesp_now_init failed\r\n", kLogPrefix);
      return false;
    }

    esp_now_register_recv_cb(&ProximityManager::onEspNowReceive);
    esp_now_register_send_cb(&ProximityManager::onEspNowSend);

    esp_now_peer_info_t peerInfo = {};
    memcpy(peerInfo.peer_addr, kBroadcastMac, sizeof(kBroadcastMac));
    peerInfo.channel = 0;
    peerInfo.ifidx = WIFI_IF_STA;
    peerInfo.encrypt = false;
    const esp_err_t addPeerResult = esp_now_add_peer(&peerInfo);
    if (addPeerResult != ESP_OK && addPeerResult != ESP_ERR_ESPNOW_EXIST) {
      Serial.printf("%sfailed to add broadcast peer: %d\r\n", kLogPrefix, static_cast<int>(addPeerResult));
      esp_now_deinit();
      return false;
    }

    wifi_promiscuous_filter_t filter = {};
    filter.filter_mask = WIFI_PROMIS_FILTER_MASK_MGMT;
    esp_wifi_set_promiscuous_filter(&filter);
    esp_wifi_set_promiscuous_rx_cb(&ProximityManager::onPromiscuousPacket);
    esp_wifi_set_promiscuous(true);

    memset(neighbors_, 0, sizeof(neighbors_));
    memset(events_, 0, sizeof(events_));
    memset(sniffCache_, 0, sizeof(sniffCache_));

    nextBeaconAtMs_ = millis() + config_.beaconPeriodMs;
    lastMaintenanceAtMs_ = millis();
    running_ = true;

    Serial.printf("%senabled node_id=%08lX mac=%02X:%02X:%02X:%02X:%02X:%02X\r\n", kLogPrefix,
                  static_cast<unsigned long>(localNodeId_), localMac_[0], localMac_[1], localMac_[2],
                  localMac_[3], localMac_[4], localMac_[5]);
    return true;
  }

  void stop() {
    if (!running_) {
      return;
    }

    esp_wifi_set_promiscuous(false);
    esp_wifi_set_promiscuous_rx_cb(nullptr);
    esp_now_unregister_recv_cb();
    esp_now_unregister_send_cb();
    esp_now_deinit();
    running_ = false;
  }

  void update() {
    if (!running_) {
      return;
    }

    const uint32_t now = millis();
    if (timeReached(now, nextBeaconAtMs_)) {
      sendBeacon(now);
      nextBeaconAtMs_ = now + config_.beaconPeriodMs;
    }

    if (timeReached(now, lastMaintenanceAtMs_ + config_.maintenancePeriodMs)) {
      performMaintenance(now);
      lastMaintenanceAtMs_ = now;
    }

    dispatchPendingEvents();
  }

  bool isRunning() const { return running_; }

  void setLocalFlags(uint16_t flags) {
    portENTER_CRITICAL(&lock_);
    localFlags_ = flags;
    portEXIT_CRITICAL(&lock_);
  }

  uint16_t getLocalFlags() const { return localFlags_; }

  uint32_t getLocalNodeId() const { return localNodeId_; }

  bool getLocalMac(uint8_t mac[6]) const {
    if (!mac) {
      return false;
    }
    memcpy(mac, localMac_, sizeof(localMac_));
    return true;
  }

  void setCallbacks(NeighborCallback onNear, NeighborCallback onFar) {
    onNear_ = onNear;
    onFar_ = onFar;
  }

  size_t getNeighborCount() const {
    size_t count = 0;
    portENTER_CRITICAL(&lock_);
    for (const NeighborInfo &neighbor : neighbors_) {
      if (neighbor.active) {
        ++count;
      }
    }
    portEXIT_CRITICAL(&lock_);
    return count;
  }

  size_t getNearNeighborCount() const {
    size_t count = 0;
    portENTER_CRITICAL(&lock_);
    for (const NeighborInfo &neighbor : neighbors_) {
      if (neighbor.active && neighbor.state == NeighborState::Near) {
        ++count;
      }
    }
    portEXIT_CRITICAL(&lock_);
    return count;
  }

  bool hasAnyNearNeighbor() const { return getNearNeighborCount() > 0; }

  size_t copyNeighbors(NeighborInfo *outNeighbors, size_t maxNeighbors) const {
    if (!outNeighbors || maxNeighbors == 0) {
      return 0;
    }

    size_t copied = 0;
    portENTER_CRITICAL(&lock_);
    for (const NeighborInfo &neighbor : neighbors_) {
      if (!neighbor.active || copied >= maxNeighbors) {
        continue;
      }
      outNeighbors[copied++] = neighbor;
    }
    portEXIT_CRITICAL(&lock_);
    return copied;
  }

 private:
  static bool timeReached(uint32_t now, uint32_t target) {
    return static_cast<int32_t>(now - target) >= 0;
  }

  static uint32_t buildNodeId(const uint8_t mac[6]) {
    return (static_cast<uint32_t>(mac[2]) << 24) | (static_cast<uint32_t>(mac[3]) << 16) |
           (static_cast<uint32_t>(mac[4]) << 8) | static_cast<uint32_t>(mac[5]);
  }

  static bool macEquals(const uint8_t *lhs, const uint8_t *rhs) {
    return memcmp(lhs, rhs, 6) == 0;
  }

  static bool isLikelyOwnPacket(const uint8_t *mac) { return macEquals(mac, instance().localMac_); }

  static void onEspNowSend(const uint8_t *, esp_now_send_status_t) {}

  static void onEspNowReceive(const uint8_t *macAddr, const uint8_t *data, int dataLen) {
    instance().handleEspNowReceive(macAddr, data, dataLen);
  }

  static void onPromiscuousPacket(void *buf, wifi_promiscuous_pkt_type_t type) {
    instance().handlePromiscuousPacket(buf, type);
  }

  void handleEspNowReceive(const uint8_t *macAddr, const uint8_t *data, int dataLen) {
    if (!running_ || !macAddr || !data || dataLen != static_cast<int>(sizeof(BeaconPacket))) {
      return;
    }

    BeaconPacket packet = {};
    memcpy(&packet, data, sizeof(packet));
    if (packet.signature != kBeaconSignature || packet.protocolVersion != kProtocolVersion) {
      return;
    }
    if (isLikelyOwnPacket(macAddr) || packet.nodeId == localNodeId_) {
      return;
    }

    const int8_t rssi = findSniffedRssi(macAddr, packet.seq);
    const uint32_t now = millis();

    portENTER_CRITICAL(&lock_);
    NeighborInfo *neighbor = findOrCreateNeighborSlot(macAddr);
    if (neighbor) {
      applyReceivedBeacon(*neighbor, macAddr, packet, rssi, now);
    }
    portEXIT_CRITICAL(&lock_);
  }

  void handlePromiscuousPacket(void *buf, wifi_promiscuous_pkt_type_t type) {
    if (!running_ || type != WIFI_PKT_MGMT || !buf) {
      return;
    }

    const wifi_promiscuous_pkt_t *packet = static_cast<const wifi_promiscuous_pkt_t *>(buf);
    const uint8_t *frame = packet->payload;
    const uint16_t frameLen = packet->rx_ctrl.sig_len;
    if (!frame || frameLen < (24U + sizeof(BeaconPacket))) {
      return;
    }

    const uint8_t *sourceMac = frame + 10;
    for (uint16_t offset = 24; offset + sizeof(BeaconPacket) <= frameLen; ++offset) {
      BeaconPacket candidate = {};
      memcpy(&candidate, frame + offset, sizeof(candidate));
      if (candidate.signature != kBeaconSignature || candidate.protocolVersion != kProtocolVersion) {
        continue;
      }

      portENTER_CRITICAL_ISR(&lock_);
      storeSniffedRssi(sourceMac, candidate.seq, packet->rx_ctrl.rssi, millis());
      portEXIT_CRITICAL_ISR(&lock_);
      return;
    }
  }

  void sendBeacon(uint32_t now) {
    BeaconPacket packet = {};
    packet.signature = kBeaconSignature;
    packet.protocolVersion = kProtocolVersion;
    packet.flags = localFlags_;
    packet.nodeId = localNodeId_;
    packet.seq = ++beaconSeq_;
    packet.uptimeMs = now;

    const esp_err_t result =
        esp_now_send(kBroadcastMac, reinterpret_cast<const uint8_t *>(&packet), sizeof(packet));
    if (result != ESP_OK && result != ESP_ERR_ESPNOW_NO_MEM) {
      Serial.printf("%sesp_now_send failed: %d\r\n", kLogPrefix, static_cast<int>(result));
    }
  }

  void performMaintenance(uint32_t now) {
    const uint32_t forgetAfterMs = config_.neighborTimeoutMs * kNeighborForgetMultiplier;

    portENTER_CRITICAL(&lock_);
    for (NeighborInfo &neighbor : neighbors_) {
      if (!neighbor.active) {
        continue;
      }

      const uint32_t ageMs = now - neighbor.lastSeenMs;
      if (ageMs <= config_.neighborTimeoutMs) {
        continue;
      }

      if (neighbor.lossCount < 0xFF) {
        ++neighbor.lossCount;
      }

      if (neighbor.state != NeighborState::Far) {
        transitionNeighborState(neighbor, NeighborState::Far);
      }

      neighbor.proximityConfirmations = 0;

      if (ageMs > forgetAfterMs) {
        memset(&neighbor, 0, sizeof(neighbor));
      }
    }
    portEXIT_CRITICAL(&lock_);
  }

  void dispatchPendingEvents() {
    PendingEvent events[kEventQueueSize];
    size_t eventCount = 0;

    portENTER_CRITICAL(&lock_);
    for (PendingEvent &event : events_) {
      if (!event.active) {
        continue;
      }
      events[eventCount++] = event;
      event.active = false;
      if (eventCount >= kEventQueueSize) {
        break;
      }
    }
    portEXIT_CRITICAL(&lock_);

    for (size_t i = 0; i < eventCount; ++i) {
      const PendingEvent &event = events[i];
      if (event.state == NeighborState::Near && onNear_) {
        onNear_(event.neighbor);
      } else if (event.state == NeighborState::Far && onFar_) {
        onFar_(event.neighbor);
      }
    }
  }

  NeighborInfo *findOrCreateNeighborSlot(const uint8_t *macAddr) {
    NeighborInfo *freeSlot = nullptr;

    for (NeighborInfo &neighbor : neighbors_) {
      if (neighbor.active && macEquals(neighbor.mac, macAddr)) {
        return &neighbor;
      }
      if (!neighbor.active && !freeSlot) {
        freeSlot = &neighbor;
      }
    }

    return freeSlot;
  }

  void applyReceivedBeacon(NeighborInfo &neighbor, const uint8_t *macAddr, const BeaconPacket &packet, int8_t rssi,
                           uint32_t now) {
    const bool isNewNeighbor = !neighbor.active;

    if (isNewNeighbor) {
      memset(&neighbor, 0, sizeof(neighbor));
      memcpy(neighbor.mac, macAddr, 6);
      neighbor.averageRssi = static_cast<float>(rssi);
      neighbor.state = NeighborState::Far;
      neighbor.active = true;
    } else {
      neighbor.averageRssi =
          (config_.rssiAlpha * static_cast<float>(rssi)) + ((1.0f - config_.rssiAlpha) * neighbor.averageRssi);
    }

    neighbor.nodeId = packet.nodeId;
    neighbor.remoteUptimeMs = packet.uptimeMs;
    neighbor.flags = packet.flags;
    neighbor.lastSeq = packet.seq;
    neighbor.lastSeenMs = now;
    neighbor.lastRssi = rssi;
    neighbor.lossCount = 0;

    if (neighbor.state != NeighborState::Near) {
      if (neighbor.averageRssi >= static_cast<float>(config_.rssiNearEnter)) {
        if (neighbor.proximityConfirmations < 0xFF) {
          ++neighbor.proximityConfirmations;
        }
        if (neighbor.proximityConfirmations >= config_.minNearConfirmations) {
          transitionNeighborState(neighbor, NeighborState::Near);
        }
      } else {
        neighbor.proximityConfirmations = 0;
      }
      return;
    }

    if (neighbor.averageRssi <= static_cast<float>(config_.rssiNearExit)) {
      neighbor.proximityConfirmations = 0;
      transitionNeighborState(neighbor, NeighborState::Far);
    }
  }

  void transitionNeighborState(NeighborInfo &neighbor, NeighborState nextState) {
    if (neighbor.state == nextState) {
      return;
    }

    neighbor.state = nextState;
    for (PendingEvent &event : events_) {
      if (!event.active) {
        event.active = true;
        event.state = nextState;
        event.neighbor = neighbor;
        return;
      }
    }
  }

  void storeSniffedRssi(const uint8_t *macAddr, uint32_t seq, int8_t rssi, uint32_t capturedAtMs) {
    SniffCacheEntry *target = nullptr;
    for (SniffCacheEntry &entry : sniffCache_) {
      if (entry.active && entry.seq == seq && macEquals(entry.mac, macAddr)) {
        target = &entry;
        break;
      }
      if (!entry.active && !target) {
        target = &entry;
      }
    }

    if (!target) {
      target = &sniffCache_[sniffCacheWriteIndex_++ % kSniffCacheSize];
    }

    target->active = true;
    memcpy(target->mac, macAddr, 6);
    target->seq = seq;
    target->rssi = rssi;
    target->capturedAtMs = capturedAtMs;
  }

  int8_t findSniffedRssi(const uint8_t *macAddr, uint32_t seq) {
    const uint32_t now = millis();
    int8_t fallbackRssi = -127;
    uint32_t newestAtMs = 0;

    for (SniffCacheEntry &entry : sniffCache_) {
      if (!entry.active) {
        continue;
      }
      if ((now - entry.capturedAtMs) > kSniffMatchWindowMs) {
        entry.active = false;
        continue;
      }
      if (!macEquals(entry.mac, macAddr)) {
        continue;
      }
      if (entry.seq == seq) {
        const int8_t rssi = entry.rssi;
        entry.active = false;
        return rssi;
      }
      if (entry.capturedAtMs >= newestAtMs) {
        newestAtMs = entry.capturedAtMs;
        fallbackRssi = entry.rssi;
      }
    }

    return fallbackRssi;
  }

  Config config_ = {};
  mutable portMUX_TYPE lock_ = portMUX_INITIALIZER_UNLOCKED;
  NeighborInfo neighbors_[PROXIMITY_ESPNOW_MAX_NEIGHBORS] = {};
  PendingEvent events_[kEventQueueSize] = {};
  SniffCacheEntry sniffCache_[kSniffCacheSize] = {};
  NeighborCallback onNear_ = nullptr;
  NeighborCallback onFar_ = nullptr;
  uint8_t localMac_[6] = {0};
  uint32_t localNodeId_ = 0;
  uint16_t localFlags_ = 0;
  uint32_t beaconSeq_ = 0;
  uint32_t nextBeaconAtMs_ = 0;
  uint32_t lastMaintenanceAtMs_ = 0;
  size_t sniffCacheWriteIndex_ = 0;
  bool running_ = false;
};

ProximityManager &manager() { return ProximityManager::instance(); }

}  // namespace

bool isBuildEnabled() { return PROXIMITY_ESPNOW_ENABLED != 0; }

bool init(const Config *config) { return manager().init(config); }

void update() { manager().update(); }

void stop() { manager().stop(); }

bool isRunning() { return manager().isRunning(); }

void setLocalFlags(uint16_t flags) { manager().setLocalFlags(flags); }

uint16_t getLocalFlags() { return manager().getLocalFlags(); }

uint32_t getLocalNodeId() { return manager().getLocalNodeId(); }

bool getLocalMac(uint8_t mac[6]) { return manager().getLocalMac(mac); }

void setCallbacks(NeighborCallback onNear, NeighborCallback onFar) { manager().setCallbacks(onNear, onFar); }

size_t getNeighborCount() { return manager().getNeighborCount(); }

size_t getNearNeighborCount() { return manager().getNearNeighborCount(); }

bool hasAnyNearNeighbor() { return manager().hasAnyNearNeighbor(); }

size_t copyNeighbors(NeighborInfo *outNeighbors, size_t maxNeighbors) {
  return manager().copyNeighbors(outNeighbors, maxNeighbors);
}

}  // namespace proximity_espnow
