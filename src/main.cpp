#include <Arduino.h>
#include <DNSServer.h>
#include <WiFi.h>
#include <WebServer.h>
#include <Wire.h>

#include <LittleFS.h>

#include <memory>

#include "AnimatedGIF.h"
#include "Arduino_DriveBus_Library.h"
#include "Arduino_GFX_Library.h"
#include "proximity_espnow.h"
#include "SensorQMI8658.hpp"

namespace {

constexpr uint16_t kPanelWidth = 240;
constexpr uint16_t kPanelHeight = 280;
constexpr uint16_t kLineBufferSize = (kPanelWidth > kPanelHeight) ? kPanelWidth : kPanelHeight;
constexpr int16_t kGifAreaSize = 240;
constexpr int16_t kReservedBandSize = 40;

constexpr int kLcdSckPin = 6;
constexpr int kLcdMosiPin = 7;
constexpr int kLcdCsPin = 5;
constexpr int kLcdDcPin = 4;
constexpr int kLcdResetPin = 8;
constexpr int kLcdBacklightPin = 15;

constexpr int kTouchSclPin = 10;
constexpr int kTouchSdaPin = 11;
constexpr int kTouchResetPin = 13;
constexpr int kTouchIntPin = 14;

constexpr uint32_t kImuPollIntervalMs = 120;
constexpr uint32_t kOrientationSettleMs = 180;
constexpr uint32_t kDizzyTurnWindowMs = 20000;
constexpr uint32_t kSynchronizedGifSlotMs = 4000;
constexpr uint32_t kNearEffectWindowMs = 30000;
constexpr uint32_t kNearEffectDurationMs = 10000;
constexpr uint8_t kNearEffectChanceDivisor = 4;
constexpr size_t kDizzyTurnThreshold = 4;
constexpr size_t kMaxGifFiles = 16;
constexpr size_t kManagedGifNameCount = 8;
constexpr float kOrientationMagnitudeThreshold = 0.60f;
constexpr float kOrientationDominanceMargin = 0.18f;
constexpr uint8_t kDefaultGifWeight = 5;
constexpr uint16_t kBroadcastGifValidMask = 0x8000U;
constexpr uint16_t kBroadcastGifIndexMask = 0x001FU;
constexpr uint32_t kTouchDebounceMs = 650;
constexpr bool kAutostartMaintenanceOnBoot = true;
constexpr uint8_t kDnsPort = 53;

constexpr char kMaintenanceSsid[] = "CubeWorld-GIF";
constexpr char kMaintenancePassword[] = "";

const char *const kManagedGifNames[kManagedGifNameCount] = {
    "default.gif",       "Scratch_head.gif", "Scratch.gif", "WakeUp_240.gif",
    "Sleep_240.gif",     "turnleft.gif",         "turnRight.gif",   "fart.gif"
};

enum class ScreenOrientation : uint8_t {
  Portrait = 0,
  LandscapeRight = 1,
  PortraitUpsideDown = 2,
  LandscapeLeft = 3,
};

struct TouchPoint {
  uint16_t x;
  uint16_t y;
};

struct Rect {
  int16_t x;
  int16_t y;
  int16_t w;
  int16_t h;
};

Arduino_DataBus *bus = new Arduino_ESP32SPI(kLcdDcPin, kLcdCsPin, kLcdSckPin, kLcdMosiPin);
Arduino_GFX *gfx =
    new Arduino_ST7789(bus, kLcdResetPin, 0, true, kPanelWidth, kPanelHeight, 0, 20, 0, 20);

std::shared_ptr<Arduino_IIC_DriveBus> i2cBus =
    std::make_shared<Arduino_HWIIC>(kTouchSdaPin, kTouchSclPin, &Wire);
std::unique_ptr<Arduino_IIC> touch;
SensorQMI8658 imu;
WebServer webServer(80);
DNSServer dnsServer;

AnimatedGIF gif;
File gifFile;
File uploadFile;
String gifPaths[kMaxGifFiles];
size_t gifCount = 0;
size_t currentGifIndex = 0;
bool gifIsOpen = false;
bool imuReady = false;
bool turnTransitionPending = false;
int16_t gifOffsetX = 0;
int16_t gifOffsetY = 0;
uint16_t lineBuffer[kLineBufferSize];
uint32_t nextFrameAtMs = 0;
uint32_t lastImuPollMs = 0;
uint32_t recentTurnStartedAtMs[kDizzyTurnThreshold] = {};
uint32_t orientationChangedAtMs = 0;
ScreenOrientation currentOrientation = ScreenOrientation::Portrait;
ScreenOrientation pendingOrientation = ScreenOrientation::Portrait;
int pendingDefaultGifIndex = -1;
uint32_t lastLoggedSyncLeaderNodeId = 0;
uint32_t lastLoggedSyncSlot = 0;
int lastLoggedSyncGifIndex = -1;
bool hasLoggedSyncState = false;
bool nearEffectWasActive = false;
uint32_t lastLoggedNearEffectWindow = 0;
uint32_t lastLoggedNearEffectLeaderNodeId = 0;
bool hasLoggedNearEffectState = false;
bool maintenanceModeActive = false;
bool touchToggleRequested = false;
uint32_t lastTouchToggleMs = 0;
String lastMaintenanceMessage;
String pendingUploadTargetPath;
String pendingUploadError;

void gifDraw(GIFDRAW *draw);
void closeCurrentGif();
bool openGifByIndex(size_t index);
void restartCurrentGif();
void drawOrientationBadge();
void drawProximityStatus();
void applyDisplayOrientation(ScreenOrientation orientation, bool force = false, bool reopenGif = true);
bool startTurnTransition(ScreenOrientation nextOrientation, const String &transitionGifName);
int findGifIndexByName(const String &filename);
bool isCounterClockwiseTurn(ScreenOrientation from, ScreenOrientation to);
bool isClockwiseTurn(ScreenOrientation from, ScreenOrientation to);
bool isTurnTransitionGif(const String &path);
bool isDizzyGif(const String &path);
bool isWakeUpGif(const String &path);
bool isSleepGif(const String &path);
bool isRoutineSelectableGif(const String &path);
void sortGifList();
int chooseNextRoutineGifIndex();
uint16_t encodeBroadcastGifFlags(int gifIndex);
int decodeBroadcastGifIndex(uint16_t flags);
void publishCurrentGifSelection();
bool getNearEffectState(bool *activeOut, uint16_t *colorOut, uint32_t *leaderNodeIdOut = nullptr,
                        uint32_t *windowOut = nullptr);
void renderNearEffectIfNeeded();
bool getSynchronizedRoutineGifIndex(size_t *indexOut);
bool getSynchronizedRoutineGifIndex(size_t *indexOut, uint32_t *leaderNodeIdOut, uint32_t *slotOut,
                                    uint32_t *timelineMsOut);
void updateSynchronizedGifPlayback();
void initProximityFeature();
void onNeighborNear(const proximity_espnow::NeighborInfo &neighbor);
void onNeighborFar(const proximity_espnow::NeighborInfo &neighbor);
void onTouchInterrupt();
void initTouch();
bool isManagedGifPath(const String &path);
String canonicalGifPath(const String &name);
bool pathMatchesAnyGifAlias(const String &path, std::initializer_list<const char *> aliases);
void removeLegacyAliasesForPath(const String &path);
int findGifIndexByAnyName(std::initializer_list<const char *> aliases);
void refreshGifListPreservingCurrent();
void drawMaintenanceStatus();
String maintenancePageHtml(const String &message = "");
void handleMaintenanceRoot();
void handleMaintenanceUpload();
void handleMaintenanceUploadData();
void handleCaptivePortalRedirect();
bool redirectToCaptivePortal();
bool startMaintenanceMode();
void stopMaintenanceMode();
void handleTouchToggle();

uint8_t displayRotationForOrientation(ScreenOrientation orientation) {
  switch (orientation) {
    case ScreenOrientation::Portrait:
      return 3;
    case ScreenOrientation::LandscapeRight:
      return 0;
    case ScreenOrientation::PortraitUpsideDown:
      return 1;
    case ScreenOrientation::LandscapeLeft:
      return 2;
  }
  return 3;
}

const char *orientationLabel(ScreenOrientation orientation) {
  switch (orientation) {
    case ScreenOrientation::Portrait:
      return "Portrait";
    case ScreenOrientation::LandscapeRight:
      return "LandscapeRight";
    case ScreenOrientation::PortraitUpsideDown:
      return "PortraitUpsideDown";
    case ScreenOrientation::LandscapeLeft:
      return "LandscapeLeft";
  }
  return "?";
}

int16_t displayLogicalWidth() {
  switch (currentOrientation) {
    case ScreenOrientation::Portrait:
    case ScreenOrientation::PortraitUpsideDown:
      return kPanelWidth;
    case ScreenOrientation::LandscapeRight:
    case ScreenOrientation::LandscapeLeft:
      return kPanelHeight;
  }
  return kPanelWidth;
}

int16_t displayLogicalHeight() {
  switch (currentOrientation) {
    case ScreenOrientation::Portrait:
    case ScreenOrientation::PortraitUpsideDown:
      return kPanelHeight;
    case ScreenOrientation::LandscapeRight:
    case ScreenOrientation::LandscapeLeft:
      return kPanelWidth;
  }
  return kPanelHeight;
}

Rect blackBandRect() {
  switch (displayRotationForOrientation(currentOrientation)) {
    case 0:
      return {0, kGifAreaSize, kGifAreaSize, kReservedBandSize};
    case 1:
      return {kGifAreaSize, 0, kReservedBandSize, kPanelWidth};
    case 2:
      return {0, 0, kGifAreaSize, kReservedBandSize};
    case 3:
      return {0, 0, kReservedBandSize, kGifAreaSize};
  }
  return {0, 0, kPanelWidth, kReservedBandSize};
}

Rect gifViewportRect() {
  switch (displayRotationForOrientation(currentOrientation)) {
    case 0:
      return {0, 0, kGifAreaSize, kGifAreaSize};
    case 1:
      return {0, 0, kGifAreaSize, kGifAreaSize};
    case 2:
    return {0, kReservedBandSize, kGifAreaSize, kGifAreaSize};
    case 3:
    return {kReservedBandSize, 0, kGifAreaSize, kGifAreaSize};
  }
  return {0, kReservedBandSize, kGifAreaSize, kGifAreaSize};
}

void clearFrameLayout() {
  const Rect band = blackBandRect();
  gfx->fillRect(band.x, band.y, band.w, band.h, BLACK);
}

TouchPoint mapTouchToRotation(uint16_t rawX, uint16_t rawY) {
  switch (displayRotationForOrientation(currentOrientation)) {
    case 0:
      return {rawX, rawY};
    case 1:
      return {rawY, static_cast<uint16_t>((kPanelWidth - 1) - rawX)};
    case 2:
      return {static_cast<uint16_t>((kPanelWidth - 1) - rawX),
              static_cast<uint16_t>((kPanelHeight - 1) - rawY)};
    case 3:
      return {static_cast<uint16_t>((kPanelHeight - 1) - rawY), rawX};
  }
  return {rawX, rawY};
}

ScreenOrientation detectOrientation(float accelX, float accelY) {
  const float absX = fabsf(accelX);
  const float absY = fabsf(accelY);

  if (absX < kOrientationMagnitudeThreshold && absY < kOrientationMagnitudeThreshold) {
    return currentOrientation;
  }

  if ((absX - absY) > kOrientationDominanceMargin) {
    return (accelX >= 0.0f) ? ScreenOrientation::LandscapeRight
                            : ScreenOrientation::LandscapeLeft;
  }

  if ((absY - absX) > kOrientationDominanceMargin) {
    return (accelY >= 0.0f) ? ScreenOrientation::PortraitUpsideDown
                            : ScreenOrientation::Portrait;
  }

  return currentOrientation;
}

void *gifOpenFile(const char *filename, int32_t *size) {
  gifFile = LittleFS.open(filename, "r");
  if (!gifFile) {
    return nullptr;
  }

  *size = gifFile.size();
  return &gifFile;
}

void gifCloseFile(void *handle) {
  auto *file = static_cast<File *>(handle);
  if (file && *file) {
    file->close();
  }
}

int32_t gifReadFile(GIFFILE *file, uint8_t *buffer, int32_t length) {
  if (!file || !file->fHandle) {
    return 0;
  }

  auto *gifFsFile = static_cast<File *>(file->fHandle);
  if (!gifFsFile || !(*gifFsFile)) {
    return 0;
  }

  const int32_t bytesRead = gifFsFile->read(buffer, length);
  file->iPos = static_cast<int32_t>(gifFsFile->position());
  yield();
  return bytesRead;
}

int32_t gifSeekFile(GIFFILE *file, int32_t position) {
  if (!file || !file->fHandle) {
    return 0;
  }

  auto *gifFsFile = static_cast<File *>(file->fHandle);
  if (!gifFsFile || !(*gifFsFile)) {
    return 0;
  }

  gifFsFile->seek(position, SeekSet);
  file->iPos = static_cast<int32_t>(gifFsFile->position());
  return file->iPos;
}

void waitForUsbSerial() {
  Serial.begin(115200);

  const uint32_t start = millis();
  while (!Serial && (millis() - start) < 3000) {
    delay(10);
  }
}

void drawStatus(const char *line1, const char *line2 = nullptr) {
  clearFrameLayout();
  gfx->setTextColor(WHITE);
  gfx->setTextSize(2);
  gfx->setCursor(16, (displayLogicalHeight() / 2) - 20);
  gfx->println(line1);

  if (line2) {
    gfx->setCursor(16, (displayLogicalHeight() / 2) + 10);
    gfx->println(line2);
  }
}

void onTouchInterrupt() {
  touchToggleRequested = true;
  if (touch) {
    touch->IIC_Interrupt_Flag = true;
  }
}

void initTouch() {
  touch.reset(new Arduino_CST816x(i2cBus, CST816T_DEVICE_ADDRESS, kTouchResetPin, kTouchIntPin, onTouchInterrupt));

  bool ready = false;
  uint8_t attempts = 0;
  while (attempts < 10) {
    ready = touch->begin();
    if (ready) {
      break;
    }
    ++attempts;
    Serial.println("Waiting for CST816T...");
    delay(300);
  }

  if (!touch || !ready) {
    Serial.println("CST816T not available, touch toggle disabled.");
    touch.reset();
    return;
  }

  touch->IIC_Write_Device_State(Arduino_IIC_Touch::Device::TOUCH_DEVICE_INTERRUPT_MODE,
                                Arduino_IIC_Touch::Device_Mode::TOUCH_DEVICE_INTERRUPT_PERIODIC);
  Serial.printf("Touch ready, id=0x%02X\r\n", static_cast<unsigned int>(touch->IIC_Read_Device_ID()));
}

void initImu() {
  imuReady = imu.begin(Wire, QMI8658_L_SLAVE_ADDRESS, kTouchSdaPin, kTouchSclPin);
  if (!imuReady) {
    imuReady = imu.begin(Wire, QMI8658_H_SLAVE_ADDRESS, kTouchSdaPin, kTouchSclPin);
  }

  if (!imuReady) {
    Serial.println("QMI8658 not found, auto-rotation disabled.");
    return;
  }

  imu.configAccelerometer(SensorQMI8658::ACC_RANGE_4G, SensorQMI8658::ACC_ODR_125Hz,
                          SensorQMI8658::LPF_MODE_0, true, false);
  imu.enableAccelerometer();

  Serial.printf("QMI8658 ready, revision=0x%02X\r\n", imu.getChipID());
}

bool hasGifExtension(const String &name) {
  String lower = name;
  lower.toLowerCase();
  return lower.endsWith(".gif");
}

String canonicalGifPath(const String &name) {
  if (name.startsWith("/")) {
    return name;
  }
  return "/" + name;
}

bool isManagedGifPath(const String &path) {
  for (size_t i = 0; i < kManagedGifNameCount; ++i) {
    if (path.endsWith(canonicalGifPath(kManagedGifNames[i]))) {
      return true;
    }
  }
  return false;
}

bool pathMatchesAnyGifAlias(const String &path, std::initializer_list<const char *> aliases) {
  for (const char *alias : aliases) {
    if (path.endsWith(canonicalGifPath(alias))) {
      return true;
    }
  }
  return false;
}

void removeLegacyAliasesForPath(const String &path) {
  if (path.endsWith("/Scratch_head_240.gif")) {
    LittleFS.remove("/Scratch Head_240.gif");
  } else if (path.endsWith("/Scratch_240.gif")) {
    LittleFS.remove("/Screcht_240.gif");
  } else if (path.endsWith("/WakeUp_240.gif")) {
    LittleFS.remove("/wakeUp_240.gif");
  } else if (path.endsWith("/turnleft.gif")) {
    LittleFS.remove("/turnLeft.gif");
  }
}

void loadGifList() {
  gifCount = 0;

  File root = LittleFS.open("/");
  if (!root || !root.isDirectory()) {
    Serial.println("Failed to open LittleFS root");
    return;
  }

  File entry = root.openNextFile();
  while (entry && gifCount < kMaxGifFiles) {
    if (!entry.isDirectory()) {
      String path = entry.name();
      if (hasGifExtension(path)) {
        if (!path.startsWith("/")) {
          path = "/" + path;
        }
        gifPaths[gifCount++] = path;
        Serial.printf("Found GIF: %s\r\n", path.c_str());
      }
    }
    entry = root.openNextFile();
  }

  sortGifList();
}

void closeCurrentGif() {
  if (gifIsOpen) {
    gif.close();
    gifIsOpen = false;
  }

  if (gifFile) {
    gifFile.close();
  }
}

bool openGifByIndex(size_t index) {
  if (gifCount == 0 || index >= gifCount) {
    return false;
  }

  closeCurrentGif();

  const String &path = gifPaths[index];
  Serial.printf("Opening GIF: %s\r\n", path.c_str());
  if (!gif.open(path.c_str(), gifOpenFile, gifCloseFile, gifReadFile, gifSeekFile, gifDraw)) {
    Serial.printf("Failed to open GIF: %s\r\n", path.c_str());
    drawStatus("GIF open failed", path.c_str());
    return false;
  }

  const Rect viewport = gifViewportRect();
  gifOffsetX = static_cast<int16_t>((static_cast<int32_t>(viewport.w) - gif.getCanvasWidth()) / 2) +
               viewport.x;
  gifOffsetY = static_cast<int16_t>((static_cast<int32_t>(viewport.h) - gif.getCanvasHeight()) / 2) +
               viewport.y;
  if (gifOffsetX < 0) {
    gifOffsetX = 0;
  }
  if (gifOffsetY < 0) {
    gifOffsetY = 0;
  }

  currentGifIndex = index;
  gifIsOpen = true;
  nextFrameAtMs = millis();
  publishCurrentGifSelection();
  clearFrameLayout();
  // drawOrientationBadge();
  return true;
}

void restartCurrentGif() {
  if (!gifIsOpen) {
    return;
  }

  gif.reset();
  nextFrameAtMs = millis();
}

void drawOrientationBadge() {
  gfx->fillRect(6, 6, 190, 20, BLACK);
  gfx->setTextColor(WHITE);
  gfx->setTextSize(1);
  gfx->setCursor(8, 12);
  gfx->print(orientationLabel(currentOrientation));
}

void drawProximityStatus() {
  if (maintenanceModeActive) {
    drawMaintenanceStatus();
    return;
  }

  const Rect band = blackBandRect();
  gfx->fillRect(band.x, band.y, band.w, band.h, BLACK);

  if (!proximity_espnow::hasAnyNearNeighbor()) {
    return;
  }

  proximity_espnow::NeighborInfo neighbors[PROXIMITY_ESPNOW_MAX_NEIGHBORS];
  const size_t neighborCount = proximity_espnow::copyNeighbors(neighbors, PROXIMITY_ESPNOW_MAX_NEIGHBORS);

  const proximity_espnow::NeighborInfo *bestNeighbor = nullptr;
  for (size_t i = 0; i < neighborCount; ++i) {
    const proximity_espnow::NeighborInfo &neighbor = neighbors[i];
    if (neighbor.state != proximity_espnow::NeighborState::Near) {
      continue;
    }
    if (!bestNeighbor || neighbor.averageRssi > bestNeighbor->averageRssi) {
      bestNeighbor = &neighbor;
    }
  }

  if (!bestNeighbor) {
    return;
  }

  gfx->setTextColor(WHITE);
  gfx->setTextSize(2);
  gfx->setCursor(band.x + 8, band.y + ((band.h > 22) ? 12 : 2));
  gfx->printf("RSSI %.0f", bestNeighbor->averageRssi);
}

void applyDisplayOrientation(ScreenOrientation orientation, bool force, bool reopenGif) {
  if (!force && orientation == currentOrientation) {
    return;
  }

  currentOrientation = orientation;
  orientationChangedAtMs = millis();
  gfx->setRotation(displayRotationForOrientation(currentOrientation));
  clearFrameLayout();
  drawProximityStatus();

  Serial.printf("Orientation -> %s (rotation=%u)\r\n", orientationLabel(currentOrientation),
                displayRotationForOrientation(currentOrientation));

  if (reopenGif && gifCount > 0) {
    openGifByIndex(currentGifIndex);
  } 
  // else {
  //   drawOrientationBadge();
  // }
}

bool startTurnTransition(ScreenOrientation nextOrientation, const String &transitionGifName) {
  int transitionGifIndex = -1;
  if (transitionGifName.endsWith("turnLeft.gif")) {
    transitionGifIndex = findGifIndexByAnyName({"turnleft.gif", "turnLeft.gif"});
  } else if (transitionGifName.endsWith("turnRight.gif")) {
    transitionGifIndex = findGifIndexByAnyName({"turnRight.gif"});
  } else {
    transitionGifIndex = findGifIndexByName(transitionGifName);
  }

  const int defaultIndex = findGifIndexByAnyName({"default.gif"});
  if (transitionGifIndex < 0 || defaultIndex < 0) {
    return false;
  }

  const uint32_t now = millis();
  for (size_t i = 0; i + 1 < kDizzyTurnThreshold; ++i) {
    recentTurnStartedAtMs[i] = recentTurnStartedAtMs[i + 1];
  }
  recentTurnStartedAtMs[kDizzyTurnThreshold - 1] = now;

  const uint32_t oldestTrackedTurnMs = recentTurnStartedAtMs[0];
  const bool shouldUseDizzyGif =
      oldestTrackedTurnMs != 0 && (now - oldestTrackedTurnMs) <= kDizzyTurnWindowMs;

  int postTurnGifIndex = defaultIndex;
  if (shouldUseDizzyGif) {
    const int dizzyIndex = findGifIndexByAnyName({"dizzy_stickman.gif"});
    if (dizzyIndex >= 0) {
      postTurnGifIndex = dizzyIndex;
    }
  }

  pendingOrientation = nextOrientation;
  pendingDefaultGifIndex = postTurnGifIndex;
  turnTransitionPending = true;
  clearFrameLayout();
  return openGifByIndex(static_cast<size_t>(transitionGifIndex));
}

bool isCounterClockwiseTurn(ScreenOrientation from, ScreenOrientation to) {
  const uint8_t fromValue = static_cast<uint8_t>(from);
  const uint8_t toValue = static_cast<uint8_t>(to);
  return static_cast<uint8_t>((fromValue + 1) % 4) == toValue;
}

bool isClockwiseTurn(ScreenOrientation from, ScreenOrientation to) {
  const uint8_t fromValue = static_cast<uint8_t>(from);
  const uint8_t toValue = static_cast<uint8_t>(to);
  return static_cast<uint8_t>((fromValue + 3) % 4) == toValue;
}

bool isTurnTransitionGif(const String &path) {
  return pathMatchesAnyGifAlias(path, {"turnLeft.gif", "turnleft.gif", "turnRight.gif"});
}

bool isDizzyGif(const String &path) {
  return pathMatchesAnyGifAlias(path, {"dizzy_stickman.gif"});
}

bool isWakeUpGif(const String &path) {
  return pathMatchesAnyGifAlias(path, {"WakeUp_240.gif", "wakeUp_240.gif"});
}

bool isSleepGif(const String &path) {
  return pathMatchesAnyGifAlias(path, {"Sleep_240.gif"});
}

bool isRoutineSelectableGif(const String &path) {
  return !isTurnTransitionGif(path) && !isDizzyGif(path) && !isWakeUpGif(path);
}

void sortGifList() {
  for (size_t i = 0; i < gifCount; ++i) {
    for (size_t j = i + 1; j < gifCount; ++j) {
      if (gifPaths[j] < gifPaths[i]) {
        const String temp = gifPaths[i];
        gifPaths[i] = gifPaths[j];
        gifPaths[j] = temp;
      }
    }
  }
}

static uint32_t mixSyncValue(uint32_t value) {
  value ^= value >> 16;
  value *= 0x7FEB352DU;
  value ^= value >> 15;
  value *= 0x846CA68BU;
  value ^= value >> 16;
  return value;
}

int chooseNextRoutineGifIndex() {
  int defaultIndex = -1;
  int totalWeight = 0;

  for (size_t i = 0; i < gifCount; ++i) {
    const String &path = gifPaths[i];
    if (!isRoutineSelectableGif(path)) {
      continue;
    }

    const bool isDefaultGif = pathMatchesAnyGifAlias(path, {"default.gif"});
    if (isDefaultGif) {
      defaultIndex = static_cast<int>(i);
      totalWeight += kDefaultGifWeight;
    } else {
      totalWeight += 1;
    }
  }

  if (totalWeight <= 0) {
    return defaultIndex;
  }

  int draw = random(totalWeight);
  for (size_t i = 0; i < gifCount; ++i) {
    const String &path = gifPaths[i];
    if (!isRoutineSelectableGif(path)) {
      continue;
    }

    const int weight = pathMatchesAnyGifAlias(path, {"default.gif"}) ? kDefaultGifWeight : 1;
    if (draw < weight) {
      return static_cast<int>(i);
    }
    draw -= weight;
  }

  return defaultIndex;
}

uint16_t encodeBroadcastGifFlags(int gifIndex) {
  if (gifIndex < 0 || gifIndex > static_cast<int>(kBroadcastGifIndexMask)) {
    return 0;
  }
  return static_cast<uint16_t>(kBroadcastGifValidMask | (static_cast<uint16_t>(gifIndex) & kBroadcastGifIndexMask));
}

int decodeBroadcastGifIndex(uint16_t flags) {
  if ((flags & kBroadcastGifValidMask) == 0) {
    return -1;
  }
  return static_cast<int>(flags & kBroadcastGifIndexMask);
}

void publishCurrentGifSelection() {
  if (!proximity_espnow::isRunning()) {
    return;
  }

  const int gifIndex = gifIsOpen ? static_cast<int>(currentGifIndex) : -1;
  const uint16_t encodedFlags = encodeBroadcastGifFlags(gifIndex);
  proximity_espnow::setLocalFlags(encodedFlags);
}

bool getNearEffectState(bool *activeOut, uint16_t *colorOut, uint32_t *leaderNodeIdOut, uint32_t *windowOut) {
  if (activeOut) {
    *activeOut = false;
  }
  if (colorOut) {
    *colorOut = BLACK;
  }

  if (!proximity_espnow::hasAnyNearNeighbor()) {
    return false;
  }

  const uint32_t localNodeId = proximity_espnow::getLocalNodeId();
  uint32_t leaderNodeId = localNodeId;
  uint32_t timelineMs = millis();

  proximity_espnow::NeighborInfo neighbors[PROXIMITY_ESPNOW_MAX_NEIGHBORS];
  const size_t neighborCount = proximity_espnow::copyNeighbors(neighbors, PROXIMITY_ESPNOW_MAX_NEIGHBORS);
  for (size_t i = 0; i < neighborCount; ++i) {
    const proximity_espnow::NeighborInfo &neighbor = neighbors[i];
    if (neighbor.state != proximity_espnow::NeighborState::Near) {
      continue;
    }

    if (neighbor.nodeId < leaderNodeId) {
      leaderNodeId = neighbor.nodeId;
      timelineMs = neighbor.remoteUptimeMs + (millis() - neighbor.lastSeenMs);
    }
  }

  const uint32_t effectWindow = timelineMs / kNearEffectWindowMs;
  const uint32_t effectSeed = mixSyncValue(effectWindow ^ leaderNodeId ^ 0x51F15EEDUL);
  const bool effectActive = (effectSeed % kNearEffectChanceDivisor) == 0 &&
                            (timelineMs % kNearEffectWindowMs) < kNearEffectDurationMs;
  const bool swapColors = (mixSyncValue(effectSeed ^ 0xA5A55A5AUL) & 0x01U) != 0;

  if (leaderNodeIdOut) {
    *leaderNodeIdOut = leaderNodeId;
  }
  if (windowOut) {
    *windowOut = effectWindow;
  }
  if (activeOut) {
    *activeOut = effectActive;
  }
  if (colorOut) {
    const bool localGetsWhite = swapColors ? (localNodeId != leaderNodeId) : (localNodeId == leaderNodeId);
    *colorOut = localGetsWhite ? WHITE : RED;
  }
  return true;
}

void renderNearEffectIfNeeded() {
  bool effectActive = false;
  uint16_t effectColor = BLACK;
  uint32_t leaderNodeId = 0;
  uint32_t effectWindow = 0;
  if (!getNearEffectState(&effectActive, &effectColor, &leaderNodeId, &effectWindow)) {
    if (nearEffectWasActive) {
      nearEffectWasActive = false;
      hasLoggedNearEffectState = false;
      if (proximity_espnow::hasAnyNearNeighbor()) {
        updateSynchronizedGifPlayback();
      } else if (gifIsOpen) {
        openGifByIndex(currentGifIndex);
      }
      drawProximityStatus();
    }
    return;
  }

  if (!hasLoggedNearEffectState || lastLoggedNearEffectWindow != effectWindow ||
      lastLoggedNearEffectLeaderNodeId != leaderNodeId || nearEffectWasActive != effectActive) {
    Serial.printf("[near-fx] leader=0x%08lX window=%lu active=%s role=%s\r\n",
                  static_cast<unsigned long>(leaderNodeId), static_cast<unsigned long>(effectWindow),
                  effectActive ? "yes" : "no", effectColor == WHITE ? "white" : "red");
    lastLoggedNearEffectWindow = effectWindow;
    lastLoggedNearEffectLeaderNodeId = leaderNodeId;
    hasLoggedNearEffectState = true;
  }

  if (effectActive) {
    if (!nearEffectWasActive) {
      Serial.printf("[near-fx] starting 10s color screen: %s\r\n", effectColor == WHITE ? "white" : "red");
    }
    gfx->fillScreen(effectColor);
    nearEffectWasActive = true;
    return;
  }

  if (nearEffectWasActive) {
    nearEffectWasActive = false;
    Serial.println("[near-fx] ending color screen");
    updateSynchronizedGifPlayback();
    if (gifIsOpen) {
      openGifByIndex(currentGifIndex);
    }
    drawProximityStatus();
  }
}

bool getSynchronizedRoutineGifIndex(size_t *indexOut) {
  return getSynchronizedRoutineGifIndex(indexOut, nullptr, nullptr, nullptr);
}

bool getSynchronizedRoutineGifIndex(size_t *indexOut, uint32_t *leaderNodeIdOut, uint32_t *slotOut,
                                    uint32_t *timelineMsOut) {
  if (!indexOut) {
    return false;
  }

  const uint32_t localNodeId = proximity_espnow::getLocalNodeId();
  uint32_t leaderNodeId = localNodeId;
  uint32_t timelineMs = millis();
  const proximity_espnow::NeighborInfo *leaderNeighbor = nullptr;

  proximity_espnow::NeighborInfo neighbors[PROXIMITY_ESPNOW_MAX_NEIGHBORS];
  const size_t neighborCount = proximity_espnow::copyNeighbors(neighbors, PROXIMITY_ESPNOW_MAX_NEIGHBORS);
  for (size_t i = 0; i < neighborCount; ++i) {
    const proximity_espnow::NeighborInfo &neighbor = neighbors[i];
    if (neighbor.state != proximity_espnow::NeighborState::Near) {
      continue;
    }

    if (neighbor.nodeId < leaderNodeId) {
      leaderNodeId = neighbor.nodeId;
      timelineMs = neighbor.remoteUptimeMs + (millis() - neighbor.lastSeenMs);
      leaderNeighbor = &neighbor;
    }
  }

  if (leaderNeighbor) {
    const int advertisedGifIndex = decodeBroadcastGifIndex(leaderNeighbor->flags);
    if (advertisedGifIndex >= 0 && advertisedGifIndex < static_cast<int>(gifCount)) {
      *indexOut = static_cast<size_t>(advertisedGifIndex);
      if (leaderNodeIdOut) {
        *leaderNodeIdOut = leaderNodeId;
      }
      if (slotOut) {
        *slotOut = timelineMs / kSynchronizedGifSlotMs;
      }
      if (timelineMsOut) {
        *timelineMsOut = timelineMs;
      }
      return true;
    }
  }

  size_t weightedPool[kMaxGifFiles * kDefaultGifWeight];
  size_t weightedCount = 0;
  for (size_t i = 0; i < gifCount && weightedCount < (kMaxGifFiles * kDefaultGifWeight); ++i) {
    const String &path = gifPaths[i];
    if (!isRoutineSelectableGif(path)) {
      continue;
    }

    const size_t weight = pathMatchesAnyGifAlias(path, {"default.gif"}) ? kDefaultGifWeight : 1;
    for (size_t copy = 0; copy < weight && weightedCount < (kMaxGifFiles * kDefaultGifWeight); ++copy) {
      weightedPool[weightedCount++] = i;
    }
  }

  if (weightedCount == 0) {
    return false;
  }

  const uint32_t slot = timelineMs / kSynchronizedGifSlotMs;
  const uint32_t syncValue = mixSyncValue(slot ^ leaderNodeId);
  *indexOut = weightedPool[syncValue % weightedCount];

  if (leaderNodeIdOut) {
    *leaderNodeIdOut = leaderNodeId;
  }
  if (slotOut) {
    *slotOut = slot;
  }
  if (timelineMsOut) {
    *timelineMsOut = timelineMs;
  }
  return true;
}

void updateSynchronizedGifPlayback() {
  if (maintenanceModeActive || turnTransitionPending || !proximity_espnow::hasAnyNearNeighbor()) {
    hasLoggedSyncState = false;
    return;
  }

  bool nearEffectActive = false;
  if (getNearEffectState(&nearEffectActive, nullptr) && nearEffectActive) {
    return;
  }

  size_t syncedGifIndex = 0;
  uint32_t leaderNodeId = 0;
  uint32_t syncSlot = 0;
  uint32_t timelineMs = 0;
  if (!getSynchronizedRoutineGifIndex(&syncedGifIndex, &leaderNodeId, &syncSlot, &timelineMs)) {
    return;
  }

  if (!hasLoggedSyncState || lastLoggedSyncLeaderNodeId != leaderNodeId || lastLoggedSyncSlot != syncSlot ||
      lastLoggedSyncGifIndex != static_cast<int>(syncedGifIndex)) {
    Serial.printf("[sync] leader=0x%08lX slot=%lu timeline=%lu gif=%s index=%u current=%u\r\n",
                  static_cast<unsigned long>(leaderNodeId), static_cast<unsigned long>(syncSlot),
                  static_cast<unsigned long>(timelineMs), gifPaths[syncedGifIndex].c_str(),
                  static_cast<unsigned int>(syncedGifIndex), static_cast<unsigned int>(currentGifIndex));
    lastLoggedSyncLeaderNodeId = leaderNodeId;
    lastLoggedSyncSlot = syncSlot;
    lastLoggedSyncGifIndex = static_cast<int>(syncedGifIndex);
    hasLoggedSyncState = true;
  }

  if (!gifIsOpen || currentGifIndex != syncedGifIndex) {
    Serial.printf("[sync] switching to synchronized gif=%s index=%u\r\n", gifPaths[syncedGifIndex].c_str(),
                  static_cast<unsigned int>(syncedGifIndex));
    openGifByIndex(syncedGifIndex);
  }
}

void updateOrientationFromImu() {
  if (!imuReady || turnTransitionPending || maintenanceModeActive) {
    return;
  }

  const uint32_t now = millis();
  if ((now - lastImuPollMs) < kImuPollIntervalMs) {
    return;
  }
  lastImuPollMs = now;

  float accelX = 0.0f;
  float accelY = 0.0f;
  float accelZ = 0.0f;
  if (!imu.getAccelerometer(accelX, accelY, accelZ)) {
    return;
  }

  const ScreenOrientation nextOrientation = detectOrientation(accelX, accelY);
  if (nextOrientation == currentOrientation) {
    return;
  }

  if ((now - orientationChangedAtMs) < kOrientationSettleMs) {
    return;
  }

  if (isCounterClockwiseTurn(currentOrientation, nextOrientation)) {
    if (startTurnTransition(nextOrientation, "/turnLeft.gif")) {
      return;
    }
  } else if (isClockwiseTurn(currentOrientation, nextOrientation)) {
    if (startTurnTransition(nextOrientation, "/turnRight.gif")) {
      return;
    }
  }

  applyDisplayOrientation(nextOrientation);
}

void gifDraw(GIFDRAW *draw) {
  const Rect viewport = gifViewportRect();
  const int16_t displayWidth = viewport.x + viewport.w;
  const int16_t displayHeight = viewport.y + viewport.h;

  int16_t y = gifOffsetY + draw->iY + draw->y;
  if (y < viewport.y || y >= displayHeight) {
    return;
  }

  int16_t x = gifOffsetX + draw->iX;
  if (x >= displayWidth) {
    return;
  }

  uint16_t *palette = draw->pPalette;
  uint8_t *source = draw->pPixels;
  int16_t width = draw->iWidth;

  if (x < viewport.x) {
    const int16_t crop = viewport.x - x;
    source += crop;
    x = viewport.x;
    width -= crop;
  }

  if ((x + width) > displayWidth) {
    width = displayWidth - x;
  }

  if (width <= 0) {
    return;
  }

  if (draw->ucHasTransparency) {
    int16_t runStart = -1;

    for (int16_t i = 0; i < width; ++i) {
      if (source[i] == draw->ucTransparent) {
        if (runStart >= 0) {
          gfx->draw16bitRGBBitmap(x + runStart, y, lineBuffer, i - runStart, 1);
          runStart = -1;
        }
        continue;
      }

      if (runStart < 0) {
        runStart = i;
      }

      lineBuffer[i - runStart] = palette[source[i]];
    }

    if (runStart >= 0) {
      gfx->draw16bitRGBBitmap(x + runStart, y, lineBuffer, width - runStart, 1);
    }
    yield();
    return;
  }

  for (int16_t i = 0; i < width; ++i) {
    lineBuffer[i] = palette[source[i]];
  }

  gfx->draw16bitRGBBitmap(x, y, lineBuffer, width, 1);
  if ((y & 0x07) == 0) {
    yield();
  }
}

void initGifPlayer() {
  if (!LittleFS.begin(true)) {
    Serial.println("LittleFS mount failed");
    drawStatus("LittleFS failed", "uploadfs needed");
    return;
  }

  gif.begin(LITTLE_ENDIAN_PIXELS);
  loadGifList();

  if (gifCount == 0) {
    Serial.println("No GIF files found in LittleFS");
    drawStatus("No GIFs found", "Use uploadfs");
    return;
  }

  const int wakeUpIndex = findGifIndexByAnyName({"WakeUp_240.gif", "wakeUp_240.gif"});
  if (wakeUpIndex >= 0) {
    openGifByIndex(static_cast<size_t>(wakeUpIndex));
    return;
  }

  const int defaultIndex = findGifIndexByAnyName({"default.gif"});
  if (defaultIndex >= 0) {
    openGifByIndex(static_cast<size_t>(defaultIndex));
    return;
  }

  const int nextGifIndex = chooseNextRoutineGifIndex();
  if (nextGifIndex >= 0) {
    openGifByIndex(static_cast<size_t>(nextGifIndex));
  } else {
    openGifByIndex(0);
  }
}

int findGifIndexByName(const String &filename) {
  for (size_t i = 0; i < gifCount; i++) {
    if (gifPaths[i].endsWith(filename)) {
      return static_cast<int>(i);
    }
  }
  return -1;
}

int findGifIndexByAnyName(std::initializer_list<const char *> aliases) {
  for (const char *alias : aliases) {
    for (size_t i = 0; i < gifCount; ++i) {
      if (gifPaths[i].endsWith(canonicalGifPath(alias))) {
        return static_cast<int>(i);
      }
    }
  }
  return -1;
}

void refreshGifListPreservingCurrent() {
  String currentPath;
  if (gifCount > 0 && currentGifIndex < gifCount) {
    currentPath = gifPaths[currentGifIndex];
  }

  loadGifList();

  if (!currentPath.isEmpty()) {
    const int preservedIndex = findGifIndexByName(currentPath);
    if (preservedIndex >= 0) {
      currentGifIndex = static_cast<size_t>(preservedIndex);
      return;
    }
  }

  const int defaultIndex = findGifIndexByAnyName({"default.gif"});
  currentGifIndex = (defaultIndex >= 0) ? static_cast<size_t>(defaultIndex) : 0;
}

void initProximityFeature() {
  if (!proximity_espnow::isBuildEnabled()) {
    return;
  }

  proximity_espnow::setCallbacks(onNeighborNear, onNeighborFar);
  if (!proximity_espnow::init()) {
    Serial.println("Proximity ESP-NOW init failed.");
    return;
  }

  Serial.printf("Proximity ESP-NOW ready, node_id=0x%08lX\r\n",
                static_cast<unsigned long>(proximity_espnow::getLocalNodeId()));
  publishCurrentGifSelection();
  drawProximityStatus();
}

void onNeighborNear(const proximity_espnow::NeighborInfo &neighbor) {
  Serial.printf("Neighbor near: node=0x%08lX avg=%.1f last=%d remote_uptime=%lu flags=0x%04X mac=%02X:%02X:%02X:%02X:%02X:%02X\r\n",
                static_cast<unsigned long>(neighbor.nodeId), neighbor.averageRssi, neighbor.lastRssi,
                static_cast<unsigned long>(neighbor.remoteUptimeMs), static_cast<unsigned int>(neighbor.flags),
                neighbor.mac[0], neighbor.mac[1], neighbor.mac[2], neighbor.mac[3], neighbor.mac[4], neighbor.mac[5]);
  renderNearEffectIfNeeded();
  updateSynchronizedGifPlayback();
  drawProximityStatus();
}

void onNeighborFar(const proximity_espnow::NeighborInfo &neighbor) {
  Serial.printf("Neighbor far: node=0x%08lX avg=%.1f last=%d remote_uptime=%lu flags=0x%04X mac=%02X:%02X:%02X:%02X:%02X:%02X\r\n",
                static_cast<unsigned long>(neighbor.nodeId), neighbor.averageRssi, neighbor.lastRssi,
                static_cast<unsigned long>(neighbor.remoteUptimeMs), static_cast<unsigned int>(neighbor.flags),
                neighbor.mac[0], neighbor.mac[1], neighbor.mac[2], neighbor.mac[3], neighbor.mac[4], neighbor.mac[5]);
  renderNearEffectIfNeeded();
  drawProximityStatus();
}

void drawMaintenanceStatus() {
  const Rect band = blackBandRect();
  gfx->fillRect(band.x, band.y, band.w, band.h, BLACK);
  gfx->setTextColor(WHITE);
  gfx->setTextSize(1);
  gfx->setCursor(band.x + 6, band.y + 8);
  gfx->print("PORTAL ");
  gfx->print(WiFi.softAPIP());
  gfx->setCursor(band.x + 6, band.y + 20);
  gfx->print("SSID:");
  gfx->print(kMaintenanceSsid);
}

String maintenancePageHtml(const String &message) {
  String html;
  html.reserve(8192);
  html += F(
      "<!DOCTYPE html><html><head><meta charset='utf-8'><meta name='viewport' "
      "content='width=device-width,initial-scale=1'><title>Trocar GIF</title><style>"
      ":root{--bg:#f6f2ea;--panel:#fffdf8;--ink:#1b1d1f;--muted:#6b7280;--accent:#0f766e;--line:#e7ddcf;--ok:#e8fbf2}"
      "*{box-sizing:border-box}body{margin:0;min-height:100vh;display:grid;place-items:center;padding:18px;"
      "font-family:Segoe UI,Arial,sans-serif;background:linear-gradient(180deg,#fbf8f1,#efe5d4);color:var(--ink)}"
      ".card{width:min(100%,460px);background:var(--panel);border:1px solid var(--line);border-radius:24px;padding:24px;"
      "box-shadow:0 20px 50px rgba(66,43,18,.12)}h1{margin:0 0 6px;font-size:32px}p{margin:0 0 18px;color:var(--muted);line-height:1.45}"
      ".msg{background:var(--ok);border:1px solid #bce9d0;color:#0d5a3d;border-radius:14px;padding:12px 14px;margin-bottom:14px;font-weight:600}"
      ".field{display:grid;gap:8px;margin-top:14px}.label{font-size:14px;font-weight:700}.select,.btn{width:100%;border-radius:16px;font-size:16px}"
      ".select{padding:14px 16px;border:1px solid var(--line);background:#fff;color:var(--ink)}"
      ".drop{display:grid;gap:6px;justify-items:center;text-align:center;padding:22px 16px;border:2px dashed var(--line);border-radius:20px;"
      "background:#fcfaf5;cursor:pointer;transition:.18s ease}.drop.active{border-color:var(--accent);background:#f3fcfa}"
      ".drop strong{font-size:18px}.drop span{font-size:14px;color:var(--muted)}input[type=file]{display:none}"
      ".file{display:none;margin-top:10px;padding:12px 14px;border-radius:14px;background:#f5f7f8;border:1px solid #e7ecef;font-size:14px}"
      ".file.show{display:block}.btn{margin-top:16px;padding:15px 18px;border:0;background:linear-gradient(135deg,#0f766e,#149b8f);color:#fff;font-weight:700;cursor:pointer}"
      ".btn:disabled{opacity:.7}.status{margin-top:12px;min-height:20px;font-size:14px;color:var(--muted)}"
      ".hint{margin-top:14px;text-align:center;font-size:13px;color:var(--muted)}"
      "</style></head><body><main class='card'><h1>Trocar GIF</h1><p>Escolha o nome e envie um novo arquivo.</p>");

  if (!message.isEmpty()) {
    html += "<div class='msg'>";
    html += message;
    html += "</div>";
  }

  html += F("<form id='uploadForm' method='POST' action='/upload' enctype='multipart/form-data'>"
            "<div class='field'><label class='label' for='target'>Arquivo de destino</label>"
            "<select class='select' id='target' name='target'>");

  for (size_t i = 0; i < kManagedGifNameCount; ++i) {
    const String path = canonicalGifPath(kManagedGifNames[i]);
    html += "<option value='";
    html += kManagedGifNames[i];
    html += "'>";
    html += kManagedGifNames[i];
    html += LittleFS.exists(path) ? " (presente)" : " (faltando)";
    html += "</option>";
  }

  html += F("</select></div><div class='field'><label class='label'>Novo arquivo GIF</label>"
            "<label class='drop' id='dropZone' for='gif'><strong>Selecionar GIF</strong>"
            "<span>Toque aqui ou arraste um arquivo .gif</span></label>"
            "<input id='gif' type='file' name='gif' accept='.gif,image/gif' required>"
            "<div class='file' id='fileChip'><span id='fileName'>Nenhum arquivo selecionado</span></div></div>"
            "<button type='submit' class='btn' id='submitBtn'>Enviar</button>"
            "<div class='status' id='statusText'>Pronto para enviar.</div></form>"
            "<div class='hint'>A tela fecha ao tocar novamente no dispositivo.</div></main><script>"
            "const dropZone=document.getElementById('dropZone');"
            "const fileInput=document.getElementById('gif');"
            "const fileChip=document.getElementById('fileChip');"
            "const fileName=document.getElementById('fileName');"
            "const form=document.getElementById('uploadForm');"
            "const statusText=document.getElementById('statusText');"
            "const submitBtn=document.getElementById('submitBtn');"
            "const target=document.getElementById('target');"
            "function syncFile(){const file=fileInput.files&&fileInput.files[0];"
            "if(!file){fileChip.classList.remove('show');fileName.textContent='Nenhum arquivo selecionado';statusText.textContent='Pronto para enviar.';return;}"
            "fileChip.classList.add('show');fileName.textContent=file.name+' - '+Math.round(file.size/1024)+' KB';"
            "statusText.textContent='Enviar para: '+target.value;}"
            "['dragenter','dragover'].forEach(evt=>dropZone.addEventListener(evt,e=>{e.preventDefault();dropZone.classList.add('active');}));"
            "['dragleave','drop'].forEach(evt=>dropZone.addEventListener(evt,e=>{e.preventDefault();dropZone.classList.remove('active');}));"
            "dropZone.addEventListener('drop',e=>{if(e.dataTransfer.files.length){fileInput.files=e.dataTransfer.files;syncFile();}});"
            "fileInput.addEventListener('change',syncFile);"
            "target.addEventListener('change',()=>{if(fileInput.files&&fileInput.files[0]){statusText.textContent='Enviar para: '+target.value;}});"
            "form.addEventListener('submit',e=>{const file=fileInput.files&&fileInput.files[0];"
            "if(!file){e.preventDefault();statusText.textContent='Escolha um arquivo GIF antes de enviar.';return;}"
            "submitBtn.disabled=true;submitBtn.textContent='Enviando...';statusText.textContent='Enviando para '+target.value+' ...';});"
            "syncFile();</script></body></html>");
  return html;
}

void handleMaintenanceRoot() {
  webServer.send(200, "text/html; charset=utf-8", maintenancePageHtml(lastMaintenanceMessage));
  lastMaintenanceMessage = "";
}

bool redirectToCaptivePortal() {
  if (!maintenanceModeActive) {
    return false;
  }

  const String hostHeader = webServer.hostHeader();
  if (hostHeader.length() == 0 || hostHeader == WiFi.softAPIP().toString()) {
    return false;
  }

  webServer.sendHeader("Location", String("http://") + WiFi.softAPIP().toString(), true);
  webServer.send(302, "text/plain", "");
  return true;
}

void handleCaptivePortalRedirect() {
  if (redirectToCaptivePortal()) {
    return;
  }
  handleMaintenanceRoot();
}

void handleMaintenanceUpload() {
  const bool ok = pendingUploadError.isEmpty();
  const String message = ok ? lastMaintenanceMessage : pendingUploadError;
  webServer.send(200, "text/html; charset=utf-8", maintenancePageHtml(message));
  pendingUploadError = "";
}

void handleMaintenanceUploadData() {
  HTTPUpload &upload = webServer.upload();

  if (upload.status == UPLOAD_FILE_START) {
    pendingUploadError = "";
    lastMaintenanceMessage = "";
    pendingUploadTargetPath = canonicalGifPath(webServer.arg("target"));

    if (!isManagedGifPath(pendingUploadTargetPath)) {
      pendingUploadError = "Arquivo de destino invalido.";
      return;
    }

    if (uploadFile) {
      uploadFile.close();
    }

    LittleFS.remove(pendingUploadTargetPath);
    removeLegacyAliasesForPath(pendingUploadTargetPath);
    uploadFile = LittleFS.open(pendingUploadTargetPath, "w");
    if (!uploadFile) {
      pendingUploadError = "Nao foi possivel abrir o arquivo para escrita.";
      return;
    }

    Serial.printf("[web] receiving %s\r\n", pendingUploadTargetPath.c_str());
    return;
  }

  if (upload.status == UPLOAD_FILE_WRITE) {
    if (!pendingUploadError.isEmpty() || !uploadFile) {
      return;
    }

    if (uploadFile.write(upload.buf, upload.currentSize) != upload.currentSize) {
      pendingUploadError = "Falha ao gravar o GIF na LittleFS.";
      uploadFile.close();
    }
    return;
  }

  if (upload.status == UPLOAD_FILE_END) {
    if (uploadFile) {
      uploadFile.close();
    }

    if (pendingUploadError.isEmpty()) {
      refreshGifListPreservingCurrent();
      lastMaintenanceMessage = "Upload concluido: " + pendingUploadTargetPath + " (" + String(upload.totalSize) + " bytes)";
      Serial.printf("[web] saved %s (%u bytes)\r\n", pendingUploadTargetPath.c_str(),
                    static_cast<unsigned int>(upload.totalSize));
    }
    drawMaintenanceStatus();
    return;
  }

  if (upload.status == UPLOAD_FILE_ABORTED) {
    if (uploadFile) {
      uploadFile.close();
    }
    pendingUploadError = "Upload interrompido.";
    LittleFS.remove(pendingUploadTargetPath);
  }
}

bool startMaintenanceMode() {
  if (maintenanceModeActive) {
    return true;
  }

  proximity_espnow::stop();
  WiFi.mode(WIFI_AP);
  WiFi.setSleep(false);
  WiFi.softAPsetHostname("cube-world-gif");

  const bool apStarted =
      (strlen(kMaintenancePassword) == 0) ? WiFi.softAP(kMaintenanceSsid) : WiFi.softAP(kMaintenanceSsid, kMaintenancePassword);
  if (!apStarted) {
    WiFi.mode(WIFI_OFF);
    initProximityFeature();
    lastMaintenanceMessage = "Falha ao iniciar o Wi-Fi.";
    return false;
  }

  closeCurrentGif();
  dnsServer.stop();
  dnsServer.start(kDnsPort, "*", WiFi.softAPIP());
  dnsServer.setErrorReplyCode(DNSReplyCode::NoError);
  webServer.on("/", HTTP_GET, handleMaintenanceRoot);
  webServer.on("/upload", HTTP_POST, handleMaintenanceUpload, handleMaintenanceUploadData);
  webServer.on("/generate_204", HTTP_GET, handleCaptivePortalRedirect);
  webServer.on("/gen_204", HTTP_GET, handleCaptivePortalRedirect);
  webServer.on("/hotspot-detect.html", HTTP_GET, handleCaptivePortalRedirect);
  webServer.on("/library/test/success.html", HTTP_GET, handleCaptivePortalRedirect);
  webServer.on("/connecttest.txt", HTTP_GET, handleCaptivePortalRedirect);
  webServer.on("/redirect", HTTP_GET, handleCaptivePortalRedirect);
  webServer.on("/ncsi.txt", HTTP_GET, handleCaptivePortalRedirect);
  webServer.on("/fwlink", HTTP_GET, handleCaptivePortalRedirect);
  webServer.onNotFound(handleCaptivePortalRedirect);
  webServer.begin();

  maintenanceModeActive = true;
  lastMaintenanceMessage = "Modo manutencao ativo. Conecte em " + String(kMaintenanceSsid) + " e abra " +
                           WiFi.softAPIP().toString();
  Serial.printf("[maintenance] web server started ssid=%s ip=http://%s/ channel=%d\r\n", kMaintenanceSsid,
                WiFi.softAPIP().toString().c_str(), static_cast<int>(WiFi.channel()));
  drawMaintenanceStatus();
  return true;
}

void stopMaintenanceMode() {
  if (!maintenanceModeActive) {
    return;
  }

  dnsServer.stop();
  webServer.stop();
  WiFi.softAPdisconnect(true);
  WiFi.mode(WIFI_STA);
  maintenanceModeActive = false;
  pendingUploadTargetPath = "";
  pendingUploadError = "";
  if (uploadFile) {
    uploadFile.close();
  }

  refreshGifListPreservingCurrent();
  if (gifCount > 0) {
    openGifByIndex(currentGifIndex < gifCount ? currentGifIndex : 0);
  }
  initProximityFeature();
  drawProximityStatus();
  Serial.println("[maintenance] web server stopped");
}

void handleTouchToggle() {
  if (!touchToggleRequested && (!touch || !touch->IIC_Interrupt_Flag)) {
    return;
  }

  if (touch) {
    touch->IIC_Interrupt_Flag = false;
  }
  touchToggleRequested = false;

  const uint32_t now = millis();
  if ((now - lastTouchToggleMs) < kTouchDebounceMs) {
    return;
  }
  lastTouchToggleMs = now;

  if (!maintenanceModeActive) {
    if (startMaintenanceMode()) {
      Serial.println("[maintenance] enabled by touch");
    }
    return;
  }

  stopMaintenanceMode();
  Serial.println("[maintenance] disabled by touch");
}

}  // namespace

void setup() {
  waitForUsbSerial();

  gfx->begin();
  pinMode(kLcdBacklightPin, OUTPUT);
  // digitalWrite(kLcdBacklightPin, HIGH);
  analogWrite(kLcdBacklightPin, 170);

  applyDisplayOrientation(ScreenOrientation::Portrait, true);
  randomSeed(static_cast<uint32_t>(micros()));

  Serial.println();
  Serial.println("Starting GIF random player");

  initTouch();
  initImu();
  initGifPlayer();
  initProximityFeature();
  drawProximityStatus();

  if (kAutostartMaintenanceOnBoot) {
    delay(400);
    startMaintenanceMode();
  }

  Serial.println("Ready: 1 touch enters GIF web upload mode, 2nd touch exits.");
}

void loop() {
  handleTouchToggle();

  if (maintenanceModeActive) {
    dnsServer.processNextRequest();
    webServer.handleClient();
    delay(2);
    yield();
    return;
  }

  updateOrientationFromImu();
  proximity_espnow::update();
  renderNearEffectIfNeeded();
  updateSynchronizedGifPlayback();

  if (gifIsOpen && !nearEffectWasActive) {
    const uint32_t now = millis();
    if (now >= nextFrameAtMs) {
      int frameDelayMs = 0;
      const int result = gif.playFrame(false, &frameDelayMs);

      if (result < 0) {
        Serial.printf("GIF decode error: %d\r\n", gif.getLastError());
        openGifByIndex(currentGifIndex);
      } 
      else if (result == 0) {
        const bool finishedSleepGif = gifPaths[currentGifIndex].length() > 0 && isSleepGif(gifPaths[currentGifIndex]);
        if (turnTransitionPending &&
            isTurnTransitionGif(gifPaths[currentGifIndex])) {
          turnTransitionPending = false;
          applyDisplayOrientation(pendingOrientation, false, false);
          clearFrameLayout();
          if (pendingDefaultGifIndex >= 0) {
            openGifByIndex(static_cast<size_t>(pendingDefaultGifIndex));
            pendingDefaultGifIndex = -1;
          } else {
            restartCurrentGif();
          }
        } else if (finishedSleepGif) {
          const int wakeUpIndex = findGifIndexByAnyName({"WakeUp_240.gif", "wakeUp_240.gif"});
          if (wakeUpIndex >= 0) {
            openGifByIndex(static_cast<size_t>(wakeUpIndex));
          } else {
            const int nextGifIndex = chooseNextRoutineGifIndex();
            if (nextGifIndex >= 0) {
              openGifByIndex(static_cast<size_t>(nextGifIndex));
            } else {
              restartCurrentGif();
            }
          }
        } else {
          if (proximity_espnow::hasAnyNearNeighbor()) {
            updateSynchronizedGifPlayback();
          } else {
            const int nextGifIndex = chooseNextRoutineGifIndex();
            if (nextGifIndex >= 0) {
              openGifByIndex(static_cast<size_t>(nextGifIndex));
            } else {
              restartCurrentGif();
            }
          }
        }
      } else {
        if (frameDelayMs < 10) {
          frameDelayMs = 10;
        }
        nextFrameAtMs = now + static_cast<uint32_t>(frameDelayMs);
      }
      drawProximityStatus();
    }
  }
  // Serial.println(orientationLabel(currentOrientation));
  delay(1);
  yield();
}
