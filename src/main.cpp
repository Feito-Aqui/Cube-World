#include <Arduino.h>
#include <Wire.h>

#include <LittleFS.h>

#include <memory>

#include "AnimatedGIF.h"
#include "Arduino_DriveBus_Library.h"
#include "Arduino_GFX_Library.h"
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

constexpr uint32_t kTouchDebounceMs = 250;
constexpr uint32_t kImuPollIntervalMs = 120;
constexpr uint32_t kOrientationSettleMs = 180;
constexpr size_t kMaxGifFiles = 16;
constexpr float kOrientationMagnitudeThreshold = 0.60f;
constexpr float kOrientationDominanceMargin = 0.18f;

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

AnimatedGIF gif;
File gifFile;
String gifPaths[kMaxGifFiles];
size_t gifCount = 0;
size_t currentGifIndex = 0;
bool gifIsOpen = false;
bool imuReady = false;
bool turnTransitionPending = false;
int16_t gifOffsetX = 0;
int16_t gifOffsetY = 0;
uint16_t lineBuffer[kLineBufferSize];
uint32_t lastTouchMs = 0;
uint32_t nextFrameAtMs = 0;
uint32_t lastImuPollMs = 0;
uint32_t orientationChangedAtMs = 0;
ScreenOrientation currentOrientation = ScreenOrientation::Portrait;
ScreenOrientation pendingOrientation = ScreenOrientation::Portrait;
int pendingDefaultGifIndex = -1;

void onTouchInterrupt();
void gifDraw(GIFDRAW *draw);
void closeCurrentGif();
bool openGifByIndex(size_t index);
void restartCurrentGif();
void drawOrientationBadge();
void applyDisplayOrientation(ScreenOrientation orientation, bool force = false, bool reopenGif = true);
bool startTurnTransition(ScreenOrientation nextOrientation, const String &transitionGifName);
int findGifIndexByName(const String &filename);
bool isCounterClockwiseTurn(ScreenOrientation from, ScreenOrientation to);
bool isClockwiseTurn(ScreenOrientation from, ScreenOrientation to);

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

void initTouch() {
  touch.reset(new Arduino_CST816x(i2cBus, CST816T_DEVICE_ADDRESS, kTouchResetPin, kTouchIntPin,
                                  onTouchInterrupt));

  while (!touch->begin()) {
    Serial.println("Waiting for CST816T...");
    delay(500);
  }

  touch->IIC_Write_Device_State(
      Arduino_IIC_Touch::Device::TOUCH_DEVICE_INTERRUPT_MODE,
      Arduino_IIC_Touch::Device_Mode::TOUCH_DEVICE_INTERRUPT_PERIODIC);
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

bool readTouchEvent() {
  if (!touch || !touch->IIC_Interrupt_Flag) {
    return false;
  }

  touch->IIC_Interrupt_Flag = false;

  const uint16_t rawX = touch->IIC_Read_Device_Value(
      Arduino_IIC_Touch::Value_Information::TOUCH_COORDINATE_X);
  const uint16_t rawY = touch->IIC_Read_Device_Value(
      Arduino_IIC_Touch::Value_Information::TOUCH_COORDINATE_Y);
  const TouchPoint rotated = mapTouchToRotation(rawX, rawY);

  Serial.printf("Touch: raw=(%u,%u) mapped=(%u,%u) rot=%u\r\n", rawX, rawY, rotated.x, rotated.y,
                static_cast<uint8_t>(currentOrientation));
  return true;
}

bool hasGifExtension(const String &name) {
  String lower = name;
  lower.toLowerCase();
  return lower.endsWith(".gif");
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
  clearFrameLayout();
  // drawOrientationBadge();
  return true;
}

void showNextGif() {
  if (turnTransitionPending) {
    return;
  }

  if (gifCount == 0) {
    return;
  }

  const size_t nextIndex = (currentGifIndex + 1) % gifCount;
  openGifByIndex(nextIndex);
}

void restartCurrentGif() {
  if (!gifIsOpen) {
    return;
  }

  gif.reset();
  nextFrameAtMs = millis();
}

void onTouchInterrupt() {
  if (touch) {
    touch->IIC_Interrupt_Flag = true;
  }
}

void drawOrientationBadge() {
  gfx->fillRect(6, 6, 190, 20, BLACK);
  gfx->setTextColor(WHITE);
  gfx->setTextSize(1);
  gfx->setCursor(8, 12);
  gfx->print(orientationLabel(currentOrientation));
}

void applyDisplayOrientation(ScreenOrientation orientation, bool force, bool reopenGif) {
  if (!force && orientation == currentOrientation) {
    return;
  }

  currentOrientation = orientation;
  orientationChangedAtMs = millis();
  gfx->setRotation(displayRotationForOrientation(currentOrientation));
  clearFrameLayout();

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
  const int transitionGifIndex = findGifIndexByName(transitionGifName);
  const int defaultIndex = findGifIndexByName("/default.gif");
  if (transitionGifIndex < 0 || defaultIndex < 0) {
    return false;
  }

  pendingOrientation = nextOrientation;
  pendingDefaultGifIndex = defaultIndex;
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

void updateOrientationFromImu() {
  if (!imuReady || turnTransitionPending) {
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

  openGifByIndex(0);
}

int findGifIndexByName(const String &filename) {
  for (size_t i = 0; i < gifCount; i++) {
    if (gifPaths[i].endsWith(filename)) {
      return static_cast<int>(i);
    }
  }
  return -1;
}

}  // namespace

void setup() {
  waitForUsbSerial();

  gfx->begin();
  pinMode(kLcdBacklightPin, OUTPUT);
  digitalWrite(kLcdBacklightPin, HIGH);

  applyDisplayOrientation(ScreenOrientation::Portrait, true);

  Serial.println();
  Serial.println("Starting GIF touch demo");

  initTouch();
  initImu();
  initGifPlayer();

  Serial.println("Ready: touch the screen to switch GIFs.");
}

void loop() {
  updateOrientationFromImu();

  if (readTouchEvent()) {
    const uint32_t now = millis();
    if ((now - lastTouchMs) >= kTouchDebounceMs) {
      lastTouchMs = now;
      showNextGif();
    }
  }

  if (gifIsOpen) {
    const uint32_t now = millis();
    if (now >= nextFrameAtMs) {
      int frameDelayMs = 0;
      const int result = gif.playFrame(false, &frameDelayMs);

      if (result < 0) {
        Serial.printf("GIF decode error: %d\r\n", gif.getLastError());
        openGifByIndex(currentGifIndex);
      } 
      else if (result == 0) {
        if (turnTransitionPending &&
            (gifPaths[currentGifIndex].endsWith("/turnLeft.gif") ||
             gifPaths[currentGifIndex].endsWith("/turnRight.gif"))) {
          turnTransitionPending = false;
          // gfx->fillScreen(WHITE);
          applyDisplayOrientation(pendingOrientation, false, false);
          clearFrameLayout();
          if (pendingDefaultGifIndex >= 0) {
            openGifByIndex(static_cast<size_t>(pendingDefaultGifIndex));
          } else {
            restartCurrentGif();
          }
        } 
        else if (gifPaths[currentGifIndex].endsWith("/wakeUp_240.gif")) {
          const int defaultIndex = findGifIndexByName("/default.gif");

          if (defaultIndex >= 0) {
            openGifByIndex(static_cast<size_t>(defaultIndex));
          } else {
            restartCurrentGif();
          }
        } else {
          restartCurrentGif();
        }
      } else {
        if (frameDelayMs < 10) {
          frameDelayMs = 10;
        }
        nextFrameAtMs = now + static_cast<uint32_t>(frameDelayMs);
      }
      // drawOrientationBadge();
    }
  }
  Serial.println(orientationLabel(currentOrientation));
  delay(1);
  yield();
}
