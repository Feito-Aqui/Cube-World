#include <Arduino.h>
#include <Wire.h>

#include <LittleFS.h>

#include <memory>

#include "AnimatedGIF.h"
#include "Arduino_DriveBus_Library.h"
#include "Arduino_GFX_Library.h"

namespace {

constexpr uint16_t kPanelWidth = 240;
constexpr uint16_t kPanelHeight = 280;

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
constexpr size_t kMaxGifFiles = 16;

Arduino_DataBus *bus = new Arduino_ESP32SPI(kLcdDcPin, kLcdCsPin, kLcdSckPin, kLcdMosiPin);
Arduino_GFX *gfx =
    new Arduino_ST7789(bus, kLcdResetPin, 0, true, kPanelWidth, kPanelHeight, 0, 20, 0, 0);

std::shared_ptr<Arduino_IIC_DriveBus> i2cBus =
    std::make_shared<Arduino_HWIIC>(kTouchSdaPin, kTouchSclPin, &Wire);
std::unique_ptr<Arduino_IIC> touch;

AnimatedGIF gif;
File gifFile;
String gifPaths[kMaxGifFiles];
size_t gifCount = 0;
size_t currentGifIndex = 0;
bool gifIsOpen = false;
int16_t gifOffsetX = 0;
int16_t gifOffsetY = 0;
uint16_t lineBuffer[kPanelWidth];
uint32_t lastTouchMs = 0;
uint32_t nextFrameAtMs = 0;

void onTouchInterrupt();
void gifDraw(GIFDRAW *draw);
void closeCurrentGif();
bool openGifByIndex(size_t index);
void restartCurrentGif();

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
  gfx->fillScreen(BLACK);
  gfx->setTextColor(WHITE);
  gfx->setTextSize(2);
  gfx->setCursor(16, 120);
  gfx->println(line1);

  if (line2) {
    gfx->setCursor(16, 150);
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

bool readTouchEvent() {
  if (!touch || !touch->IIC_Interrupt_Flag) {
    return false;
  }

  touch->IIC_Interrupt_Flag = false;

  const uint16_t x = touch->IIC_Read_Device_Value(
      Arduino_IIC_Touch::Value_Information::TOUCH_COORDINATE_X);
  const uint16_t y = touch->IIC_Read_Device_Value(
      Arduino_IIC_Touch::Value_Information::TOUCH_COORDINATE_Y);

  Serial.printf("Touch: x=%u y=%u\r\n", x, y);
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

  // gfx->fillScreen(BLACK);

  const String &path = gifPaths[index];
  Serial.printf("Opening GIF: %s\r\n", path.c_str());
  if (!gif.open(path.c_str(), gifOpenFile, gifCloseFile, gifReadFile, gifSeekFile, gifDraw)) {
    Serial.printf("Failed to open GIF: %s\r\n", path.c_str());
    drawStatus("GIF open failed", path.c_str());
    return false;
  }

  gifOffsetX = (kPanelWidth - gif.getCanvasWidth()) / 2;
  gifOffsetY = (kPanelHeight - gif.getCanvasHeight()) / 2;
  if (gifOffsetX < 0) {
    gifOffsetX = 0;
  }
  if (gifOffsetY < 0) {
    gifOffsetY = 0;
  }

  currentGifIndex = index;
  gifIsOpen = true;
  nextFrameAtMs = millis();
  return true;
}

void showNextGif() {
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

void gifDraw(GIFDRAW *draw) {
  int16_t y = gifOffsetY + draw->iY + draw->y;
  if (y < 0 || y >= static_cast<int16_t>(kPanelHeight)) {
    return;
  }

  int16_t x = gifOffsetX + draw->iX;
  if (x >= static_cast<int16_t>(kPanelWidth)) {
    return;
  }

  uint16_t *palette = draw->pPalette;
  uint8_t *source = draw->pPixels;
  int16_t width = draw->iWidth;

  if ((x + width) > static_cast<int16_t>(kPanelWidth)) {
    width = kPanelWidth - x;
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

}  // namespace

int findGifIndexByName(const String &filename) {
  for (size_t i = 0; i < gifCount; i++) {
    if (gifPaths[i].endsWith(filename)) {
      return i;
    }
  }
  return -1; // Não encontrou
}

void setup() {
  waitForUsbSerial();

  gfx->begin();
  gfx->setRotation(0);

  pinMode(kLcdBacklightPin, OUTPUT);
  digitalWrite(kLcdBacklightPin, HIGH);

  Serial.println();
  Serial.println("Starting GIF touch demo");

  initTouch();
  initGifPlayer();

  Serial.println("Ready: touch the screen to switch GIFs.");
}

void loop() {
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
        // --- INÍCIO DA MUDANÇA ---
        // Se o GIF que acabou de terminar for o Sleep_240.gif
        if (gifPaths[currentGifIndex].endsWith("/wakeUp_240.gif")) {
          int defaultIndex = findGifIndexByName("/default.gif");
          
          if (defaultIndex >= 0) {
            openGifByIndex(defaultIndex); // Troca automaticamente para o default.gif
          } else {
            restartCurrentGif(); // Prevenção de erro: se default.gif não existir, repete o atual
          }
        } else {
          restartCurrentGif(); // Comportamento normal para os outros GIFs: fica em loop
        }
        // --- FIM DA MUDANÇA ---
      } 
      else {
        if (frameDelayMs < 10) {
          frameDelayMs = 10;
        }
        nextFrameAtMs = now + static_cast<uint32_t>(frameDelayMs);
      }
    }
  }

  delay(1);
  yield();
}
