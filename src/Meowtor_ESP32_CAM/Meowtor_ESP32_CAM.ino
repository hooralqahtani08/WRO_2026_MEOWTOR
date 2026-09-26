/*
  Meowtor 2026 - ESP32-CAM vision controller

  Target: AI Thinker ESP32-CAM with OV3660/OV2640, Arduino-ESP32 3.x.
  This controller performs vision only. The Raspberry Pi Pico 2 owns the
  motor, steering servo, ToF sensors, IMU, start button and run state machine.

  WRO 2026 compliance:
    - Wi-Fi and Bluetooth are explicitly disabled (rules 11.6, 11.10, 11.17).
    - UART is the only link to the Pico 2.
    - Red traffic sign => vehicle must pass on its RIGHT.
    - Green traffic sign => vehicle must pass on its LEFT.

  ANTi-inspired techniques adapted to our hardware:
    - low-resolution ROI processing, largest connected blob selection,
      detection memory, fixed-rate packets and lighting-tolerant chromaticity.
  This is an independent C++ implementation for ESP32-CAM, not ANTi code.

  UART wiring after uploading:
    ESP32-CAM U0T / GPIO1 -> Pico GP1 / UART0 RX
    ESP32-CAM U0R / GPIO3 <- Pico GP0 / UART0 TX (optional telemetry)
    ESP32-CAM GND          -> Pico GND
  Disconnect the Pico UART wires while uploading this sketch.
*/

#include <Arduino.h>
#include "esp_camera.h"
#include <WiFi.h>
#include "esp32-hal-bt.h"

// ----------------------- CALIBRATION SETTINGS -----------------------
constexpr bool CAMERA_VFLIP = true;
constexpr bool CAMERA_HMIRROR = false;
// The esp32-camera RGB565 framebuffer normally stores the high byte first.
// If a bench test reports blue when shown red (and red when shown blue),
// change this one value to true and upload again.
constexpr bool RGB565_SWAP_BYTES = false;
constexpr bool FLASH_ALWAYS_ON = true;
constexpr uint8_t FLASH_PIN = 4;
constexpr uint32_t UART_BAUD = 115200;
constexpr uint32_t PACKET_INTERVAL_MS = 60;   // about 16 packets/s

// The classifier uses normalized colour, so brightness may change without
// changing the class. Tune these two values using real competition objects.
constexpr uint8_t MIN_BRIGHTNESS = 28;
constexpr uint16_t MAX_COLOUR_DISTANCE = 2500;
constexpr uint16_t MIN_BLOB_CELLS = 7;

// ----------------------- AI THINKER CAMERA PINS ---------------------
constexpr int PWDN_GPIO_NUM = 32;
constexpr int RESET_GPIO_NUM = -1;
constexpr int XCLK_GPIO_NUM = 0;
constexpr int SIOD_GPIO_NUM = 26;
constexpr int SIOC_GPIO_NUM = 27;
constexpr int Y9_GPIO_NUM = 35;
constexpr int Y8_GPIO_NUM = 34;
constexpr int Y7_GPIO_NUM = 39;
constexpr int Y6_GPIO_NUM = 36;
constexpr int Y5_GPIO_NUM = 21;
constexpr int Y4_GPIO_NUM = 19;
constexpr int Y3_GPIO_NUM = 18;
constexpr int Y2_GPIO_NUM = 5;
constexpr int VSYNC_GPIO_NUM = 25;
constexpr int HREF_GPIO_NUM = 23;
constexpr int PCLK_GPIO_NUM = 22;

constexpr int FRAME_W = 160;
constexpr int FRAME_H = 120;
constexpr int STEP = 2;
constexpr int ROI_Y0 = 18;
constexpr int GRID_W = FRAME_W / STEP;
constexpr int GRID_H = (FRAME_H - ROI_Y0) / STEP;
constexpr int GRID_N = GRID_W * GRID_H;

enum PixelClass : uint8_t {
  PX_NONE = 0,
  PX_RED = 1,
  PX_GREEN = 2,
  PX_MAGENTA = 3,
  PX_ORANGE = 4,
  PX_BLUE = 5
};

struct Prototype {
  uint8_t r;
  uint8_t g;
  uint8_t b;
  PixelClass label;
};

// Normalized RGB prototypes. The first red and green samples are derived from
// the official WRO colours RGB(238,39,55) and RGB(68,214,44). Extra samples
// cover shadows/highlights. Replace/add samples after collecting field data.
const Prototype TRAINING_SET[] = {
  {183, 30, 42, PX_RED}, {170, 43, 42, PX_RED},
  {155, 48, 52, PX_RED}, {193, 34, 28, PX_RED},
  {53, 167, 35, PX_GREEN}, {45, 159, 51, PX_GREEN},
  {63, 145, 47, PX_GREEN}, {39, 178, 38, PX_GREEN},
  {128, 0, 127, PX_MAGENTA}, {150, 20, 85, PX_MAGENTA},
  {176, 70, 9, PX_ORANGE}, {161, 83, 11, PX_ORANGE},
  {20, 67, 168, PX_BLUE}, {29, 82, 144, PX_BLUE}
};

struct Blob {
  bool valid = false;
  uint16_t cells = 0;
  int16_t cx = -1;
  int16_t cy = -1;
  int16_t width = 0;
  int16_t height = 0;
};

static uint8_t classGrid[GRID_N];
static uint8_t visited[GRID_N];
static uint16_t floodQueue[GRID_N];
uint32_t frameSequence = 0;
uint32_t lastPacketAt = 0;

PixelClass classifyPixel(uint8_t r, uint8_t g, uint8_t b) {
  const uint16_t sum = uint16_t(r) + g + b;
  const uint8_t brightest = max(r, max(g, b));
  const uint8_t darkest = min(r, min(g, b));
  if (brightest < MIN_BRIGHTNESS || brightest - darkest < 22 || sum < 45) {
    return PX_NONE;
  }

  const int rn = (uint32_t(r) * 255U) / sum;
  const int gn = (uint32_t(g) * 255U) / sum;
  const int bn = 255 - rn - gn;
  uint32_t bestDistance = UINT32_MAX;
  PixelClass bestClass = PX_NONE;

  for (const Prototype &p : TRAINING_SET) {
    const int dr = rn - p.r;
    const int dg = gn - p.g;
    const int db = bn - p.b;
    const uint32_t distance = uint32_t(dr * dr + dg * dg + db * db);
    if (distance < bestDistance) {
      bestDistance = distance;
      bestClass = p.label;
    }
  }
  return bestDistance <= MAX_COLOUR_DISTANCE ? bestClass : PX_NONE;
}

void decodeFrame(const camera_fb_t *fb) {
  memset(classGrid, 0, sizeof(classGrid));
  if (!fb || fb->format != PIXFORMAT_RGB565 || fb->width != FRAME_W ||
      fb->height != FRAME_H) return;

  for (int gy = 0; gy < GRID_H; ++gy) {
    const int y = ROI_Y0 + gy * STEP;
    for (int gx = 0; gx < GRID_W; ++gx) {
      const int x = gx * STEP;
      const size_t offset = (size_t(y) * FRAME_W + x) * 2;
      const uint8_t first = fb->buf[offset];
      const uint8_t second = fb->buf[offset + 1];
      const uint16_t raw = RGB565_SWAP_BYTES
                               ? (uint16_t(second) << 8) | first
                               : (uint16_t(first) << 8) | second;
      const uint8_t r5 = (raw >> 11) & 0x1F;
      const uint8_t g6 = (raw >> 5) & 0x3F;
      const uint8_t b5 = raw & 0x1F;
      const uint8_t r = (r5 << 3) | (r5 >> 2);
      const uint8_t g = (g6 << 2) | (g6 >> 4);
      const uint8_t b = (b5 << 3) | (b5 >> 2);
      classGrid[gy * GRID_W + gx] = classifyPixel(r, g, b);
    }
  }
}

void insertBlob(Blob candidate, Blob &first, Blob &second) {
  if (!candidate.valid) return;
  if (!first.valid || candidate.cells > first.cells) {
    second = first;
    first = candidate;
  } else if (!second.valid || candidate.cells > second.cells) {
    second = candidate;
  }
}

void findTwoLargest(PixelClass target, Blob &first, Blob &second) {
  first = Blob{};
  second = Blob{};
  memset(visited, 0, sizeof(visited));

  for (int start = 0; start < GRID_N; ++start) {
    if (visited[start] || classGrid[start] != target) continue;

    int head = 0, tail = 0;
    floodQueue[tail++] = start;
    visited[start] = 1;
    uint32_t sumX = 0, sumY = 0;
    int minX = GRID_W, maxX = 0, minY = GRID_H, maxY = 0;

    while (head < tail) {
      const int index = floodQueue[head++];
      const int x = index % GRID_W;
      const int y = index / GRID_W;
      sumX += x;
      sumY += y;
      minX = min(minX, x); maxX = max(maxX, x);
      minY = min(minY, y); maxY = max(maxY, y);

      const int neighbours[4] = {index - 1, index + 1, index - GRID_W,
                                 index + GRID_W};
      for (int n = 0; n < 4; ++n) {
        const int ni = neighbours[n];
        if (ni < 0 || ni >= GRID_N) continue;
        const int nx = ni % GRID_W;
        if ((n == 0 || n == 1) && abs(nx - x) != 1) continue;
        if (!visited[ni] && classGrid[ni] == target) {
          visited[ni] = 1;
          floodQueue[tail++] = ni;
        }
      }
    }

    if (tail < MIN_BLOB_CELLS) continue;
    Blob candidate;
    candidate.valid = true;
    candidate.cells = tail;
    candidate.cx = int16_t((sumX / tail) * STEP + STEP / 2);
    candidate.cy = int16_t(ROI_Y0 + (sumY / tail) * STEP + STEP / 2);
    candidate.width = int16_t((maxX - minX + 1) * STEP);
    candidate.height = int16_t((maxY - minY + 1) * STEP);
    insertBlob(candidate, first, second);
  }
}

int valueOrMinusOne(const Blob &b, int16_t Blob::*member) {
  return b.valid ? b.*member : -1;
}

void sendVisionPacket(const Blob &red, const Blob &green,
                      const Blob &magenta1, const Blob &magenta2) {
  const int magCount = int(magenta1.valid) + int(magenta2.valid);
  const int magHeight = max(valueOrMinusOne(magenta1, &Blob::height),
                            valueOrMinusOne(magenta2, &Blob::height));
  // V,seq,redX,redH,redArea,greenX,greenH,greenArea,
  //   magentaCount,magentaX1,magentaX2,magentaHeight\n
  Serial.printf("V,%lu,%d,%d,%u,%d,%d,%u,%d,%d,%d,%d\n",
                (unsigned long)frameSequence,
                valueOrMinusOne(red, &Blob::cx),
                valueOrMinusOne(red, &Blob::height), red.cells,
                valueOrMinusOne(green, &Blob::cx),
                valueOrMinusOne(green, &Blob::height), green.cells,
                magCount, valueOrMinusOne(magenta1, &Blob::cx),
                valueOrMinusOne(magenta2, &Blob::cx), magHeight);
}

bool startCamera() {
  camera_config_t c = {};
  c.ledc_channel = LEDC_CHANNEL_0;
  c.ledc_timer = LEDC_TIMER_0;
  c.pin_d0 = Y2_GPIO_NUM; c.pin_d1 = Y3_GPIO_NUM;
  c.pin_d2 = Y4_GPIO_NUM; c.pin_d3 = Y5_GPIO_NUM;
  c.pin_d4 = Y6_GPIO_NUM; c.pin_d5 = Y7_GPIO_NUM;
  c.pin_d6 = Y8_GPIO_NUM; c.pin_d7 = Y9_GPIO_NUM;
  c.pin_xclk = XCLK_GPIO_NUM; c.pin_pclk = PCLK_GPIO_NUM;
  c.pin_vsync = VSYNC_GPIO_NUM; c.pin_href = HREF_GPIO_NUM;
  c.pin_sccb_sda = SIOD_GPIO_NUM; c.pin_sccb_scl = SIOC_GPIO_NUM;
  c.pin_pwdn = PWDN_GPIO_NUM; c.pin_reset = RESET_GPIO_NUM;
  c.xclk_freq_hz = 20000000;
  c.pixel_format = PIXFORMAT_RGB565;
  c.frame_size = FRAMESIZE_QQVGA;
  c.fb_count = psramFound() ? 2 : 1;
  c.grab_mode = CAMERA_GRAB_LATEST;
  c.fb_location = psramFound() ? CAMERA_FB_IN_PSRAM : CAMERA_FB_IN_DRAM;

  if (esp_camera_init(&c) != ESP_OK) return false;
  sensor_t *camera = esp_camera_sensor_get();
  if (camera) {
    camera->set_vflip(camera, CAMERA_VFLIP);
    camera->set_hmirror(camera, CAMERA_HMIRROR);
    camera->set_saturation(camera, 1);
    camera->set_whitebal(camera, 1);
    camera->set_awb_gain(camera, 1);
    camera->set_exposure_ctrl(camera, 1);
    camera->set_gain_ctrl(camera, 1);
  }
  return true;
}

void fatalSignal(uint8_t flashes) {
  for (;;) {
    for (uint8_t i = 0; i < flashes; ++i) {
      digitalWrite(FLASH_PIN, HIGH); delay(120);
      digitalWrite(FLASH_PIN, LOW); delay(180);
    }
    delay(900);
  }
}

void setup() {
  pinMode(FLASH_PIN, OUTPUT);
  digitalWrite(FLASH_PIN, LOW);
  Serial.begin(UART_BAUD);
  delay(300);

  // Required for competition: do not create an access point or use RF.
  WiFi.disconnect(true, true);
  WiFi.mode(WIFI_OFF);
  btStop();

  if (!startCamera()) fatalSignal(3);
  digitalWrite(FLASH_PIN, FLASH_ALWAYS_ON ? HIGH : LOW);
  Serial.println("READY,VISION,MEOWTOR2026");
}

void loop() {
  camera_fb_t *frame = esp_camera_fb_get();
  if (!frame) {
    Serial.println("E,CAMERA_FRAME");
    delay(20);
    return;
  }

  decodeFrame(frame);
  Blob red1, red2, green1, green2, magenta1, magenta2;
  findTwoLargest(PX_RED, red1, red2);
  findTwoLargest(PX_GREEN, green1, green2);
  findTwoLargest(PX_MAGENTA, magenta1, magenta2);
  esp_camera_fb_return(frame);

  ++frameSequence;
  if (millis() - lastPacketAt >= PACKET_INTERVAL_MS) {
    lastPacketAt = millis();
    sendVisionPacket(red1, green1, magenta1, magenta2);
  }
}
