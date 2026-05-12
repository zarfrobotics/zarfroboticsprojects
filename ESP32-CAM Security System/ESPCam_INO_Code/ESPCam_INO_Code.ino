/*
  ╔══════════════════════════════════════════════════════════════╗
  ║           ESPCam — AI Thinker ESP32-CAM                      ║
  ║           Full Featured Security Camera Firmware             ║
  ╠══════════════════════════════════════════════════════════════╣
  ║  • 30 fps MJPEG live stream  (port 81)                       ║
  ║  • 15 fps AVI recording to SD (1-min segments)               ║
  ║  • Snapshot: save to SD  /  download to browser              ║
  ║  • Pan/Tilt servo  (GPIO14=pan, GPIO15=tilt)                 ║
  ║    — attached with 500-2500 µs pulse range (SG90 safe)       ║
  ║  • PIR motion detection (GPIO2) → auto snapshot              ║
  ║  • LED flash with PWM brightness  (GPIO4)                    ║
  ║  • SD browser: list / download / delete                      ║
  ║  • Stop-recording endpoint (/record?cmd=stop)                ║
  ║  • SPIFFS web UI served from /  (port 82)                    ║
  ║  • /ping  /status  /pir  /camset  endpoints                  ║
  ╠══════════════════════════════════════════════════════════════╣
  ║  BOARD  : AI Thinker ESP32-CAM                               ║
  ║  LIBS   : ESP32Servo, ArduinoJson, SPIFFS (built-in)         ║
  ║  AVI fmt: byte-exact                                         ║
  ╠══════════════════════════════════════════════════════════════╣
  ║  © 2026 Zarf Robotics — youtube.com/@ZarfRobotics2011        ║
  ╚══════════════════════════════════════════════════════════════╝
*/

#include "esp_camera.h"
#include "esp_http_server.h"
#include <WiFi.h>
#include <SD_MMC.h>
#include <FS.h>
#include <SPIFFS.h>
#include <ArduinoJson.h>
#include <ESP32Servo.h>
#include "time.h"

// ════════════════════════════════════════════════════════════════
//  USER CONFIG
// ════════════════════════════════════════════════════════════════
#define WIFI_SSID    "YOUR_WIFI_SSID"
#define WIFI_PASS    "YOUR_WIFI_PASSWORD"
#define DEVICE_NAME  "ESPCam"

// ════════════════════════════════════════════════════════════════
//  CAMERA PINS — AI Thinker ESP32-CAM
// ════════════════════════════════════════════════════════════════
#define PWDN_GPIO_NUM   32
#define RESET_GPIO_NUM  -1
#define XCLK_GPIO_NUM    0
#define SIOD_GPIO_NUM   26
#define SIOC_GPIO_NUM   27
#define Y9_GPIO_NUM     35
#define Y8_GPIO_NUM     34
#define Y7_GPIO_NUM     39
#define Y6_GPIO_NUM     36
#define Y5_GPIO_NUM     21
#define Y4_GPIO_NUM     19
#define Y3_GPIO_NUM     18
#define Y2_GPIO_NUM      5
#define VSYNC_GPIO_NUM  25
#define HREF_GPIO_NUM   23
#define PCLK_GPIO_NUM   22

// ════════════════════════════════════════════════════════════════
//  PERIPHERAL PINS
// ════════════════════════════════════════════════════════════════
// IMPORTANT: GPIO12 is the VDD_SDIO voltage-select strapping pin.
// If it is HIGH at boot, SD gets 1.8V and won't init.
// Servo MUST NOT be attached until AFTER SD_MMC.begin() returns.
#define PAN_PIN    12   // Pan  servo signal
#define TILT_PIN   13   // Tilt servo signal
#define PIR_PIN     2   // PIR HC-SR501 output
#define LED_PIN     4   // Flash LED (also SD_MMC CLK conflict — keep LOW)

// ════════════════════════════════════════════════════════════════
//  SERVER PORTS
// ════════════════════════════════════════════════════════════════
#define STREAM_PORT  81
#define CTRL_PORT    82

// ════════════════════════════════════════════════════════════════
//  AVI / SD CONFIG  
// ════════════════════════════════════════════════════════════════
#define RAMSIZE          (1024 * 8)   // 8 KB sector-aligned write buffer
#define CHUNK_HDR        8            // "00dc" (4) + size field (4)
#define AVI_HEADER_LEN   310          // exact — do not change
#define IDX_ENTRY        16           // bytes per idx1 entry
#define MAX_AVI_FRAMES   1800         // 15 fps × 120 s headroom
#define AVI_DURATION_MS  60000        // 1-minute segments
#define AVI_FPS          15           // recording fps
#define STREAM_FPS       30           // live stream fps
#define AVITEMP          "/avi_tmp.avi"

// LED PWM
#define LED_LEDC_CHANNEL  LEDC_CHANNEL_3   // camera uses 0-2
#define LED_LEDC_TIMER    LEDC_TIMER_1
#define LED_LEDC_FREQ     1000
#define LED_LEDC_RES      8                // 8-bit → 0-255

// ════════════════════════════════════════════════════════════════
//  MJPEG STREAM BOUNDARY
// ════════════════════════════════════════════════════════════════
#define PART_BOUNDARY "123456789000000000000987654321"
static const char* STREAM_CONTENT_TYPE =
  "multipart/x-mixed-replace;boundary=" PART_BOUNDARY;
static const char* STREAM_BOUNDARY = "\r\n--" PART_BOUNDARY "\r\n";
static const char* STREAM_PART =
  "Content-Type: image/jpeg\r\nContent-Length: %u\r\n\r\n";

// ════════════════════════════════════════════════════════════════
//  AVI HEADER  — exact byte array
//
//  Verified patch offsets (byte-counted):
//   0x04        RIFF payload size
//   0x20        usecs per frame
//   0x30        total frames  (avih, 2 bytes)
//   0x40/0x44   width / height (avih)
//   0x84        FPS            (strh rate, 1 byte)
//   0x8C        total frames   (strh length, 2 bytes)
//   0xA8/0xAC   width / height (strf BITMAPINFOHEADER)
//   0x12E       movi LIST data size
// ════════════════════════════════════════════════════════════════
static uint8_t aviHeader[AVI_HEADER_LEN] = {
  0x52,0x49,0x46,0x46, 0x00,0x00,0x00,0x00, 0x41,0x56,0x49,0x20, 0x4C,0x49,0x53,0x54,
  0x16,0x01,0x00,0x00, 0x68,0x64,0x72,0x6C, 0x61,0x76,0x69,0x68, 0x38,0x00,0x00,0x00,
  0x00,0x00,0x00,0x00, 0x00,0x00,0x00,0x00, 0x00,0x00,0x00,0x00, 0x10,0x00,0x00,0x00,
  0x00,0x00,0x00,0x00, 0x00,0x00,0x00,0x00, 0x01,0x00,0x00,0x00, 0x00,0x00,0x00,0x00,
  0x00,0x00,0x00,0x00, 0xe0,0x00,0x00,0x00, 0x00,0x00,0x00,0x00, 0x00,0x00,0x00,0x00,
  0x00,0x00,0x00,0x00, 0x00,0x00,0x00,0x00, 0x4C,0x49,0x53,0x54, 0x6C,0x00,0x00,0x00,
  0x73,0x74,0x72,0x6C, 0x73,0x74,0x72,0x68, 0x30,0x00,0x00,0x00, 0x76,0x69,0x64,0x73,
  0x4D,0x4A,0x50,0x47, 0x00,0x00,0x00,0x00, 0x00,0x00,0x00,0x00, 0x00,0x00,0x00,0x00,
  0x01,0x00,0x00,0x00, 0x00,0x00,0x00,0x00, 0x00,0x00,0x00,0x00, 0x0A,0x00,0x00,0x00,
  0x00,0x00,0x00,0x00, 0x00,0x00,0x00,0x00, 0x00,0x00,0x00,0x00, 0x73,0x74,0x72,0x66,
  0x28,0x00,0x00,0x00, 0x28,0x00,0x00,0x00, 0x00,0x00,0x00,0x00, 0x00,0x00,0x00,0x00,
  0x01,0x00,0x18,0x00, 0x4D,0x4A,0x50,0x47, 0x00,0x00,0x00,0x00, 0x00,0x00,0x00,0x00,
  0x00,0x00,0x00,0x00, 0x00,0x00,0x00,0x00, 0x00,0x00,0x00,0x00,
  0x4C,0x49,0x53,0x54, 0x56,0x00,0x00,0x00,
  0x73,0x74,0x72,0x6C, 0x73,0x74,0x72,0x68, 0x30,0x00,0x00,0x00, 0x61,0x75,0x64,0x73,
  0x00,0x00,0x00,0x00, 0x00,0x00,0x00,0x00, 0x00,0x00,0x00,0x00, 0x00,0x00,0x00,0x00,
  0x01,0x00,0x00,0x00, 0x11,0x2B,0x00,0x00, 0x00,0x00,0x00,0x00, 0x00,0x00,0x00,0x00,
  0x11,0x2B,0x00,0x00, 0x00,0x00,0x00,0x00, 0x02,0x00,0x00,0x00, 0x73,0x74,0x72,0x66,
  0x12,0x00,0x00,0x00, 0x01,0x00,0x01,0x00, 0x11,0x2B,0x00,0x00, 0x11,0x2B,0x00,0x00,
  0x02,0x00,0x10,0x00, 0x00,0x00,
  0x4C,0x49,0x53,0x54, 0x00,0x00,0x00,0x00, 0x6D,0x6F,0x76,0x69,
};

// VGA 640×480 byte-pairs for width/height patching
static const uint8_t VGA_W[2] = {0x80, 0x02};  // 640
static const uint8_t VGA_H[2] = {0xE0, 0x01};  // 480

// AVI markers
static const uint8_t dcBuf[4]   = {0x30,0x30,0x64,0x63}; // 00dc
static const uint8_t idx1Buf[4] = {0x69,0x64,0x78,0x31}; // idx1
static const uint8_t zeroBuf[4] = {0x00,0x00,0x00,0x00};

// ════════════════════════════════════════════════════════════════
//  GLOBALS
// ════════════════════════════════════════════════════════════════
Servo panServo, tiltServo;
int   panAngle  = 90;
int   tiltAngle = 90;
#define SERVO_PAN_LEDC   4
#define SERVO_TILT_LEDC  5

httpd_handle_t  streamServer = NULL;
httpd_handle_t  ctrlServer   = NULL;
SemaphoreHandle_t camMutex   = NULL;
SemaphoreHandle_t aviMutex   = NULL;

// ── AVI state ──────────────────────────────────────────────────
static uint8_t  iSDbuffer[RAMSIZE * 2 + CHUNK_HDR];
static size_t   highPoint    = 0;
static File     aviFile;
static uint32_t aviStartMs   = 0;
static uint16_t aviFrameCnt  = 0;
static size_t   aviMoViSize  = 0;
static bool     aviRecording = false;
static bool     userStopReq  = false;   // set by /record?cmd=stop
static char     aviFileName[200];

// AVI index (PSRAM preferred)
static uint8_t* idxBuf    = NULL;
static size_t   idxPtr    = 0;
static size_t   idxOffset = 4;
static size_t   idxLen    = 0;

// ── Misc state ─────────────────────────────────────────────────
bool          sdReady       = false;
int           snapCount     = 0;
uint8_t       ledBrightness = 0;         // 0 = off
unsigned long lastMotionSave = 0;
bool          timeSet        = false;
struct tm     timeinfo;

TaskHandle_t aviTaskHandle = NULL;
TaskHandle_t pirTaskHandle = NULL;

// ════════════════════════════════════════════════════════════════
//  CHIP TEMPERATURE  (ESP32 internal sensor)
//  Uses Espressif's undocumented but stable API.
//  Typical idle: 40–50°C, under load (stream+record): 55–75°C
// ════════════════════════════════════════════════════════════════
static float getChipTemp() {
  // ESP32 core v3.x uses temperatureRead() which returns °C directly
  return temperatureRead();
}

// ════════════════════════════════════════════════════════════════
//  UTILITY
// ════════════════════════════════════════════════════════════════
static bool getTimeStr(char* dayDir, char* hourDir,
                       char* timestamp, char* datestamp) {
  if (!getLocalTime(&timeinfo)) return false;
  strftime(dayDir,    20, "%d_%m_%Y",       &timeinfo);
  int h = timeinfo.tm_hour;
  snprintf(hourDir,   20, "%02d00-%02d00",  h, h + 1);
  strftime(timestamp, 30, "%d%m%Y_%H%M_%S", &timeinfo);
  strftime(datestamp, 25, "%d%m%Y_%H%M",    &timeinfo);
  return true;
}

static void sanitizePath(const char* in, char* out, size_t sz) {
  const char* p = in;
  if (strncmp(p, "/sdcard", 7) == 0) p += 7;
  if (*p == '\0') p = "/";
  strncpy(out, p, sz - 1);
  out[sz - 1] = '\0';
}

// ════════════════════════════════════════════════════════════════
//  LED CONTROL  (PWM brightness via LEDC)
// ════════════════════════════════════════════════════════════════
static void ledInit() {
  ledc_timer_config_t t = {};
  t.speed_mode      = LEDC_HIGH_SPEED_MODE;
  t.timer_num       = LED_LEDC_TIMER;
  t.duty_resolution = (ledc_timer_bit_t)LED_LEDC_RES;
  t.freq_hz         = LED_LEDC_FREQ;
  t.clk_cfg         = LEDC_AUTO_CLK;
  ledc_timer_config(&t);

  ledc_channel_config_t c = {};
  c.speed_mode = LEDC_HIGH_SPEED_MODE;
  c.channel    = LED_LEDC_CHANNEL;
  c.timer_sel  = LED_LEDC_TIMER;
  c.intr_type  = LEDC_INTR_DISABLE;
  c.gpio_num   = LED_PIN;
  c.duty       = 0;
  c.hpoint     = 0;
  ledc_channel_config(&c);
}

static void ledSet(uint8_t brightness) {
  ledBrightness = brightness;
  ledc_set_duty(LEDC_HIGH_SPEED_MODE, LED_LEDC_CHANNEL, brightness);
  ledc_update_duty(LEDC_HIGH_SPEED_MODE, LED_LEDC_CHANNEL);
}

// ════════════════════════════════════════════════════════════════
//  CAMERA INIT
// ════════════════════════════════════════════════════════════════
bool initCamera() {
  camera_config_t cfg = {};
  cfg.ledc_channel = LEDC_CHANNEL_0;
  cfg.ledc_timer   = LEDC_TIMER_0;
  cfg.pin_d0=Y2_GPIO_NUM; cfg.pin_d1=Y3_GPIO_NUM;
  cfg.pin_d2=Y4_GPIO_NUM; cfg.pin_d3=Y5_GPIO_NUM;
  cfg.pin_d4=Y6_GPIO_NUM; cfg.pin_d5=Y7_GPIO_NUM;
  cfg.pin_d6=Y8_GPIO_NUM; cfg.pin_d7=Y9_GPIO_NUM;
  cfg.pin_xclk=XCLK_GPIO_NUM;   cfg.pin_pclk=PCLK_GPIO_NUM;
  cfg.pin_vsync=VSYNC_GPIO_NUM; cfg.pin_href=HREF_GPIO_NUM;
  cfg.pin_sscb_sda=SIOD_GPIO_NUM; cfg.pin_sscb_scl=SIOC_GPIO_NUM;
  cfg.pin_pwdn=PWDN_GPIO_NUM;   cfg.pin_reset=RESET_GPIO_NUM;
  cfg.xclk_freq_hz = 20000000;
  cfg.pixel_format = PIXFORMAT_JPEG;
  cfg.frame_size   = FRAMESIZE_VGA;
  cfg.jpeg_quality = 10;    // quality 10 → good image, manageable size
  cfg.fb_count     = 2;     // 2 buffers: stream reads one while camera fills other
  cfg.grab_mode    = CAMERA_GRAB_LATEST;
  if (esp_camera_init(&cfg) != ESP_OK) {
    Serial.println("[CAM] Init failed");
    return false;
  }
  sensor_t* s = esp_camera_sensor_get();
  s->set_brightness(s,1);  s->set_contrast(s,0);
  s->set_saturation(s,-1); s->set_whitebal(s,1);
  s->set_awb_gain(s,1);    s->set_exposure_ctrl(s,1);
  s->set_gain_ctrl(s,1);   s->set_raw_gma(s,1);
  s->set_lenc(s,1);        s->set_dcw(s,1);  s->set_wpc(s,1);
  Serial.println("[CAM] OK — VGA 640×480, 2 frame buffers");
  return true;
}

// ════════════════════════════════════════════════════════════════
//  SD INIT  (5-attempt retry — handles intermittent boot failures)
// ════════════════════════════════════════════════════════════════
bool initSD() {
  // GPIO4 (flash LED) can corrupt SD CLK on some AI Thinker revisions
  // GPIO2 is SD D0 — needs pull-up for reliable 1-bit mode
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);


  for (int attempt = 1; attempt <= 5; attempt++) {
    Serial.printf("[SD] Attempt %d/5\n", attempt);
    SD_MMC.end();
    delay(300);
    digitalWrite(LED_PIN, LOW);  // re-assert after end()

    if (SD_MMC.begin("/sdcard", true, false, SDMMC_FREQ_DEFAULT)) {
      if (SD_MMC.cardType() != CARD_NONE) {
        SD_MMC.mkdir("/videos");
        SD_MMC.mkdir("/photos");
        Serial.printf("[SD] OK — %.1f GB total, %.1f GB free\n",
          SD_MMC.totalBytes() / 1073741824.0,
          (SD_MMC.totalBytes() - SD_MMC.usedBytes()) / 1073741824.0);
        return true;
      }
    }
    Serial.printf("[SD] Attempt %d failed\n", attempt);
    delay(500 * attempt);
  }
  Serial.println("[SD] All attempts failed — recording disabled");
  return false;
}

// ════════════════════════════════════════════════════════════════
//  AVI INDEX  
// ════════════════════════════════════════════════════════════════
static void prepAviIndex() {
  if (!idxBuf) {
    size_t need = (size_t)(MAX_AVI_FRAMES + 2) * IDX_ENTRY;
    idxBuf = psramFound() ? (uint8_t*)ps_malloc(need) : (uint8_t*)malloc(need);
    if (!idxBuf) { Serial.println("[AVI] idx malloc failed!"); return; }
  }
  memcpy(idxBuf, idx1Buf, 4);  // idx1 marker
  idxPtr      = CHUNK_HDR;     // leave room for 4-byte size field
  idxOffset   = 4;             // initial offset = 4 (past 'movi' fourCC)
  idxLen      = 0;
  aviMoViSize = 0;
}

static void buildAviIdx(size_t dataSize) {
  // dataSize = 4-byte-aligned jpeg size (NO chunk header)
  aviMoViSize += dataSize;
  if (idxPtr + IDX_ENTRY > (size_t)(MAX_AVI_FRAMES + 2) * IDX_ENTRY) {
    Serial.println("[AVI] index overflow — frame dropped from index");
    return;
  }
  memcpy(idxBuf + idxPtr,      dcBuf,      4);
  memcpy(idxBuf + idxPtr + 4,  zeroBuf,    4);
  memcpy(idxBuf + idxPtr + 8,  &idxOffset, 4);
  memcpy(idxBuf + idxPtr + 12, &dataSize,  4);
  idxOffset += dataSize + CHUNK_HDR;  // advance: chunk header + data
  idxPtr    += IDX_ENTRY;
}

static void finalizeAviIndex(uint16_t frameCnt) {
  uint32_t sz = (uint32_t)frameCnt * IDX_ENTRY;
  memcpy(idxBuf + 4, &sz, 4);         // patch idx1 size field
  idxLen = sz + CHUNK_HDR;            // total idx1 block (header + entries)
  idxPtr = 0;                         // reset for write-out
}

// ════════════════════════════════════════════════════════════════
//  AVI HEADER PATCH  
// ════════════════════════════════════════════════════════════════
static void patchAviHeader(uint8_t fps, uint16_t frameCnt,
                           size_t moviSize) {
  xSemaphoreTake(aviMutex, portMAX_DELAY);
  if (fps < 1) fps = 1;

  // RIFF payload = everything after the 8-byte "RIFF ????" header
  // = AVI_HEADER_LEN-8 + frameCnt*(CHUNK_HDR+IDX_ENTRY) + moviSize
  size_t aviSize = moviSize + AVI_HEADER_LEN
                 + (size_t)(CHUNK_HDR + IDX_ENTRY) * frameCnt;
  memcpy(aviHeader + 4, &aviSize, 4);

  // Microseconds per frame
  uint32_t usecs = (uint32_t)round(1000000.0f / fps);
  memcpy(aviHeader + 0x20, &usecs, 4);

  // Frame counts (2-byte fields)
  memcpy(aviHeader + 0x30, &frameCnt, 2);   // avih dwTotalFrames
  memcpy(aviHeader + 0x8C, &frameCnt, 2);   // strh dwLength

  // FPS — 1-byte rate field in strh
  memcpy(aviHeader + 0x84, &fps, 1);

  // Width / height in both avih and strf BITMAPINFOHEADER
  memcpy(aviHeader + 0x40, VGA_W, 2);  memcpy(aviHeader + 0x44, VGA_H, 2);
  memcpy(aviHeader + 0xA8, VGA_W, 2);  memcpy(aviHeader + 0xAC, VGA_H, 2);

  // movi LIST data size = moviSize + frameCnt*CHUNK_HDR + 4 ('movi' tag)
  uint32_t moviDataSz = (uint32_t)(moviSize + (size_t)frameCnt * CHUNK_HDR + 4);
  memcpy(aviHeader + 0x12E, &moviDataSz, 4);

  xSemaphoreGive(aviMutex);
}

// ════════════════════════════════════════════════════════════════
//  openAvi / saveAviFrame / closeAvi
// ════════════════════════════════════════════════════════════════
static void openAvi() {
  char dayDir[20], hourDir[20], ts[30], ds[25];
  if (!getTimeStr(dayDir, hourDir, ts, ds)) {
    snprintf(ds,      sizeof(ds),      "notime_%lu", millis() / 1000);
    snprintf(dayDir,  sizeof(dayDir),  "00_00_0000");
    snprintf(hourDir, sizeof(hourDir), "0000-0100");
  }

  char d1[80], d2[80];
  snprintf(d1, sizeof(d1), "/videos/%s", dayDir);         SD_MMC.mkdir(d1);
  snprintf(d2, sizeof(d2), "/videos/%s/%s", dayDir, hourDir); SD_MMC.mkdir(d2);

  snprintf(aviFileName, sizeof(aviFileName),
           "/videos/%s/%s/%s.avi", dayDir, hourDir, ds);

  if (SD_MMC.exists(AVITEMP)) SD_MMC.remove(AVITEMP);
  aviFile = SD_MMC.open(AVITEMP, FILE_WRITE);
  if (!aviFile) {
    Serial.println("[AVI] Cannot open temp file!");
    return;
  }

  aviFile.write(aviHeader, AVI_HEADER_LEN);  // placeholder header
  aviStartMs  = millis();
  aviFrameCnt = 0;
  highPoint   = 0;
  userStopReq = false;
  prepAviIndex();
  aviRecording = true;
  Serial.printf("[AVI] Started: %s\n", aviFileName);
}

static void saveAviFrame(camera_fb_t* fb) {
  if (!aviFile || !fb || !fb->len) return;

  // Align to 4-byte boundary
  uint16_t filler = (4 - (fb->len & 3)) & 3;
  size_t   jpegSz = fb->len + filler;

  // Write 8-byte chunk header into sector-aligned buffer
  memcpy(iSDbuffer + highPoint,     dcBuf,  4);
  uint32_t jsz32 = (uint32_t)jpegSz;
  memcpy(iSDbuffer + highPoint + 4, &jsz32, 4);
  highPoint += CHUNK_HDR;
  if (highPoint >= RAMSIZE) {
    highPoint -= RAMSIZE;
    aviFile.write(iSDbuffer, RAMSIZE);
    memcpy(iSDbuffer, iSDbuffer + RAMSIZE, highPoint);
  }

  // Copy jpeg data through sector buffer
  size_t remain = jpegSz;
  while (remain >= RAMSIZE - highPoint) {
    memcpy(iSDbuffer + highPoint,
           fb->buf + (jpegSz - remain), RAMSIZE - highPoint);
    aviFile.write(iSDbuffer, RAMSIZE);
    remain    -= (RAMSIZE - highPoint);
    highPoint  = 0;
  }
  memcpy(iSDbuffer + highPoint, fb->buf + (jpegSz - remain), remain);
  highPoint += remain;

  buildAviIdx(jpegSz);
  aviFrameCnt++;
}

static void closeAvi() {
  if (!aviFile) return;
  aviRecording = false;
  if (highPoint) { aviFile.write(iSDbuffer, highPoint); highPoint = 0; }

  uint32_t elapsed   = millis() - aviStartMs;
  float    actualFPS = (elapsed > 0 && aviFrameCnt > 0)
                       ? (1000.0f * aviFrameCnt / elapsed) : (float)AVI_FPS;
  uint8_t  iFPS      = (uint8_t)round(actualFPS);
  if (iFPS < 1) iFPS = 1;

  finalizeAviIndex(aviFrameCnt);
  aviFile.write(idxBuf, idxLen);          // write idx1 block

  patchAviHeader(iFPS, aviFrameCnt, aviMoViSize);
  aviFile.seek(0);
  aviFile.write(aviHeader, AVI_HEADER_LEN); // overwrite with patched header
  aviFile.close();

  if (SD_MMC.exists(aviFileName)) SD_MMC.remove(aviFileName);
  SD_MMC.rename(AVITEMP, aviFileName);

  Serial.printf("[AVI] Saved: %s | %u frames | %.1f fps | %lu s\n",
                aviFileName, aviFrameCnt, actualFPS, elapsed / 1000);
}

// ════════════════════════════════════════════════════════════════
//  AVI RECORDER TASK  (core 1, priority 2)
//  Records at AVI_FPS, rotates every AVI_DURATION_MS or on demand
// ════════════════════════════════════════════════════════════════
void aviRecorderTask(void* pv) {
  Serial.printf("[AVI Task] Running on core %d\n", xPortGetCoreID());

  // Wait for SD to be confirmed ready
  while (!sdReady) delay(500);
  delay(1000);  // extra settle

  const uint32_t frameIntervalMs = 1000 / AVI_FPS;

  while (true) {
    openAvi();
    if (!aviRecording) { delay(10000); continue; }

    uint32_t segStart = millis();
    while (millis() - segStart < AVI_DURATION_MS && !userStopReq) {
      uint32_t fStart = millis();

      xSemaphoreTake(camMutex, portMAX_DELAY);
      camera_fb_t* fb = esp_camera_fb_get();
      if (fb) { saveAviFrame(fb); esp_camera_fb_return(fb); }
      xSemaphoreGive(camMutex);

      uint32_t took = millis() - fStart;
      if (took < frameIntervalMs) delay(frameIntervalMs - took);
    }

    closeAvi();

    // If user pressed Stop, pause until a start is requested again
    if (userStopReq) {
      Serial.println("[AVI] Recording stopped by user");
      while (userStopReq) delay(500);
      Serial.println("[AVI] Recording restarted by user");
    }
  }
}

// ════════════════════════════════════════════════════════════════
//  PIR MOTION SNAPSHOT  (core 0, priority 1)
// ════════════════════════════════════════════════════════════════
static void saveMotionSnapshot() {
  char dayDir[20], hourDir[20], ts[30], ds[25];
  if (!getTimeStr(dayDir, hourDir, ts, ds)) {
    snprintf(ts, sizeof(ts), "motion_%lu", millis() / 1000);
    snprintf(dayDir, sizeof(dayDir), "00_00_0000");
    snprintf(hourDir, sizeof(hourDir), "0000-0100");
  }
  char d1[80], d2[80], jpgPath[120];
  snprintf(d1, sizeof(d1), "/photos/%s", dayDir);             SD_MMC.mkdir(d1);
  snprintf(d2, sizeof(d2), "/photos/%s/%s", dayDir, hourDir); SD_MMC.mkdir(d2);
  snprintf(jpgPath, sizeof(jpgPath), "/photos/%s/%s/%s.jpg",  dayDir, hourDir, ts);

  xSemaphoreTake(camMutex, portMAX_DELAY);
  camera_fb_t* fb = esp_camera_fb_get();
  if (fb) {
    File f = SD_MMC.open(jpgPath, FILE_WRITE);
    if (f) { f.write(fb->buf, fb->len); f.close();
      Serial.printf("[PIR] Snapshot: %s\n", jpgPath); }
    esp_camera_fb_return(fb);
  }
  xSemaphoreGive(camMutex);
}

void pirTaskFn(void* pv) {
  while (true) {
    if (sdReady && digitalRead(PIR_PIN) == HIGH) {
      if (millis() - lastMotionSave > 5000) {
        lastMotionSave = millis();
        Serial.println("[PIR] Motion detected!");
        saveMotionSnapshot();
      }
    }
    delay(200);
  }
}

// ════════════════════════════════════════════════════════════════
//  HTTP HANDLERS
// ════════════════════════════════════════════════════════════════

// ── MJPEG stream — 30 fps ───────────────────────────────────────
static esp_err_t stream_handler(httpd_req_t* req) {
  char part_buf[64];
  const uint32_t frameMs = 1000 / STREAM_FPS;

  httpd_resp_set_type(req, STREAM_CONTENT_TYPE);
  httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
  httpd_resp_set_hdr(req, "Cache-Control", "no-cache, no-store");

  while (true) {
    uint32_t t0 = millis();

    xSemaphoreTake(camMutex, portMAX_DELAY);
    camera_fb_t* fb = esp_camera_fb_get();
    xSemaphoreGive(camMutex);
    if (!fb) break;

    size_t hlen = snprintf(part_buf, sizeof(part_buf), STREAM_PART, fb->len);
    esp_err_t res = httpd_resp_send_chunk(req, STREAM_BOUNDARY, strlen(STREAM_BOUNDARY));
    if (res == ESP_OK) res = httpd_resp_send_chunk(req, part_buf, hlen);
    if (res == ESP_OK) res = httpd_resp_send_chunk(req, (const char*)fb->buf, fb->len);
    esp_camera_fb_return(fb);
    if (res != ESP_OK) break;

    uint32_t elapsed = millis() - t0;
    if (elapsed < frameMs) delay(frameMs - elapsed);
  }
  return ESP_OK;
}

// ── Single JPEG download ────────────────────────────────────────
static esp_err_t snapshot_handler(httpd_req_t* req) {
  xSemaphoreTake(camMutex, portMAX_DELAY);
  camera_fb_t* fb = esp_camera_fb_get();
  xSemaphoreGive(camMutex);
  if (!fb) { httpd_resp_send_500(req); return ESP_FAIL; }
  httpd_resp_set_type(req, "image/jpeg");
  httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
  httpd_resp_set_hdr(req, "Content-Disposition", "attachment; filename=snapshot.jpg");
  esp_err_t res = httpd_resp_send(req, (const char*)fb->buf, fb->len);
  esp_camera_fb_return(fb);
  return res;
}

// ── Save snapshot to SD ─────────────────────────────────────────
static esp_err_t save_snapshot_handler(httpd_req_t* req) {
  char dayDir[20], hourDir[20], ts[30], ds[25];
  getTimeStr(dayDir, hourDir, ts, ds);
  char d1[80], d2[80], jpgPath[120];
  snprintf(d1, sizeof(d1), "/photos/%s", dayDir);             SD_MMC.mkdir(d1);
  snprintf(d2, sizeof(d2), "/photos/%s/%s", dayDir, hourDir); SD_MMC.mkdir(d2);
  snprintf(jpgPath, sizeof(jpgPath), "/photos/%s/%s/%s.jpg",  dayDir, hourDir, ts);

  xSemaphoreTake(camMutex, portMAX_DELAY);
  camera_fb_t* fb = esp_camera_fb_get();
  bool ok = false;
  if (fb) {
    File f = SD_MMC.open(jpgPath, FILE_WRITE);
    if (f) { f.write(fb->buf, fb->len); f.close(); ok = true; snapCount++; }
    esp_camera_fb_return(fb);
  }
  xSemaphoreGive(camMutex);

  httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
  httpd_resp_set_type(req, "application/json");
  char resp[200];
  if (ok) snprintf(resp, sizeof(resp), "{\"ok\":true,\"path\":\"%s\"}", jpgPath);
  else    snprintf(resp, sizeof(resp), "{\"ok\":false}");
  httpd_resp_send(req, resp, strlen(resp));
  return ESP_OK;
}

// ── Recording control: /record?cmd=start|stop|status ───────────
static esp_err_t record_handler(httpd_req_t* req) {
  char qbuf[40], cmd[16] = {};
  httpd_req_get_url_query_str(req, qbuf, sizeof(qbuf));
  httpd_query_key_value(qbuf, "cmd", cmd, sizeof(cmd));

  if (strcmp(cmd, "stop") == 0 && aviRecording) {
    userStopReq = true;
  } else if (strcmp(cmd, "start") == 0 && userStopReq) {
    userStopReq = false;  // AVI task will restart on next loop
  }

  httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
  httpd_resp_set_type(req, "application/json");
  char resp[80];
  snprintf(resp, sizeof(resp),
           "{\"recording\":%s,\"sd\":%s}",
           aviRecording ? "true" : "false",
           sdReady      ? "true" : "false");
  httpd_resp_send(req, resp, strlen(resp));
  return ESP_OK;
}

// ── Servo control: /servo?pan=90&tilt=45 ───────────────────────
static esp_err_t servo_handler(httpd_req_t* req) {
  char buf[60], param[10];
  httpd_req_get_url_query_str(req, buf, sizeof(buf));
  if (httpd_query_key_value(buf, "pan",  param, sizeof(param)) == ESP_OK)
    panAngle  = constrain(atoi(param), 0, 180);
  if (httpd_query_key_value(buf, "tilt", param, sizeof(param)) == ESP_OK)
    tiltAngle = constrain(atoi(param), 0, 180);
  panServo.write(panAngle);
  tiltServo.write(tiltAngle);
  httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
  httpd_resp_set_type(req, "application/json");
  char r[60];
  snprintf(r, sizeof(r), "{\"pan\":%d,\"tilt\":%d}", panAngle, tiltAngle);
  httpd_resp_send(req, r, strlen(r));
  return ESP_OK;
}

// ── LED: /led?state=0..255  (0=off, 1-255=PWM brightness) ──────
static esp_err_t led_handler(httpd_req_t* req) {
  char buf[30], param[10];
  httpd_req_get_url_query_str(req, buf, sizeof(buf));
  if (httpd_query_key_value(buf, "state", param, sizeof(param)) == ESP_OK)
    ledSet((uint8_t)constrain(atoi(param), 0, 255));
  httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
  char r[30]; snprintf(r, sizeof(r), "{\"brightness\":%d}", ledBrightness);
  httpd_resp_set_type(req, "application/json");
  httpd_resp_send(req, r, strlen(r));
  return ESP_OK;
}

// ── Ping ────────────────────────────────────────────────────────
static esp_err_t ping_handler(httpd_req_t* req) {
  httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
  httpd_resp_send(req, "pong", 4);
  return ESP_OK;
}

// ── Status JSON ─────────────────────────────────────────────────
static esp_err_t status_handler(httpd_req_t* req) {
  uint64_t tot  = sdReady ? SD_MMC.totalBytes() : 0;
  uint64_t used = sdReady ? SD_MMC.usedBytes()  : 0;
  char resp[512];
  snprintf(resp, sizeof(resp),
    "{\"name\":\"%s\","
    "\"ip\":\"%s\","
    "\"recording\":%s,"
    "\"pan\":%d,\"tilt\":%d,"
    "\"sd_total_mb\":%llu,\"sd_used_mb\":%llu,"
    "\"uptime_s\":%lu,\"rssi\":%ld,"
    "\"snaps\":%d,\"led\":%d,"
    "\"stream_fps\":%d,\"rec_fps\":%d,"
    "\"chip_temp\":%.1f}",
    DEVICE_NAME,
    WiFi.localIP().toString().c_str(),
    aviRecording ? "true" : "false",
    panAngle, tiltAngle,
    tot / 1048576ULL, used / 1048576ULL,
    millis() / 1000UL, (long)WiFi.RSSI(),
    snapCount, ledBrightness,
    STREAM_FPS, AVI_FPS,
    getChipTemp());
  httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
  httpd_resp_set_type(req, "application/json");
  httpd_resp_send(req, resp, strlen(resp));
  return ESP_OK;
}

// ── PIR state ───────────────────────────────────────────────────
static esp_err_t pir_handler(httpd_req_t* req) {
  httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
  httpd_resp_set_type(req, "application/json");
  char r[30];
  snprintf(r, sizeof(r), "{\"motion\":%s}",
           digitalRead(PIR_PIN) == HIGH ? "true" : "false");
  httpd_resp_send(req, r, strlen(r));
  return ESP_OK;
}

// ── Camera image settings ───────────────────────────────────────
static esp_err_t cam_settings_handler(httpd_req_t* req) {
  char buf[150], param[10];
  httpd_req_get_url_query_str(req, buf, sizeof(buf));
  sensor_t* s = esp_camera_sensor_get();
  if (httpd_query_key_value(buf,"quality",   param,sizeof(param))==ESP_OK) s->set_quality(s,   constrain(atoi(param),4,63));
  if (httpd_query_key_value(buf,"brightness",param,sizeof(param))==ESP_OK) s->set_brightness(s,constrain(atoi(param),-2,2));
  if (httpd_query_key_value(buf,"contrast",  param,sizeof(param))==ESP_OK) s->set_contrast(s,  constrain(atoi(param),-2,2));
  if (httpd_query_key_value(buf,"saturation",param,sizeof(param))==ESP_OK) s->set_saturation(s,constrain(atoi(param),-2,2));
  if (httpd_query_key_value(buf,"hmirror",   param,sizeof(param))==ESP_OK) s->set_hmirror(s,   atoi(param) ? 1 : 0);
  if (httpd_query_key_value(buf,"vflip",     param,sizeof(param))==ESP_OK) s->set_vflip(s,     atoi(param) ? 1 : 0);
  if (httpd_query_key_value(buf,"nightmode", param,sizeof(param))==ESP_OK) {
    if (atoi(param)) { s->set_gain_ctrl(s,0); s->set_agc_gain(s,30); s->set_brightness(s,2); s->set_saturation(s,-2); }
    else             { s->set_gain_ctrl(s,1); s->set_agc_gain(s,0);  s->set_brightness(s,1); s->set_saturation(s,-1); }
  }
  httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
  httpd_resp_send(req, "{\"ok\":true}", 11);
  return ESP_OK;
}

// ════════════════════════════════════════════════════════════════
//  SD BROWSER HANDLERS
// ════════════════════════════════════════════════════════════════

// Returns JSON array with name, path, isDir, size
static esp_err_t sd_list_handler(httpd_req_t* req) {
  char qbuf[220], rawPath[120] = "/", param[120];
  httpd_req_get_url_query_str(req, qbuf, sizeof(qbuf));
  if (httpd_query_key_value(qbuf, "path", param, sizeof(param)) == ESP_OK)
    strncpy(rawPath, param, sizeof(rawPath) - 1);

  char sdPath[120];
  sanitizePath(rawPath, sdPath, sizeof(sdPath));
  Serial.printf("[SD LIST] %s\n", sdPath);

  if (!sdReady) {
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
    httpd_resp_set_type(req, "application/json");
    httpd_resp_send(req, "[]", 2);
    return ESP_OK;
  }

  File dir = SD_MMC.open(sdPath);
  if (!dir || !dir.isDirectory()) {
    if (dir) dir.close();
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
    httpd_resp_set_type(req, "application/json");
    httpd_resp_send(req, "[]", 2);
    return ESP_OK;
  }

  String out = "[";
  bool first = true;
  File entry = dir.openNextFile();
  while (entry) {
    String name = String(entry.name());
    int sl = name.lastIndexOf('/');
    if (sl >= 0) name = name.substring(sl + 1);

    if (name.length() > 0 && name[0] != '.') {
      String fullPath = String(sdPath);
      if (!fullPath.endsWith("/")) fullPath += "/";
      fullPath += name;

      bool     isDir = entry.isDirectory();
      uint32_t size  = isDir ? 0 : (uint32_t)entry.size();

      if (!first) out += ",";
      first = false;
      out += "{\"name\":\"";   out += name;
      out += "\",\"path\":\""; out += fullPath;
      out += "\",\"isDir\":";  out += (isDir ? "true" : "false");
      out += ",\"size\":";     out += String(size);
      out += "}";
    }
    entry.close();
    entry = dir.openNextFile();
  }
  dir.close();
  out += "]";

  httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
  httpd_resp_set_type(req, "application/json");
  httpd_resp_send(req, out.c_str(), out.length());
  return ESP_OK;
}

static esp_err_t sd_download_handler(httpd_req_t* req) {
  char qbuf[220], rawPath[160];
  httpd_req_get_url_query_str(req, qbuf, sizeof(qbuf));
  if (httpd_query_key_value(qbuf, "path", rawPath, sizeof(rawPath)) != ESP_OK) {
    httpd_resp_send_404(req); return ESP_FAIL; }
  char sdPath[160]; sanitizePath(rawPath, sdPath, sizeof(sdPath));

  File f = SD_MMC.open(sdPath, FILE_READ);
  if (!f || f.isDirectory()) { if(f)f.close(); httpd_resp_send_404(req); return ESP_FAIL; }

  String p  = String(sdPath);
  const char* ct = "application/octet-stream";
  if (p.endsWith(".avi"))  ct = "video/x-msvideo";
  else if (p.endsWith(".jpg") || p.endsWith(".jpeg")) ct = "image/jpeg";

  httpd_resp_set_type(req, ct);
  httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
  String fname = p.substring(p.lastIndexOf('/') + 1);
  String cd = "attachment; filename=\"" + fname + "\"";
  httpd_resp_set_hdr(req, "Content-Disposition", cd.c_str());
  char clBuf[20]; snprintf(clBuf, sizeof(clBuf), "%u", (unsigned)f.size());
  httpd_resp_set_hdr(req, "Content-Length", clBuf);

  uint8_t buf[4096]; size_t n;
  while ((n = f.read(buf, sizeof(buf))) > 0)
    httpd_resp_send_chunk(req, (const char*)buf, n);
  httpd_resp_send_chunk(req, NULL, 0);
  f.close();
  return ESP_OK;
}

static esp_err_t sd_delete_handler(httpd_req_t* req) {
  char qbuf[220], rawPath[160];
  httpd_req_get_url_query_str(req, qbuf, sizeof(qbuf));
  if (httpd_query_key_value(qbuf, "path", rawPath, sizeof(rawPath)) != ESP_OK) {
    httpd_resp_send_404(req); return ESP_FAIL; }
  char sdPath[160]; sanitizePath(rawPath, sdPath, sizeof(sdPath));
  bool ok = SD_MMC.remove(sdPath);
  httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
  httpd_resp_set_type(req, "application/json");
  char resp[30]; snprintf(resp, sizeof(resp), "{\"ok\":%s}", ok?"true":"false");
  httpd_resp_send(req, resp, strlen(resp));
  return ESP_OK;
}

// ── Serve index.html from SPIFFS ────────────────────────────────
static esp_err_t index_handler(httpd_req_t* req) {
  File f = SPIFFS.open("/index.html", "r");
  if (!f) { httpd_resp_send_404(req); return ESP_FAIL; }
  httpd_resp_set_type(req, "text/html");
  httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
  uint8_t buf[2048]; size_t n;
  while ((n = f.read(buf, sizeof(buf))) > 0)
    httpd_resp_send_chunk(req, (const char*)buf, n);
  httpd_resp_send_chunk(req, NULL, 0);
  f.close();
  return ESP_OK;
}

// ════════════════════════════════════════════════════════════════
//  START HTTP SERVERS
// ════════════════════════════════════════════════════════════════
void startStreamServer() {
  httpd_config_t cfg  = HTTPD_DEFAULT_CONFIG();
  cfg.server_port     = STREAM_PORT;
  cfg.ctrl_port       = 32768;
  cfg.max_uri_handlers = 4;
  cfg.stack_size      = 8192;

  httpd_uri_t uris[] = {
    { "/stream",   HTTP_GET, stream_handler,   NULL },
    { "/snapshot", HTTP_GET, snapshot_handler, NULL },
  };
  if (httpd_start(&streamServer, &cfg) == ESP_OK) {
    for (auto& u : uris) httpd_register_uri_handler(streamServer, &u);
    Serial.printf("[HTTP] Stream   → http://%s:%d/stream\n",
                  WiFi.localIP().toString().c_str(), STREAM_PORT);
  }
}

void startCtrlServer() {
  httpd_config_t cfg   = HTTPD_DEFAULT_CONFIG();
  cfg.server_port      = CTRL_PORT;
  cfg.ctrl_port        = 32769;
  cfg.max_uri_handlers = 20;
  cfg.max_open_sockets = 7;
  cfg.stack_size       = 8192;
  cfg.recv_wait_timeout = 10;
  cfg.send_wait_timeout = 30;

  httpd_uri_t uris[] = {
    { "/",            HTTP_GET, index_handler,         NULL },
    { "/ping",         HTTP_GET, ping_handler,          NULL },
    { "/status",       HTTP_GET, status_handler,        NULL },
    { "/servo",        HTTP_GET, servo_handler,         NULL },
    { "/led",          HTTP_GET, led_handler,           NULL },
    { "/pir",          HTTP_GET, pir_handler,           NULL },
    { "/camset",       HTTP_GET, cam_settings_handler,  NULL },
    { "/savesnap",     HTTP_GET, save_snapshot_handler, NULL },
    { "/record",       HTTP_GET, record_handler,        NULL },
    { "/sd/list",      HTTP_GET, sd_list_handler,       NULL },
    { "/sd/download",  HTTP_GET, sd_download_handler,   NULL },
    { "/sd/delete",    HTTP_GET, sd_delete_handler,     NULL },
  };
  if (httpd_start(&ctrlServer, &cfg) == ESP_OK) {
    for (auto& u : uris) httpd_register_uri_handler(ctrlServer, &u);
    Serial.printf("[HTTP] Control  → http://%s:%d\n",
                  WiFi.localIP().toString().c_str(), CTRL_PORT);
  }
}

// ════════════════════════════════════════════════════════════════
//  SETUP
// ════════════════════════════════════════════════════════════════
void setup() {
  Serial.begin(115200);
  Serial.println();
  Serial.println("╔══════════════════════════════════════════════╗");
  Serial.println("║   ESPCam — Booting...  Zarf Robotics         ║");
  Serial.println("╚══════════════════════════════════════════════╝");

  // PIR input
  pinMode(PIR_PIN, INPUT);

  // LED — PWM brightness init (also ensures GPIO4 stays LOW during SD init)
  ledInit();
  ledSet(0);

  // SPIFFS for web UI
  if (!SPIFFS.begin(true)) Serial.println("[SPIFFS] Failed — web UI unavailable");
  else                      Serial.println("[SPIFFS] OK");

  // Camera
  if (!initCamera()) {
    Serial.println("[BOOT] Camera FAILED — halted");
    while (true) delay(1000);
  }
  delay(100);  // let LEDC stabilise before SD init

  // WiFi
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  Serial.print("[WiFi] Connecting");
  for (int t = 0; WiFi.status() != WL_CONNECTED && t < 40; t++) {
    delay(500); Serial.print(".");
  }
  Serial.println();
  if (WiFi.status() == WL_CONNECTED)
    Serial.printf("[WiFi] IP: %s\n", WiFi.localIP().toString().c_str());
  else
    Serial.println("[WiFi] Failed — running offline");

  // NTP (IST = UTC+5:30)
  configTime(5 * 3600 + 30 * 60, 0, "pool.ntp.org", "time.nist.gov");
  Serial.print("[NTP] Syncing");
  for (int i = 0; i < 10; i++) {
    if (getLocalTime(&timeinfo)) { timeSet = true; break; }
    delay(1000); Serial.print(".");
  }
  Serial.println(timeSet ? " OK" : " Failed");

  // SD — MUST come before servo attach (GPIO12 strapping pin conflict)
  sdReady = initSD();
  if (!sdReady) Serial.println("[BOOT] SD unavailable — recording disabled");

  // ── SERVO ATTACH — after SD init ───────────────────────────
  // Use 500-2400 µs pulse range (safe for SG90, MG90S, TowerPro)
  // Uses LEDC channels 4 & 5 (camera uses 0, LED uses 3)
  ESP32PWM::allocateTimer(0);
  ESP32PWM::allocateTimer(1);
  ESP32PWM::allocateTimer(2);
  ESP32PWM::allocateTimer(3);
  panServo.setPeriodHertz(50);
  tiltServo.setPeriodHertz(50);
  panServo.attach(PAN_PIN,  500, 2500);
  tiltServo.attach(TILT_PIN, 500, 2500);
  panServo.write(90);
  tiltServo.write(90);
  panServo.write(panAngle);
  tiltServo.write(tiltAngle);
  Serial.println("[Servo] Pan/Tilt ready (GPIO14/GPIO15, 500-2500µs)");

  // Mutexes
  camMutex = xSemaphoreCreateMutex();
  aviMutex = xSemaphoreCreateMutex();

  // HTTP servers
  startStreamServer();
  startCtrlServer();

  // FreeRTOS tasks
  xTaskCreatePinnedToCore(aviRecorderTask, "AVI", 8192, NULL, 2, &aviTaskHandle, 1);
  xTaskCreatePinnedToCore(pirTaskFn,       "PIR", 6144, NULL, 1, &pirTaskHandle, 0);

  Serial.println();
  Serial.println("╔══════════════════════════════════════════════╗");
  Serial.println("║           ESPCam — Ready!                    ║");
  Serial.println("║           Zarf Robotics                      ║");
  Serial.printf( "║  IP      : %-33s║\n", WiFi.localIP().toString().c_str());
  Serial.printf( "║  Web UI  : http://%-24s:82 ║\n", WiFi.localIP().toString().c_str());
  Serial.printf( "║  Stream  : http://%-24s:81/stream ║\n", WiFi.localIP().toString().c_str());
  Serial.printf( "║  SD      : %-33s║\n", sdReady ? "Ready" : "NOT FOUND");
  Serial.println("╚══════════════════════════════════════════════╝");
}

// ════════════════════════════════════════════════════════════════
//  LOOP — WiFi watchdog only (all real work in tasks / handlers)
// ════════════════════════════════════════════════════════════════
void loop() {
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("[WiFi] Reconnecting...");
    WiFi.reconnect();
    delay(5000);
  }
  delay(10000);
}
