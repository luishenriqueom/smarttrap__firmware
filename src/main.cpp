#include <HTTPClient.h>
#include <LoRa.h>
#include <Preferences.h>
#include <SD_MMC.h>
#include <SPI.h>
#include <Update.h>
#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <esp_camera.h>
#include <soc/rtc_cntl_reg.h>
#include <soc/soc.h>
#include <sys/time.h>
#include <time.h>

#include <cstring>

#include "config.h"
#include "inference.h"
#include "protocol.h"

#define PWDN_GPIO_NUM 32
#define RESET_GPIO_NUM -1
#define XCLK_GPIO_NUM 0
#define SIOD_GPIO_NUM 26
#define SIOC_GPIO_NUM 27
#define Y9_GPIO_NUM 35
#define Y8_GPIO_NUM 34
#define Y7_GPIO_NUM 39
#define Y6_GPIO_NUM 36
#define Y5_GPIO_NUM 21
#define Y4_GPIO_NUM 19
#define Y3_GPIO_NUM 18
#define Y2_GPIO_NUM 5
#define VSYNC_GPIO_NUM 25
#define HREF_GPIO_NUM 23
#define PCLK_GPIO_NUM 22

static const uint8_t MODE_COLLECT = 0;
static const uint8_t MODE_INFER = 1;
static const uint8_t MODE_UNSET = 255;

static_assert(MODE_COLLECT == SMARTTRAP_MODE_COLLECT, "modo coleta");
static_assert(MODE_INFER == SMARTTRAP_MODE_INFER, "modo inferência");

static uint8_t storedMode = MODE_COLLECT;
static uint8_t activeMode = MODE_COLLECT;

static_assert(sizeof(DEVICE_CODE) <= sizeof(SmartTrapUplink::device_code), "DEVICE_CODE cabe em 15 caracteres");
static bool loraReady = false;
static bool cameraReady = false;

struct FrameOption {
  const char *token;
  framesize_t size;
};

static const FrameOption FRAME_OPTIONS[] = {
    {"qqvga", FRAMESIZE_QQVGA},
    {"qvga", FRAMESIZE_QVGA},
    {"vga", FRAMESIZE_VGA},
    {"svga", FRAMESIZE_SVGA},
    {"xga", FRAMESIZE_XGA},
    {"sxga", FRAMESIZE_SXGA},
    {"uxga", FRAMESIZE_UXGA},
};

static framesize_t activeSize = FRAMESIZE_SVGA;
static char activeToken[8] = "svga";
static uint8_t activeQuality = 12;
static uint32_t activeSleep = SLEEP_FALLBACK_SECONDS;

static void loadStoredSleep() {
  Preferences prefs;
  if (!prefs.begin("smarttrap", true)) {
    return;
  }
  uint32_t stored = prefs.getUInt("slp", SLEEP_FALLBACK_SECONDS);
  prefs.end();
  if (stored >= 60 && stored <= 86400) {
    activeSleep = stored;
  }
}

static void storeSleep(uint32_t seconds) {
  if (seconds < 60 || seconds > 86400) {
    return;
  }
  activeSleep = seconds;
  Preferences prefs;
  if (!prefs.begin("smarttrap", false)) {
    return;
  }
  prefs.putUInt("slp", seconds);
  prefs.end();
}

static void enterSleep(uint32_t seconds) {
  if (loraReady) {
    LoRa.sleep();
  }
  if (cameraReady) {
    esp_camera_deinit();
  }
  WiFi.disconnect(true);
  WiFi.mode(WIFI_OFF);
  Serial.flush();
  esp_sleep_enable_timer_wakeup((uint64_t)seconds * 1000000ULL);
  esp_deep_sleep_start();
}

static void sleepUntilNext() {
  enterSleep(activeSleep);
}

static const char *modeToken(uint8_t mode) {
  return mode == MODE_INFER ? "infer" : "collect";
}

static uint8_t modeFromToken(const char *token) {
  if (strcmp(token, "infer") == 0) {
    return MODE_INFER;
  }
  if (strcmp(token, "collect") == 0) {
    return MODE_COLLECT;
  }
  return MODE_UNSET;
}

static void storeMode(uint8_t mode) {
  if (mode != MODE_COLLECT && mode != MODE_INFER) {
    return;
  }
  if (mode == storedMode) {
    return;
  }
  Preferences prefs;
  if (!prefs.begin("smarttrap", false)) {
    return;
  }
  prefs.putUChar("mode", mode);
  prefs.end();
  storedMode = mode;
  Serial.printf("Modo %s gravado para o próximo ciclo\n", modeToken(mode));
}

static uint8_t currentMode() {
  Preferences prefs;
  if (!prefs.begin("smarttrap", false)) {
    storedMode = FW_MODE_INFER ? MODE_INFER : MODE_COLLECT;
    return storedMode;
  }
  if (prefs.getUShort("ver", 0) != FW_VERSION) {
    prefs.putUShort("ver", FW_VERSION);
  }
  uint8_t mode = prefs.getUChar("mode", MODE_UNSET);
  if (mode != MODE_COLLECT && mode != MODE_INFER) {
    mode = FW_MODE_INFER ? MODE_INFER : MODE_COLLECT;
    prefs.putUChar("mode", mode);
  }
  prefs.end();
  storedMode = mode;
  return mode;
}

static const time_t CLOCK_VALID_AFTER = 1700000000;

static uint32_t unixNow() {
  time_t now = time(nullptr);
  if (now <= CLOCK_VALID_AFTER) {
    return 0;
  }
  return (uint32_t)now;
}

static void applyUnixTime(uint32_t unixTime) {
  if (unixTime <= (uint32_t)CLOCK_VALID_AFTER) {
    return;
  }
  struct timeval value = {};
  value.tv_sec = (time_t)unixTime;
  settimeofday(&value, nullptr);
  Serial.printf("Relógio ajustado: %u\n", unixTime);
}

static bool localCivilTime(struct tm *local) {
  time_t now = time(nullptr);
  if (now <= CLOCK_VALID_AFTER) {
    return false;
  }
  time_t shifted = now + LOCAL_UTC_OFFSET_SECONDS;
  return gmtime_r(&shifted, local) != nullptr;
}

static bool inNightWindow(const struct tm *local) {
  int minute = local->tm_hour * 60 + local->tm_min;
  return minute >= NIGHT_START_MINUTE || minute < NIGHT_END_MINUTE;
}

static uint32_t secondsUntilDay(const struct tm *local) {
  int minute = local->tm_hour * 60 + local->tm_min;
  int until = minute < NIGHT_END_MINUTE ? NIGHT_END_MINUTE - minute : (24 * 60 - minute) + NIGHT_END_MINUTE;
  uint32_t seconds = (uint32_t)until * 60u;
  if (seconds > (uint32_t)local->tm_sec) {
    seconds -= (uint32_t)local->tm_sec;
  }
  if (seconds < 60) {
    seconds = 60;
  }
  return seconds;
}

static void pauseForNight(bool maintenance) {
  struct tm local;
  if (!localCivilTime(&local) || !inNightWindow(&local)) {
    return;
  }
  if (maintenance) {
    Serial.println("Noite, manutenção segue o ciclo");
    return;
  }
  uint32_t seconds = secondsUntilDay(&local);
  Serial.printf("Noite %02d:%02d, sem foto; dorme %lu s até %02d:%02d\n", local.tm_hour, local.tm_min, (unsigned long)seconds, NIGHT_END_MINUTE / 60, NIGHT_END_MINUTE % 60);
  enterSleep(seconds);
}

static bool maintenanceHeld() {
  pinMode(MAINT_BUTTON_PIN, INPUT_PULLUP);
  delay(30);
  if (digitalRead(MAINT_BUTTON_PIN) != LOW) {
    return false;
  }
  delay(30);
  return digitalRead(MAINT_BUTTON_PIN) == LOW;
}

static bool initCamera() {
  camera_config_t config = {};
  config.ledc_channel = LEDC_CHANNEL_0;
  config.ledc_timer = LEDC_TIMER_0;
  config.pin_d0 = Y2_GPIO_NUM;
  config.pin_d1 = Y3_GPIO_NUM;
  config.pin_d2 = Y4_GPIO_NUM;
  config.pin_d3 = Y5_GPIO_NUM;
  config.pin_d4 = Y6_GPIO_NUM;
  config.pin_d5 = Y7_GPIO_NUM;
  config.pin_d6 = Y8_GPIO_NUM;
  config.pin_d7 = Y9_GPIO_NUM;
  config.pin_xclk = XCLK_GPIO_NUM;
  config.pin_pclk = PCLK_GPIO_NUM;
  config.pin_vsync = VSYNC_GPIO_NUM;
  config.pin_href = HREF_GPIO_NUM;
  config.pin_sccb_sda = SIOD_GPIO_NUM;
  config.pin_sccb_scl = SIOC_GPIO_NUM;
  config.pin_pwdn = PWDN_GPIO_NUM;
  config.pin_reset = RESET_GPIO_NUM;
  config.xclk_freq_hz = 20000000;
  config.frame_size = activeSize;
  config.pixel_format = PIXFORMAT_JPEG;
  config.grab_mode = CAMERA_GRAB_LATEST;
  config.fb_location = CAMERA_FB_IN_PSRAM;
  config.jpeg_quality = activeQuality;
  config.fb_count = 2;

  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
    Serial.printf("Falha ao iniciar câmera: 0x%x\n", err);
    return false;
  }
  cameraReady = true;
  return true;
}

static const FrameOption *findFrame(const char *token) {
  for (const FrameOption &option : FRAME_OPTIONS) {
    if (strcmp(option.token, token) == 0) {
      return &option;
    }
  }
  return nullptr;
}

static void rememberToken(const char *token) {
  strncpy(activeToken, token, sizeof(activeToken) - 1);
  activeToken[sizeof(activeToken) - 1] = '\0';
}

static bool validQuality(int quality) {
  return quality >= 4 && quality <= 63;
}

static void loadStoredFrameSize() {
  Preferences prefs;
  const char *stored = "svga";
  String saved;
  if (prefs.begin("smarttrap", true)) {
    saved = prefs.getString("fs", "svga");
    int quality = prefs.getUChar("jq", 12);
    prefs.end();
    stored = saved.c_str();
    if (validQuality(quality)) {
      activeQuality = (uint8_t)quality;
    }
  }
  const FrameOption *option = findFrame(stored);
  if (option == nullptr) {
    option = findFrame("svga");
  }
  activeSize = option->size;
  rememberToken(option->token);
}

static void storeFrameToken(const char *token) {
  Preferences prefs;
  if (!prefs.begin("smarttrap", false)) {
    return;
  }
  prefs.putString("fs", token);
  prefs.end();
}

static void storeQuality(uint8_t quality) {
  Preferences prefs;
  if (!prefs.begin("smarttrap", false)) {
    return;
  }
  prefs.putUChar("jq", quality);
  prefs.end();
}

static void discardFrames(int count) {
  for (int index = 0; index < count; index++) {
    camera_fb_t *frame = esp_camera_fb_get();
    if (frame == nullptr) {
      break;
    }
    esp_camera_fb_return(frame);
  }
}

static bool applyQuality(uint8_t quality) {
  if (!validQuality(quality) || quality == activeQuality) {
    return quality == activeQuality;
  }
  sensor_t *sensor = esp_camera_sensor_get();
  if (sensor == nullptr || sensor->set_quality == nullptr || sensor->set_quality(sensor, quality) != 0) {
    return false;
  }
  activeQuality = quality;
  storeQuality(quality);
  discardFrames(1);
  return true;
}

static bool applyFrameToken(const char *token) {
  const FrameOption *option = findFrame(token);
  if (option == nullptr) {
    return false;
  }
  if (cameraReady && strcmp(activeToken, option->token) == 0) {
    return true;
  }
  framesize_t previousSize = activeSize;
  char previousToken[8];
  strncpy(previousToken, activeToken, sizeof(previousToken));
  previousToken[sizeof(previousToken) - 1] = '\0';

  if (cameraReady) {
    esp_camera_deinit();
    cameraReady = false;
  }
  activeSize = option->size;
  rememberToken(option->token);
  if (initCamera()) {
    discardFrames(2);
    storeFrameToken(activeToken);
    return true;
  }

  esp_camera_deinit();
  activeSize = previousSize;
  rememberToken(previousToken);
  initCamera();
  return false;
}

static bool waitForWifi(uint32_t timeoutMs) {
  uint32_t start = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - start < timeoutMs) {
    delay(250);
  }
  return WiFi.status() == WL_CONNECTED;
}

static bool connectWifi() {
  WiFi.persistent(false);
  WiFi.mode(WIFI_STA);
  WiFi.setSleep(false);
  const char *ssids[] = {HELTEC_AP_SSID, WIFI_SSID};
  const char *passwords[] = {HELTEC_AP_PASSWORD, WIFI_PASSWORD};
  for (size_t i = 0; i < 2; i++) {
    if (ssids[i][0] == '\0') {
      continue;
    }
    WiFi.begin(ssids[i], passwords[i]);
    Serial.printf("Wi-Fi \"%s\"\n", ssids[i]);
    if (waitForWifi(WIFI_TIMEOUT_MS)) {
      Serial.println(WiFi.localIP());
      configTime(0, 0, "pool.ntp.org");
      for (int attempt = 0; attempt < 20 && time(nullptr) < 1700000000; attempt++) {
        delay(250);
      }
      return true;
    }
    WiFi.disconnect(false);
  }
  Serial.println("Wi-Fi indisponível");
  return false;
}

static String apiBase() {
  if (WiFi.SSID() == HELTEC_AP_SSID) {
    return "http://192.168.4.1";
  }
  return API_BASE;
}

static bool beginHttp(HTTPClient &http, WiFiClient &plain, WiFiClientSecure &secure, const String &url) {
  http.setTimeout(65000);
  http.setFollowRedirects(HTTPC_STRICT_FOLLOW_REDIRECTS);
  bool https = url.startsWith("https://");
  if (https) {
    secure.setInsecure();
    if (!http.begin(secure, url)) {
      return false;
    }
  } else if (!http.begin(plain, url)) {
    return false;
  }
  http.addHeader("X-API-Key", API_KEY);
  return true;
}

static bool jsonString(const String &body, const char *key, char *out, size_t outLen) {
  String needle = String("\"") + key + "\"";
  int at = body.indexOf(needle);
  if (at < 0) {
    return false;
  }
  at = body.indexOf(':', at + needle.length());
  if (at < 0) {
    return false;
  }
  at++;
  while (at < (int)body.length() && (body[at] == ' ' || body[at] == '\t')) {
    at++;
  }
  if (at >= (int)body.length() || body[at] != '"') {
    return false;
  }
  at++;
  int end = body.indexOf('"', at);
  if (end <= at || (size_t)(end - at) >= outLen) {
    return false;
  }
  memcpy(out, body.c_str() + at, end - at);
  out[end - at] = '\0';
  return true;
}

static int jsonInt(const String &body, const char *key) {
  String needle = String("\"") + key + "\"";
  int at = body.indexOf(needle);
  if (at < 0) {
    return -1;
  }
  at = body.indexOf(':', at + needle.length());
  if (at < 0) {
    return -1;
  }
  return body.substring(at + 1).toInt();
}

static bool fetchCameraConfig(char *token, size_t tokenLen, int *quality, int *sleepSeconds) {
  String url = apiBase() + "/api/v1/ingest/config?device_code=" + DEVICE_CODE + "&device_time=" + String(unixNow()) + "&operating_mode=" + modeToken(activeMode);
  WiFiClient plain;
  WiFiClientSecure secure;
  HTTPClient http;
  if (!beginHttp(http, plain, secure, url)) {
    return false;
  }
  http.setTimeout(12000);
  int code = http.GET();
  if (code != 200) {
    Serial.printf("Configuração: HTTP %d\n", code);
    http.end();
    return false;
  }
  String body = http.getString();
  http.end();
  int setTime = jsonInt(body, "set_time");
  if (setTime > CLOCK_VALID_AFTER) {
    applyUnixTime((uint32_t)setTime);
  }
  char modeName[16] = "";
  if (jsonString(body, "operating_mode", modeName, sizeof(modeName))) {
    storeMode(modeFromToken(modeName));
  }
  *quality = jsonInt(body, "jpeg_quality");
  *sleepSeconds = jsonInt(body, "sleep_seconds");
  return jsonString(body, "frame_size", token, tokenLen);
}

static void syncCameraConfig(bool applyNow) {
  char token[8] = "";
  int quality = -1;
  int sleepSeconds = -1;
  if (!fetchCameraConfig(token, sizeof(token), &quality, &sleepSeconds)) {
    Serial.printf("Configuração mantida: %s q%d sono %lu s\n", activeToken, activeQuality, (unsigned long)activeSleep);
    return;
  }
  if (sleepSeconds >= 60 && sleepSeconds <= 86400) {
    storeSleep((uint32_t)sleepSeconds);
  }
  bool sizeOk = token[0] != '\0' && findFrame(token) != nullptr;
  bool qualityOk = validQuality(quality);
  bool sizeChanged = sizeOk && strcmp(token, activeToken) != 0;
  bool qualityChanged = qualityOk && (uint8_t)quality != activeQuality;
  if (token[0] != '\0' && !sizeOk) {
    Serial.printf("Resolução ignorada: %s\n", token);
  }
  if (!sizeChanged && !qualityChanged) {
    Serial.printf("Configuração %s q%d\n", activeToken, activeQuality);
    return;
  }
  if (!applyNow) {
    if (sizeChanged) {
      storeFrameToken(token);
    }
    if (qualityChanged) {
      storeQuality((uint8_t)quality);
    }
    Serial.println("Configuração gravada para o próximo ciclo");
    return;
  }
  if (sizeChanged) {
    uint8_t previousQuality = activeQuality;
    if (qualityChanged) {
      activeQuality = (uint8_t)quality;
    }
    if (!applyFrameToken(token)) {
      activeQuality = previousQuality;
      if (cameraReady) {
        esp_camera_deinit();
        cameraReady = false;
      }
      initCamera();
      Serial.printf("Falha ao aplicar %s, permanece %s q%d\n", token, activeToken, activeQuality);
      return;
    }
    if (qualityChanged) {
      storeQuality(activeQuality);
    }
    Serial.printf("Configuração alterada para %s q%d\n", activeToken, activeQuality);
    return;
  }
  if (applyQuality((uint8_t)quality)) {
    Serial.printf("Qualidade JPEG alterada para %u\n", activeQuality);
  } else {
    Serial.printf("Falha ao aplicar qualidade %d, permanece %u\n", quality, activeQuality);
  }
}

static bool applyOta() {
  String url = apiBase() + "/api/v1/ingest/firmware/smarttrap/bin?current=" + String(FW_VERSION);
  WiFiClient plain;
  WiFiClientSecure secure;
  HTTPClient http;
  if (!beginHttp(http, plain, secure, url)) {
    Serial.println("OTA: falha ao abrir a URL");
    return false;
  }

  const char *headerKeys[] = {"x-MD5"};
  http.collectHeaders(headerKeys, 1);
  int code = http.GET();
  if (code == 304 || code == 204) {
    Serial.printf("OTA: firmware %d já é o publicado\n", FW_VERSION);
    http.end();
    return false;
  }
  if (code != 200) {
    Serial.printf("OTA: HTTP %d\n", code);
    http.end();
    return false;
  }

  int length = http.getSize();
  String md5 = http.header("x-MD5");
  if (!Update.begin(length > 0 ? length : UPDATE_SIZE_UNKNOWN)) {
    Serial.printf("OTA: Update.begin falhou (%s)\n", Update.errorString());
    http.end();
    return false;
  }
  if (md5.length() == 32) {
    Update.setMD5(md5.c_str());
  }

  WiFiClient *stream = http.getStreamPtr();
  uint32_t readyAt = millis();
  while (stream->available() == 0 && millis() - readyAt < 10000) {
    delay(10);
  }
  int first = stream->peek();
  if (first != 0xE9) {
    Serial.printf("OTA: primeiro byte 0x%02X\n", first < 0 ? 0 : first);
    Update.abort();
    http.end();
    return false;
  }
  size_t written = Update.writeStream(*stream);
  bool finished = written > 0 && Update.end();
  http.end();
  if (!finished || !Update.isFinished()) {
    Serial.printf("OTA: gravação falhou (%s)\n", Update.errorString());
    Update.abort();
    return false;
  }

  Serial.println("OTA aplicado, reiniciando");
  Serial.flush();
  ESP.restart();
  return true;
}

static String isoNow() {
  time_t now = time(nullptr);
  struct tm clock;
  gmtime_r(&now, &clock);
  char iso[32];
  strftime(iso, sizeof(iso), "%Y-%m-%dT%H:%M:%SZ", &clock);
  return String(iso);
}

static bool uploadJpeg(const uint8_t *data, size_t length, const char *detections) {
  const String boundary = "SmartTrapBoundary7f3a9c";
  String detectionPart = "";
  if (detections != nullptr && detections[0] != '\0') {
    detectionPart = "--" + boundary + "\r\n"
                    "Content-Disposition: form-data; name=\"detections\"\r\n\r\n" +
                    String(detections) + "\r\n";
  }
  String head = "--" + boundary + "\r\n"
                "Content-Disposition: form-data; name=\"device_code\"\r\n\r\n" +
                String(DEVICE_CODE) + "\r\n--" + boundary + "\r\n"
                "Content-Disposition: form-data; name=\"captured_at\"\r\n\r\n" +
                isoNow() + "\r\n--" + boundary + "\r\n"
                "Content-Disposition: form-data; name=\"device_time\"\r\n\r\n" +
                String(unixNow()) + "\r\n--" + boundary + "\r\n"
                "Content-Disposition: form-data; name=\"operating_mode\"\r\n\r\n" +
                modeToken(activeMode) + "\r\n--" + boundary + "\r\n"
                "Content-Disposition: form-data; name=\"frame_size\"\r\n\r\n" +
                String(activeToken) + "\r\n--" + boundary + "\r\n"
                "Content-Disposition: form-data; name=\"jpeg_quality\"\r\n\r\n" +
                String(activeQuality) + "\r\n" + detectionPart + "--" + boundary + "\r\n"
                "Content-Disposition: form-data; name=\"image\"; filename=\"capture.jpg\"\r\n"
                "Content-Type: image/jpeg\r\n\r\n";
  String tail = "\r\n--" + boundary + "--\r\n";
  size_t total = head.length() + length + tail.length();
  uint8_t *body = (uint8_t *)ps_malloc(total);
  if (body == nullptr) {
    body = (uint8_t *)malloc(total);
  }
  if (body == nullptr) {
    Serial.println("Sem memória para o JPEG");
    return false;
  }
  memcpy(body, head.c_str(), head.length());
  memcpy(body + head.length(), data, length);
  memcpy(body + head.length() + length, tail.c_str(), tail.length());

  String url = apiBase() + "/api/v1/ingest/images";
  WiFiClient plain;
  WiFiClientSecure secure;
  HTTPClient http;
  bool sent = false;
  if (beginHttp(http, plain, secure, url)) {
    http.addHeader("Content-Type", "multipart/form-data; boundary=" + boundary);
    int code = http.POST(body, total);
    Serial.printf("POST imagem -> HTTP %d\n", code);
    sent = code == 201;
    http.end();
  }
  free(body);
  return sent;
}

static void releaseSdBus() {
  pinMode(LORA_NSS, OUTPUT);
  digitalWrite(LORA_NSS, HIGH);
}

static String nextCapturePath() {
  Preferences prefs;
  prefs.begin("smarttrap", false);
  uint32_t seq = prefs.getUInt("sdseq", 0) + 1;
  prefs.putUInt("sdseq", seq);
  prefs.end();

  char path[48];
  time_t now = time(nullptr);
  if (now > 1700000000) {
    struct tm clock;
    gmtime_r(&now, &clock);
    snprintf(path, sizeof(path), "/captures/%04d%02d%02d_%02d%02d%02d_%lu.jpg", clock.tm_year + 1900, clock.tm_mon + 1, clock.tm_mday, clock.tm_hour, clock.tm_min, clock.tm_sec, (unsigned long)seq);
  } else {
    snprintf(path, sizeof(path), "/captures/capture_%06lu.jpg", (unsigned long)seq);
  }
  return String(path);
}

static bool saveJpeg(const uint8_t *data, size_t length, String *savedPath, const char *sidecar) {
  releaseSdBus();
  if (!SD_MMC.setPins(SD_CLK, SD_CMD, SD_D0) || !SD_MMC.begin("/sdcard", true, false, SD_FREQ_KHZ)) {
    Serial.println("SD: cartão indisponível");
    return false;
  }
  if (!SD_MMC.exists("/captures") && !SD_MMC.mkdir("/captures")) {
    Serial.println("SD: pasta /captures não criada");
    SD_MMC.end();
    return false;
  }

  String path = nextCapturePath();
  File file = SD_MMC.open(path.c_str(), FILE_WRITE);
  if (!file) {
    Serial.printf("SD: não abriu %s\n", path.c_str());
    SD_MMC.end();
    return false;
  }

  static uint8_t chunk[1024];
  size_t written = 0;
  while (written < length) {
    size_t count = length - written;
    if (count > sizeof(chunk)) {
      count = sizeof(chunk);
    }
    memcpy(chunk, data + written, count);
    if (file.write(chunk, count) != count) {
      break;
    }
    written += count;
  }
  file.close();
  if (written != length) {
    SD_MMC.remove(path.c_str());
    SD_MMC.end();
    Serial.printf("SD: gravação incompleta (%u/%u)\n", (unsigned)written, (unsigned)length);
    return false;
  }
  if (sidecar != nullptr) {
    String sidePath = path;
    sidePath.replace(".jpg", ".json");
    File side = SD_MMC.open(sidePath.c_str(), FILE_WRITE);
    if (!side || side.print(sidecar) != (size_t)strlen(sidecar)) {
      Serial.println("SD: sidecar não gravado");
    }
    if (side) {
      side.close();
    }
  }
  SD_MMC.end();
  if (savedPath != nullptr) {
    *savedPath = path;
  }
  Serial.printf("SD: %s (%u bytes)\n", path.c_str(), (unsigned)written);
  return true;
}

static bool captureAndUpload(bool upload) {
  camera_fb_t *frame = esp_camera_fb_get();
  if (frame == nullptr || frame->format != PIXFORMAT_JPEG) {
    Serial.println("Captura falhou");
    if (frame != nullptr) {
      esp_camera_fb_return(frame);
    }
    return false;
  }
  saveJpeg(frame->buf, frame->len, nullptr, nullptr);
  bool sent = !upload || uploadJpeg(frame->buf, frame->len, nullptr);
  esp_camera_fb_return(frame);
  return sent;
}

static bool startLora() {
  pinMode(LORA_NSS, OUTPUT);
  digitalWrite(LORA_NSS, HIGH);
  SPI.begin(LORA_SCK, LORA_MISO, LORA_MOSI, LORA_NSS);
  LoRa.setSPI(SPI);
  LoRa.setPins(LORA_NSS, LORA_RST, LORA_DIO0);
  if (!LoRa.begin(LORA_FREQUENCY)) {
    Serial.println("LoRa não iniciou");
    return false;
  }
  LoRa.setSpreadingFactor(LORA_SF);
  LoRa.setSignalBandwidth(LORA_BANDWIDTH);
  LoRa.setCodingRate4(LORA_CODING_RATE);
  LoRa.setSyncWord(LORA_SYNC_WORD);
  LoRa.setTxPower(LORA_TX_POWER);
  loraReady = true;
  return true;
}

static bool otaCommandReceived() {
  uint32_t start = millis();
  while (millis() - start < LORA_RX_WINDOW_MS) {
    int length = LoRa.parsePacket();
    if (length <= 0) {
      delay(10);
      continue;
    }
    uint8_t buffer[8];
    int cap = length < (int)sizeof(buffer) ? length : (int)sizeof(buffer);
    int got = LoRa.readBytes(buffer, cap);
    while (LoRa.available()) {
      LoRa.read();
    }
    if (got == (int)sizeof(SmartTrapModeDownlink)) {
      SmartTrapModeDownlink command;
      memcpy(&command, buffer, sizeof(command));
      if (command.magic == SMARTTRAP_MAGIC && command.kind == SMARTTRAP_KIND_MODE) {
        storeMode(command.operating_mode);
      }
    } else if (got == (int)sizeof(SmartTrapTimeDownlink)) {
      SmartTrapTimeDownlink command;
      memcpy(&command, buffer, sizeof(command));
      if (command.magic == SMARTTRAP_MAGIC && command.kind == SMARTTRAP_KIND_TIME) {
        applyUnixTime(command.unix_time);
      }
    } else if (got == (int)sizeof(SmartTrapDownlink)) {
      SmartTrapDownlink command;
      memcpy(&command, buffer, sizeof(command));
      if (command.magic == SMARTTRAP_MAGIC && command.kind == SMARTTRAP_KIND_OTA) {
        return true;
      }
    }
  }
  return false;
}

static String detectionsJson(const Inference &result) {
  String json = "{\"boxes\":[";
  for (int index = 0; index < result.boxCount; index++) {
    if (index > 0) {
      json += ',';
    }
    const Detection &box = result.boxes[index];
    json += "{\"species\":\"";
    json += box.species;
    json += "\",\"x\":";
    json += box.x;
    json += ",\"y\":";
    json += box.y;
    json += ",\"width\":";
    json += box.width;
    json += ",\"height\":";
    json += box.height;
    json += ",\"confidence\":";
    json += String(box.confidence, 3);
    json += '}';
  }
  json += "]}";
  return json;
}

static String swapExtension(const String &path, const char *extension) {
  int dot = path.lastIndexOf('.');
  if (dot < 0) {
    return path + extension;
  }
  return path.substring(0, dot) + extension;
}

static bool mountSd() {
  releaseSdBus();
  return SD_MMC.setPins(SD_CLK, SD_CMD, SD_D0) && SD_MMC.begin("/sdcard", true, false, SD_FREQ_KHZ);
}

static void markUploaded(const String &jpgPath) {
  if (jpgPath.length() == 0 || !mountSd()) {
    return;
  }
  File mark = SD_MMC.open(swapExtension(jpgPath, ".up").c_str(), FILE_WRITE);
  if (mark) {
    mark.close();
  }
  SD_MMC.end();
}

static bool syncModel() {
  String url = apiBase() + "/api/v1/ingest/models/current?device_code=" + DEVICE_CODE;
  WiFiClient plain;
  WiFiClientSecure secure;
  HTTPClient http;
  if (!beginHttp(http, plain, secure, url)) {
    return false;
  }
  int code = http.GET();
  if (code == 404) {
    http.end();
    Serial.println("Modelo: nenhum atribuído");
    return false;
  }
  if (code != 200) {
    Serial.printf("Modelo: HTTP %d\n", code);
    http.end();
    return false;
  }
  String body = http.getString();
  http.end();
  char sha[80] = "";
  if (!jsonString(body, "sha256", sha, sizeof(sha))) {
    return false;
  }
  Preferences prefs;
  prefs.begin("smarttrap", true);
  String current = prefs.getString("sha", "");
  prefs.end();
  if (current == sha && mountSd() && SD_MMC.exists("/model.tflite")) {
    SD_MMC.end();
    Serial.println("Modelo já está no cartão");
    return true;
  }
  if (SD_MMC.cardType() != CARD_NONE) {
    SD_MMC.end();
  }

  String fileUrl = apiBase() + "/api/v1/ingest/models/current/file?device_code=" + DEVICE_CODE;
  if (!beginHttp(http, plain, secure, fileUrl)) {
    return false;
  }
  code = http.GET();
  if (code != 200) {
    Serial.printf("Modelo arquivo: HTTP %d\n", code);
    http.end();
    return false;
  }
  if (!mountSd()) {
    http.end();
    return false;
  }
  SD_MMC.remove("/model.tflite.tmp");
  File model = SD_MMC.open("/model.tflite.tmp", FILE_WRITE);
  int written = model ? http.writeToStream(&model) : -1;
  if (model) {
    model.close();
  }
  http.end();
  if (written <= 0) {
    SD_MMC.remove("/model.tflite.tmp");
    SD_MMC.end();
    Serial.println("Modelo: download falhou");
    return false;
  }
  SD_MMC.remove("/model.tflite");
  if (!SD_MMC.rename("/model.tflite.tmp", "/model.tflite")) {
    SD_MMC.end();
    Serial.println("Modelo: não renomeou o arquivo");
    return false;
  }
  File meta = SD_MMC.open("/model.json", FILE_WRITE);
  bool metaOk = meta && meta.print(body) == body.length();
  if (meta) {
    meta.close();
  }
  SD_MMC.end();
  if (!metaOk) {
    Serial.println("Modelo: metadados não gravados");
    return false;
  }
  prefs.begin("smarttrap", false);
  prefs.putString("sha", sha);
  prefs.end();
  Serial.printf("Modelo gravado (%d bytes)\n", written);
  return true;
}

static void uploadPending(int limit) {
  if (!mountSd()) {
    return;
  }
  File folder = SD_MMC.open("/captures");
  if (!folder) {
    SD_MMC.end();
    return;
  }
  String pending[3];
  int found = 0;
  File entry = folder.openNextFile();
  while (entry && found < limit) {
    bool directory = entry.isDirectory();
    String name = entry.name();
    entry.close();
    if (directory) {
      entry = folder.openNextFile();
      continue;
    }
    if (!name.startsWith("/")) {
      name = "/captures/" + name;
    }
    if (name.endsWith(".jpg") && !SD_MMC.exists(swapExtension(name, ".up").c_str())) {
      String side = swapExtension(name, ".json");
      if (SD_MMC.exists(side.c_str())) {
        pending[found++] = name;
      }
    }
    entry = folder.openNextFile();
  }
  folder.close();
  SD_MMC.end();

  for (int index = 0; index < found; index++) {
    if (!mountSd()) {
      return;
    }
    File image = SD_MMC.open(pending[index].c_str(), FILE_READ);
    File side = SD_MMC.open(swapExtension(pending[index], ".json").c_str(), FILE_READ);
    size_t length = image ? image.size() : 0;
    String detections = side ? side.readString() : String("{\"boxes\":[]}");
    uint8_t *data = nullptr;
    if (image && length > 0 && length <= 1500000) {
      data = (uint8_t *)ps_malloc(length);
      if (data == nullptr) {
        data = (uint8_t *)malloc(length);
      }
      if (data != nullptr && image.read(data, length) != length) {
        free(data);
        data = nullptr;
      }
    }
    if (image) {
      image.close();
    }
    if (side) {
      side.close();
    }
    SD_MMC.end();
    if (data != nullptr && uploadJpeg(data, length, detections.c_str())) {
      markUploaded(pending[index]);
    }
    free(data);
  }
}

static void inferAndMaybeUpdate() {
  if (connectWifi()) {
    syncModel();
    syncCameraConfig(false);
    WiFi.disconnect(true);
    WiFi.mode(WIFI_OFF);
  }

  camera_fb_t *frame = esp_camera_fb_get();
  Inference result = {};
  result.confidence = -1.0f;
  uint8_t *jpeg = nullptr;
  size_t length = 0;
  String savedPath;
  String detections = "{\"boxes\":[]}";
  if (frame != nullptr && frame->format == PIXFORMAT_JPEG && frame->len > 0) {
    jpeg = (uint8_t *)ps_malloc(frame->len);
    if (jpeg == nullptr) {
      jpeg = (uint8_t *)malloc(frame->len);
    }
    if (jpeg != nullptr) {
      memcpy(jpeg, frame->buf, frame->len);
      length = frame->len;
    }
    esp_camera_fb_return(frame);
  } else if (frame != nullptr) {
    esp_camera_fb_return(frame);
  }

  if (jpeg != nullptr) {
    result = classifyFrame(jpeg, length);
    detections = detectionsJson(result);
    saveJpeg(jpeg, length, &savedPath, detections.c_str());
  }

  bool heardOta = false;
  if (startLora()) {
    SmartTrapUplink packet = {};
    packet.magic = SMARTTRAP_MAGIC;
    packet.kind = SMARTTRAP_KIND_METRIC;
    packet.fw_version = FW_VERSION;
    packet.pest_count = result.ready && result.pestCount > 0 ? (uint16_t)result.pestCount : 0;
    if (result.ready && result.confidence >= 0.0f && result.confidence <= 1.0f) {
      packet.confidence = (uint8_t)(result.confidence * 100.0f + 0.5f);
    } else {
      packet.confidence = SMARTTRAP_CONF_NONE;
    }
    packet.model_ok = result.ready ? 1 : 0;
    packet.battery_mv = 0;
    packet.unix_time = unixNow();
    packet.operating_mode = activeMode;
    strncpy(packet.device_code, DEVICE_CODE, sizeof(packet.device_code) - 1);
    LoRa.beginPacket();
    LoRa.write((const uint8_t *)&packet, sizeof(packet));
    LoRa.endPacket();
    LoRa.receive();
    Serial.printf("LoRa enviado, fw %u, relógio %u, escuta %u ms\n", FW_VERSION, packet.unix_time, LORA_RX_WINDOW_MS);
    heardOta = otaCommandReceived();
  }

  if (connectWifi()) {
    if (jpeg != nullptr && uploadJpeg(jpeg, length, detections.c_str())) {
      markUploaded(savedPath);
    }
    uploadPending(3);
    if (heardOta) {
      Serial.println("Comando OTA recebido");
      applyOta();
    }
  }
  free(jpeg);
}

void setup() {
  WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0);

  Serial.begin(115200);
  delay(200);
  Serial.printf("\nSmartTrap fw %d\n", FW_VERSION);

  bool maintenance = maintenanceHeld();
  uint8_t mode = currentMode();
#if !FW_MODE_INFER
  if (mode == MODE_INFER) {
    Serial.println("Inferência pedida, firmware sem modelo; segue em coleta");
    mode = MODE_COLLECT;
  }
#endif
  activeMode = mode;
  Serial.printf("Modo %s%s\n", modeToken(mode), maintenance ? " + manutenção" : "");

  loadStoredFrameSize();
  loadStoredSleep();
  pauseForNight(maintenance);
  if (!initCamera()) {
    Serial.printf("Câmera recusou %s, volta para svga\n", activeToken);
    esp_camera_deinit();
    activeSize = FRAMESIZE_SVGA;
    rememberToken("svga");
    storeFrameToken(activeToken);
    if (!initCamera()) {
      sleepUntilNext();
    }
  }

  if (maintenance || mode == MODE_COLLECT) {
    bool online = connectWifi();
    if (online) {
      syncCameraConfig(mode == MODE_COLLECT);
    }
    if (mode == MODE_COLLECT) {
      captureAndUpload(online);
    }
    if (online) {
      applyOta();
    }
  }

  if (mode == MODE_INFER && !maintenance) {
    inferAndMaybeUpdate();
  }

  sleepUntilNext();
}

void loop() {}
