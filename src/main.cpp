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

static_assert(sizeof(DEVICE_CODE) <= sizeof(SmartTrapUplink::device_code), "DEVICE_CODE cabe em 15 caracteres");
static bool loraReady = false;
static bool cameraReady = false;

static void sleepUntilNext() {
  if (loraReady) {
    LoRa.sleep();
  }
  if (cameraReady) {
    esp_camera_deinit();
  }
  WiFi.disconnect(true);
  WiFi.mode(WIFI_OFF);
  Serial.flush();
  esp_sleep_enable_timer_wakeup((uint64_t)SLEEP_SECONDS * 1000000ULL);
  esp_deep_sleep_start();
}

static uint8_t currentMode() {
  Preferences prefs;
  prefs.begin("smarttrap", false);
  uint16_t applied = prefs.getUShort("ver", 0);
  uint8_t mode = prefs.getUChar("mode", MODE_COLLECT);
  if (applied != FW_VERSION) {
    mode = FW_MODE_INFER ? MODE_INFER : MODE_COLLECT;
    prefs.putUShort("ver", FW_VERSION);
    prefs.putUChar("mode", mode);
  }
  prefs.end();
  return mode;
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
  config.frame_size = FRAMESIZE_SVGA;
  config.pixel_format = PIXFORMAT_JPEG;
  config.grab_mode = CAMERA_GRAB_LATEST;
  config.fb_location = CAMERA_FB_IN_PSRAM;
  config.jpeg_quality = 12;
  config.fb_count = 2;

  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
    Serial.printf("Falha ao iniciar câmera: 0x%x\n", err);
    return false;
  }
  cameraReady = true;
  return true;
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

static bool uploadJpeg(const uint8_t *data, size_t length) {
  const String boundary = "SmartTrapBoundary7f3a9c";
  String head = "--" + boundary + "\r\n"
                "Content-Disposition: form-data; name=\"device_code\"\r\n\r\n" +
                String(DEVICE_CODE) + "\r\n--" + boundary + "\r\n"
                "Content-Disposition: form-data; name=\"captured_at\"\r\n\r\n" +
                isoNow() + "\r\n--" + boundary + "\r\n"
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

static bool saveJpeg(const uint8_t *data, size_t length) {
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
  SD_MMC.end();
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
  saveJpeg(frame->buf, frame->len);
  bool sent = !upload || uploadJpeg(frame->buf, frame->len);
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
    if (got == (int)sizeof(SmartTrapDownlink)) {
      SmartTrapDownlink command;
      memcpy(&command, buffer, sizeof(command));
      if (command.magic == SMARTTRAP_MAGIC && command.kind == SMARTTRAP_KIND_OTA) {
        return true;
      }
    }
  }
  return false;
}

static void inferAndMaybeUpdate() {
  camera_fb_t *frame = esp_camera_fb_get();
  Inference result{0, -1.0f, false};
  if (frame != nullptr && frame->format == PIXFORMAT_JPEG) {
    saveJpeg(frame->buf, frame->len);
    result = classifyFrame(frame->buf, frame->len);
    esp_camera_fb_return(frame);
  } else if (frame != nullptr) {
    esp_camera_fb_return(frame);
  }

  if (!startLora()) {
    return;
  }

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
  strncpy(packet.device_code, DEVICE_CODE, sizeof(packet.device_code) - 1);

  LoRa.beginPacket();
  LoRa.write((const uint8_t *)&packet, sizeof(packet));
  LoRa.endPacket();
  LoRa.receive();
  Serial.printf("LoRa enviado, fw %u, escuta %u ms\n", FW_VERSION, LORA_RX_WINDOW_MS);

  if (!otaCommandReceived()) {
    return;
  }
  Serial.println("Comando OTA recebido");
  if (connectWifi()) {
    applyOta();
  }
}

void setup() {
  WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0);

  Serial.begin(115200);
  delay(200);
  Serial.printf("\nSmartTrap fw %d\n", FW_VERSION);

  bool maintenance = maintenanceHeld();
  uint8_t mode = currentMode();
  Serial.printf("Modo %s%s\n", mode == MODE_INFER ? "infer" : "coleta", maintenance ? " + manutenção" : "");

  if (!initCamera()) {
    sleepUntilNext();
  }

  if (maintenance || mode == MODE_COLLECT) {
    bool online = connectWifi();
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
