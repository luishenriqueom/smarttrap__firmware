#pragma once

#ifndef FW_VERSION
#define FW_VERSION 1
#endif
#ifndef FW_MODE_INFER
#define FW_MODE_INFER 0
#endif

#define WIFI_SSID "Network - 2.4G"
#define WIFI_PASSWORD "qwert4321"
#define HELTEC_AP_SSID "smarttrap-gw"
#define HELTEC_AP_PASSWORD "smarttrap-gw"

#define API_BASE "http://192.168.0.4:8000"
#define API_KEY "CHANGE_ME"
#define DEVICE_CODE "trap-02"

#define SLEEP_SECONDS 60
#define WIFI_TIMEOUT_MS 15000

// GPIO0 preso em LOW no reset entra no bootloader ROM deste módulo.
// O botão de manutenção liga GPIO33 em GND.
#define MAINT_BUTTON_PIN 33

#define LORA_MOSI 2
#define LORA_MISO 13
#define LORA_SCK 4
#define LORA_NSS 12
#define LORA_RST 15
#define LORA_DIO0 -1
#define LORA_FREQUENCY 915E6
#define LORA_SF 7
#define LORA_BANDWIDTH 125E3
#define LORA_CODING_RATE 5
#define LORA_SYNC_WORD 0x12
#define LORA_TX_POWER 17
#define LORA_RX_WINDOW_MS 2000

// Slot SD nativo do ESP32-CAM, em 1 bit. CMD e D0 coincidem com RESET e MOSI do LoRa;
// o rádio permanece em repouso (NSS alto, sem SPI) enquanto o cartão é usado.
#define SD_CLK 14
#define SD_CMD 15
#define SD_D0 2
#define SD_FREQ_KHZ 20000
