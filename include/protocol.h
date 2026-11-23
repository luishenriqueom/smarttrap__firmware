#pragma once

#include <stddef.h>
#include <stdint.h>

static const uint8_t SMARTTRAP_MAGIC = 0x53;
static const uint8_t SMARTTRAP_KIND_METRIC = 0x01;
static const uint8_t SMARTTRAP_KIND_OTA = 0x10;
static const uint8_t SMARTTRAP_KIND_TIME = 0x11;
static const uint8_t SMARTTRAP_CONF_NONE = 255;

// Pacotes antigos têm 26 bytes e não trazem unix_time.
static const size_t SMARTTRAP_UPLINK_LEGACY_SIZE = 26;

struct __attribute__((packed)) SmartTrapUplink {
  uint8_t magic;
  uint8_t kind;
  uint16_t fw_version;
  uint16_t pest_count;
  uint8_t confidence;
  uint8_t model_ok;
  uint16_t battery_mv;
  char device_code[16];
  uint32_t unix_time;
};

struct __attribute__((packed)) SmartTrapDownlink {
  uint8_t magic;
  uint8_t kind;
};

struct __attribute__((packed)) SmartTrapTimeDownlink {
  uint8_t magic;
  uint8_t kind;
  uint32_t unix_time;
};

static_assert(sizeof(SmartTrapUplink) == 30, "uplink");
static_assert(sizeof(SmartTrapDownlink) == 2, "downlink");
static_assert(sizeof(SmartTrapTimeDownlink) == 6, "time downlink");
