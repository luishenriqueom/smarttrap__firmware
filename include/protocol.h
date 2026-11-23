#pragma once

#include <stddef.h>
#include <stdint.h>

static const uint8_t SMARTTRAP_MAGIC = 0x53;
static const uint8_t SMARTTRAP_KIND_METRIC = 0x01;
static const uint8_t SMARTTRAP_KIND_OTA = 0x10;
static const uint8_t SMARTTRAP_KIND_TIME = 0x11;
static const uint8_t SMARTTRAP_KIND_MODE = 0x12;
static const uint8_t SMARTTRAP_MODE_COLLECT = 0;
static const uint8_t SMARTTRAP_MODE_INFER = 1;
static const uint8_t SMARTTRAP_CONF_NONE = 255;

// 26 bytes: firmware sem relógio. 30 bytes: relógio, sem modo.
static const size_t SMARTTRAP_UPLINK_LEGACY_SIZE = 26;
static const size_t SMARTTRAP_UPLINK_CLOCK_SIZE = 30;

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
  uint8_t operating_mode;
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

struct __attribute__((packed)) SmartTrapModeDownlink {
  uint8_t magic;
  uint8_t kind;
  uint8_t operating_mode;
};

static_assert(sizeof(SmartTrapUplink) == 31, "uplink");
static_assert(sizeof(SmartTrapDownlink) == 2, "downlink");
static_assert(sizeof(SmartTrapTimeDownlink) == 6, "time downlink");
static_assert(sizeof(SmartTrapModeDownlink) == 3, "mode downlink");
