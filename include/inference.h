#pragma once

#include <stddef.h>
#include <stdint.h>

struct Inference {
  int pestCount;
  float confidence;
  bool ready;
};

Inference classifyFrame(const uint8_t *jpeg, size_t length);
