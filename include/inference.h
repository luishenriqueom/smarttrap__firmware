#pragma once

#include <stddef.h>
#include <stdint.h>

static const int kMaxDetections = 48;

struct Detection {
  char species[24];
  int x;
  int y;
  int width;
  int height;
  float confidence;
};

struct Inference {
  int pestCount;
  float confidence;
  bool ready;
  int boxCount;
  Detection boxes[kMaxDetections];
};

Inference classifyFrame(const uint8_t *jpeg, size_t length);
