#include "inference.h"

Inference classifyFrame(const uint8_t *jpeg, size_t length) {
  (void)jpeg;
  (void)length;
  return Inference{3, 0.8f, true};
}
