#include "inference.h"

#include <Arduino.h>
#include <SD_MMC.h>
#include <esp_heap_caps.h>
#include <img_converters.h>

#include <cstring>

#include "config.h"

#if FW_MODE_INFER

#include "tensorflow/lite/micro/micro_interpreter.h"
#include "tensorflow/lite/micro/micro_mutable_op_resolver.h"
#include "tensorflow/lite/schema/schema_generated.h"

static const int kWindow = 96;
static const int kMaxClasses = 8;
static const int kArenaBytes = 300 * 1024;

static uint8_t *modelBytes = nullptr;
static uint8_t *arena = nullptr;
static tflite::MicroInterpreter *interpreter = nullptr;
static tflite::MicroMutableOpResolver<32> resolver;
static bool resolverReady = false;
static int classCount = 0;
static char classNames[kMaxClasses][24];
static int scanWindow = 96;
static int scanStride = 64;
static float scoreThreshold = 0.6f;

static bool mountCard() {
  pinMode(LORA_NSS, OUTPUT);
  digitalWrite(LORA_NSS, HIGH);
  return SD_MMC.setPins(SD_CLK, SD_CMD, SD_D0) && SD_MMC.begin("/sdcard", true, false, SD_FREQ_KHZ);
}

static int jsonIntField(const String &body, const char *key, int fallback) {
  String needle = String("\"") + key + "\"";
  int at = body.indexOf(needle);
  if (at < 0) {
    return fallback;
  }
  at = body.indexOf(':', at + needle.length());
  if (at < 0) {
    return fallback;
  }
  return body.substring(at + 1).toInt();
}

static float jsonFloatField(const String &body, const char *key, float fallback) {
  String needle = String("\"") + key + "\"";
  int at = body.indexOf(needle);
  if (at < 0) {
    return fallback;
  }
  at = body.indexOf(':', at + needle.length());
  if (at < 0) {
    return fallback;
  }
  return body.substring(at + 1).toFloat();
}

static void readClasses(const String &body) {
  classCount = 0;
  int at = body.indexOf("\"classes\"");
  if (at < 0) {
    return;
  }
  at = body.indexOf('[', at);
  if (at < 0) {
    return;
  }
  while (classCount < kMaxClasses) {
    int open = body.indexOf('"', at + 1);
    if (open < 0) {
      break;
    }
    int close = body.indexOf('"', open + 1);
    if (close < 0 || close - open > 24) {
      break;
    }
    int colon = body.indexOf(':', at + 1);
    if (colon >= 0 && colon < open) {
      break;
    }
    memset(classNames[classCount], 0, sizeof(classNames[classCount]));
    body.substring(open + 1, close).toCharArray(classNames[classCount], sizeof(classNames[classCount]));
    classCount += 1;
    at = close;
    int comma = body.indexOf(',', close);
    int end = body.indexOf(']', close);
    if (end >= 0 && (comma < 0 || end < comma)) {
      break;
    }
  }
}

static bool loadModel() {
  if (interpreter != nullptr) {
    return true;
  }
  if (!mountCard()) {
    Serial.println("Modelo: cartão indisponível");
    return false;
  }
  if (!SD_MMC.exists("/model.tflite") || !SD_MMC.exists("/model.json")) {
    SD_MMC.end();
    Serial.println("Modelo: /model.tflite ausente");
    return false;
  }
  File meta = SD_MMC.open("/model.json", FILE_READ);
  String json = meta.readString();
  meta.close();
  scanWindow = jsonIntField(json, "window", 96);
  scanStride = jsonIntField(json, "stride", 64);
  scoreThreshold = jsonFloatField(json, "threshold", 0.6f);
  if (scanWindow != kWindow || scanStride < 1) {
    SD_MMC.end();
    Serial.println("Modelo: janela diferente de 96");
    return false;
  }
  readClasses(json);

  File file = SD_MMC.open("/model.tflite", FILE_READ);
  size_t length = file.size();
  if (length < 32 || length > 1500 * 1024) {
    file.close();
    SD_MMC.end();
    Serial.println("Modelo: tamanho inesperado");
    return false;
  }
  modelBytes = (uint8_t *)heap_caps_malloc(length, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
  if (modelBytes == nullptr) {
    modelBytes = (uint8_t *)malloc(length);
  }
  size_t got = file.read(modelBytes, length);
  file.close();
  SD_MMC.end();
  if (modelBytes == nullptr || got != length) {
    Serial.println("Modelo: leitura incompleta");
    return false;
  }

  const tflite::Model *model = tflite::GetModel(modelBytes);
  if (model->version() != TFLITE_SCHEMA_VERSION) {
    Serial.println("Modelo: schema incompatível");
    return false;
  }
  if (!resolverReady) {
    resolver.AddConv2D();
    resolver.AddDepthwiseConv2D();
    resolver.AddAdd();
    resolver.AddMul();
    resolver.AddPad();
    resolver.AddMean();
    resolver.AddReshape();
    resolver.AddLogistic();
    resolver.AddSoftmax();
    resolver.AddQuantize();
    resolver.AddDequantize();
    resolver.AddAveragePool2D();
    resolver.AddMaxPool2D();
    resolver.AddFullyConnected();
    resolver.AddConcatenation();
    resolver.AddRelu();
    resolver.AddRelu6();
    resolver.AddResizeNearestNeighbor();
    resolver.AddTranspose();
    resolver.AddStridedSlice();
    resolver.AddPack();
    resolver.AddShape();
    resolver.AddExpandDims();
    resolver.AddSqueeze();
    resolver.AddCast();
    resolver.AddSlice();
    resolver.AddGather();
    resolver.AddSub();
    resolver.AddLeakyRelu();
    resolverReady = true;
  }
  arena = (uint8_t *)heap_caps_aligned_alloc(16, kArenaBytes, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
  if (arena == nullptr) {
    arena = (uint8_t *)heap_caps_aligned_alloc(16, kArenaBytes, MALLOC_CAP_8BIT);
  }
  if (arena == nullptr) {
    Serial.println("Modelo: sem arena");
    return false;
  }
  static tflite::MicroInterpreter staticInterpreter(model, resolver, arena, kArenaBytes);
  interpreter = &staticInterpreter;
  if (interpreter->AllocateTensors() != kTfLiteOk) {
    Serial.println("Modelo: AllocateTensors falhou");
    interpreter = nullptr;
    return false;
  }
  TfLiteTensor *input = interpreter->input(0);
  if (input->dims->size != 4 || input->dims->data[1] != kWindow || input->dims->data[2] != kWindow) {
    Serial.println("Modelo: entrada diferente de 96x96");
    interpreter = nullptr;
    return false;
  }
  Serial.printf("Modelo carregado, %d classes\n", classCount);
  return true;
}

static int originCount(int length) {
  if (length <= kWindow) {
    return 1;
  }
  int count = 0;
  int last = -1;
  for (int at = 0; at <= length - kWindow; at += scanStride) {
    count += 1;
    last = at;
  }
  if (last != length - kWindow) {
    count += 1;
  }
  return count;
}

static int originAt(int length, int index) {
  if (length <= kWindow) {
    return 0;
  }
  int at = index * scanStride;
  int last = length - kWindow;
  return at > last ? last : at;
}

static void quantizeInput(TfLiteTensor *input, const uint8_t *rgb) {
  const int channels = input->dims->data[3];
  const float scale = input->params.scale == 0.0f ? 1.0f : input->params.scale;
  const int zero = input->params.zero_point;
  for (int y = 0; y < kWindow; y++) {
    for (int x = 0; x < kWindow; x++) {
      const uint8_t *pixel = rgb + (y * kWindow + x) * 3;
      for (int channel = 0; channel < channels; channel++) {
        uint8_t value = pixel[channel];
        if (channels == 1) {
          value = (uint8_t)((pixel[0] * 30 + pixel[1] * 59 + pixel[2] * 11) / 100);
        }
        int quantized = (int)(value / 255.0f / scale + 0.5f) + zero;
        if (quantized < -128) {
          quantized = -128;
        }
        if (quantized > 127) {
          quantized = 127;
        }
        int index = (y * kWindow + x) * channels + channel;
        if (input->type == kTfLiteInt8) {
          input->data.int8[index] = (int8_t)quantized;
        } else if (input->type == kTfLiteUInt8) {
          input->data.uint8[index] = (uint8_t)(quantized + 128);
        } else if (input->type == kTfLiteFloat32) {
          input->data.f[index] = value / 255.0f;
        }
      }
    }
  }
}

static float outputScore(const TfLiteTensor *output, int index) {
  if (output->type == kTfLiteFloat32) {
    return output->data.f[index];
  }
  if (output->type == kTfLiteInt8) {
    return (output->data.int8[index] - output->params.zero_point) * output->params.scale;
  }
  if (output->type == kTfLiteUInt8) {
    return (output->data.uint8[index] - output->params.zero_point) * output->params.scale;
  }
  return 0.0f;
}

static void remember(Inference *result, const Detection &found) {
  if (result->boxCount < kMaxDetections) {
    result->boxes[result->boxCount] = found;
    result->boxCount += 1;
    return;
  }
  int weakest = 0;
  for (int index = 1; index < result->boxCount; index++) {
    if (result->boxes[index].confidence < result->boxes[weakest].confidence) {
      weakest = index;
    }
  }
  if (found.confidence > result->boxes[weakest].confidence) {
    result->boxes[weakest] = found;
  }
}

static void considerWindow(Inference *result, const uint8_t *rgb, int ox, int oy) {
  TfLiteTensor *input = interpreter->input(0);
  TfLiteTensor *output = interpreter->output(0);
  quantizeInput(input, rgb);
  if (interpreter->Invoke() != kTfLiteOk) {
    return;
  }
  const bool nchw = output->dims->size == 4 && classCount > 0 && output->dims->data[1] == classCount && output->dims->data[3] != classCount;
  int gridH = 1;
  int gridW = 1;
  int classes = 1;
  if (output->dims->size == 4 && nchw) {
    classes = output->dims->data[1];
    gridH = output->dims->data[2];
    gridW = output->dims->data[3];
  } else if (output->dims->size == 4) {
    gridH = output->dims->data[1];
    gridW = output->dims->data[2];
    classes = output->dims->data[3];
  } else if (output->dims->size == 3) {
    gridH = output->dims->data[1];
    gridW = output->dims->data[2];
  }
  if (gridH < 1 || gridW < 1) {
    return;
  }
  const int cellW = kWindow / gridW > 0 ? kWindow / gridW : 1;
  const int cellH = kWindow / gridH > 0 ? kWindow / gridH : 1;
  for (int row = 0; row < gridH; row++) {
    for (int col = 0; col < gridW; col++) {
      int bestClass = -1;
      float best = scoreThreshold;
      for (int cls = 0; cls < classes && cls < kMaxClasses; cls++) {
        int index = nchw ? (cls * gridH + row) * gridW + col : (row * gridW + col) * classes + cls;
        float score = outputScore(output, index);
        if (score >= best) {
          best = score;
          bestClass = cls;
        }
      }
      if (bestClass < 0) {
        continue;
      }
      Detection found = {};
      if (bestClass < classCount) {
        strncpy(found.species, classNames[bestClass], sizeof(found.species) - 1);
      }
      found.x = ox + col * cellW;
      found.y = oy + row * cellH;
      found.width = cellW;
      found.height = cellH;
      found.confidence = best;
      remember(result, found);
    }
  }
}

static void mergeBoxes(Inference *result) {
  bool drop[kMaxDetections] = {};
  for (int i = 0; i < result->boxCount; i++) {
    if (drop[i]) {
      continue;
    }
    for (int j = i + 1; j < result->boxCount; j++) {
      if (drop[j] || strcmp(result->boxes[i].species, result->boxes[j].species) != 0) {
        continue;
      }
      int dx = (result->boxes[i].x + result->boxes[i].width / 2) - (result->boxes[j].x + result->boxes[j].width / 2);
      int dy = (result->boxes[i].y + result->boxes[i].height / 2) - (result->boxes[j].y + result->boxes[j].height / 2);
      if (dx < 0) {
        dx = -dx;
      }
      if (dy < 0) {
        dy = -dy;
      }
      if (dx > 8 || dy > 8) {
        continue;
      }
      int loser = result->boxes[i].confidence >= result->boxes[j].confidence ? j : i;
      drop[loser] = true;
      if (loser == i) {
        break;
      }
    }
  }
  int kept = 0;
  float lowest = 1.0f;
  for (int index = 0; index < result->boxCount; index++) {
    if (drop[index]) {
      continue;
    }
    result->boxes[kept] = result->boxes[index];
    if (result->boxes[kept].confidence < lowest) {
      lowest = result->boxes[kept].confidence;
    }
    kept += 1;
  }
  result->boxCount = kept;
  result->pestCount = kept;
  result->confidence = kept == 0 ? -1.0f : lowest;
}

struct JpegJob {
  const uint8_t *jpeg;
  size_t length;
  uint8_t *rgb;
  int width;
  int row0;
  int row1;
};

static size_t readJpeg(void *arg, size_t index, uint8_t *buf, size_t len) {
  JpegJob *job = (JpegJob *)arg;
  if (index >= job->length) {
    return 0;
  }
  if (index + len > job->length) {
    len = job->length - index;
  }
  memcpy(buf, job->jpeg + index, len);
  return len;
}

static bool writeJpeg(void *arg, uint16_t x, uint16_t y, uint16_t w, uint16_t h, uint8_t *data) {
  JpegJob *job = (JpegJob *)arg;
  for (uint16_t row = 0; row < h; row++) {
    int yy = y + row;
    if (yy < job->row0 || yy >= job->row1 || x >= job->width) {
      continue;
    }
    int copy = w;
    if (x + copy > job->width) {
      copy = job->width - x;
    }
    memcpy(job->rgb + ((yy - job->row0) * job->width + x) * 3, data + row * w * 3, copy * 3);
  }
  return true;
}

static bool jpegSize(const uint8_t *jpeg, size_t length, int *width, int *height) {
  size_t at = 2;
  while (at + 8 < length) {
    if (jpeg[at] != 0xFF) {
      at += 1;
      continue;
    }
    uint8_t marker = jpeg[at + 1];
    if (marker == 0xD8 || marker == 0xD9 || marker == 0x01 || (marker >= 0xD0 && marker <= 0xD7)) {
      at += 2;
      continue;
    }
    uint16_t segment = (jpeg[at + 2] << 8) | jpeg[at + 3];
    if (segment < 2) {
      return false;
    }
    if (marker == 0xC0 || marker == 0xC2) {
      *height = (jpeg[at + 5] << 8) | jpeg[at + 6];
      *width = (jpeg[at + 7] << 8) | jpeg[at + 8];
      return *width >= kWindow && *height >= kWindow;
    }
    at += 2 + segment;
  }
  return false;
}

static void copyWindow(const uint8_t *rgb, int imageWidth, int ox, int localY, uint8_t *window) {
  for (int row = 0; row < kWindow; row++) {
    memcpy(window + row * kWindow * 3, rgb + ((localY + row) * imageWidth + ox) * 3, kWindow * 3);
  }
}

static bool decodeInto(JpegJob *job) {
  return esp_jpg_decode(job->length, JPG_SCALE_NONE, readJpeg, writeJpeg, job) == ESP_OK;
}

static void scanPhoto(Inference *result, const uint8_t *jpeg, size_t length, int width, int height) {
  size_t fullBytes = (size_t)width * (size_t)height * 3;
  uint8_t *full = (uint8_t *)heap_caps_malloc(fullBytes, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
  if (full != nullptr) {
    JpegJob job{jpeg, length, full, width, 0, height};
    if (decodeInto(&job)) {
      const int rows = originCount(height);
      const int cols = originCount(width);
      static uint8_t window[kWindow * kWindow * 3];
      for (int yIndex = 0; yIndex < rows; yIndex++) {
        int oy = originAt(height, yIndex);
        for (int xIndex = 0; xIndex < cols; xIndex++) {
          int ox = originAt(width, xIndex);
          copyWindow(full, width, ox, oy, window);
          considerWindow(result, window, ox, oy);
          delay(1);
        }
      }
      heap_caps_free(full);
      return;
    }
    heap_caps_free(full);
  }

  static uint8_t bandWindow[kWindow * kWindow * 3];
  const int rows = originCount(height);
  const int cols = originCount(width);
  size_t bandBytes = (size_t)width * kWindow * 3;
  uint8_t *band = (uint8_t *)heap_caps_malloc(bandBytes, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
  if (band == nullptr) {
    band = (uint8_t *)malloc(bandBytes);
  }
  if (band == nullptr) {
    Serial.println("Inferência: sem faixa RGB");
    return;
  }
  for (int yIndex = 0; yIndex < rows; yIndex++) {
    int oy = originAt(height, yIndex);
    memset(band, 0, bandBytes);
    JpegJob job{jpeg, length, band, width, oy, oy + kWindow};
    if (!decodeInto(&job)) {
      continue;
    }
    for (int xIndex = 0; xIndex < cols; xIndex++) {
      int ox = originAt(width, xIndex);
      copyWindow(band, width, ox, 0, bandWindow);
      considerWindow(result, bandWindow, ox, oy);
      delay(1);
    }
  }
  free(band);
}

Inference classifyFrame(const uint8_t *jpeg, size_t length) {
  Inference result = {};
  result.confidence = -1.0f;
  if (!loadModel()) {
    return result;
  }
  int width = 0;
  int height = 0;
  if (!jpegSize(jpeg, length, &width, &height)) {
    Serial.println("Inferência: JPEG sem tamanho");
    return result;
  }
  scanPhoto(&result, jpeg, length, width, height);
  mergeBoxes(&result);
  result.ready = true;
  Serial.printf("Inferência: %d pragas\n", result.pestCount);
  return result;
}

#else

Inference classifyFrame(const uint8_t *jpeg, size_t length) {
  (void)jpeg;
  (void)length;
  Inference result = {};
  result.confidence = -1.0f;
  return result;
}

#endif
