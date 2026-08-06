/* ============================================================================
   테스트용 Adafruit NeoPixel 스텁 헤더

   실제 라이브러리 대신 픽셀 색만 기록해 두어, 경고 상태에서 스트립이
   전체 빨간색이 되는지와 종료 시 모든 픽셀이 꺼지는지 검증합니다.
   Arduino IDE 빌드에는 사용되지 않으며, `tests/run-tests.sh`에서만 사용합니다.
   ========================================================================== */

#ifndef U_CAST_TEST_NEOPIXEL_H
#define U_CAST_TEST_NEOPIXEL_H

#include "Arduino.h"

#define NEO_GRB 0x52
#define NEO_KHZ800 0x0000

#define STUB_MAX_PIXELS 64

class Adafruit_NeoPixel {
 public:
  Adafruit_NeoPixel(uint16_t pixels, uint8_t pin, uint16_t type)
      : numPixels_(pixels), pin_(pin), type_(type), brightness_(255), showCount_(0) {
    for (uint16_t i = 0; i < STUB_MAX_PIXELS; i++) buffer_[i] = 0;
    for (uint16_t i = 0; i < STUB_MAX_PIXELS; i++) shown_[i] = 0;
  }

  void begin() {}
  void setBrightness(uint8_t value) { brightness_ = value; }

  uint32_t Color(uint8_t r, uint8_t g, uint8_t b) {
    return ((uint32_t)r << 16) | ((uint32_t)g << 8) | (uint32_t)b;
  }

  void setPixelColor(uint16_t index, uint32_t color) {
    if (index < STUB_MAX_PIXELS) buffer_[index] = color;
  }

  void clear() {
    for (uint16_t i = 0; i < STUB_MAX_PIXELS; i++) buffer_[i] = 0;
  }

  void show() {
    for (uint16_t i = 0; i < STUB_MAX_PIXELS; i++) shown_[i] = buffer_[i];
    showCount_++;
  }

  // --- 테스트 전용 조회 함수 ---
  uint16_t testPixelCount() const { return numPixels_; }
  uint8_t testPin() const { return pin_; }
  uint32_t testShownPixel(uint16_t index) const {
    return (index < STUB_MAX_PIXELS) ? shown_[index] : 0;
  }
  bool testAllShown(uint32_t color) const {
    for (uint16_t i = 0; i < numPixels_; i++) {
      if (shown_[i] != color) return false;
    }
    return true;
  }
  uint32_t testShowCount() const { return showCount_; }

 private:
  uint16_t numPixels_;
  uint8_t pin_;
  uint16_t type_;
  uint8_t brightness_;
  uint32_t showCount_;
  uint32_t buffer_[STUB_MAX_PIXELS];
  uint32_t shown_[STUB_MAX_PIXELS];
};

#endif  // U_CAST_TEST_NEOPIXEL_H
