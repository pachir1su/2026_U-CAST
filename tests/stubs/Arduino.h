/* ============================================================================
   테스트용 Arduino 스텁 헤더

   실물 보드 없이 PC에서 `main.ino`의 상태 머신을 검증하기 위한 최소 구현입니다.
   Arduino IDE 빌드에는 사용되지 않으며, `tests/run-tests.sh`에서만 사용합니다.
   ========================================================================== */

#ifndef U_CAST_TEST_ARDUINO_H
#define U_CAST_TEST_ARDUINO_H

#include <stdint.h>
#include <stdio.h>
#include <string.h>

#define HIGH 1
#define LOW 0
#define INPUT 0
#define OUTPUT 1
#define INPUT_PULLUP 2

// 아두이노 우노에서 A0, A1은 디지털 14, 15번 핀과 같습니다.
static const uint8_t A0 = 14;
static const uint8_t A1 = 15;

#define STUB_PIN_COUNT 24

class __FlashStringHelper;
#define F(string_literal) (reinterpret_cast<const __FlashStringHelper *>(string_literal))

namespace ucast_stub {

struct Board {
  uint8_t pinMode[STUB_PIN_COUNT];
  uint8_t digitalIn[STUB_PIN_COUNT];
  uint8_t digitalOut[STUB_PIN_COUNT];
  uint32_t clockMs;
  uint16_t toneFreq;   // 0이면 부저 정지
  bool toneOn;
  bool serialLogging;  // true면 로그를 실제로 화면에 출력
  uint32_t serialLines;  // println() 호출 횟수(화면 출력 여부와 무관하게 셈)
};

extern Board board;

inline void reset() {
  memset(&board, 0, sizeof(board));
  for (int i = 0; i < STUB_PIN_COUNT; i++) {
    board.digitalIn[i] = HIGH;  // 풀업 기본 상태(입력 없음)
  }
}

}  // namespace ucast_stub

inline void pinMode(uint8_t pin, uint8_t mode) {
  if (pin < STUB_PIN_COUNT) ucast_stub::board.pinMode[pin] = mode;
}

inline int digitalRead(uint8_t pin) {
  return (pin < STUB_PIN_COUNT) ? ucast_stub::board.digitalIn[pin] : HIGH;
}

inline void digitalWrite(uint8_t pin, uint8_t value) {
  if (pin < STUB_PIN_COUNT) ucast_stub::board.digitalOut[pin] = value;
}

inline uint32_t millis() { return ucast_stub::board.clockMs; }

inline void tone(uint8_t pin, uint16_t frequency) {
  (void)pin;
  ucast_stub::board.toneOn = true;
  ucast_stub::board.toneFreq = frequency;
}

inline void noTone(uint8_t pin) {
  (void)pin;
  ucast_stub::board.toneOn = false;
  ucast_stub::board.toneFreq = 0;
}

class SerialStub {
 public:
  void begin(long) {}
  void print(const __FlashStringHelper *text) { emit(text); }
  void println(const __FlashStringHelper *text) {
    emit(text);
    endLine();
  }
  void println(const char *text) {
    if (ucast_stub::board.serialLogging) printf("%s", text);
    endLine();
  }

  // 실제 Arduino의 Serial은 정수도 그대로 출력할 수 있습니다.
  // 스케치가 digitalRead() 결과를 찍는 로그를 쓰므로 같은 오버로드를 둡니다.
  void print(int value) {
    if (ucast_stub::board.serialLogging) printf("%d", value);
  }
  void println(int value) {
    if (ucast_stub::board.serialLogging) printf("%d", value);
    endLine();
  }

 private:
  // 로그가 몇 줄 나왔는지는 화면 출력 여부와 상관없이 세어 둡니다.
  // 로그 간격 제한 같은 검증이 출력 설정에 좌우되지 않게 하기 위해서입니다.
  void endLine() {
    ucast_stub::board.serialLines++;
    if (ucast_stub::board.serialLogging) printf("\n");
  }

  void emit(const __FlashStringHelper *text) {
    if (ucast_stub::board.serialLogging) {
      printf("%s", reinterpret_cast<const char *>(text));
    }
  }
};

extern SerialStub Serial;

#endif  // U_CAST_TEST_ARDUINO_H
