/* ============================================================================
   워크북 기준 진입·통과 IR 경고 알고리즘 회귀 테스트

   실물 보드 없이 PC에서 `main.ino`의 상태 머신을 검증합니다.
   실행 방법: tests/run-tests.sh

   여기서 검증하는 것은 상태 전이와 출력 논리이며,
   실제 IR 모듈의 출력 논리나 스트립 픽셀 수 같은 실물 값은 검증하지 않습니다.
   ========================================================================== */

#include "stubs/Arduino.h"

namespace ucast_stub {
Board board;
}
SerialStub Serial;

#include "../main.ino"

#include <stdio.h>

static int testsRun = 0;
static int testsFailed = 0;
static const char *currentTest = "";

static void check(bool condition, const char *label) {
  if (!condition) {
    testsFailed++;
    printf("  [FAIL] %s — %s\n", currentTest, label);
  }
}

static void beginTest(const char *name) {
  testsRun++;
  currentTest = name;
}

/* ---------------------------------------------------------------------------
   테스트 보조 함수
   --------------------------------------------------------------------------- */

// 스케치의 전역 상태를 초기값으로 되돌리고 setup()을 다시 실행합니다.
static void resetSystem() {
  ucast_stub::reset();

  systemState = STATE_IDLE;
  warningTimeoutAt = 0;
  warningBlinkOn = false;
  warningBlinkChangedAt = 0;

  irEntry1 = {PIN_IR_ENTRY_1, false, false, 0};
  irEntry2 = {PIN_IR_ENTRY_2, false, false, 0};
  irExit1  = {PIN_IR_EXIT_1,  false, false, 0};
  irExit2  = {PIN_IR_EXIT_2,  false, false, 0};
  btnStart = {PIN_BTN_START, false, false, 0};
  btnStop  = {PIN_BTN_STOP,  false, false, 0};

  setup();
}

static void setIr(uint8_t pin, bool detected) {
  bool low = IR_ACTIVE_LOW ? detected : !detected;
  ucast_stub::board.digitalIn[pin] = low ? LOW : HIGH;
}

static void setButton(uint8_t pin, bool pressed) {
  bool low = BUTTON_ACTIVE_LOW ? pressed : !pressed;
  ucast_stub::board.digitalIn[pin] = low ? LOW : HIGH;
}

// 지정한 시간만큼 시간을 흘리며 loop()를 반복 실행합니다.
static void advance(uint32_t ms, uint32_t stepMs = 10) {
  uint32_t elapsed = 0;
  while (elapsed < ms) {
    ucast_stub::board.clockMs += stepMs;
    elapsed += stepMs;
    loop();
  }
}

static bool warnLedOn() {
  return ucast_stub::board.digitalOut[PIN_WARN_LED] == HIGH;
}

static uint32_t redColor() { return strip1.Color(WARN_COLOR_R, WARN_COLOR_G, WARN_COLOR_B); }

static bool bothStripsRed() {
  return strip1.testAllShown(redColor()) && strip2.testAllShown(redColor());
}

static bool bothStripsOff() {
  return strip1.testAllShown(0) && strip2.testAllShown(0);
}

// 센서를 감지 → 확정 대기 → 해제 순서로 한 번 통과시킵니다.
static void pulseIr(uint8_t pin, uint32_t holdMs) {
  setIr(pin, true);
  advance(holdMs);
  setIr(pin, false);
  advance(IR_CONFIRM_MS + 40);
}

static void pressButton(uint8_t pin) {
  setButton(pin, true);
  advance(BUTTON_DEBOUNCE_MS + 30);
  setButton(pin, false);
  advance(BUTTON_DEBOUNCE_MS + 30);
}

// 대기 상태에서 경고 상태로 진입시킵니다(진입 IR 1 사용).
static void startWarning() {
  pulseIr(PIN_IR_ENTRY_1, IR_CONFIRM_MS + 40);
}

/* ---------------------------------------------------------------------------
   1~3. 기본 시작
   --------------------------------------------------------------------------- */

static void testStartByEntry1() {
  beginTest("1. 진입 IR 1 감지 → 경고 시작");
  resetSystem();
  pulseIr(PIN_IR_ENTRY_1, IR_CONFIRM_MS + 40);
  check(systemState == STATE_WARNING, "경고 상태로 전환되어야 함");
}

static void testStartByEntry2() {
  beginTest("2. 진입 IR 2 감지 → 경고 시작");
  resetSystem();
  pulseIr(PIN_IR_ENTRY_2, IR_CONFIRM_MS + 40);
  check(systemState == STATE_WARNING, "경고 상태로 전환되어야 함");
}

static void testStartByRedButton() {
  beginTest("3. 빨간 버튼 → 경고 시작");
  resetSystem();
  pressButton(PIN_BTN_START);
  check(systemState == STATE_WARNING, "경고 상태로 전환되어야 함");
}

/* ---------------------------------------------------------------------------
   4~6. 기본 종료
   --------------------------------------------------------------------------- */

static void testStopByExit1() {
  beginTest("4. 경고 중 통과 IR 1 감지 → 경고 종료");
  resetSystem();
  startWarning();
  pulseIr(PIN_IR_EXIT_1, IR_CONFIRM_MS + 40);
  check(systemState == STATE_IDLE, "대기 상태로 복귀해야 함");
  check(bothStripsOff(), "두 스트립이 모두 꺼져야 함");
}

static void testStopByExit2() {
  beginTest("5. 경고 중 통과 IR 2 감지 → 경고 종료");
  resetSystem();
  startWarning();
  pulseIr(PIN_IR_EXIT_2, IR_CONFIRM_MS + 40);
  check(systemState == STATE_IDLE, "대기 상태로 복귀해야 함");
}

static void testStopByGreenButton() {
  beginTest("6. 경고 중 초록 버튼 → 경고 종료");
  resetSystem();
  startWarning();
  pressButton(PIN_BTN_STOP);
  check(systemState == STATE_IDLE, "대기 상태로 복귀해야 함");
}

/* ---------------------------------------------------------------------------
   7~10. 무효 입력
   --------------------------------------------------------------------------- */

static void testIdleIgnoresExitIr() {
  beginTest("7. 대기 중 통과 IR 감지 → 아무 변화 없음");
  resetSystem();
  pulseIr(PIN_IR_EXIT_1, IR_CONFIRM_MS + 40);
  pulseIr(PIN_IR_EXIT_2, IR_CONFIRM_MS + 40);
  check(systemState == STATE_IDLE, "대기 상태를 유지해야 함");
  check(!warnLedOn(), "빨간 경고 LED가 꺼져 있어야 함");
  check(!ucast_stub::board.toneOn, "부저가 울리지 않아야 함");
}

static void testIdleIgnoresGreenButton() {
  beginTest("8. 대기 중 초록 버튼 → 아무 변화 없음");
  resetSystem();
  pressButton(PIN_BTN_STOP);
  check(systemState == STATE_IDLE, "대기 상태를 유지해야 함");
  check(bothStripsOff(), "스트립이 꺼져 있어야 함");
}

static void testWarningKeepsOnEntryReentry() {
  beginTest("9. 경고 중 진입 IR 재감지 → 경고 유지");
  resetSystem();
  startWarning();
  pulseIr(PIN_IR_ENTRY_2, IR_CONFIRM_MS + 40);
  pulseIr(PIN_IR_ENTRY_1, IR_CONFIRM_MS + 40);
  check(systemState == STATE_WARNING, "경고 상태를 유지해야 함");
}

static void testWarningKeepsOnRedButtonRepeat() {
  beginTest("10. 경고 중 빨간 버튼 재입력 → 경고 유지");
  resetSystem();
  startWarning();
  pressButton(PIN_BTN_START);
  pressButton(PIN_BTN_START);
  check(systemState == STATE_WARNING, "경고 상태를 유지해야 함");
}

/* ---------------------------------------------------------------------------
   11~16. 입력 안정성
   --------------------------------------------------------------------------- */

static void testLongButtonPress() {
  beginTest("11. 버튼 길게 누르기 → 한 번만 처리");
  resetSystem();

  // 빨간 버튼을 계속 누르고 있어도 시작은 한 번만 처리됩니다.
  setButton(PIN_BTN_START, true);
  advance(3000);
  check(systemState == STATE_WARNING, "경고가 시작되어야 함");

  // 빨간 버튼을 누른 채로 초록 버튼을 누르면 종료되고,
  // 눌린 상태가 유지되는 빨간 버튼이 경고를 다시 켜지 않아야 합니다.
  pressButton(PIN_BTN_STOP);
  check(systemState == STATE_IDLE, "초록 버튼으로 종료되어야 함");
  advance(2000);
  check(systemState == STATE_IDLE, "누른 채로 유지되는 빨간 버튼이 재시작하지 않아야 함");
}

static void testHeldIrDoesNotRepeat() {
  beginTest("12. IR 센서가 계속 감지된 상태 → 이벤트 반복 없음");
  resetSystem();

  setIr(PIN_IR_ENTRY_1, true);
  advance(2000);
  check(systemState == STATE_WARNING, "경고가 시작되어야 함");

  setIr(PIN_IR_EXIT_1, true);
  advance(2000);
  check(systemState == STATE_IDLE, "통과 감지로 종료되어야 함");

  // 두 센서가 계속 감지 상태로 유지되어도 시작/종료가 반복되지 않아야 합니다.
  advance(5000);
  check(systemState == STATE_IDLE, "감지 유지 상태에서 재시작하지 않아야 함");
  check(bothStripsOff(), "출력이 꺼진 상태를 유지해야 함");
}

static void testShortNoiseIgnored() {
  beginTest("13. 짧은 센서 노이즈 → 무시");
  resetSystem();
  setIr(PIN_IR_ENTRY_1, true);
  advance(IR_CONFIRM_MS - 40);
  setIr(PIN_IR_ENTRY_1, false);
  advance(500);
  check(systemState == STATE_IDLE, "확정 시간 미만 신호는 무시해야 함");
}

static void testEntryAndExitTogetherDoesNotStart() {
  beginTest("14. 진입과 통과 동시 입력 → 시작하지 않음");
  resetSystem();
  setIr(PIN_IR_ENTRY_1, true);
  setIr(PIN_IR_EXIT_1, true);
  advance(1000);
  check(systemState == STATE_IDLE, "모호한 입력이므로 대기를 유지해야 함");
  check(bothStripsOff(), "스트립이 켜졌다 꺼지는 동작이 없어야 함");
}

static void testRedButtonStartsDespiteAmbiguousIr() {
  beginTest("14-2. 모호한 IR 입력 중에도 빨간 버튼은 강제 시작");
  resetSystem();
  setIr(PIN_IR_ENTRY_1, true);
  setIr(PIN_IR_EXIT_1, true);
  advance(1000);
  check(systemState == STATE_IDLE, "센서만으로는 시작하지 않아야 함");

  pressButton(PIN_BTN_START);
  check(systemState == STATE_WARNING, "빨간 버튼은 항상 경고를 시작해야 함");
}

static void testBothEntrySensorsTogether() {
  beginTest("15. 진입 센서 2개 동시 입력 → 한 번만 시작");
  resetSystem();
  setIr(PIN_IR_ENTRY_1, true);
  setIr(PIN_IR_ENTRY_2, true);
  advance(1000);
  check(systemState == STATE_WARNING, "경고가 시작되어야 함");
  advance(3000);
  check(systemState == STATE_WARNING, "경고 상태를 그대로 유지해야 함");
}

static void testBothExitSensorsTogether() {
  beginTest("16. 통과 센서 2개 동시 입력 → 종료");
  resetSystem();
  startWarning();
  setIr(PIN_IR_EXIT_1, true);
  setIr(PIN_IR_EXIT_2, true);
  advance(1000);
  check(systemState == STATE_IDLE, "대기 상태로 복귀해야 함");
}

/* ---------------------------------------------------------------------------
   17~20. 출력
   --------------------------------------------------------------------------- */

static void testStripsFullRed() {
  beginTest("17. 경고 중 두 스트립 전체 빨간색");
  resetSystem();
  startWarning();
  check(bothStripsRed(), "스트립 1·2의 모든 픽셀이 빨간색이어야 함");

  // 방향 애니메이션이 없으므로 시간이 지나도 색 구성이 변하지 않아야 합니다.
  advance(2000);
  check(bothStripsRed(), "경고 중 스트립 색이 계속 유지되어야 함");
}

static void testWarnLedBlinkInterval() {
  beginTest("18. 빨간 경고 LED가 설정 간격으로 점멸");
  resetSystem();
  startWarning();
  check(warnLedOn(), "경고 시작 직후 LED가 켜져야 함");

  bool previous = warnLedOn();
  int toggles = 0;
  for (uint32_t elapsed = 0; elapsed < 900; elapsed += 10) {
    advance(10);
    if (warnLedOn() != previous) {
      toggles++;
      previous = warnLedOn();
    }
  }
  // 900ms / 300ms = 3회 전환
  check(toggles == 3, "900ms 동안 3회 점멸 전환이 있어야 함");
}

static void testBuzzerFollowsWarnLed() {
  beginTest("19. 부저가 설정 주기·주파수로 작동");
  resetSystem();
  startWarning();

  bool mismatched = false;
  bool wrongFreq = false;
  for (uint32_t elapsed = 0; elapsed < 1200; elapsed += 10) {
    advance(10);
    if (ucast_stub::board.toneOn != warnLedOn()) mismatched = true;
    if (ucast_stub::board.toneOn && ucast_stub::board.toneFreq != BUZZER_FREQUENCY_HZ) {
      wrongFreq = true;
    }
  }
  check(!mismatched, "부저는 빨간 경고 LED와 같은 주기로 동작해야 함");
  check(!wrongFreq, "부저 주파수가 BUZZER_FREQUENCY_HZ여야 함");
}

static void testAllOutputsOffOnStop() {
  beginTest("20. 종료 시 모든 출력 소등");
  resetSystem();
  startWarning();
  pressButton(PIN_BTN_STOP);

  check(bothStripsOff(), "두 스트립의 모든 픽셀이 꺼져야 함");
  check(!warnLedOn(), "빨간 경고 LED가 꺼져야 함");
  check(!ucast_stub::board.toneOn, "부저가 꺼져야 함");

  advance(2000);
  check(!warnLedOn(), "대기 중 LED가 다시 켜지지 않아야 함");
  check(!ucast_stub::board.toneOn, "대기 중 부저가 다시 울리지 않아야 함");
}

/* ---------------------------------------------------------------------------
   추가. 핀 배정과 선택 기능
   --------------------------------------------------------------------------- */

static void testPinAssignment() {
  beginTest("A. 워크북 8장 핀 배정");
  check(PIN_IR_ENTRY_1 == 2, "진입 IR 1 = D2");
  check(PIN_IR_ENTRY_2 == 3, "진입 IR 2 = D3");
  check(PIN_IR_EXIT_1 == 4, "통과 IR 1 = D4");
  check(PIN_IR_EXIT_2 == 5, "통과 IR 2 = D5");
  check(PIN_STRIP_1 == 6, "스트립 1 = D6");
  check(PIN_STRIP_2 == 7, "스트립 2 = D7");
  check(PIN_WARN_LED == 8, "빨간 경고 LED = D8");
  check(PIN_BUZZER == 9, "부저 = D9");
  check(PIN_BTN_START == A0, "빨간 시작 버튼 = A0");
  check(PIN_BTN_STOP == A1, "초록 종료 버튼 = A1");
  check(strip1.testPin() == PIN_STRIP_1, "스트립 1 객체가 D6를 사용해야 함");
  check(strip2.testPin() == PIN_STRIP_2, "스트립 2 객체가 D7을 사용해야 함");
}

static void testTimeoutDisabledByDefault() {
  beginTest("B. 자동 종료는 기본 비활성");
  check(WARNING_TIMEOUT_MS == 0, "WARNING_TIMEOUT_MS 기본값은 0이어야 함");

  resetSystem();
  startWarning();
  advance(90000, 100);
  check(systemState == STATE_WARNING, "종료 입력 없이 자동으로 꺼지지 않아야 함");
}

int main() {
  ucast_stub::board.serialLogging = false;

  testStartByEntry1();
  testStartByEntry2();
  testStartByRedButton();
  testStopByExit1();
  testStopByExit2();
  testStopByGreenButton();
  testIdleIgnoresExitIr();
  testIdleIgnoresGreenButton();
  testWarningKeepsOnEntryReentry();
  testWarningKeepsOnRedButtonRepeat();
  testLongButtonPress();
  testHeldIrDoesNotRepeat();
  testShortNoiseIgnored();
  testEntryAndExitTogetherDoesNotStart();
  testRedButtonStartsDespiteAmbiguousIr();
  testBothEntrySensorsTogether();
  testBothExitSensorsTogether();
  testStripsFullRed();
  testWarnLedBlinkInterval();
  testBuzzerFollowsWarnLed();
  testAllOutputsOffOnStop();
  testPinAssignment();
  testTimeoutDisabledByDefault();

  printf("테스트 %d개 실행, 실패 %d개\n", testsRun, testsFailed);
  return (testsFailed == 0) ? 0 : 1;
}
