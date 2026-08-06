/* ============================================================================
   양방향 진입·도착 IR 경고 알고리즘 회귀 테스트

   실물 보드 없이 PC에서 `main.ino`의 상태 머신을 검증합니다.
   실행 방법: tests/run-tests.sh

   여기서 검증하는 것은 상태 전이와 출력 논리이며,
   실제 IR 모듈의 출력 논리나 부저 음량 같은 실물 값은 검증하지 않습니다.
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
  startedFrom = SIDE_NONE;
  warningTimeoutAt = 0;
  warningBlinkOn = false;
  warningBlinkChangedAt = 0;

  irLeft1  = {PIN_IR_LEFT_1,  false, false, 0};
  irLeft2  = {PIN_IR_LEFT_2,  false, false, 0};
  irRight1 = {PIN_IR_RIGHT_1, false, false, 0};
  irRight2 = {PIN_IR_RIGHT_2, false, false, 0};
  btnStart = {PIN_BTN_START, false, false, 0};
  btnStop  = {PIN_BTN_STOP,  false, false, 0};

  // 모든 IR을 "감지 없음" 상태로 둡니다.
  const int idle = IR_ACTIVE_LOW ? HIGH : LOW;
  ucast_stub::board.digitalIn[PIN_IR_LEFT_1]  = idle;
  ucast_stub::board.digitalIn[PIN_IR_LEFT_2]  = idle;
  ucast_stub::board.digitalIn[PIN_IR_RIGHT_1] = idle;
  ucast_stub::board.digitalIn[PIN_IR_RIGHT_2] = idle;

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

static bool stripsRed() { return strip1.testAllShown(redColor()); }
static bool stripsOff() { return strip1.testAllShown(0); }

// 센서를 감지 → 확정 대기 → 해제 순서로 한 번 통과시킵니다(손을 휘젓는 동작).
static void pulseIr(uint8_t pin) {
  setIr(pin, true);
  advance(IR_CONFIRM_MS + 40);
  setIr(pin, false);
  advance(IR_CONFIRM_MS + 40);
}

static void pressButton(uint8_t pin) {
  setButton(pin, true);
  advance(BUTTON_DEBOUNCE_MS + 30);
  setButton(pin, false);
  advance(BUTTON_DEBOUNCE_MS + 30);
}

/* ---------------------------------------------------------------------------
   1~4. 양방향 시작·종료
   --------------------------------------------------------------------------- */

static void testLeftToRight() {
  beginTest("1. 왼쪽 → 오른쪽 횡단");
  resetSystem();

  pulseIr(PIN_IR_LEFT_1);
  check(systemState == STATE_WARNING, "왼쪽 감지로 경고가 시작되어야 함");
  check(startedFrom == SIDE_LEFT, "출발 쪽을 왼쪽으로 기억해야 함");
  check(stripsRed(), "경고 중 스트립이 전체 빨간색이어야 함");

  pulseIr(PIN_IR_RIGHT_1);
  check(systemState == STATE_IDLE, "반대편(오른쪽) 도착으로 종료되어야 함");
  check(stripsOff(), "종료 시 스트립이 꺼져야 함");
}

static void testRightToLeft() {
  beginTest("2. 오른쪽 → 왼쪽 횡단 (반대 방향)");
  resetSystem();

  pulseIr(PIN_IR_RIGHT_1);
  check(systemState == STATE_WARNING, "오른쪽 감지로도 경고가 시작되어야 함");
  check(startedFrom == SIDE_RIGHT, "출발 쪽을 오른쪽으로 기억해야 함");

  pulseIr(PIN_IR_LEFT_1);
  check(systemState == STATE_IDLE, "반대편(왼쪽) 도착으로 종료되어야 함");
}

static void testEachSensorOfPairStarts() {
  beginTest("3. 같은 연석의 두 번째 센서로도 시작·종료");
  resetSystem();
  pulseIr(PIN_IR_LEFT_2);
  check(systemState == STATE_WARNING && startedFrom == SIDE_LEFT, "왼쪽 2번으로 시작");
  pulseIr(PIN_IR_RIGHT_2);
  check(systemState == STATE_IDLE, "오른쪽 2번으로 종료");

  resetSystem();
  pulseIr(PIN_IR_RIGHT_2);
  check(systemState == STATE_WARNING && startedFrom == SIDE_RIGHT, "오른쪽 2번으로 시작");
  pulseIr(PIN_IR_LEFT_2);
  check(systemState == STATE_IDLE, "왼쪽 2번으로 종료");
}

static void testConsecutiveCrossingsBothWays() {
  beginTest("4. 왼→오 다음에 오→왼을 연달아");
  resetSystem();

  pulseIr(PIN_IR_LEFT_1);
  pulseIr(PIN_IR_RIGHT_1);
  check(systemState == STATE_IDLE, "첫 횡단이 끝나야 함");
  check(startedFrom == SIDE_NONE, "출발 쪽 기억이 지워져야 함");

  // 이번에는 반대 방향으로 건넙니다.
  pulseIr(PIN_IR_RIGHT_1);
  check(systemState == STATE_WARNING && startedFrom == SIDE_RIGHT,
        "두 번째 횡단은 오른쪽 출발로 잡혀야 함");
  pulseIr(PIN_IR_LEFT_1);
  check(systemState == STATE_IDLE, "두 번째 횡단도 정상 종료되어야 함");
}

/* ---------------------------------------------------------------------------
   5~7. 출발 쪽 재감지는 종료가 아니다 (양방향의 핵심 회귀)
   --------------------------------------------------------------------------- */

static void testSameSideDoesNotStopLeft() {
  beginTest("5. 왼쪽 출발 중 왼쪽 재감지 → 종료되지 않음");
  resetSystem();
  pulseIr(PIN_IR_LEFT_1);
  check(systemState == STATE_WARNING, "경고 시작");

  // 한 번 지나갈 때마다 확인합니다. 두 번 확인하고 끝내면 "꺼졌다가 다시 켜진"
  // 잘못된 구현도 통과해 버리기 때문입니다.
  pulseIr(PIN_IR_LEFT_2);
  check(systemState == STATE_WARNING, "왼쪽 2번 재감지 후에도 경고가 유지되어야 함");
  check(startedFrom == SIDE_LEFT, "출발 쪽 기억이 유지되어야 함");

  pulseIr(PIN_IR_LEFT_1);
  check(systemState == STATE_WARNING, "왼쪽 1번 재감지 후에도 경고가 유지되어야 함");
  check(startedFrom == SIDE_LEFT, "출발 쪽 기억이 유지되어야 함");
  check(stripsRed(), "경고 출력이 유지되어야 함");
}

static void testSameSideDoesNotStopRight() {
  beginTest("6. 오른쪽 출발 중 오른쪽 재감지 → 종료되지 않음");
  resetSystem();
  pulseIr(PIN_IR_RIGHT_1);
  check(systemState == STATE_WARNING, "경고 시작");

  pulseIr(PIN_IR_RIGHT_2);
  check(systemState == STATE_WARNING, "오른쪽 2번 재감지 후에도 경고가 유지되어야 함");
  check(startedFrom == SIDE_RIGHT, "출발 쪽 기억이 유지되어야 함");

  pulseIr(PIN_IR_RIGHT_1);
  check(systemState == STATE_WARNING, "오른쪽 1번 재감지 후에도 경고가 유지되어야 함");
}

static void testIdleIgnoresNothing() {
  beginTest("7. 대기 중에는 어느 쪽이든 시작 조건이 됨");
  resetSystem();
  pulseIr(PIN_IR_RIGHT_2);
  check(systemState == STATE_WARNING, "대기 중 오른쪽 입력도 무시되지 않아야 함");
}

/* ---------------------------------------------------------------------------
   8~11. 버튼
   --------------------------------------------------------------------------- */

static void testStartByRedButton() {
  beginTest("8. 빨간 버튼 → 경고 시작 (출발 쪽 없음)");
  resetSystem();
  pressButton(PIN_BTN_START);
  check(systemState == STATE_WARNING, "경고 상태로 전환되어야 함");
  check(startedFrom == SIDE_NONE, "버튼 시작은 출발 쪽이 없어야 함");
}

static void testButtonStartStopsFromEitherSide() {
  beginTest("9. 버튼으로 시작한 경고는 어느 쪽 센서로도 종료");
  resetSystem();
  pressButton(PIN_BTN_START);
  pulseIr(PIN_IR_LEFT_1);
  check(systemState == STATE_IDLE, "왼쪽으로 종료되어야 함");

  resetSystem();
  pressButton(PIN_BTN_START);
  pulseIr(PIN_IR_RIGHT_1);
  check(systemState == STATE_IDLE, "오른쪽으로도 종료되어야 함");
}

static void testStopByGreenButton() {
  beginTest("10. 초록 버튼 → 경고 종료");
  resetSystem();
  pulseIr(PIN_IR_LEFT_1);
  pressButton(PIN_BTN_STOP);
  check(systemState == STATE_IDLE, "대기 상태로 복귀해야 함");
}

static void testIdleIgnoresGreenButton() {
  beginTest("11. 대기 중 초록 버튼 → 아무 변화 없음");
  resetSystem();
  pressButton(PIN_BTN_STOP);
  check(systemState == STATE_IDLE, "대기 상태를 유지해야 함");
  check(stripsOff(), "스트립이 꺼져 있어야 함");
}

/* ---------------------------------------------------------------------------
   12~16. 모호 입력과 입력 안정성
   --------------------------------------------------------------------------- */

static void testBothSidesStableDoesNotStart() {
  beginTest("12. 양쪽이 계속 감지 중 → 시작하지 않음 (마주보게 둔 경우)");
  resetSystem();
  setIr(PIN_IR_LEFT_1, true);
  setIr(PIN_IR_RIGHT_1, true);
  advance(2000);
  check(systemState == STATE_IDLE, "모호한 입력이므로 대기를 유지해야 함");
  check(stripsOff(), "스트립이 켜졌다 꺼지는 동작이 없어야 함");
}

static void testRedButtonStartsDespiteAmbiguousIr() {
  beginTest("13. 모호한 IR 입력 중에도 빨간 버튼은 강제 시작");
  resetSystem();
  setIr(PIN_IR_LEFT_1, true);
  setIr(PIN_IR_RIGHT_1, true);
  advance(2000);
  check(systemState == STATE_IDLE, "센서만으로는 시작하지 않아야 함");

  pressButton(PIN_BTN_START);
  check(systemState == STATE_WARNING, "빨간 버튼은 항상 경고를 시작해야 함");
}

static void testLongButtonPress() {
  beginTest("14. 버튼 길게 누르기 → 한 번만 처리");
  resetSystem();

  setButton(PIN_BTN_START, true);
  advance(3000);
  check(systemState == STATE_WARNING, "경고가 시작되어야 함");

  pressButton(PIN_BTN_STOP);
  check(systemState == STATE_IDLE, "초록 버튼으로 종료되어야 함");
  advance(2000);
  check(systemState == STATE_IDLE, "누른 채로 유지되는 빨간 버튼이 재시작하지 않아야 함");
}

static void testHeldIrDoesNotRepeat() {
  beginTest("15. IR이 계속 감지된 상태 → 이벤트 반복 없음");
  resetSystem();

  setIr(PIN_IR_LEFT_1, true);
  advance(2000);
  check(systemState == STATE_WARNING, "경고가 시작되어야 함");

  setIr(PIN_IR_RIGHT_1, true);
  advance(2000);
  check(systemState == STATE_IDLE, "반대편 도착으로 종료되어야 함");

  // 두 센서가 계속 감지 상태로 유지되어도 시작/종료가 반복되지 않아야 합니다.
  advance(5000);
  check(systemState == STATE_IDLE, "감지 유지 상태에서 재시작하지 않아야 함");
  check(stripsOff(), "출력이 꺼진 상태를 유지해야 함");
}

static void testShortNoiseIgnored() {
  beginTest("16. 짧은 센서 노이즈 → 무시");
  resetSystem();
  setIr(PIN_IR_LEFT_1, true);
  advance(IR_CONFIRM_MS - 30);
  setIr(PIN_IR_LEFT_1, false);
  advance(500);
  check(systemState == STATE_IDLE, "확정 시간 미만 신호는 무시해야 함");
}

/* ---------------------------------------------------------------------------
   17~20. 출력
   --------------------------------------------------------------------------- */

static void testStripsFullRed() {
  beginTest("17. 경고 중 스트립 전체 빨간색");
  resetSystem();
  pulseIr(PIN_IR_LEFT_1);
  check(stripsRed(), "스트립의 모든 픽셀이 빨간색이어야 함");
  check(strip1.testShownPixel(STRIP_1_PIXELS - 1) == redColor(), "마지막 픽셀도 빨간색이어야 함");

  // 방향 애니메이션이 없으므로 시간이 지나도 색 구성이 변하지 않아야 합니다.
  advance(2000);
  check(stripsRed(), "경고 중 스트립 색이 계속 유지되어야 함");
}

static void testWarnLedBlinkInterval() {
  beginTest("18. 빨간 경고 LED가 설정 간격으로 점멸");
  resetSystem();
  pulseIr(PIN_IR_LEFT_1);
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
  pulseIr(PIN_IR_LEFT_1);

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
  pulseIr(PIN_IR_LEFT_1);
  pressButton(PIN_BTN_STOP);

  check(stripsOff(), "스트립의 모든 픽셀이 꺼져야 함");
  check(!warnLedOn(), "빨간 경고 LED가 꺼져야 함");
  check(!ucast_stub::board.toneOn, "부저가 꺼져야 함");

  advance(2000);
  check(!warnLedOn(), "대기 중 LED가 다시 켜지지 않아야 함");
  check(!ucast_stub::board.toneOn, "대기 중 부저가 다시 울리지 않아야 함");
}

/* ---------------------------------------------------------------------------
   추가. 핀 배정, 스트립 2줄, 선택 기능
   --------------------------------------------------------------------------- */

static void testPinAssignment() {
  beginTest("A. 실물 배선 기준 핀 배정");
  check(PIN_IR_LEFT_1 == 8, "왼쪽 IR 1 = D8");
  check(PIN_IR_LEFT_2 == 9, "왼쪽 IR 2 = D9");
  check(PIN_IR_RIGHT_1 == 10, "오른쪽 IR 1 = D10");
  check(PIN_IR_RIGHT_2 == 11, "오른쪽 IR 2 = D11");
  check(PIN_WARN_LED == 3, "빨간 경고 LED = D3");
  check(PIN_STRIP_1 == 4, "스트립 1 = D4");
  check(PIN_BUZZER == 5, "부저 = D5");
  check(PIN_BTN_START == A0, "빨간 시작 버튼 = A0");
  check(PIN_BTN_STOP == A1, "초록 종료 버튼 = A1");
  check(strip1.testPin() == PIN_STRIP_1, "스트립 1 객체가 D4를 사용해야 함");

  // test/test.ino가 쓰는 D8·D10 배선을 그대로 두고 D9·D11만 추가하면 되는지
  // (센서 증설 시 기존 선을 옮기지 않아도 되는지) 확인합니다.
  check(PIN_IR_LEFT_1 == 8 && PIN_IR_RIGHT_1 == 10,
        "축소판이 쓰던 D8·D10이 그대로 유지되어야 함");
}

static void testSecondStripIsOptional() {
  beginTest("B. 두 번째 스트립은 핀을 정하기 전까지 건너뜀");
  // PIN_STRIP_2가 0이면 사용하지 않는 것으로 봅니다.
  check(STRIP_2_CONNECTED == (PIN_STRIP_2 != 0),
        "STRIP_2_CONNECTED가 PIN_STRIP_2로부터 결정되어야 함");

  resetSystem();
  const uint32_t showsBefore = strip2.testShowCount();
  pulseIr(PIN_IR_LEFT_1);
  advance(1000);
  pressButton(PIN_BTN_STOP);

  if (!STRIP_2_CONNECTED) {
    check(strip2.testShowCount() == showsBefore,
          "핀 미정일 때는 두 번째 스트립에 아무것도 보내지 않아야 함");
  } else {
    check(strip2.testShowCount() > showsBefore,
          "핀을 정했으면 두 번째 스트립도 갱신되어야 함");
  }
}

static void testStripPixelCount() {
  beginTest("C. 스트립 픽셀 수 = 실물 10개");
  check(STRIP_1_PIXELS == 10, "STRIP_1_PIXELS는 실물 스트립 LED 개수인 10이어야 함");
  check(strip1.testPixelCount() == STRIP_1_PIXELS, "스트립 객체가 STRIP_1_PIXELS개를 가져야 함");
}

static void testTimeoutDisabledByDefault() {
  beginTest("D. 자동 종료는 기본 비활성");
  check(WARNING_TIMEOUT_MS == 0, "WARNING_TIMEOUT_MS 기본값은 0이어야 함");

  resetSystem();
  pulseIr(PIN_IR_LEFT_1);
  advance(90000, 100);
  check(systemState == STATE_WARNING, "종료 입력 없이 자동으로 꺼지지 않아야 함");
}

int main() {
  ucast_stub::board.serialLogging = false;

  testLeftToRight();
  testRightToLeft();
  testEachSensorOfPairStarts();
  testConsecutiveCrossingsBothWays();
  testSameSideDoesNotStopLeft();
  testSameSideDoesNotStopRight();
  testIdleIgnoresNothing();
  testStartByRedButton();
  testButtonStartStopsFromEitherSide();
  testStopByGreenButton();
  testIdleIgnoresGreenButton();
  testBothSidesStableDoesNotStart();
  testRedButtonStartsDespiteAmbiguousIr();
  testLongButtonPress();
  testHeldIrDoesNotRepeat();
  testShortNoiseIgnored();
  testStripsFullRed();
  testWarnLedBlinkInterval();
  testBuzzerFollowsWarnLed();
  testAllOutputsOffOnStop();
  testPinAssignment();
  testSecondStripIsOptional();
  testStripPixelCount();
  testTimeoutDisabledByDefault();

  printf("테스트 %d개 실행, 실패 %d개\n", testsRun, testsFailed);
  return (testsFailed == 0) ? 0 : 1;
}
