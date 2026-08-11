/* ============================================================================
   축소판(test/test.ino) 회귀 테스트

   실물에서 확인한 단일 채널 계약:
   D8 시작 IR → D4 스트립/ D3 LED 점멸 시작
   D9 끝 IR   → 경고 종료
   ========================================================================== */

#include "stubs/Arduino.h"

namespace ucast_stub { Board board; }
SerialStub Serial;

#include "../test/test.ino"

#include <stdio.h>

static int testsRun = 0;
static int testsFailed = 0;
static const char *currentTest = "";

static void beginTest(const char *name) {
  testsRun++;
  currentTest = name;
}

static void check(bool condition, const char *label) {
  if (!condition) {
    testsFailed++;
    printf("  [FAIL] %s — %s\n", currentTest, label);
  }
}

static void advance(uint32_t ms, uint32_t stepMs = 10) {
  uint32_t elapsed = 0;
  while (elapsed < ms) {
    ucast_stub::board.clockMs += stepMs;
    elapsed += stepMs;
    loop();
  }
}

static void setIr(uint8_t pin, bool detected) {
  bool low = IR_ACTIVE_LOW ? detected : !detected;
  ucast_stub::board.digitalIn[pin] = low ? LOW : HIGH;
}

static void pulseIr(uint8_t pin) {
  setIr(pin, true);
  advance(IR_CONFIRM_MS + 40);
  setIr(pin, false);
  advance(IR_CONFIRM_MS + 40);
}

static uint32_t redColor() { return strip.Color(WARN_COLOR_R, WARN_COLOR_G, WARN_COLOR_B); }
static bool stripRed() { return strip.testAllShown(redColor()); }
static bool stripOff() { return strip.testAllShown(0); }

struct SirenScan {
  uint16_t lowest;
  uint16_t highest;
  int turns;
  bool inRange;
};

static SirenScan scanSiren(uint32_t durationMs) {
  SirenScan scan = {SIREN_MAX_HZ, SIREN_MIN_HZ, 0, true};

  uint16_t previous = ucast_stub::board.toneFreq;
  int direction = 0;

  for (uint32_t elapsed = 0; elapsed < durationMs; elapsed += 10) {
    advance(10);
    uint16_t frequency = ucast_stub::board.toneFreq;
    if (!ucast_stub::board.toneOn) continue;

    if (frequency < SIREN_MIN_HZ || frequency > SIREN_MAX_HZ) scan.inRange = false;
    if (frequency < scan.lowest) scan.lowest = frequency;
    if (frequency > scan.highest) scan.highest = frequency;

    if (frequency != previous) {
      int nowDirection = (frequency > previous) ? 1 : -1;
      if (direction != 0 && nowDirection != direction) scan.turns++;
      direction = nowDirection;
      previous = frequency;
    }
  }
  return scan;
}

static void resetSystem() {
  ucast_stub::reset();
  systemState = STATE_IDLE;
  warningTimeoutAt = 0;
  warningBlinkOn = false;
  warningBlinkChangedAt = 0;
  buzzerOn = false;
  sirenRising = true;
  sirenHz = 0;
  sirenSweepStartedAt = 0;
  sirenUpdatedAt = 0;
  selftestDone = false;
  irLoggedAt = 0;
  irEntry = {PIN_IR_ENTRY, false, false, 0};
  irExit = {PIN_IR_EXIT, false, false, 0};
  btnStart = {PIN_BTN_START, false, false, 0};
  btnStop = {PIN_BTN_STOP, false, false, 0};

  const int idle = IR_ACTIVE_LOW ? HIGH : LOW;
  ucast_stub::board.digitalIn[PIN_IR_ENTRY] = idle;
  ucast_stub::board.digitalIn[PIN_IR_EXIT] = idle;

  setup();
  advance(SELFTEST_END_MS + 100);
}

static void testPins() {
  beginTest("1. 실물 축소판 핀 고정");
  check(PIN_IR_ENTRY == 8, "시작 IR=D8");
  check(PIN_IR_EXIT == 9, "끝 IR=D9");
  check(PIN_STRIP == 4, "스트립=D4");
  check(PIN_WARN_LED == 3, "경고 LED=D3");
  check(PIN_BUZZER == 5, "부저=D5");
  check(!USE_BUTTONS, "버튼 미사용");
}

static void testEntryStartsExitStops() {
  beginTest("2. D8 시작 → D9 종료");
  resetSystem();
  pulseIr(PIN_IR_ENTRY);
  check(systemState == STATE_WARNING, "D8로 경고 시작");
  check(stripRed(), "스트립 빨강 ON 위상");
  check(ucast_stub::board.toneOn, "사이렌 시작");

  pulseIr(PIN_IR_EXIT);
  check(systemState == STATE_IDLE, "D9로 종료");
  check(stripOff(), "스트립 종료");
  check(!ucast_stub::board.toneOn, "사이렌 즉시 정지(noTone)");
  check(ucast_stub::board.toneFreq == 0, "정지 후 주파수가 남아 있지 않아야 함");
}

static void testExitDoesNotStart() {
  beginTest("3. 대기 중 D9 끝 센서는 시작하지 않음");
  resetSystem();
  pulseIr(PIN_IR_EXIT);
  check(systemState == STATE_IDLE, "끝 센서 단독 무시");
}

static void testStripAndLedBlinkTogether() {
  beginTest("4. 스트립·빨간 LED 300ms 동시 점멸");
  resetSystem();
  pulseIr(PIN_IR_ENTRY);
  check(warningBlinkOn && stripRed(), "시작 직후 ON");
  check(ucast_stub::board.digitalOut[PIN_WARN_LED] == HIGH, "LED ON");

  advance(WARNING_BLINK_MS);
  check(!warningBlinkOn && stripOff(), "300ms 후 스트립 OFF");
  check(ucast_stub::board.digitalOut[PIN_WARN_LED] == LOW, "300ms 후 LED OFF");
  check(ucast_stub::board.toneOn, "OFF 위상에도 사이렌 유지");

  advance(WARNING_BLINK_MS);
  check(warningBlinkOn && stripRed(), "600ms 후 다시 ON");
}

static void testSirenAndTimeout() {
  beginTest("5. 사이렌 범위와 10초 안전 복귀");
  resetSystem();
  pulseIr(PIN_IR_ENTRY);
  advance(1100);
  check(ucast_stub::board.toneOn, "사이렌 연속");
  check(ucast_stub::board.toneFreq >= SIREN_MIN_HZ &&
        ucast_stub::board.toneFreq <= SIREN_MAX_HZ, "사이렌 범위");

  advance(WARNING_TIMEOUT_MS + 1000);
  check(systemState == STATE_IDLE, "10초 뒤 자동 복귀");
  check(!ucast_stub::board.toneOn, "자동 복귀 뒤에도 소리가 남아 있으면 안 됨");
}

static void testSirenSweepMatchesMain() {
  beginTest("6. 사이렌 상승·하강이 main.ino와 같은 주기로 뒤바뀜");
  resetSystem();
  pulseIr(PIN_IR_ENTRY);

  SirenScan scan = scanSiren(2000);
  check(scan.inRange, "주파수가 SIREN_MIN_HZ~SIREN_MAX_HZ를 벗어나지 않아야 함");
  check(scan.turns >= 3 && scan.turns <= 4, "0.5초마다 상승↔하강이 뒤바뀌어야 함");
}

static void testLogIsRateLimited() {
  beginTest("7. 주기 로그가 IR_LOG_INTERVAL_MS 간격으로만 나옴");
  resetSystem();

  const uint32_t before = ucast_stub::board.serialLines;
  advance(2000);
  const uint32_t lines = ucast_stub::board.serialLines - before;

  if (IR_LOG_INTERVAL_MS == 0) {
    check(lines == 0, "로그를 껐으면 주기 출력이 없어야 함");
  } else {
    uint32_t interval = IR_LOG_INTERVAL_MS;
    const uint32_t expected = 2000 / interval;
    check(lines >= expected - 1 && lines <= expected + 1,
          "2초 동안 2000/IR_LOG_INTERVAL_MS줄 안팎이어야 함");
    check(lines < 200, "loop()마다 출력하면 안 됨");
  }
}

static void testButtonsAreNotRead() {
  beginTest("8. 버튼 미연결 시 핀 입력으로 오작동하지 않음");
  resetSystem();
  ucast_stub::board.digitalIn[PIN_BTN_START] = LOW;
  ucast_stub::board.digitalIn[PIN_BTN_STOP] = LOW;
  advance(500);
  check(systemState == STATE_IDLE, "A0/A1 값과 무관하게 대기");
}

int main() {
  ucast_stub::board.serialLogging = false;

  testPins();
  testEntryStartsExitStops();
  testExitDoesNotStart();
  testStripAndLedBlinkTogether();
  testSirenAndTimeout();
  testSirenSweepMatchesMain();
  testLogIsRateLimited();
  testButtonsAreNotRead();

  printf("축소판 테스트 %d개 실행, 실패 %d개\n", testsRun, testsFailed);
  return testsFailed == 0 ? 0 : 1;
}
