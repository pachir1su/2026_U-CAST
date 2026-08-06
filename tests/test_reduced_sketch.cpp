/* ============================================================================
   축소판 스케치(`test/test.ino`) 회귀 테스트

   `main.ino`와 `test/test.ino`는 같은 알고리즘을 써야 합니다. 두 파일은 각각
   setup()/loop()를 정의하므로 한 실행 파일에 함께 넣을 수 없어, 축소판은 이
   파일에서 따로 컴파일해 **부저 사이렌과 진단 로그가 같은 값·같은 동작인지**를
   검증합니다.

   실행 방법: tests/run-tests.sh
   여기서도 실물 IR 모듈의 출력 논리나 부저 음량 같은 값은 검증하지 않습니다.
   ========================================================================== */

#include "stubs/Arduino.h"

namespace ucast_stub {
Board board;
}
SerialStub Serial;

#include "../test/test.ino"

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

// 전역 상태를 초기값으로 되돌리고 setup()을 다시 실행한 뒤,
// 부팅 자가진단이 끝날 때까지 시간을 흘려 보냅니다.
static void resetSystem() {
  ucast_stub::reset();

  systemState = STATE_IDLE;
  startedFrom = SIDE_NONE;
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

  irLeft  = {PIN_IR_LEFT,  false, false, 0};
  irRight = {PIN_IR_RIGHT, false, false, 0};
  btnStart = {PIN_BTN_START, false, false, 0};
  btnStop  = {PIN_BTN_STOP,  false, false, 0};

  const int idle = IR_ACTIVE_LOW ? HIGH : LOW;
  ucast_stub::board.digitalIn[PIN_IR_LEFT]  = idle;
  ucast_stub::board.digitalIn[PIN_IR_RIGHT] = idle;

  setup();
  advance(SELFTEST_END_MS + 100);  // 부팅 자가진단이 끝난 뒤부터 검증합니다
}

static void pulseIr(uint8_t pin) {
  setIr(pin, true);
  advance(IR_CONFIRM_MS + 40);
  setIr(pin, false);
  advance(IR_CONFIRM_MS + 40);
}

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

/* ---------------------------------------------------------------------------
   1~2. main.ino와 값·동작이 같은지
   --------------------------------------------------------------------------- */

static void testSirenConstantsMatchMain() {
  beginTest("1. 사이렌 상수가 main.ino와 같음");
  // main.ino의 테스트 J와 같은 기대값입니다. 한쪽만 고치면 여기서 걸립니다.
  check(SIREN_MIN_HZ == 800, "SIREN_MIN_HZ = 800");
  check(SIREN_MAX_HZ == 1800, "SIREN_MAX_HZ = 1800");
  check(SIREN_SWEEP_MS == 500, "SIREN_SWEEP_MS = 500");
  check(SIREN_UPDATE_MS >= 20 && SIREN_UPDATE_MS <= 30, "SIREN_UPDATE_MS는 20~30ms");
  check(WARNING_BLINK_MS == 300, "빨간 LED 점멸 간격도 main.ino와 같아야 함");
}

static void testSirenSweepMatchesMain() {
  beginTest("2. 사이렌 상승·하강 동작이 main.ino와 같음");
  resetSystem();

  // 경고 시작 직후를 보기 위해 손을 대고 확정 시간만 지난 시점에서 확인합니다.
  setIr(PIN_IR_LEFT, true);
  advance(IR_CONFIRM_MS + 10);
  check(systemState == STATE_WARNING, "왼쪽 감지로 경고가 시작되어야 함");
  check(ucast_stub::board.toneOn, "경고 시작 즉시 소리가 나야 함");
  check(ucast_stub::board.toneFreq == SIREN_MIN_HZ, "최저 주파수에서 시작해야 함");
  setIr(PIN_IR_LEFT, false);

  const uint16_t step = (uint16_t)(((uint32_t)(SIREN_MAX_HZ - SIREN_MIN_HZ) *
                                    SIREN_UPDATE_MS) / SIREN_SWEEP_MS);

  SirenScan scan = scanSiren(2000);
  check(scan.inRange, "주파수가 SIREN_MIN_HZ~SIREN_MAX_HZ를 벗어나지 않아야 함");
  check(scan.lowest <= SIREN_MIN_HZ + step, "최저 주파수 근처까지 내려가야 함");
  check(scan.highest >= SIREN_MAX_HZ - step, "최고 주파수 근처까지 올라가야 함");
  check(scan.turns >= 3 && scan.turns <= 4, "0.5초마다 상승↔하강이 뒤바뀌어야 함");
}

/* ---------------------------------------------------------------------------
   3~5. 종료와 로그
   --------------------------------------------------------------------------- */

static void testSirenStopsImmediately() {
  beginTest("3. 사이렌 중 반대편 도착 → 즉시 정지");
  resetSystem();
  pulseIr(PIN_IR_LEFT);
  advance(700);
  check(ucast_stub::board.toneOn, "종료 전에는 소리가 나고 있어야 함");

  pulseIr(PIN_IR_RIGHT);
  check(systemState == STATE_IDLE, "사이렌 중에도 반대편 도착으로 종료되어야 함");
  check(!ucast_stub::board.toneOn, "종료 즉시 noTone()이 되어야 함");
  check(ucast_stub::board.toneFreq == 0, "정지 후 주파수가 남아 있지 않아야 함");
}

static void testAutoReturnStopsSiren() {
  beginTest("4. 자동 복귀로 종료돼도 사이렌이 멈춤");
  resetSystem();
  pulseIr(PIN_IR_LEFT);
  check(systemState == STATE_WARNING, "경고가 시작되어야 함");

  advance(WARNING_TIMEOUT_MS + 500, 50);
  check(systemState == STATE_IDLE, "자동 복귀로 대기 상태가 되어야 함");
  check(!ucast_stub::board.toneOn, "자동 복귀 뒤에도 소리가 남아 있으면 안 됨");
}

static void testLogIsRateLimited() {
  beginTest("5. 주기 로그가 IR_LOG_INTERVAL_MS 간격으로만 나옴");
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

int main() {
  ucast_stub::board.serialLogging = false;

  testSirenConstantsMatchMain();
  testSirenSweepMatchesMain();
  testSirenStopsImmediately();
  testAutoReturnStopsSiren();
  testLogIsRateLimited();

  printf("축소판 테스트 %d개 실행, 실패 %d개\n", testsRun, testsFailed);
  return (testsFailed == 0) ? 0 : 1;
}
