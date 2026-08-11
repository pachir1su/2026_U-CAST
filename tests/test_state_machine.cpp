/* ============================================================================
   멈춰 ! — 독립 2채널 상태 머신 회귀 테스트

   이슈 #18 계약:
   - 채널1: ENTRY_1(D8) → STRIP_1 시작 / EXIT_1(D10) → STRIP_1 종료
   - 채널2: ENTRY_2(D9) → STRIP_2 시작 / EXIT_2(D11) → STRIP_2 종료
   - 두 채널은 서로 독립적으로 동시에 동작 가능
   - 공용 빨간 LED·부저는 하나라도 활성 채널이 있으면 유지
   ========================================================================== */

#include "stubs/Arduino.h"

namespace ucast_stub { Board board; }
SerialStub Serial;

#include "../main.ino"

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

static void setButton(uint8_t pin, bool pressed) {
  bool low = BUTTON_ACTIVE_LOW ? pressed : !pressed;
  ucast_stub::board.digitalIn[pin] = low ? LOW : HIGH;
}

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

static uint32_t redColor() {
  return strip1.Color(WARN_COLOR_R, WARN_COLOR_G, WARN_COLOR_B);
}

static bool strip1Red() { return strip1.testAllShown(redColor()); }
static bool strip1Off() { return strip1.testAllShown(0); }
static bool strip2Red() { return strip2.testAllShown(redColor()); }
static bool strip2Off() { return strip2.testAllShown(0); }
static bool warnLedOn() { return ucast_stub::board.digitalOut[PIN_WARN_LED] == HIGH; }

// 사이렌 주파수를 지정한 시간 동안 훑으며 최저·최고값과 방향 전환 횟수를 셉니다.
struct SirenScan {
  uint16_t lowest;
  uint16_t highest;
  int turns;      // 상승 → 하강 또는 하강 → 상승으로 바뀐 횟수
  bool inRange;   // 모든 표본이 SIREN_MIN_HZ ~ SIREN_MAX_HZ 안에 있었는지
};

static SirenScan scanSiren(uint32_t durationMs) {
  SirenScan scan = {SIREN_MAX_HZ, SIREN_MIN_HZ, 0, true};

  uint16_t previous = ucast_stub::board.toneFreq;
  int direction = 0;  // +1 = 올라가는 중, -1 = 내려가는 중

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
  channel1Active = false;
  channel2Active = false;
  channel1TimeoutAt = 0;
  channel2TimeoutAt = 0;
  warningBlinkOn = false;
  warningBlinkChangedAt = 0;
  buzzerOn = false;
  sirenRising = true;
  sirenHz = 0;
  sirenSweepStartedAt = 0;
  sirenUpdatedAt = 0;
  irLoggedAt = 0;

  irEntry1 = {PIN_IR_ENTRY_1, false, false, 0};
  irEntry2 = {PIN_IR_ENTRY_2, false, false, 0};
  irExit1  = {PIN_IR_EXIT_1,  false, false, 0};
  irExit2  = {PIN_IR_EXIT_2,  false, false, 0};
  btnStart = {PIN_BTN_START, false, false, 0};
  btnStop  = {PIN_BTN_STOP,  false, false, 0};

  const int idle = IR_ACTIVE_LOW ? HIGH : LOW;
  ucast_stub::board.digitalIn[PIN_IR_ENTRY_1] = idle;
  ucast_stub::board.digitalIn[PIN_IR_ENTRY_2] = idle;
  ucast_stub::board.digitalIn[PIN_IR_EXIT_1] = idle;
  ucast_stub::board.digitalIn[PIN_IR_EXIT_2] = idle;

  setup();
}

static void testChannel1Sequence() {
  beginTest("1. 채널1 시작/끝 센서가 스트립1만 제어");
  resetSystem();

  pulseIr(PIN_IR_ENTRY_1);
  check(channel1Active, "ENTRY_1 감지 후 채널1 활성");
  check(!channel2Active, "채널2는 비활성 유지");
  check(systemState == STATE_WARNING, "시스템 경고 상태");
  check(strip1Red(), "스트립1이 빨간색으로 켜짐");
  if (STRIP_2_CONNECTED) check(strip2Off(), "스트립2는 꺼져 있어야 함");

  pulseIr(PIN_IR_EXIT_1);
  check(!channel1Active, "EXIT_1 감지 후 채널1 종료");
  check(systemState == STATE_IDLE, "활성 채널이 없으면 대기");
  check(strip1Off(), "스트립1 종료");
}

static void testChannel2Sequence() {
  beginTest("2. 채널2 시작/끝 센서가 스트립2만 제어");
  resetSystem();

  pulseIr(PIN_IR_ENTRY_2);
  check(!channel1Active && channel2Active, "ENTRY_2는 채널2만 활성화");
  check(strip1Off(), "스트립1은 꺼져 있음");
  if (STRIP_2_CONNECTED) check(strip2Red(), "연결된 스트립2가 빨간색으로 켜짐");

  pulseIr(PIN_IR_EXIT_2);
  check(!channel2Active, "EXIT_2로 채널2 종료");
  if (STRIP_2_CONNECTED) check(strip2Off(), "스트립2 종료");
}

static void testWrongExitDoesNotStopOtherChannel() {
  beginTest("3. 다른 채널 끝 센서는 활성 채널을 끄지 않음");
  resetSystem();

  pulseIr(PIN_IR_ENTRY_1);
  pulseIr(PIN_IR_EXIT_2);
  check(channel1Active, "EXIT_2는 채널1을 종료하지 않음");
  check(warningBlinkOn ? strip1Red() : strip1Off(), "스트립1은 현재 점멸 위상을 유지");

  pulseIr(PIN_IR_EXIT_1);
  check(!channel1Active, "자기 EXIT_1에서만 종료");
}

static void testBothChannelsIndependent() {
  beginTest("4. 두 채널 동시 활성 후 하나만 독립 종료");
  resetSystem();

  pulseIr(PIN_IR_ENTRY_1);
  pulseIr(PIN_IR_ENTRY_2);
  check(channel1Active && channel2Active, "두 채널 동시 활성 가능");
  check(warningBlinkOn ? strip1Red() : strip1Off(), "스트립1은 현재 점멸 위상 반영");
  if (STRIP_2_CONNECTED) check(warningBlinkOn ? strip2Red() : strip2Off(), "스트립2도 같은 점멸 위상 반영");
  check(ucast_stub::board.toneOn, "공용 사이렌 활성");

  pulseIr(PIN_IR_EXIT_1);
  check(!channel1Active && channel2Active, "채널1만 종료");
  check(strip1Off(), "스트립1만 꺼짐");
  if (STRIP_2_CONNECTED) check(warningBlinkOn ? strip2Red() : strip2Off(), "스트립2는 활성 상태와 점멸 위상 유지");
  check(ucast_stub::board.toneOn, "채널2가 남아 사이렌 유지");

  pulseIr(PIN_IR_EXIT_2);
  check(!channel1Active && !channel2Active, "마지막 채널 종료");
  check(!ucast_stub::board.toneOn, "모두 종료되면 사이렌 정지");
}

static void testIdleExitIgnored() {
  beginTest("5. 대기 중 끝 센서 단독 감지는 무시");
  resetSystem();
  pulseIr(PIN_IR_EXIT_1);
  pulseIr(PIN_IR_EXIT_2);
  check(systemState == STATE_IDLE, "끝 센서만으로 시작하지 않음");
  check(!channel1Active && !channel2Active, "채널 비활성 유지");
}

static void testEntryRepeatDoesNotStop() {
  beginTest("6. 활성 중 시작 센서 재감지는 유지");
  resetSystem();
  pulseIr(PIN_IR_ENTRY_1);
  pulseIr(PIN_IR_ENTRY_1);
  check(channel1Active, "ENTRY_1 재감지 후에도 채널1 유지");
}

static void testPairAmbiguityGuard() {
  beginTest("7. 끝 센서가 이미 붙어 있으면 같은 채널 시작 차단");
  resetSystem();

  setIr(PIN_IR_EXIT_1, true);
  advance(IR_CONFIRM_MS + 40);
  pulseIr(PIN_IR_ENTRY_1);
  check(!channel1Active, "EXIT_1 안정 감지 중 ENTRY_1 시작 차단");
}

static void testCrossChannelSimultaneousStartAllowed() {
  beginTest("8. 서로 다른 채널의 시작 센서 동시 감지는 허용");
  resetSystem();

  setIr(PIN_IR_ENTRY_1, true);
  setIr(PIN_IR_ENTRY_2, true);
  advance(IR_CONFIRM_MS + 40);
  check(channel1Active && channel2Active, "두 시작 센서를 동시에 처리");
}

static void testRedButtonStartsBoth() {
  beginTest("9. 빨간 버튼은 두 채널 수동 시작");
  resetSystem();
  pressButton(PIN_BTN_START);
  check(channel1Active && channel2Active, "두 채널 모두 활성");
  check(systemState == STATE_WARNING, "시스템 경고");
}

static void testGreenButtonStopsBoth() {
  beginTest("10. 초록 버튼은 두 채널 수동 종료");
  resetSystem();
  pressButton(PIN_BTN_START);
  pressButton(PIN_BTN_STOP);
  check(!channel1Active && !channel2Active, "두 채널 모두 종료");
  check(strip1Off(), "스트립1 꺼짐");
  if (STRIP_2_CONNECTED) check(strip2Off(), "스트립2 꺼짐");
  check(!ucast_stub::board.toneOn, "부저 꺼짐");
}

static void testShortNoiseIgnored() {
  beginTest("11. IR_CONFIRM_MS 미만 노이즈 무시");
  resetSystem();

  setIr(PIN_IR_ENTRY_1, true);
  advance(IR_CONFIRM_MS / 2);
  setIr(PIN_IR_ENTRY_1, false);
  advance(IR_CONFIRM_MS + 30);
  check(!channel1Active, "짧은 신호로 시작하지 않음");
}

static void testStripBlinkTogetherWithWarnLed() {
  beginTest("12. 활성 스트립과 공용 LED 300ms 동시 점멸");
  resetSystem();
  pulseIr(PIN_IR_ENTRY_1);

  bool phase0 = warningBlinkOn;
  bool led0 = warnLedOn();
  bool strip0 = strip1Red();
  check(phase0 && led0 && strip0, "시작 직후 ON 위상");

  advance(WARNING_BLINK_MS);
  check(!warningBlinkOn && !warnLedOn() && strip1Off(), "300ms 후 함께 OFF");
  check(ucast_stub::board.toneOn, "스트립 OFF 위상에도 사이렌 지속");

  advance(WARNING_BLINK_MS);
  check(warningBlinkOn && warnLedOn() && strip1Red(), "600ms 후 함께 ON");
}

static void testOnlyActiveStripBlinks() {
  beginTest("13. 비활성 채널 스트립은 점멸하지 않음");
  resetSystem();
  pulseIr(PIN_IR_ENTRY_1);
  advance(WARNING_BLINK_MS * 2);
  check(strip1Red(), "활성 스트립1 ON 위상");
  if (STRIP_2_CONNECTED) check(strip2Off(), "비활성 스트립2는 계속 OFF");
}

static void testSirenRangeAndContinuity() {
  beginTest("14. 사이렌 800~1800Hz 연속 동작");
  resetSystem();
  pulseIr(PIN_IR_ENTRY_1);

  bool alwaysOn = true;
  bool inRange = true;
  uint16_t low = SIREN_MAX_HZ;
  uint16_t high = SIREN_MIN_HZ;
  for (int i = 0; i < 120; i++) {
    advance(10);
    if (!ucast_stub::board.toneOn) alwaysOn = false;
    uint16_t f = ucast_stub::board.toneFreq;
    if (f < SIREN_MIN_HZ || f > SIREN_MAX_HZ) inRange = false;
    if (f < low) low = f;
    if (f > high) high = f;
  }
  check(alwaysOn, "경고 중 tone이 끊기지 않음");
  check(inRange, "주파수 범위 준수");
  check(low <= SIREN_MIN_HZ + 100, "최저 주파수 근처 도달");
  check(high >= SIREN_MAX_HZ - 100, "최고 주파수 근처 도달");
}

static void testSirenSurvivesOneChannelStop() {
  beginTest("15. 한 채널 종료 후 다른 채널이 남으면 사이렌 유지");
  resetSystem();
  pulseIr(PIN_IR_ENTRY_1);
  pulseIr(PIN_IR_ENTRY_2);
  pulseIr(PIN_IR_EXIT_1);
  check(channel2Active, "채널2 활성 유지");
  check(ucast_stub::board.toneOn, "사이렌 유지");
}

static void testPinAssignment() {
  beginTest("16. 이슈 #18 채널 핀 배정");
  check(PIN_IR_ENTRY_1 == 8, "ENTRY_1=D8");
  check(PIN_IR_ENTRY_2 == 9, "ENTRY_2=D9");
  check(PIN_IR_EXIT_1 == 10, "EXIT_1=D10");
  check(PIN_IR_EXIT_2 == 11, "EXIT_2=D11");
  check(PIN_STRIP_1 == 4, "STRIP_1=D4");
  check(PIN_WARN_LED == 3, "WARN_LED=D3");
  check(PIN_BUZZER == 5, "BUZZER=D5");
}

static void testSecondStripConfiguration() {
  beginTest("17. 스트립2 연결/미연결 설정");
  resetSystem();
  check(STRIP_2_CONNECTED == (PIN_STRIP_2 != 0), "연결 플래그가 핀 설정과 일치");
  if (STRIP_2_CONNECTED) {
    check(strip2.testPin() == PIN_STRIP_2, "스트립2 객체가 지정 핀 사용");
    pulseIr(PIN_IR_ENTRY_2);
    check(strip2Red(), "연결 시 채널2가 스트립2를 켬");
  } else {
    check(PIN_STRIP_2 == 0, "기본값은 미확정(0)");
  }
}

static void testTimeoutDisabledByDefault() {
  beginTest("18. main 자동 종료 기본 비활성");
  resetSystem();
  check(WARNING_TIMEOUT_MS == 0, "기본값 0");
  pulseIr(PIN_IR_ENTRY_1);
  advance(2000);
  check(channel1Active, "시간만으로 종료되지 않음");
}

static void testLogRateLimit() {
  beginTest("19. 시리얼 주기 로그 간격 제한");
  resetSystem();
  uint32_t before = ucast_stub::board.serialLines;
  advance(1200, 10);
  uint32_t lines = ucast_stub::board.serialLines - before;
  if (IR_LOG_INTERVAL_MS == 0) {
    check(lines == 0, "로그 간격 0이면 주기 로그 없음");
  } else {
    check(lines <= 4, "loop마다 로그하지 않음");
    check(lines >= 2, "주기 로그는 출력됨");
  }
}

static void testMillisOverflowSiren() {
  beginTest("20. millis 오버플로 근처에서도 사이렌 유지");
  resetSystem();
  ucast_stub::board.clockMs = 0xFFFFFF00UL;
  pulseIr(PIN_IR_ENTRY_1);
  advance(1200);
  check(channel1Active, "오버플로 뒤에도 채널 유지");
  check(ucast_stub::board.toneOn, "오버플로 뒤에도 사이렌 유지");
  check(ucast_stub::board.toneFreq >= SIREN_MIN_HZ &&
        ucast_stub::board.toneFreq <= SIREN_MAX_HZ, "주파수 범위 유지");
}

static void testHeldIrDoesNotRepeat() {
  beginTest("21. 시작 IR이 계속 감지된 상태 → 이벤트 반복 없음");
  resetSystem();

  setIr(PIN_IR_ENTRY_1, true);
  advance(2000);
  check(channel1Active, "채널1 경고가 시작되어야 함");

  setIr(PIN_IR_EXIT_1, true);
  advance(2000);
  check(!channel1Active, "끝 IR 감지로 채널1이 종료되어야 함");

  // 두 센서가 계속 감지 상태로 유지되어도 재시작/재종료가 반복되지 않아야 합니다.
  advance(5000);
  check(!channel1Active, "감지 유지 상태에서 재시작하지 않아야 함");
  check(strip1Off(), "출력이 꺼진 상태를 유지해야 함");
}

static void testLongButtonPress() {
  beginTest("22. 버튼 길게 누르기 → 한 번만 처리");
  resetSystem();

  setButton(PIN_BTN_START, true);
  advance(3000);
  check(channel1Active && channel2Active, "두 채널 모두 경고가 시작되어야 함");

  pressButton(PIN_BTN_STOP);
  check(!channel1Active && !channel2Active, "초록 버튼으로 종료되어야 함");
  advance(2000);
  check(!channel1Active && !channel2Active, "누른 채로 유지되는 빨간 버튼이 재시작하지 않아야 함");
}

static void testAllOutputsOffOnStop() {
  beginTest("23. 종료 시 모든 출력 즉시 소등");
  resetSystem();
  pulseIr(PIN_IR_ENTRY_1);
  pressButton(PIN_BTN_STOP);

  check(strip1Off(), "스트립1이 꺼져야 함");
  check(!warnLedOn(), "빨간 경고 LED가 꺼져야 함");
  check(!ucast_stub::board.toneOn, "부저가 꺼져야 함");

  advance(2000);
  check(!warnLedOn(), "대기 중 LED가 다시 켜지지 않아야 함");
  check(!ucast_stub::board.toneOn, "대기 중 부저가 다시 울리지 않아야 함");
}

static void testSirenSweepTurnsEvery500ms() {
  beginTest("24. 상승·하강이 SIREN_SWEEP_MS마다 뒤바뀜");
  resetSystem();
  pulseIr(PIN_IR_ENTRY_1);

  // 2000ms 동안이면 500ms 구간이 4번 지나가므로 방향 전환은 3~4회입니다.
  SirenScan scan = scanSiren(2000);
  check(scan.turns >= 3 && scan.turns <= 4, "0.5초마다 상승↔하강이 뒤바뀌어야 함");
}

static void testSirenStopsImmediatelyOnExit() {
  beginTest("25. 사이렌 중 끝 IR 감지 → 즉시 정지");
  resetSystem();
  pulseIr(PIN_IR_ENTRY_1);
  advance(700);  // 하강 구간 한가운데
  check(ucast_stub::board.toneOn, "종료 전에는 소리가 나고 있어야 함");

  pulseIr(PIN_IR_EXIT_1);
  check(!channel1Active, "채널1이 종료되어야 함");
  check(!ucast_stub::board.toneOn, "종료 즉시 noTone()이 되어야 함");
  check(ucast_stub::board.toneFreq == 0, "정지 후 주파수가 남아 있지 않아야 함");
}

static void testSirenStopsImmediatelyOnGreenButton() {
  beginTest("26. 사이렌 중 초록 버튼 → 즉시 정지");
  resetSystem();
  pulseIr(PIN_IR_ENTRY_2);
  advance(1300);  // 두 번째 구간 한가운데
  check(ucast_stub::board.toneOn, "종료 전에는 소리가 나고 있어야 함");

  pressButton(PIN_BTN_STOP);
  check(!channel2Active, "채널2가 종료되어야 함");
  check(!ucast_stub::board.toneOn, "종료 즉시 noTone()이 되어야 함");
}

static void testSirenConstants() {
  beginTest("27. 사이렌 상수 (test/test.ino와 같은 값)");
  check(SIREN_MIN_HZ == 800, "SIREN_MIN_HZ = 800");
  check(SIREN_MAX_HZ == 1800, "SIREN_MAX_HZ = 1800");
  check(SIREN_SWEEP_MS == 500, "SIREN_SWEEP_MS = 500");
  check(SIREN_UPDATE_MS >= 20 && SIREN_UPDATE_MS <= 30, "SIREN_UPDATE_MS는 20~30ms");
  check(SIREN_MIN_HZ < SIREN_MAX_HZ, "최저 주파수가 최고 주파수보다 낮아야 함");
}

static void testStripPixelCount() {
  beginTest("28. 스트립 픽셀 수 = 실물 10개");
  check(STRIP_1_PIXELS == 10, "STRIP_1_PIXELS는 실물 스트립 LED 개수인 10이어야 함");
  check(strip1.testPixelCount() == STRIP_1_PIXELS, "스트립 객체가 STRIP_1_PIXELS개를 가져야 함");
}

static void testLogReportsWarningState() {
  beginTest("29. 경고 중에도 주기 로그가 계속 나옴");
  resetSystem();
  pulseIr(PIN_IR_ENTRY_1);

  uint32_t before = ucast_stub::board.serialLines;
  advance(2000);
  uint32_t lines = ucast_stub::board.serialLines - before;

  if (IR_LOG_INTERVAL_MS == 0) {
    check(lines == 0, "로그를 껐으면 경고 중에도 주기 출력이 없어야 함");
  } else {
    check(lines > 0, "경고 중에도 주기 로그가 나와야 함");
  }
}

int main() {
  testChannel1Sequence();
  testChannel2Sequence();
  testWrongExitDoesNotStopOtherChannel();
  testBothChannelsIndependent();
  testIdleExitIgnored();
  testEntryRepeatDoesNotStop();
  testPairAmbiguityGuard();
  testCrossChannelSimultaneousStartAllowed();
  testRedButtonStartsBoth();
  testGreenButtonStopsBoth();
  testShortNoiseIgnored();
  testStripBlinkTogetherWithWarnLed();
  testOnlyActiveStripBlinks();
  testSirenRangeAndContinuity();
  testSirenSurvivesOneChannelStop();
  testPinAssignment();
  testSecondStripConfiguration();
  testTimeoutDisabledByDefault();
  testLogRateLimit();
  testMillisOverflowSiren();
  testHeldIrDoesNotRepeat();
  testLongButtonPress();
  testAllOutputsOffOnStop();
  testSirenSweepTurnsEvery500ms();
  testSirenStopsImmediatelyOnExit();
  testSirenStopsImmediatelyOnGreenButton();
  testSirenConstants();
  testStripPixelCount();
  testLogReportsWarningState();

  printf("\nmain.ino 테스트 %d개 실행, 실패 %d개\n", testsRun, testsFailed);
  return testsFailed == 0 ? 0 : 1;
}
