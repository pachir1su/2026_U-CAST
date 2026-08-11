/* ============================================================================
   멈춰 ! — 돌발 보행자 감지 및 경고 시스템

   천안시 청소년 도시재생 챌린지 (U-CAST) / 5팀 「건영아 잘하자」

   [최종 실물 스케치]
   IR 센서 4개를 "시작/끝 센서 1쌍 + LED 스트립 1줄"의 독립 채널 2개로
   사용합니다. 이 구조는 GitHub 이슈 #18에서 확정한 의도를 반영합니다.

     채널 1: 시작 IR 1(D8)  → LED 스트립 1(D4) 점멸 시작
             끝   IR 1(D10) → LED 스트립 1 점멸 종료

     채널 2: 시작 IR 2(D9)  → LED 스트립 2(PIN_STRIP_2) 점멸 시작
             끝   IR 2(D11) → LED 스트립 2 점멸 종료

   두 채널은 서로 독립적입니다. 예를 들어 채널 1이 경고 중이어도 채널 2는 별도로
   시작·종료할 수 있습니다. 나중에 센서/스트립 세트를 늘릴 때도 같은 구조를 반복합니다.

   빨간 경고 LED와 부저는 현재 실물에서 각각 1개뿐인 공용 출력입니다.
   따라서 채널 1 또는 채널 2 중 하나라도 경고 중이면 공용 LED와 사이렌이 작동하고,
   두 채널이 모두 종료되어야 공용 출력도 꺼집니다.

   빨간 버튼은 두 채널을 모두 수동 시작하고, 초록 버튼은 두 채널을 모두 수동 종료합니다.
   `delay()`는 사용하지 않습니다. 센서·버튼·점멸·사이렌은 모두 millis() 기반입니다.

   두 번째 스트립의 실제 DIN 핀이 아직 확정되지 않았으므로 PIN_STRIP_2 기본값은 0입니다.
   최종 배선이 확정되면 [설정 1]의 PIN_STRIP_2_VALUE만 실제 핀 번호로 바꾸세요.
   ========================================================================== */

#include <Adafruit_NeoPixel.h>

/* ---------------------------------------------------------------------------
   [설정 1] 핀 번호
   --------------------------------------------------------------------------- */
const uint8_t PIN_IR_ENTRY_1 = 8;   // 채널 1 시작 IR
const uint8_t PIN_IR_ENTRY_2 = 9;   // 채널 2 시작 IR
const uint8_t PIN_IR_EXIT_1  = 10;  // 채널 1 끝 IR
const uint8_t PIN_IR_EXIT_2  = 11;  // 채널 2 끝 IR

const uint8_t PIN_WARN_LED   = 3;   // 공용 빨간 경고 LED
const uint8_t PIN_STRIP_1    = 4;   // 채널 1 네오픽셀 DIN
const uint8_t PIN_BUZZER     = 5;   // 공용 수동 부저
const uint8_t PIN_STRIP_2    = 6;   // 채널 2 네오픽셀 DIN
const uint8_t PIN_BTN_START  = A0;  // 빨간 버튼: 두 채널 수동 시작
const uint8_t PIN_BTN_STOP   = A1;  // 초록 버튼: 두 채널 수동 종료

// 실물 핀이 확정되면 0 대신 실제 핀을 적습니다.
// PC 회귀 테스트는 -DPIN_STRIP_2_VALUE=6 같은 방식으로 2번 스트립 연결 상태도 검증합니다.
#ifndef PIN_STRIP_2_VALUE
#define PIN_STRIP_2_VALUE 0
#endif
const uint8_t PIN_STRIP_2 = PIN_STRIP_2_VALUE;
const bool STRIP_2_CONNECTED = (PIN_STRIP_2 != 0);

/* ---------------------------------------------------------------------------
   [설정 2] 네오픽셀
   --------------------------------------------------------------------------- */
const uint16_t STRIP_1_PIXELS = 10;
const uint16_t STRIP_2_PIXELS = 10;
const uint8_t STRIP_BRIGHTNESS = 120;

const uint8_t WARN_COLOR_R = 255;
const uint8_t WARN_COLOR_G = 0;
const uint8_t WARN_COLOR_B = 0;

// 각 활성 채널의 스트립과 공용 빨간 LED가 함께 켜짐↔꺼짐 전환되는 간격입니다.
const uint16_t WARNING_BLINK_MS = 300;

/* ---------------------------------------------------------------------------
   [설정 3] 수동 부저 사이렌
   --------------------------------------------------------------------------- */
const uint16_t SIREN_MIN_HZ    = 800;
const uint16_t SIREN_MAX_HZ    = 1800;
const uint16_t SIREN_SWEEP_MS  = 500;
const uint16_t SIREN_UPDATE_MS = 25;

/* ---------------------------------------------------------------------------
   [설정 4] IR 센서
   --------------------------------------------------------------------------- */
const bool IR_ACTIVE_LOW = true;
const bool IR_USE_INTERNAL_PULLUP = false;
const uint16_t IR_CONFIRM_MS = 60;

// 같은 채널의 시작·끝 센서가 동시에 잡히거나 끝 센서가 이미 감지 중이면
// 시작하지 않습니다. 서로 다른 채널의 동시 감지는 정상이며 각각 독립 처리합니다.
const bool IR_PAIR_AMBIGUITY_GUARD = true;

#ifndef IR_LOG_INTERVAL_MS
const uint16_t IR_LOG_INTERVAL_MS = 400;
#endif

/* ---------------------------------------------------------------------------
   [설정 5] 안전 복귀
   0이면 자동 종료를 사용하지 않습니다. 값이 있으면 채널별로 독립 적용됩니다.
   --------------------------------------------------------------------------- */
const uint32_t WARNING_TIMEOUT_MS = 0UL;

/* ---------------------------------------------------------------------------
   [설정 6] 버튼·시리얼
   --------------------------------------------------------------------------- */
const bool BUTTON_ACTIVE_LOW = true;
const uint16_t BUTTON_DEBOUNCE_MS = 40;
const bool SERIAL_DEBUG = true;
const long SERIAL_BAUD = 9600;

/* ===========================================================================
   상태와 입력 필터
   =========================================================================== */

enum SystemState {
  STATE_IDLE,
  STATE_WARNING
};

SystemState systemState = STATE_IDLE;
bool channel1Active = false;
bool channel2Active = false;
uint32_t channel1TimeoutAt = 0;
uint32_t channel2TimeoutAt = 0;

struct HoldFilter {
  uint8_t pin;
  bool stable;
  bool lastRaw;
  uint32_t changedAt;
};

HoldFilter irEntry1 = {PIN_IR_ENTRY_1, false, false, 0};
HoldFilter irEntry2 = {PIN_IR_ENTRY_2, false, false, 0};
HoldFilter irExit1  = {PIN_IR_EXIT_1,  false, false, 0};
HoldFilter irExit2  = {PIN_IR_EXIT_2,  false, false, 0};

struct Button {
  uint8_t pin;
  bool stable;
  bool lastRaw;
  uint32_t changedAt;
};

Button btnStart = {PIN_BTN_START, false, false, 0};
Button btnStop  = {PIN_BTN_STOP,  false, false, 0};

bool warningBlinkOn = false;
uint32_t warningBlinkChangedAt = 0;

bool buzzerOn = false;
bool sirenRising = true;
uint16_t sirenHz = 0;
uint32_t sirenSweepStartedAt = 0;
uint32_t sirenUpdatedAt = 0;

uint32_t irLoggedAt = 0;

Adafruit_NeoPixel strip1(STRIP_1_PIXELS, PIN_STRIP_1, NEO_GRB + NEO_KHZ800);
Adafruit_NeoPixel strip2(STRIP_2_PIXELS, PIN_STRIP_2, NEO_GRB + NEO_KHZ800);

void logLine(const __FlashStringHelper *message) {
  if (SERIAL_DEBUG) Serial.println(message);
}

bool anyChannelActive() {
  return channel1Active || channel2Active;
}

void updateSystemState() {
  systemState = anyChannelActive() ? STATE_WARNING : STATE_IDLE;
}

bool readIrDetected(uint8_t pin) {
  bool low = (digitalRead(pin) == LOW);
  return IR_ACTIVE_LOW ? low : !low;
}

bool detectedEdge(HoldFilter &filter, uint32_t now) {
  bool raw = readIrDetected(filter.pin);

  if (raw != filter.lastRaw) {
    filter.lastRaw = raw;
    filter.changedAt = now;
  }

  if (raw != filter.stable && (now - filter.changedAt) >= IR_CONFIRM_MS) {
    bool previous = filter.stable;
    filter.stable = raw;
    return (!previous && filter.stable);
  }

  return false;
}

bool buttonPressedEdge(Button &button, uint32_t now) {
  bool raw = (digitalRead(button.pin) == LOW);
  if (!BUTTON_ACTIVE_LOW) raw = !raw;

  if (raw != button.lastRaw) {
    button.lastRaw = raw;
    button.changedAt = now;
    return false;
  }

  if (raw != button.stable && (now - button.changedAt) >= BUTTON_DEBOUNCE_MS) {
    button.stable = raw;
    return raw;
  }

  return false;
}

/* ===========================================================================
   출력 제어
   =========================================================================== */

void applyStrip1(bool on) {
  uint32_t color = on ? strip1.Color(WARN_COLOR_R, WARN_COLOR_G, WARN_COLOR_B) : 0;
  for (uint16_t i = 0; i < STRIP_1_PIXELS; i++) strip1.setPixelColor(i, color);
  strip1.show();
}

void applyStrip2(bool on) {
  if (!STRIP_2_CONNECTED) return;
  uint32_t color = on ? strip2.Color(WARN_COLOR_R, WARN_COLOR_G, WARN_COLOR_B) : 0;
  for (uint16_t i = 0; i < STRIP_2_PIXELS; i++) strip2.setPixelColor(i, color);
  strip2.show();
}

void applyActiveChannelStrips(bool phaseOn) {
  applyStrip1(channel1Active && phaseOn);
  applyStrip2(channel2Active && phaseOn);
}

void applyWarnLed(bool phaseOn) {
  warningBlinkOn = phaseOn;
  digitalWrite(PIN_WARN_LED, (anyChannelActive() && phaseOn) ? HIGH : LOW);
}

uint16_t sirenFrequencyAt(uint32_t elapsed) {
  if (elapsed > SIREN_SWEEP_MS) elapsed = SIREN_SWEEP_MS;
  uint32_t span = (uint32_t)SIREN_MAX_HZ - (uint32_t)SIREN_MIN_HZ;
  uint32_t moved = (span * elapsed) / SIREN_SWEEP_MS;
  return sirenRising ? (uint16_t)(SIREN_MIN_HZ + moved)
                     : (uint16_t)(SIREN_MAX_HZ - moved);
}

void startBuzzer(uint32_t now) {
  if (buzzerOn) return;
  buzzerOn = true;
  sirenRising = true;
  sirenSweepStartedAt = now;
  sirenUpdatedAt = now;
  sirenHz = SIREN_MIN_HZ;
  tone(PIN_BUZZER, sirenHz);
}

void stopBuzzer() {
  buzzerOn = false;
  sirenHz = 0;
  noTone(PIN_BUZZER);
}

void updateBuzzerSiren(uint32_t now) {
  if (!buzzerOn) return;
  if ((now - sirenUpdatedAt) < SIREN_UPDATE_MS) return;
  sirenUpdatedAt = now;

  while ((now - sirenSweepStartedAt) >= SIREN_SWEEP_MS) {
    sirenSweepStartedAt += SIREN_SWEEP_MS;
    sirenRising = !sirenRising;
  }

  sirenHz = sirenFrequencyAt(now - sirenSweepStartedAt);
  tone(PIN_BUZZER, sirenHz);
}

void allOutputsOff() {
  channel1Active = false;
  channel2Active = false;
  channel1TimeoutAt = 0;
  channel2TimeoutAt = 0;
  updateSystemState();

  warningBlinkOn = false;
  digitalWrite(PIN_WARN_LED, LOW);
  applyStrip1(false);
  applyStrip2(false);
  stopBuzzer();
}

/* ===========================================================================
   채널별 상태 전이
   =========================================================================== */

uint32_t makeTimeoutAt(uint32_t now) {
  if (WARNING_TIMEOUT_MS == 0) return 0;
  uint32_t timeoutAt = now + WARNING_TIMEOUT_MS;
  return (timeoutAt == 0) ? 1 : timeoutAt;
}

bool warningTimedOut(uint32_t timeoutAt, uint32_t now) {
  if (timeoutAt == 0) return false;
  return ((int32_t)(now - timeoutAt) >= 0);
}

void startChannel1(uint32_t now, const __FlashStringHelper *reason) {
  if (channel1Active) return;
  bool hadActiveChannel = anyChannelActive();

  channel1Active = true;
  channel1TimeoutAt = makeTimeoutAt(now);
  updateSystemState();

  if (!hadActiveChannel) {
    warningBlinkOn = true;
    warningBlinkChangedAt = now;
    applyWarnLed(true);
    startBuzzer(now);
  }
  applyStrip1(warningBlinkOn);

  if (SERIAL_DEBUG) {
    Serial.print(F("채널1 시작 - "));
    Serial.println(reason);
  }
}

void startChannel2(uint32_t now, const __FlashStringHelper *reason) {
  if (channel2Active) return;
  bool hadActiveChannel = anyChannelActive();

  channel2Active = true;
  channel2TimeoutAt = makeTimeoutAt(now);
  updateSystemState();

  if (!hadActiveChannel) {
    warningBlinkOn = true;
    warningBlinkChangedAt = now;
    applyWarnLed(true);
    startBuzzer(now);
  }
  applyStrip2(warningBlinkOn);

  if (SERIAL_DEBUG) {
    Serial.print(F("채널2 시작 - "));
    Serial.println(reason);
  }
}

void stopChannel1(const __FlashStringHelper *reason) {
  if (!channel1Active) return;
  channel1Active = false;
  channel1TimeoutAt = 0;
  applyStrip1(false);
  updateSystemState();

  if (SERIAL_DEBUG) {
    Serial.print(F("채널1 종료 - "));
    Serial.println(reason);
  }

  if (!anyChannelActive()) {
    warningBlinkOn = false;
    digitalWrite(PIN_WARN_LED, LOW);
    stopBuzzer();
  }
}

void stopChannel2(const __FlashStringHelper *reason) {
  if (!channel2Active) return;
  channel2Active = false;
  channel2TimeoutAt = 0;
  applyStrip2(false);
  updateSystemState();

  if (SERIAL_DEBUG) {
    Serial.print(F("채널2 종료 - "));
    Serial.println(reason);
  }

  if (!anyChannelActive()) {
    warningBlinkOn = false;
    digitalWrite(PIN_WARN_LED, LOW);
    stopBuzzer();
  }
}

void startAllChannels(uint32_t now) {
  startChannel1(now, F("빨간 버튼 수동 시작"));
  startChannel2(now, F("빨간 버튼 수동 시작"));
}

void stopAllChannels(const __FlashStringHelper *reason) {
  if (channel1Active) stopChannel1(reason);
  if (channel2Active) stopChannel2(reason);
}

void updateBlinkOutputs(uint32_t now) {
  if (!anyChannelActive()) return;
  if ((now - warningBlinkChangedAt) < WARNING_BLINK_MS) return;

  warningBlinkChangedAt = now;
  bool nextPhase = !warningBlinkOn;
  applyWarnLed(nextPhase);
  applyActiveChannelStrips(nextPhase);
}

/* ===========================================================================
   진단 로그
   =========================================================================== */

void logIrSensor(const __FlashStringHelper *label, const HoldFilter &filter) {
  Serial.print(label);
  Serial.print(digitalRead(filter.pin));
  Serial.print(filter.stable ? F(" 감지  ") : F(" 없음  "));
}

void logIrValues(uint32_t now) {
  if (!SERIAL_DEBUG || IR_LOG_INTERVAL_MS == 0) return;
  if ((now - irLoggedAt) < IR_LOG_INTERVAL_MS) return;
  irLoggedAt = now;

  logIrSensor(F("E1(D8)="), irEntry1);
  logIrSensor(F("E2(D9)="), irEntry2);
  logIrSensor(F("X1(D10)="), irExit1);
  logIrSensor(F("X2(D11)="), irExit2);

  Serial.print(channel1Active ? F("CH1=WARNING ") : F("CH1=IDLE "));
  Serial.print(channel2Active ? F("CH2=WARNING ") : F("CH2=IDLE "));
  Serial.print(systemState == STATE_WARNING ? F("SYSTEM=WARNING ") : F("SYSTEM=IDLE "));

  Serial.print((channel1Active && warningBlinkOn) ? F("S1=RED ") : F("S1=OFF "));
  if (!STRIP_2_CONNECTED) {
    Serial.print(F("S2=NC "));
  } else {
    Serial.print((channel2Active && warningBlinkOn) ? F("S2=RED ") : F("S2=OFF "));
  }

  Serial.print(warningBlinkOn && anyChannelActive() ? F("LED=ON ") : F("LED=OFF "));
  Serial.print(F("BUZZER="));
  if (buzzerOn) {
    Serial.print((int)sirenHz);
    Serial.println(F("Hz"));
  } else {
    Serial.println(F("OFF"));
  }
}

/* ===========================================================================
   Arduino 진입점
   =========================================================================== */

void setup() {
  uint8_t irMode = IR_USE_INTERNAL_PULLUP ? INPUT_PULLUP : INPUT;
  pinMode(PIN_IR_ENTRY_1, irMode);
  pinMode(PIN_IR_ENTRY_2, irMode);
  pinMode(PIN_IR_EXIT_1, irMode);
  pinMode(PIN_IR_EXIT_2, irMode);

  pinMode(PIN_BTN_START, INPUT_PULLUP);
  pinMode(PIN_BTN_STOP, INPUT_PULLUP);
  pinMode(PIN_WARN_LED, OUTPUT);
  pinMode(PIN_BUZZER, OUTPUT);

  strip1.begin();
  strip1.setBrightness(STRIP_BRIGHTNESS);
  if (STRIP_2_CONNECTED) {
    strip2.begin();
    strip2.setBrightness(STRIP_BRIGHTNESS);
  }

  allOutputsOff();

  if (SERIAL_DEBUG) {
    Serial.begin(SERIAL_BAUD);
    Serial.println(F("멈춰 ! - 돌발 보행자 감지 및 경고 시스템 시작"));
    Serial.println(F("채널1: 시작 D8 / 끝 D10 / 스트립1 D4"));
    Serial.println(F("채널2: 시작 D9 / 끝 D11 / 스트립2 PIN_STRIP_2"));
    Serial.println(F("각 채널은 독립적으로 시작·종료됩니다 (#18)"));
    if (!STRIP_2_CONNECTED) {
      Serial.println(F("주의: PIN_STRIP_2=0, 채널2 센서는 동작하지만 스트립2 출력은 건너뜁니다"));
    }
  }
}

void loop() {
  uint32_t now = millis();

  // 모든 필터는 매 loop마다 갱신해야 한 센서가 true일 때 다른 센서 상태가 멈추지 않습니다.
  bool entry1Event = detectedEdge(irEntry1, now);
  bool entry2Event = detectedEdge(irEntry2, now);
  bool exit1Event  = detectedEdge(irExit1, now);
  bool exit2Event  = detectedEdge(irExit2, now);

  logIrValues(now);

  bool redButtonPressed = buttonPressedEdge(btnStart, now);
  bool greenButtonPressed = buttonPressedEdge(btnStop, now);

  // 공용 초록 버튼은 활성 채널 전체를 우선 종료합니다.
  if (greenButtonPressed && anyChannelActive()) {
    stopAllChannels(F("초록 버튼 수동 종료"));
  } else if (redButtonPressed) {
    // 공용 빨간 버튼은 두 채널을 모두 켜는 발표용 강제 시작입니다.
    startAllChannels(now);
  } else {
    // 채널 1: ENTRY_1 → STRIP_1 시작, EXIT_1 → STRIP_1 종료
    if (!channel1Active) {
      if (entry1Event && exit1Event) {
        logLine(F("채널1 입력 무시: 시작·끝 센서 동시 감지"));
      } else if (entry1Event) {
        if (IR_PAIR_AMBIGUITY_GUARD && irExit1.stable) {
          logLine(F("채널1 입력 무시: 끝 센서가 이미 감지 중"));
        } else {
          startChannel1(now, F("시작 IR 1 감지"));
        }
      }
      // 대기 중 EXIT_1 단독 이벤트는 의도적으로 무시합니다.
    } else {
      if (exit1Event) {
        stopChannel1(F("끝 IR 1 감지"));
      } else if (entry1Event) {
        logLine(F("채널1 유지: 시작 IR 1 재감지"));
      }
    }

    // 채널 2: ENTRY_2 → STRIP_2 시작, EXIT_2 → STRIP_2 종료
    if (!channel2Active) {
      if (entry2Event && exit2Event) {
        logLine(F("채널2 입력 무시: 시작·끝 센서 동시 감지"));
      } else if (entry2Event) {
        if (IR_PAIR_AMBIGUITY_GUARD && irExit2.stable) {
          logLine(F("채널2 입력 무시: 끝 센서가 이미 감지 중"));
        } else {
          startChannel2(now, F("시작 IR 2 감지"));
        }
      }
      // 대기 중 EXIT_2 단독 이벤트는 의도적으로 무시합니다.
    } else {
      if (exit2Event) {
        stopChannel2(F("끝 IR 2 감지"));
      } else if (entry2Event) {
        logLine(F("채널2 유지: 시작 IR 2 재감지"));
      }
    }
  }

  if (channel1Active && warningTimedOut(channel1TimeoutAt, now)) {
    stopChannel1(F("채널1 제한시간 경과"));
  }
  if (channel2Active && warningTimedOut(channel2TimeoutAt, now)) {
    stopChannel2(F("채널2 제한시간 경과"));
  }

  updateBlinkOutputs(now);
  updateBuzzerSiren(now);
}
