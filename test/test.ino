/* ============================================================================
   멈춰 ! — IR 센서 2개 / LED 스트립 1줄 축소판

   실제 테스트 회로에서 확인한 핀:
     시작 IR OUT → D8
     끝   IR OUT → D9
     네오픽셀 DIN → D4
     빨간 경고 LED → D3
     수동 부저 → D5
     버튼 → 현재 미연결

   GitHub 이슈 #18의 최종 의도대로 이 축소판은 "IR 한 쌍 + 스트립 한 줄"인
   채널 하나를 그대로 검증합니다.

     [대기] D8 시작 IR 감지 → 스트립(D4) + 빨간 LED(D3) 300ms 점멸,
                              부저(D5) 800~1800Hz 사이렌
     [경고] D9 끝 IR 감지   → 스트립·LED·부저 즉시 종료

   D9는 시작 센서가 아닙니다. 대기 중 D9만 감지해도 경고를 시작하지 않습니다.
   버튼은 USE_BUTTONS=false라 pinMode/digitalRead 자체를 하지 않습니다.
   ========================================================================== */

#include <Adafruit_NeoPixel.h>

const uint8_t PIN_IR_ENTRY   = 8;
const uint8_t PIN_IR_EXIT    = 9;
const uint8_t PIN_WARN_LED   = 3;
const uint8_t PIN_STRIP      = 4;
const uint8_t PIN_BUZZER     = 5;
const uint8_t PIN_BTN_START  = A0;
const uint8_t PIN_BTN_STOP   = A1;

const bool USE_BUTTONS = false;

const uint16_t STRIP_PIXELS = 10;
const uint8_t STRIP_BRIGHTNESS = 120;
const uint8_t WARN_COLOR_R = 255;
const uint8_t WARN_COLOR_G = 0;
const uint8_t WARN_COLOR_B = 0;
const uint16_t WARNING_BLINK_MS = 300;

const uint16_t SIREN_MIN_HZ    = 800;
const uint16_t SIREN_MAX_HZ    = 1800;
const uint16_t SIREN_SWEEP_MS  = 500;
const uint16_t SIREN_UPDATE_MS = 25;

const bool IR_ACTIVE_LOW = true;
const bool IR_USE_INTERNAL_PULLUP = false;
const uint16_t IR_CONFIRM_MS = 60;

#ifndef IR_LOG_INTERVAL_MS
const uint16_t IR_LOG_INTERVAL_MS = 400;
#endif

// 버튼이 없는 축소판이므로 도착 센서를 놓쳐도 10초 뒤 자동 복귀합니다.
const uint32_t WARNING_TIMEOUT_MS = 10000UL;

const bool STARTUP_SELFTEST = true;
const uint16_t SELFTEST_ON_MS = 700;
const uint16_t SELFTEST_END_MS = 1400;

const bool BUTTON_ACTIVE_LOW = true;
const uint16_t BUTTON_DEBOUNCE_MS = 40;
const bool SERIAL_DEBUG = true;
const long SERIAL_BAUD = 9600;

Adafruit_NeoPixel strip(STRIP_PIXELS, PIN_STRIP, NEO_GRB + NEO_KHZ800);

enum SystemState {
  STATE_IDLE,
  STATE_WARNING
};

SystemState systemState = STATE_IDLE;
uint32_t warningTimeoutAt = 0;

struct HoldFilter {
  uint8_t pin;
  bool stable;
  bool lastRaw;
  uint32_t changedAt;
};

HoldFilter irEntry = {PIN_IR_ENTRY, false, false, 0};
HoldFilter irExit  = {PIN_IR_EXIT,  false, false, 0};

struct Button {
  uint8_t pin;
  bool stable;
  bool lastRaw;
  uint32_t changedAt;
};

Button btnStart = {PIN_BTN_START, false, false, 0};
Button btnStop  = {PIN_BTN_STOP, false, false, 0};

bool warningBlinkOn = false;
uint32_t warningBlinkChangedAt = 0;

bool buzzerOn = false;
bool sirenRising = true;
uint16_t sirenHz = 0;
uint32_t sirenSweepStartedAt = 0;
uint32_t sirenUpdatedAt = 0;

bool selftestDone = false;
uint32_t irLoggedAt = 0;

void logLine(const __FlashStringHelper *message) {
  if (SERIAL_DEBUG) Serial.println(message);
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
  if (!USE_BUTTONS) return false;

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

void applyStrip(bool on) {
  uint32_t color = on ? strip.Color(WARN_COLOR_R, WARN_COLOR_G, WARN_COLOR_B) : 0;
  for (uint16_t i = 0; i < STRIP_PIXELS; i++) strip.setPixelColor(i, color);
  strip.show();
}

void applyBlinkOutputs(bool on) {
  warningBlinkOn = on;
  digitalWrite(PIN_WARN_LED, on ? HIGH : LOW);
  applyStrip(on);
}

uint16_t sirenFrequencyAt(uint32_t elapsed) {
  if (elapsed > SIREN_SWEEP_MS) elapsed = SIREN_SWEEP_MS;
  uint32_t span = (uint32_t)SIREN_MAX_HZ - (uint32_t)SIREN_MIN_HZ;
  uint32_t moved = (span * elapsed) / SIREN_SWEEP_MS;
  return sirenRising ? (uint16_t)(SIREN_MIN_HZ + moved)
                     : (uint16_t)(SIREN_MAX_HZ - moved);
}

void startBuzzer(uint32_t now) {
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
  warningBlinkOn = false;
  digitalWrite(PIN_WARN_LED, LOW);
  applyStrip(false);
  stopBuzzer();
}

void enterWarning(uint32_t now, const __FlashStringHelper *reason) {
  systemState = STATE_WARNING;
  warningBlinkChangedAt = now;
  warningBlinkOn = true;
  warningTimeoutAt = now + WARNING_TIMEOUT_MS;
  if (warningTimeoutAt == 0) warningTimeoutAt = 1;

  applyBlinkOutputs(true);
  startBuzzer(now);

  if (SERIAL_DEBUG) {
    Serial.print(F("상태: 경고 시작 - "));
    Serial.println(reason);
  }
}

void enterIdle(const __FlashStringHelper *reason) {
  systemState = STATE_IDLE;
  warningTimeoutAt = 0;
  allOutputsOff();

  if (SERIAL_DEBUG) {
    Serial.print(F("상태: 대기 복귀 - "));
    Serial.println(reason);
  }
}

bool warningTimedOut(uint32_t now) {
  if (warningTimeoutAt == 0) return false;
  return ((int32_t)(now - warningTimeoutAt) >= 0);
}

void updateBlinkOutputs(uint32_t now) {
  if (systemState != STATE_WARNING) return;
  if ((now - warningBlinkChangedAt) < WARNING_BLINK_MS) return;
  warningBlinkChangedAt = now;
  applyBlinkOutputs(!warningBlinkOn);
}

void logIrSensor(const __FlashStringHelper *label, const HoldFilter &filter) {
  Serial.print(label);
  Serial.print(digitalRead(filter.pin));
  Serial.print(filter.stable ? F(" 감지  ") : F(" 없음  "));
}

void logIrValues(uint32_t now) {
  if (!SERIAL_DEBUG || IR_LOG_INTERVAL_MS == 0) return;
  if ((now - irLoggedAt) < IR_LOG_INTERVAL_MS) return;
  irLoggedAt = now;

  logIrSensor(F("ENTRY(D8)="), irEntry);
  logIrSensor(F("EXIT(D9)="), irExit);
  Serial.print(systemState == STATE_WARNING ? F("STATE_WARNING ") : F("STATE_IDLE "));
  Serial.print((systemState == STATE_WARNING && warningBlinkOn) ? F("STRIP=RED ") : F("STRIP=OFF "));
  Serial.print(warningBlinkOn ? F("LED=ON ") : F("LED=OFF "));
  Serial.print(F("BUZZER="));
  if (buzzerOn) {
    Serial.print((int)sirenHz);
    Serial.println(F("Hz"));
  } else {
    Serial.println(F("OFF"));
  }
}

bool runSelftest(uint32_t now) {
  if (!STARTUP_SELFTEST || selftestDone) return false;

  if (now < SELFTEST_ON_MS) {
    applyBlinkOutputs(true);
    if (!buzzerOn) startBuzzer(now);
    updateBuzzerSiren(now);
    return true;
  }

  if (now < SELFTEST_END_MS) {
    allOutputsOff();
    return true;
  }

  selftestDone = true;
  allOutputsOff();
  logLine(F("자가진단 완료 - 지금부터 시작/끝 IR을 읽습니다"));
  return false;
}

void setup() {
  uint8_t irMode = IR_USE_INTERNAL_PULLUP ? INPUT_PULLUP : INPUT;
  pinMode(PIN_IR_ENTRY, irMode);
  pinMode(PIN_IR_EXIT, irMode);

  if (USE_BUTTONS) {
    pinMode(PIN_BTN_START, INPUT_PULLUP);
    pinMode(PIN_BTN_STOP, INPUT_PULLUP);
  }

  pinMode(PIN_WARN_LED, OUTPUT);
  pinMode(PIN_BUZZER, OUTPUT);

  strip.begin();
  strip.setBrightness(STRIP_BRIGHTNESS);
  allOutputsOff();

  if (SERIAL_DEBUG) {
    Serial.begin(SERIAL_BAUD);
    Serial.println(F("멈춰 ! - 축소판 시작"));
    Serial.println(F("시작 IR=D8, 끝 IR=D9, LED=D3, 스트립=D4, 부저=D5"));
    Serial.println(F("D8 감지→경고 시작 / D9 감지→경고 종료"));
    if (!USE_BUTTONS) Serial.println(F("버튼 미사용 - A0/A1을 읽지 않습니다"));
  }
}

void loop() {
  uint32_t now = millis();
  if (runSelftest(now)) return;

  bool entryEvent = detectedEdge(irEntry, now);
  bool exitEvent = detectedEdge(irExit, now);

  logIrValues(now);

  bool redButtonPressed = buttonPressedEdge(btnStart, now);
  bool greenButtonPressed = buttonPressedEdge(btnStop, now);

  if (systemState == STATE_IDLE) {
    if (redButtonPressed) {
      enterWarning(now, F("빨간 버튼 수동 시작"));
    } else if (entryEvent && exitEvent) {
      logLine(F("입력 무시: 시작·끝 센서 동시 감지"));
    } else if (entryEvent) {
      if (irExit.stable) {
        logLine(F("입력 무시: 끝 센서가 이미 감지 중"));
      } else {
        enterWarning(now, F("시작 IR 감지"));
      }
    }
    // 대기 중 끝 IR 단독 감지는 무시합니다.
  } else {
    if (exitEvent) {
      enterIdle(F("끝 IR 감지 - 통과 완료"));
      return;
    }
    if (greenButtonPressed) {
      enterIdle(F("초록 버튼 수동 종료"));
      return;
    }
    if (entryEvent) logLine(F("경고 유지: 시작 IR 재감지"));
    if (redButtonPressed) logLine(F("경고 유지: 빨간 버튼 재입력"));

    if (warningTimedOut(now)) {
      enterIdle(F("제한시간 경과 - 자동 복귀"));
      return;
    }

    updateBlinkOutputs(now);
    updateBuzzerSiren(now);
  }
}
