/* ============================================================================
   무단횡단 방지 시스템 (Jaywalking Warning System)

   천안시 청소년 도시재생 챌린지 (U-CAST) / 5팀 「건영아 잘하자」

   [최종 MVP 동작]
   1) 왼쪽·오른쪽 인도에 IR 브레이크빔 센서를 1세트씩 설치한다.
      각 세트는 송신부 1개와 수신부 1개로 구성되고, 수신부 OUT만 아두이노가 읽는다.
   2) 대기 상태에서 왼쪽 빔이 먼저 끊기면 왼쪽→오른쪽 횡단으로,
      오른쪽 빔이 먼저 끊기면 오른쪽→왼쪽 횡단으로 판단한다.
   3) 횡단 중에는 차도 위 네오픽셀 스트립 2개와 「보행자 주의」 표지판 내부
      빨간 LED가 점멸하고, 부저가 보행자에게 경고한다.
   4) 반대편 인도의 빔이 끊기면 도착으로 판단하고 모든 출력을 끈다.
   5) 빨간 버튼은 왼쪽 IR 감지, 초록 버튼은 오른쪽 IR 감지를 모의한다.
      버튼 색은 시작/종료를 뜻하지 않는다.

   [이번 MVP에서 제외]
   - 횡단보도 방향 유도
   - 초음파 센서
   - 펜스 / 압력 센서(FSR)

   ※ 조정값은 아래 [설정] 구역의 상수만 수정합니다.
   ========================================================================== */

#include <Adafruit_NeoPixel.h>

/* ---------------------------------------------------------------------------
   [설정 1] 핀 번호
   --------------------------------------------------------------------------- */
const uint8_t PIN_IR_LEFT       = 2;   // 왼쪽 IR 브레이크빔 수신부 OUT
const uint8_t PIN_IR_RIGHT      = 3;   // 오른쪽 IR 브레이크빔 수신부 OUT
const uint8_t PIN_STRIP_1       = 6;   // 차도 경고 스트립 1 DIN
const uint8_t PIN_STRIP_2       = 7;   // 차도 경고 스트립 2 DIN
const uint8_t PIN_SIGN_LED      = 8;   // 「보행자 주의」 표지판 내부 빨간 LED
const uint8_t PIN_BUZZER        = 9;   // 보행자 경고용 수동 부저
const uint8_t PIN_BTN_LEFT      = 10;  // 빨간 버튼: 왼쪽 IR 감지 모의
const uint8_t PIN_BTN_RIGHT     = 11;  // 초록 버튼: 오른쪽 IR 감지 모의

/* ---------------------------------------------------------------------------
   [설정 2] 네오픽셀 스트립
   두 스트립 모두 차도 위 경고용이며 방향 유도에는 사용하지 않습니다.
   --------------------------------------------------------------------------- */
const uint16_t STRIP1_PIXELS = 10;  // 실물 LED 개수로 수정
const uint16_t STRIP2_PIXELS = 10;  // 실물 LED 개수로 수정

const uint8_t STRIP_BRIGHTNESS = 120;  // 0~255
const uint8_t WARN_COLOR_R = 255;
const uint8_t WARN_COLOR_G = 0;
const uint8_t WARN_COLOR_B = 0;

// 도로 스트립과 표지판을 동시에 켰다 끄는 점멸 간격
const uint16_t WARNING_LIGHT_BLINK_MS = 300;

/* ---------------------------------------------------------------------------
   [설정 3] IR 브레이크빔 판정
   --------------------------------------------------------------------------- */
// 대부분의 브레이크빔 수신부는 빔이 끊기면 LOW를 출력합니다.
// 실물에서 반대로 동작하면 false로 변경합니다.
const bool IR_ACTIVE_LOW = true;

// 수신부 OUT가 오픈 컬렉터 방식이면 내부 풀업이 필요할 수 있습니다.
const bool IR_USE_INTERNAL_PULLUP = true;

// 신호가 이 시간 이상 유지되어야 빔 차단으로 확정합니다.
const uint16_t IR_CONFIRM_MS = 80;

/* ---------------------------------------------------------------------------
   [설정 4] 상태 전환과 안전 복귀
   --------------------------------------------------------------------------- */
// 반대편 빔이 너무 빨리 들어온 경우 오감지로 보고 종료하지 않습니다.
// 모형 크기에 맞춰 줄이거나 늘릴 수 있습니다.
const uint16_t CROSSING_MIN_MS = 800;

// 도착 신호가 없을 때 자동으로 대기 상태로 돌아가는 시간입니다.
// 0이면 자동 복귀하지 않습니다.
const uint32_t CROSSING_TIMEOUT_MS = 60000UL;

/* ---------------------------------------------------------------------------
   [설정 5] 부저
   --------------------------------------------------------------------------- */
const uint16_t BUZZER_FREQ_HZ = 2000;
const uint16_t BUZZER_ON_MS   = 180;
const uint16_t BUZZER_OFF_MS  = 320;

/* ---------------------------------------------------------------------------
   [설정 6] 버튼·시리얼
   --------------------------------------------------------------------------- */
const bool BUTTON_ACTIVE_LOW = true;
const uint16_t BUTTON_DEBOUNCE_MS = 40;
const bool SERIAL_DEBUG = true;
const long SERIAL_BAUD = 9600;

/* ===========================================================================
   아래는 동작 로직입니다.
   =========================================================================== */

Adafruit_NeoPixel strip1(STRIP1_PIXELS, PIN_STRIP_1, NEO_GRB + NEO_KHZ800);
Adafruit_NeoPixel strip2(STRIP2_PIXELS, PIN_STRIP_2, NEO_GRB + NEO_KHZ800);

enum SystemState {
  STATE_IDLE,
  STATE_LEFT_TO_RIGHT,
  STATE_RIGHT_TO_LEFT
};

SystemState systemState = STATE_IDLE;
uint32_t crossingStartedAt = 0;
bool pendingArrival = false;

struct HoldFilter {
  bool stable;
  bool lastRaw;
  uint32_t changedAt;
};

HoldFilter leftBeamFilter  = {false, false, 0};
HoldFilter rightBeamFilter = {false, false, 0};

struct Button {
  uint8_t pin;
  bool stable;
  bool lastRaw;
  uint32_t changedAt;
};

Button btnLeft  = {PIN_BTN_LEFT,  false, false, 0};
Button btnRight = {PIN_BTN_RIGHT, false, false, 0};

bool warningLightsOn = false;
bool buzzerOn = false;
uint32_t warningLightsChangedAt = 0;
uint32_t buzzerChangedAt = 0;

void logLine(const __FlashStringHelper *message) {
  if (SERIAL_DEBUG) Serial.println(message);
}

bool readBeamBlocked(uint8_t pin) {
  bool low = (digitalRead(pin) == LOW);
  return IR_ACTIVE_LOW ? low : !low;
}

// 안정화된 상태가 false→true로 바뀌는 순간에만 true를 반환합니다.
bool blockedEdge(HoldFilter &filter, bool rawBlocked, uint16_t confirmMs, uint32_t now) {
  if (rawBlocked != filter.lastRaw) {
    filter.lastRaw = rawBlocked;
    filter.changedAt = now;
  }

  if (rawBlocked != filter.stable && (now - filter.changedAt) >= confirmMs) {
    bool previous = filter.stable;
    filter.stable = rawBlocked;
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

void setStripColor(Adafruit_NeoPixel &strip, uint16_t pixels, bool on) {
  uint32_t color = on ? strip.Color(WARN_COLOR_R, WARN_COLOR_G, WARN_COLOR_B) : 0;
  for (uint16_t i = 0; i < pixels; i++) {
    strip.setPixelColor(i, color);
  }
  strip.show();
}

void applyWarningLights(bool on) {
  warningLightsOn = on;
  digitalWrite(PIN_SIGN_LED, on ? HIGH : LOW);
  setStripColor(strip1, STRIP1_PIXELS, on);
  setStripColor(strip2, STRIP2_PIXELS, on);
}

void allOutputsOff() {
  applyWarningLights(false);
  noTone(PIN_BUZZER);
  buzzerOn = false;
}

void enterCrossing(SystemState direction, uint32_t now) {
  systemState = direction;
  crossingStartedAt = now;
  pendingArrival = false;
  warningLightsChangedAt = now;
  buzzerChangedAt = now;

  applyWarningLights(true);
  if (BUZZER_FREQ_HZ > 0) {
    tone(PIN_BUZZER, BUZZER_FREQ_HZ);
    buzzerOn = true;
  }

  if (direction == STATE_LEFT_TO_RIGHT) {
    logLine(F("상태: 왼쪽 → 오른쪽 횡단 시작"));
  } else {
    logLine(F("상태: 오른쪽 → 왼쪽 횡단 시작"));
  }
}

void enterIdle(const __FlashStringHelper *reason) {
  systemState = STATE_IDLE;
  pendingArrival = false;
  allOutputsOff();
  if (SERIAL_DEBUG) {
    Serial.print(F("상태: 대기 복귀 - "));
    Serial.println(reason);
  }
}

void updateWarningLights(uint32_t now) {
  if ((now - warningLightsChangedAt) < WARNING_LIGHT_BLINK_MS) return;
  warningLightsChangedAt = now;
  applyWarningLights(!warningLightsOn);
}

void updateBuzzer(uint32_t now) {
  if (BUZZER_FREQ_HZ == 0) return;

  uint16_t interval = buzzerOn ? BUZZER_ON_MS : BUZZER_OFF_MS;
  if ((now - buzzerChangedAt) < interval) return;

  buzzerChangedAt = now;
  buzzerOn = !buzzerOn;
  if (buzzerOn) tone(PIN_BUZZER, BUZZER_FREQ_HZ);
  else          noTone(PIN_BUZZER);
}

void setup() {
  pinMode(PIN_IR_LEFT,  IR_USE_INTERNAL_PULLUP ? INPUT_PULLUP : INPUT);
  pinMode(PIN_IR_RIGHT, IR_USE_INTERNAL_PULLUP ? INPUT_PULLUP : INPUT);

  pinMode(PIN_BTN_LEFT,  INPUT_PULLUP);
  pinMode(PIN_BTN_RIGHT, INPUT_PULLUP);

  pinMode(PIN_SIGN_LED, OUTPUT);
  pinMode(PIN_BUZZER,   OUTPUT);

  strip1.begin();
  strip1.setBrightness(STRIP_BRIGHTNESS);
  strip2.begin();
  strip2.setBrightness(STRIP_BRIGHTNESS);

  allOutputsOff();

  if (SERIAL_DEBUG) {
    Serial.begin(SERIAL_BAUD);
    Serial.println(F("무단횡단 방지 시스템 시작 - 양방향 대기"));
    Serial.println(F("빨간 버튼=왼쪽 IR 모의, 초록 버튼=오른쪽 IR 모의"));
  }
}

void loop() {
  uint32_t now = millis();

  bool leftBeamEvent = blockedEdge(
    leftBeamFilter,
    readBeamBlocked(PIN_IR_LEFT),
    IR_CONFIRM_MS,
    now
  );

  bool rightBeamEvent = blockedEdge(
    rightBeamFilter,
    readBeamBlocked(PIN_IR_RIGHT),
    IR_CONFIRM_MS,
    now
  );

  bool leftButtonEvent  = buttonPressedEdge(btnLeft, now);
  bool rightButtonEvent = buttonPressedEdge(btnRight, now);

  // 버튼은 해당 인도의 IR 빔이 끊긴 것과 동일하게 처리합니다.
  bool leftEvent  = leftBeamEvent  || leftButtonEvent;
  bool rightEvent = rightBeamEvent || rightButtonEvent;

  switch (systemState) {
    case STATE_IDLE:
      if (leftEvent && rightEvent) {
        logLine(F("입력 무시: 양쪽 감지선이 동시에 작동함"));
      } else if (leftEvent) {
        enterCrossing(STATE_LEFT_TO_RIGHT, now);
      } else if (rightEvent) {
        enterCrossing(STATE_RIGHT_TO_LEFT, now);
      }
      break;

    case STATE_LEFT_TO_RIGHT: {
      uint32_t elapsed = now - crossingStartedAt;

      if (rightEvent) {
        if (elapsed >= CROSSING_MIN_MS) {
          enterIdle(F("오른쪽 인도 도착 감지"));
          break;
        }
        pendingArrival = true;
        logLine(F("오른쪽 도착 입력 보류: 최소 횡단시간 대기"));
      }

      if (pendingArrival && elapsed >= CROSSING_MIN_MS) {
        enterIdle(F("보류된 오른쪽 도착 입력 처리"));
        break;
      }

      if (leftEvent) {
        logLine(F("반복 입력 무시: 왼쪽 감지선"));
      }

      if (CROSSING_TIMEOUT_MS > 0 && elapsed >= CROSSING_TIMEOUT_MS) {
        enterIdle(F("도착 신호 없음 - 안전 자동 복귀"));
      }
      break;
    }

    case STATE_RIGHT_TO_LEFT: {
      uint32_t elapsed = now - crossingStartedAt;

      if (leftEvent) {
        if (elapsed >= CROSSING_MIN_MS) {
          enterIdle(F("왼쪽 인도 도착 감지"));
          break;
        }
        pendingArrival = true;
        logLine(F("왼쪽 도착 입력 보류: 최소 횡단시간 대기"));
      }

      if (pendingArrival && elapsed >= CROSSING_MIN_MS) {
        enterIdle(F("보류된 왼쪽 도착 입력 처리"));
        break;
      }

      if (rightEvent) {
        logLine(F("반복 입력 무시: 오른쪽 감지선"));
      }

      if (CROSSING_TIMEOUT_MS > 0 && elapsed >= CROSSING_TIMEOUT_MS) {
        enterIdle(F("도착 신호 없음 - 안전 자동 복귀"));
      }
      break;
    }
  }

  if (systemState != STATE_IDLE) {
    updateWarningLights(now);
    updateBuzzer(now);
  }
}
