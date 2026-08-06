/* ============================================================================
   무단횡단 방지 시스템 (Jaywalking Warning System)

   천안시 청소년 도시재생 챌린지 (U-CAST) / 5팀 「건영아 잘하자」

   [최종 MVP 동작] — 3차시 워크북 5·6·7·8장 기준
   1) 차도 진입 구간에 진입 감지 IR 센서 2개(D2, D3)를 설치한다.
   2) 도로 반대편 통과 지점에 통과(도착) 감지 IR 센서 2개(D4, D5)를 설치한다.
   3) 대기 상태에서 진입 IR 중 하나가 감지되거나 빨간 버튼을 누르면 경고를 시작한다.
   4) 경고 중에는 네오픽셀 스트립 2줄이 전체 빨간색으로 켜지고,
      운전자 경고용 빨간 LED와 부저가 같은 주기로 점멸한다.
   5) 통과 IR 중 하나가 감지되거나 초록 버튼을 누르면 모든 출력을 끄고 대기로 돌아간다.

   빨간 버튼 = 경고 수동 시작, 초록 버튼 = 경고 수동 종료.
   두 버튼 모두 특정 방향이나 특정 센서를 모의하지 않는다.

   [이번 MVP에서 제외] — 초기 검토 후 제외한 기능
   - 보행자의 횡단 방향 계산과 방향 상태 저장
   - 횡단보도 방향 유도와 흐르는 방향성 LED 애니메이션
   - 펜스, 압력센서(FSR), 초음파센서

   ※ 조정값은 아래 [설정] 구역의 상수만 수정합니다.
   ========================================================================== */

#include <Adafruit_NeoPixel.h>

/* ---------------------------------------------------------------------------
   [설정 1] 핀 번호
   워크북 8장 「코드 기반 핀 연결표」와 같은 배정입니다.
   --------------------------------------------------------------------------- */
const uint8_t PIN_IR_ENTRY_1 = 2;   // 진입 감지 IR 센서 1
const uint8_t PIN_IR_ENTRY_2 = 3;   // 진입 감지 IR 센서 2
const uint8_t PIN_IR_EXIT_1  = 4;   // 통과(도착) 감지 IR 센서 1
const uint8_t PIN_IR_EXIT_2  = 5;   // 통과(도착) 감지 IR 센서 2
const uint8_t PIN_STRIP_1    = 6;   // 네오픽셀 LED 스트립 1 DIN
const uint8_t PIN_STRIP_2    = 7;   // 네오픽셀 LED 스트립 2 DIN
const uint8_t PIN_WARN_LED   = 8;   // 운전자 경고용 빨간 LED(표지판 LED)
const uint8_t PIN_BUZZER     = 9;   // 경고용 부저
const uint8_t PIN_BTN_START  = A0;  // 빨간 버튼: 경고 수동 시작
const uint8_t PIN_BTN_STOP   = A1;  // 초록 버튼: 경고 수동 종료

/* ---------------------------------------------------------------------------
   [설정 2] 네오픽셀 스트립
   두 스트립 모두 경고 상태에서 전체 빨간색으로 켜집니다. 방향 애니메이션은 없습니다.

   ※ STRIP1_PIXELS, STRIP2_PIXELS는 아직 실물로 확정하지 않은 값입니다.
     스트립의 낱개 LED 개수를 직접 세어 두 값을 각각 수정해야 합니다.
   --------------------------------------------------------------------------- */
const uint16_t STRIP1_PIXELS = 15;  // 임시값 — 실물 스트립 1의 LED 개수로 수정
const uint16_t STRIP2_PIXELS = 15;  // 임시값 — 실물 스트립 2의 LED 개수로 수정

const uint8_t STRIP_BRIGHTNESS = 120;  // 0~255
const uint8_t WARN_COLOR_R = 255;
const uint8_t WARN_COLOR_G = 0;
const uint8_t WARN_COLOR_B = 0;

/* ---------------------------------------------------------------------------
   [설정 3] 경고 출력 주기
   빨간 경고 LED와 부저가 이 간격으로 함께 점멸합니다.
   --------------------------------------------------------------------------- */
const uint16_t WARNING_BLINK_MS = 300;
const uint16_t BUZZER_FREQUENCY_HZ = 1000;

/* ---------------------------------------------------------------------------
   [설정 4] IR 센서 판정
   --------------------------------------------------------------------------- */
// 많은 디지털 IR 모듈은 물체를 감지하면 LOW를 출력합니다.
// 실물에서 반대로 동작하면 false로 변경합니다. (실물 확인 전까지는 미확정값)
const bool IR_ACTIVE_LOW = true;

// 모듈 출력이 오픈 컬렉터 방식이면 내부 풀업이 필요할 수 있습니다.
const bool IR_USE_INTERNAL_PULLUP = true;

// 신호가 이 시간 이상 유지되어야 감지로 확정합니다(짧은 노이즈 무시).
const uint16_t IR_CONFIRM_MS = 80;

/* ---------------------------------------------------------------------------
   [설정 5] 안전 복귀(선택 기능)
   워크북 원본에는 없는 선택 설정입니다. 기본값 0은 자동 종료를 사용하지 않습니다.
   0보다 큰 값을 넣으면 그 시간이 지난 뒤 경고를 자동으로 끄고 대기로 돌아갑니다.
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
   아래는 동작 로직입니다.
   =========================================================================== */

Adafruit_NeoPixel strip1(STRIP1_PIXELS, PIN_STRIP_1, NEO_GRB + NEO_KHZ800);
Adafruit_NeoPixel strip2(STRIP2_PIXELS, PIN_STRIP_2, NEO_GRB + NEO_KHZ800);

enum SystemState {
  STATE_IDLE,
  STATE_WARNING
};

SystemState systemState = STATE_IDLE;

// WARNING_TIMEOUT_MS가 0이면 항상 0으로 유지되며 자동 종료를 사용하지 않습니다.
uint32_t warningTimeoutAt = 0;

// IR 신호를 일정 시간 유지될 때만 확정하는 필터입니다.
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

Button btnStart = {PIN_BTN_START, false, false, 0};  // 빨간 버튼
Button btnStop  = {PIN_BTN_STOP,  false, false, 0};  // 초록 버튼

bool warningBlinkOn = false;
uint32_t warningBlinkChangedAt = 0;

void logLine(const __FlashStringHelper *message) {
  if (SERIAL_DEBUG) Serial.println(message);
}

bool readIrDetected(uint8_t pin) {
  bool low = (digitalRead(pin) == LOW);
  return IR_ACTIVE_LOW ? low : !low;
}

// 확정된 상태가 false→true로 바뀌는 순간에만 true를 반환합니다.
// 센서가 계속 감지된 상태로 유지되어도 이벤트는 다시 발생하지 않습니다.
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

// 길게 누르고 있어도 누르는 순간 한 번만 true를 반환합니다.
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

// 두 스트립을 경고 색으로 켜거나 완전히 끕니다.
void applyStrips(bool on) {
  setStripColor(strip1, STRIP1_PIXELS, on);
  setStripColor(strip2, STRIP2_PIXELS, on);
}

// 빨간 경고 LED와 부저는 항상 같은 상태로 함께 동작합니다.
void applyBlinkOutputs(bool on) {
  warningBlinkOn = on;
  digitalWrite(PIN_WARN_LED, on ? HIGH : LOW);
  if (on && BUZZER_FREQUENCY_HZ > 0) {
    tone(PIN_BUZZER, BUZZER_FREQUENCY_HZ);
  } else {
    noTone(PIN_BUZZER);
  }
}

void allOutputsOff() {
  applyBlinkOutputs(false);
  applyStrips(false);
}

void enterWarning(uint32_t now, const __FlashStringHelper *reason) {
  systemState = STATE_WARNING;
  warningBlinkChangedAt = now;

  if (WARNING_TIMEOUT_MS > 0) {
    warningTimeoutAt = now + WARNING_TIMEOUT_MS;
    if (warningTimeoutAt == 0) warningTimeoutAt = 1;  // 0은 "사용 안 함"이므로 피합니다
  } else {
    warningTimeoutAt = 0;
  }

  applyStrips(true);
  applyBlinkOutputs(true);

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

// millis() 오버플로에 안전하도록 부호 있는 차이로 비교합니다.
bool warningTimedOut(uint32_t now) {
  if (warningTimeoutAt == 0) return false;
  return ((int32_t)(now - warningTimeoutAt) >= 0);
}

void updateBlinkOutputs(uint32_t now) {
  if ((now - warningBlinkChangedAt) < WARNING_BLINK_MS) return;
  warningBlinkChangedAt = now;
  applyBlinkOutputs(!warningBlinkOn);
}

void setup() {
  uint8_t irMode = IR_USE_INTERNAL_PULLUP ? INPUT_PULLUP : INPUT;
  pinMode(PIN_IR_ENTRY_1, irMode);
  pinMode(PIN_IR_ENTRY_2, irMode);
  pinMode(PIN_IR_EXIT_1,  irMode);
  pinMode(PIN_IR_EXIT_2,  irMode);

  pinMode(PIN_BTN_START, INPUT_PULLUP);
  pinMode(PIN_BTN_STOP,  INPUT_PULLUP);

  pinMode(PIN_WARN_LED, OUTPUT);
  pinMode(PIN_BUZZER,   OUTPUT);

  strip1.begin();
  strip1.setBrightness(STRIP_BRIGHTNESS);
  strip2.begin();
  strip2.setBrightness(STRIP_BRIGHTNESS);

  allOutputsOff();

  if (SERIAL_DEBUG) {
    Serial.begin(SERIAL_BAUD);
    Serial.println(F("무단횡단 방지 시스템 시작 - 대기 상태"));
    Serial.println(F("빨간 버튼=경고 수동 시작, 초록 버튼=경고 수동 종료"));
  }
}

void loop() {
  uint32_t now = millis();

  bool entryEvent1 = detectedEdge(irEntry1, now);
  bool entryEvent2 = detectedEdge(irEntry2, now);
  bool exitEvent1  = detectedEdge(irExit1,  now);
  bool exitEvent2  = detectedEdge(irExit2,  now);

  bool entryDetected = entryEvent1 || entryEvent2;
  bool exitDetected  = exitEvent1  || exitEvent2;

  bool redButtonPressed   = buttonPressedEdge(btnStart, now);
  bool greenButtonPressed = buttonPressedEdge(btnStop,  now);

  switch (systemState) {
    case STATE_IDLE:
      // 대기 상태에서 통과 센서 입력만 들어온 경우는 아무 일도 하지 않습니다.
      if (redButtonPressed) {
        // 빨간 버튼은 방향·센서와 무관한 강제 시작이므로 항상 우선합니다.
        enterWarning(now, F("빨간 버튼 수동 시작"));
      } else if (entryDetected) {
        // 진입과 통과가 함께 감지되면 어느 상황인지 판단할 수 없으므로
        // 켰다가 바로 끄는 동작 대신 보수적으로 시작하지 않습니다.
        if (irExit1.stable || irExit2.stable) {
          logLine(F("입력 무시: 진입과 통과가 동시에 감지됨"));
        } else {
          enterWarning(now, F("진입 IR 감지"));
        }
      }
      break;

    case STATE_WARNING:
      if (exitDetected) {
        enterIdle(F("통과 IR 감지"));
        break;
      }

      if (greenButtonPressed) {
        // 초록 버튼은 방향·센서와 무관한 강제 종료입니다.
        enterIdle(F("초록 버튼 수동 종료"));
        break;
      }

      // 경고 중 새로운 진입 감지나 빨간 버튼은 현재 경고 상태를 그대로 유지합니다.
      if (entryDetected)   logLine(F("경고 유지: 진입 IR 재감지"));
      if (redButtonPressed) logLine(F("경고 유지: 빨간 버튼 재입력"));

      if (warningTimedOut(now)) {
        enterIdle(F("제한시간 경과 - 선택 기능 자동 복귀"));
        break;
      }

      updateBlinkOutputs(now);
      break;
  }
}
