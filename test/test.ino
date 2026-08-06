/* ============================================================================
   무단횡단 방지 시스템 — Tinkercad 시뮬레이션 스케치

   천안시 청소년 도시재생 챌린지 (U-CAST) / 5팀 「건영아 잘하자」

   [이 파일의 역할]
   실물 회로에는 IR 센서 4개를 씁니다. 그 배선과 코드는 저장소 루트의 `main.ino`가
   기준이며, 이 파일은 그것을 대체하지 않습니다.
   Tinkercad에는 디지털 IR 센서 모듈과 Adafruit NeoPixel 라이브러리가 없으므로,
   시뮬레이터에서 알고리즘만 확인하려고 다음 두 가지를 바꾼 버전입니다.

     - IR 센서 4개  →  가변저항 4개(A0~A3). 손잡이를 돌리면 감지/비감지가 됩니다.
     - 스트립 2줄   →  스트립 1줄(D4)

   상태 전이 규칙과 동시 입력 정책은 `main.ino`와 완전히 같습니다.
   알고리즘을 바꿀 때는 두 파일을 함께 고쳐야 합니다.

   [동작]
   1) 대기 상태에서 진입 가변저항(A0·A1) 중 하나가 임계값을 넘거나
      빨간 버튼(D9)을 누르면 경고를 시작한다.
   2) 경고 중에는 네오픽셀 스트립이 전체 빨간색으로 켜지고,
      운전자 경고용 빨간 LED(D3)와 부저(D5)가 같은 주기로 점멸한다.
   3) 통과 가변저항(A2·A3) 중 하나가 임계값을 넘거나
      초록 버튼(D10)을 누르면 모든 출력을 끄고 대기로 돌아간다.

   ※ 조정값은 아래 [설정] 구역의 상수만 수정합니다.
   ========================================================================== */

#include <Adafruit_NeoPixel.h>

/* ---------------------------------------------------------------------------
   [설정 1] 핀 번호
   `docs/images/5조 아두이노 스키매틱.pdf`의 넷 이름과 같은 배정입니다.
   실물 IR 배선의 핀 번호(D2~D5 등)와 다르므로 `main.ino`와 혼동하지 않습니다.
   --------------------------------------------------------------------------- */
const uint8_t PIN_POT_ENTRY_1 = A0;  // 진입 감지 IR 1 대역품(가변저항)
const uint8_t PIN_POT_ENTRY_2 = A1;  // 진입 감지 IR 2 대역품(가변저항)
const uint8_t PIN_POT_EXIT_1  = A2;  // 통과 감지 IR 1 대역품(가변저항)
const uint8_t PIN_POT_EXIT_2  = A3;  // 통과 감지 IR 2 대역품(가변저항)

const uint8_t PIN_WARN_LED = 3;      // 운전자 경고용 빨간 LED(220Ω 직렬)
const uint8_t PIN_STRIP    = 4;      // 네오픽셀 스트립 DIN
const uint8_t PIN_BUZZER   = 5;      // 경고용 부저

const uint8_t PIN_BTN_START = 9;     // 빨간 버튼(S1): 경고 수동 시작
const uint8_t PIN_BTN_STOP  = 10;    // 초록 버튼(S2): 경고 수동 종료

/* ---------------------------------------------------------------------------
   [설정 2] 네오픽셀 스트립
   경고 상태에서 전체 빨간색으로 켜집니다. 방향 애니메이션은 없습니다.

   ※ STRIP_PIXELS는 Tinkercad에 올려둔 스트립의 낱개 LED 개수와 맞춰야 합니다.
   --------------------------------------------------------------------------- */
const uint16_t STRIP_PIXELS = 12;

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
   [설정 4] 가변저항 판정
   가변저항은 IR 센서의 대역품입니다. 값이 임계값을 넘으면 "감지"로 봅니다.

   실물 IR 모듈은 감지 시 LOW/HIGH 둘 중 하나를 내보내는 디지털 출력이므로,
   `main.ino`에는 IR_ACTIVE_LOW가 있고 여기에는 없습니다.
   --------------------------------------------------------------------------- */
const uint16_t POT_THRESHOLD = 512;

// 임계값 바로 근처에서 값이 떨리며 감지/해제를 반복하는 것을 막습니다.
// 감지는 (임계값 + 히스테리시스) 이상, 해제는 (임계값 − 히스테리시스) 이하에서만
// 일어납니다.
const uint16_t POT_HYSTERESIS = 40;

// 신호가 이 시간 이상 유지되어야 감지로 확정합니다(짧은 흔들림 무시).
const uint16_t POT_CONFIRM_MS = 80;

/* ---------------------------------------------------------------------------
   [설정 5] 안전 복귀(선택 기능)
   기본값 0은 자동 종료를 사용하지 않습니다. 0보다 큰 값을 넣으면 그 시간이 지난 뒤
   경고를 자동으로 끄고 대기로 돌아갑니다. `main.ino`와 같은 규칙입니다.
   --------------------------------------------------------------------------- */
const uint32_t WARNING_TIMEOUT_MS = 0UL;

/* ---------------------------------------------------------------------------
   [설정 6] 버튼·시리얼
   버튼은 INPUT_PULLUP을 쓰므로 한쪽 다리를 GND에 연결합니다(눌리면 LOW).
   --------------------------------------------------------------------------- */
const bool BUTTON_ACTIVE_LOW = true;
const uint16_t BUTTON_DEBOUNCE_MS = 40;

const bool SERIAL_DEBUG = true;
const long SERIAL_BAUD = 9600;

// 가변저항 4개의 현재 값을 이 간격으로 시리얼 모니터에 찍습니다.
// 0을 넣으면 값 출력을 끄고 상태 변화 로그만 남깁니다.
const uint16_t POT_LOG_INTERVAL_MS = 500;

/* ===========================================================================
   아래는 동작 로직입니다. `main.ino`와 같은 구조를 유지합니다.
   =========================================================================== */

Adafruit_NeoPixel strip(STRIP_PIXELS, PIN_STRIP, NEO_GRB + NEO_KHZ800);

enum SystemState {
  STATE_IDLE,
  STATE_WARNING
};

SystemState systemState = STATE_IDLE;

// WARNING_TIMEOUT_MS가 0이면 항상 0으로 유지되며 자동 종료를 사용하지 않습니다.
uint32_t warningTimeoutAt = 0;

// 가변저항 값을 일정 시간 유지될 때만 확정하는 필터입니다.
struct HoldFilter {
  uint8_t pin;
  bool stable;
  bool lastRaw;
  uint32_t changedAt;
};

HoldFilter potEntry1 = {PIN_POT_ENTRY_1, false, false, 0};
HoldFilter potEntry2 = {PIN_POT_ENTRY_2, false, false, 0};
HoldFilter potExit1  = {PIN_POT_EXIT_1,  false, false, 0};
HoldFilter potExit2  = {PIN_POT_EXIT_2,  false, false, 0};

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

uint32_t potLoggedAt = 0;

void logLine(const __FlashStringHelper *message) {
  if (SERIAL_DEBUG) Serial.println(message);
}

// 현재 확정 상태에 따라 서로 다른 문턱을 씁니다(히스테리시스).
bool readPotDetected(uint8_t pin, bool currentlyDetected) {
  int value = analogRead(pin);

  if (currentlyDetected) {
    return value > (int)POT_THRESHOLD - (int)POT_HYSTERESIS;
  }
  return value >= (int)POT_THRESHOLD + (int)POT_HYSTERESIS;
}

// 확정된 상태가 false→true로 바뀌는 순간에만 true를 반환합니다.
// 가변저항을 돌린 채로 두어도 이벤트는 다시 발생하지 않습니다.
bool detectedEdge(HoldFilter &filter, uint32_t now) {
  bool raw = readPotDetected(filter.pin, filter.stable);

  if (raw != filter.lastRaw) {
    filter.lastRaw = raw;
    filter.changedAt = now;
  }

  if (raw != filter.stable && (now - filter.changedAt) >= POT_CONFIRM_MS) {
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

// 스트립을 경고 색으로 켜거나 완전히 끕니다.
void applyStrip(bool on) {
  uint32_t color = on ? strip.Color(WARN_COLOR_R, WARN_COLOR_G, WARN_COLOR_B) : 0;
  for (uint16_t i = 0; i < STRIP_PIXELS; i++) {
    strip.setPixelColor(i, color);
  }
  strip.show();
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
  applyStrip(false);
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

  applyStrip(true);
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

// 시뮬레이터에서 손잡이를 얼마나 돌렸는지 확인하기 위한 값 출력입니다.
void logPotValues(uint32_t now) {
  if (!SERIAL_DEBUG || POT_LOG_INTERVAL_MS == 0) return;
  if ((now - potLoggedAt) < POT_LOG_INTERVAL_MS) return;
  potLoggedAt = now;

  Serial.print(F("진입1:"));
  Serial.print(analogRead(PIN_POT_ENTRY_1));
  Serial.print(F(" 진입2:"));
  Serial.print(analogRead(PIN_POT_ENTRY_2));
  Serial.print(F(" 통과1:"));
  Serial.print(analogRead(PIN_POT_EXIT_1));
  Serial.print(F(" 통과2:"));
  Serial.println(analogRead(PIN_POT_EXIT_2));
}

void setup() {
  pinMode(PIN_BTN_START, INPUT_PULLUP);
  pinMode(PIN_BTN_STOP,  INPUT_PULLUP);

  pinMode(PIN_WARN_LED, OUTPUT);
  pinMode(PIN_BUZZER,   OUTPUT);

  strip.begin();
  strip.setBrightness(STRIP_BRIGHTNESS);

  allOutputsOff();

  if (SERIAL_DEBUG) {
    Serial.begin(SERIAL_BAUD);
    Serial.println(F("무단횡단 방지 시스템(시뮬레이션) 시작 - 대기 상태"));
    Serial.println(F("가변저항 A0·A1=진입, A2·A3=통과 / 빨간 버튼 D9=시작, 초록 버튼 D10=종료"));
  }
}

void loop() {
  uint32_t now = millis();

  logPotValues(now);

  bool entryEvent1 = detectedEdge(potEntry1, now);
  bool entryEvent2 = detectedEdge(potEntry2, now);
  bool exitEvent1  = detectedEdge(potExit1,  now);
  bool exitEvent2  = detectedEdge(potExit2,  now);

  bool entryDetected = entryEvent1 || entryEvent2;
  bool exitDetected  = exitEvent1  || exitEvent2;

  bool redButtonPressed   = buttonPressedEdge(btnStart, now);
  bool greenButtonPressed = buttonPressedEdge(btnStop,  now);

  switch (systemState) {
    case STATE_IDLE:
      // 대기 상태에서 통과 입력만 들어온 경우는 아무 일도 하지 않습니다.
      if (redButtonPressed) {
        // 빨간 버튼은 방향·센서와 무관한 강제 시작이므로 항상 우선합니다.
        enterWarning(now, F("빨간 버튼 수동 시작"));
      } else if (entryDetected) {
        // 진입과 통과가 함께 감지되면 어느 상황인지 판단할 수 없으므로
        // 켰다가 바로 끄는 동작 대신 보수적으로 시작하지 않습니다.
        if (potExit1.stable || potExit2.stable) {
          logLine(F("입력 무시: 진입과 통과가 동시에 감지됨"));
        } else {
          enterWarning(now, F("진입 감지"));
        }
      }
      break;

    case STATE_WARNING:
      if (exitDetected) {
        enterIdle(F("통과 감지"));
        break;
      }

      if (greenButtonPressed) {
        // 초록 버튼은 방향·센서와 무관한 강제 종료입니다.
        enterIdle(F("초록 버튼 수동 종료"));
        break;
      }

      // 경고 중 새로운 진입 감지나 빨간 버튼은 현재 경고 상태를 그대로 유지합니다.
      if (entryDetected)    logLine(F("경고 유지: 진입 재감지"));
      if (redButtonPressed) logLine(F("경고 유지: 빨간 버튼 재입력"));

      if (warningTimedOut(now)) {
        enterIdle(F("제한시간 경과 - 선택 기능 자동 복귀"));
        break;
      }

      updateBlinkOutputs(now);
      break;
  }
}
