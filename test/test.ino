/* ============================================================================
   무단횡단 방지 시스템 — IR 센서 2개 축소판 스케치

   천안시 청소년 도시재생 챌린지 (U-CAST) / 5팀 「건영아 잘하자」

   ────────────────────────────────────────────────────────────────────────────
   [이 파일의 역할]
   최종 발표용 회로는 IR 센서를 4개(진입 2 + 통과 2) 씁니다. 그 기준 스케치는
   저장소 루트의 `main.ino`이며, 이 파일은 그것을 대체하지 않습니다.

   지금은 IR 센서가 2개뿐이라, **가진 부품만으로 먼저 조립하고 알고리즘을 확인**
   하려고 만든 축소판입니다.

     - 진입 감지 IR 1개 → D2 (`main.ino`의 진입 IR 1과 같은 핀)
     - 통과 감지 IR 1개 → D3 (`main.ino`에서는 진입 IR 2가 쓰던 핀)

   나머지 부품(스트립 1줄, 빨간 LED, 부저, 버튼 2개)은 `main.ino`와 핀이 완전히
   같습니다. 그래서 IR 센서 2개를 더 구해 D3의 통과 IR을 D4로 옮기고 D3·D5에
   센서를 추가한 뒤 `main.ino`를 올리면, 나머지 배선은 그대로 두고 최종 구성이
   됩니다.

   상태 전이 규칙과 동시 입력 정책은 `main.ino`와 완전히 같습니다.
   **알고리즘을 바꿀 때는 두 파일을 함께 고쳐야 합니다.**

   [동작]
   1) 대기 상태에서 진입 IR(D2)이 감지되거나 빨간 버튼(A0)을 누르면 경고를 시작한다.
   2) 경고 중에는 네오픽셀 스트립이 전체 빨간색으로 켜지고,
      운전자 경고용 빨간 LED(D8)와 부저(D9)가 같은 주기로 점멸한다.
   3) 통과 IR(D3)이 감지되거나 초록 버튼(A1)을 누르면 모든 출력을 끄고 대기로 돌아간다.

   ※ 조정값은 아래 [설정] 구역의 상수만 수정합니다.
   ※ 이 파일은 반드시 `test/` 폴더 안에 두어야 합니다. Arduino IDE는 한 스케치
     폴더의 모든 `.ino`를 합쳐 컴파일하므로, `main.ino` 옆에 두면 `setup()`과
     `loop()`가 중복 정의되어 빌드가 실패합니다.
   ========================================================================== */

/* 네오픽셀(WS2812) 스트립 제어 라이브러리입니다.
   Arduino IDE의 라이브러리 관리에서 "Adafruit NeoPixel"을 설치해야 합니다. */
#include <Adafruit_NeoPixel.h>

/* ---------------------------------------------------------------------------
   [설정 1] 핀 번호
   IR 2개를 제외하면 `main.ino`와 같은 배정입니다.
   --------------------------------------------------------------------------- */
const uint8_t PIN_IR_ENTRY = 2;   // 진입 감지 IR 센서 1개
const uint8_t PIN_IR_EXIT  = 3;   // 통과(도착) 감지 IR 센서 1개
const uint8_t PIN_STRIP    = 6;   // 네오픽셀 LED 스트립 DIN (1줄)
const uint8_t PIN_WARN_LED = 8;   // 운전자 경고용 빨간 LED(표지판 LED)
const uint8_t PIN_BUZZER   = 9;   // 경고용 부저
const uint8_t PIN_BTN_START = A0; // 빨간 버튼: 경고 수동 시작
const uint8_t PIN_BTN_STOP  = A1; // 초록 버튼: 경고 수동 종료

// D4·D5·D7은 비어 있습니다. 센서를 4개로 늘릴 때 D4·D5를 통과 IR로 씁니다.

/* ---------------------------------------------------------------------------
   [설정 2] 네오픽셀 스트립
   경고 상태에서 전체가 빨간색으로 켜집니다. 방향 애니메이션은 없습니다.
   --------------------------------------------------------------------------- */

// 스트립에 박혀 있는 낱개 LED(픽셀) 개수입니다.
const uint16_t STRIP_PIXELS = 10;

// 밝기(0~255). 전원이 부족해 색이 흔들리면 낮춥니다.
const uint8_t STRIP_BRIGHTNESS = 120;

// 경고 색상을 빨강·초록·파랑 세기(각 0~255)로 지정합니다.
const uint8_t WARN_COLOR_R = 255;
const uint8_t WARN_COLOR_G = 0;
const uint8_t WARN_COLOR_B = 0;

/* ---------------------------------------------------------------------------
   [설정 3] 경고 출력 주기
   --------------------------------------------------------------------------- */
const uint16_t WARNING_BLINK_MS = 300;      // 0.3초마다 켜짐↔꺼짐
const uint16_t BUZZER_FREQUENCY_HZ = 1000;  // 부저 음 높이(Hz)

/* ---------------------------------------------------------------------------
   [설정 4] IR 센서 판정
   `main.ino`와 같은 규칙입니다.
   --------------------------------------------------------------------------- */

// 많은 디지털 IR 모듈은 감지 시 LOW(0V)를 출력합니다.
// 손을 대지 않았는데 계속 감지된다고 나오면 false로 바꿔 보세요.
const bool IR_ACTIVE_LOW = true;

// 출력이 오픈 컬렉터 방식이면 내부 풀업이 필요할 수 있습니다.
const bool IR_USE_INTERNAL_PULLUP = true;

// 신호가 이 시간(ms) 이상 유지되어야 감지로 확정합니다(짧은 노이즈 무시).
const uint16_t IR_CONFIRM_MS = 80;

/* ---------------------------------------------------------------------------
   [설정 5] 안전 복귀(선택 기능)
   기본값 0은 자동 종료를 사용하지 않습니다. `main.ino`와 같은 규칙입니다.
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

/* ===========================================================================
   아래는 동작 로직입니다. `main.ino`와 같은 구조를 유지합니다.
   =========================================================================== */

/* 스트립 객체를 만듭니다. (픽셀 수, 핀, 스트립 종류) 순서입니다.
   NEO_GRB는 색 데이터 순서, NEO_KHZ800은 통신 속도입니다. */
Adafruit_NeoPixel strip(STRIP_PIXELS, PIN_STRIP, NEO_GRB + NEO_KHZ800);

// 이 장치가 가질 수 있는 상태는 딱 두 가지입니다.
enum SystemState {
  STATE_IDLE,     // 대기: 모든 출력 꺼짐
  STATE_WARNING   // 경고 중: 스트립·빨간 LED·부저 작동
};

SystemState systemState = STATE_IDLE;

// WARNING_TIMEOUT_MS가 0이면 항상 0으로 유지되며 자동 종료를 사용하지 않습니다.
uint32_t warningTimeoutAt = 0;

/* IR 신호가 잠깐 튄 것인지 진짜 감지인지 가려내는 필터입니다.
   센서마다 자기 상태를 따로 기억해야 하므로 struct로 묶어 센서 수만큼 만듭니다. */
struct HoldFilter {
  uint8_t pin;         // 이 필터가 담당하는 핀 번호
  bool stable;         // 확정된 상태(true = 감지 중)
  bool lastRaw;        // 직전에 읽은 날것 그대로의 값
  uint32_t changedAt;  // 날것 값이 마지막으로 바뀐 시각
};

HoldFilter irEntry = {PIN_IR_ENTRY, false, false, 0};
HoldFilter irExit  = {PIN_IR_EXIT,  false, false, 0};

// 버튼도 채터링을 걸러야 하므로 같은 모양의 묶음을 씁니다.
struct Button {
  uint8_t pin;
  bool stable;
  bool lastRaw;
  uint32_t changedAt;
};

Button btnStart = {PIN_BTN_START, false, false, 0};  // 빨간 버튼
Button btnStop  = {PIN_BTN_STOP,  false, false, 0};  // 초록 버튼

bool warningBlinkOn = false;         // 지금 점멸 출력이 켜진 상태인지
uint32_t warningBlinkChangedAt = 0;  // 점멸 상태를 마지막으로 뒤집은 시각

// F()와 __FlashStringHelper는 문자열을 RAM 대신 플래시 메모리에 두는 방법입니다.
// 우노는 RAM이 2KB뿐이라 한글 로그를 많이 쓰면 금방 부족해집니다.
void logLine(const __FlashStringHelper *message) {
  if (SERIAL_DEBUG) Serial.println(message);
}

// 핀을 읽어 "지금 감지 중인가?"를 true/false로 바꿔 줍니다.
// 감지 시 LOW인지 HIGH인지에 대한 판단을 IR_ACTIVE_LOW 한 곳에만 둡니다.
bool readIrDetected(uint8_t pin) {
  bool low = (digitalRead(pin) == LOW);
  return IR_ACTIVE_LOW ? low : !low;
}

/* 센서가 "감지 없음 → 감지"로 바뀌는 순간에만 딱 한 번 true를 반환합니다.

   loop()는 1초에 수천 번 돕니다. 값을 그대로 쓰면 손을 1초만 대고 있어도
   "감지!"가 수천 번 발생합니다. 그래서 (1) 값이 IR_CONFIRM_MS 이상 유지돼야
   확정하고, (2) 확정 상태가 실제로 바뀐 그 순간에만 이벤트를 냅니다.

   `HoldFilter &filter`의 &는 "복사본 말고 원본을 직접 고치겠다"는 뜻입니다.
   이게 없으면 필터가 기억한 내용이 함수를 나가는 순간 사라집니다. */
bool detectedEdge(HoldFilter &filter, uint32_t now) {
  bool raw = readIrDetected(filter.pin);

  // 값이 방금 바뀌었다면 바뀐 시각을 기록하고 확정 시간을 처음부터 다시 셉니다.
  if (raw != filter.lastRaw) {
    filter.lastRaw = raw;
    filter.changedAt = now;
  }

  // 확정 상태와 다른 값이 IR_CONFIRM_MS 이상 유지되면 확정 상태를 바꿉니다.
  if (raw != filter.stable && (now - filter.changedAt) >= IR_CONFIRM_MS) {
    bool previous = filter.stable;
    filter.stable = raw;
    return (!previous && filter.stable);  // 꺼짐 → 켜짐일 때만 이벤트
  }

  return false;
}

// 버튼도 같은 원리입니다. 길게 누르고 있어도 누르는 순간 한 번만 true입니다.
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
    return raw;  // 눌림(true)일 때만 이벤트, 뗄 때는 false
  }

  return false;
}

/* 스트립 전체를 경고 색으로 켜거나(on = true) 완전히 끕니다(on = false).

   setPixelColor()는 메모리에 색을 적어 두기만 하고, 실제로 스트립에 데이터를
   쏘는 것은 마지막 show() 한 번입니다. */
void applyStrip(bool on) {
  uint32_t color = on ? strip.Color(WARN_COLOR_R, WARN_COLOR_G, WARN_COLOR_B) : 0;
  for (uint16_t i = 0; i < STRIP_PIXELS; i++) {
    strip.setPixelColor(i, color);
  }
  strip.show();
}

// 빨간 경고 LED와 부저는 항상 같은 상태로 함께 동작합니다.
// 둘을 따로 관리하면 타이밍이 어긋나므로 한 함수에서 같이 처리합니다.
void applyBlinkOutputs(bool on) {
  warningBlinkOn = on;
  digitalWrite(PIN_WARN_LED, on ? HIGH : LOW);
  if (on && BUZZER_FREQUENCY_HZ > 0) {
    tone(PIN_BUZZER, BUZZER_FREQUENCY_HZ);  // 소리 시작(계속 울림)
  } else {
    noTone(PIN_BUZZER);                     // 소리 정지
  }
}

void allOutputsOff() {
  applyBlinkOutputs(false);
  applyStrip(false);
}

// 대기 → 경고로 넘어갈 때 해야 할 일을 모아 둔 함수입니다.
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

// 경고 → 대기로 돌아갈 때의 처리입니다.
void enterIdle(const __FlashStringHelper *reason) {
  systemState = STATE_IDLE;
  warningTimeoutAt = 0;
  allOutputsOff();

  if (SERIAL_DEBUG) {
    Serial.print(F("상태: 대기 복귀 - "));
    Serial.println(reason);
  }
}

/* millis()는 전원을 켠 뒤 흐른 밀리초인데, 약 49.7일이 지나면 0으로 되돌아갑니다.
   뺄셈 결과를 부호 있는 정수로 보면 되돌아가는 순간에도 정확히 비교됩니다. */
bool warningTimedOut(uint32_t now) {
  if (warningTimeoutAt == 0) return false;  // 0 = 자동 종료 사용 안 함
  return ((int32_t)(now - warningTimeoutAt) >= 0);
}

/* 점멸을 담당합니다. delay()를 쓰면 기다리는 동안 센서와 버튼을 못 읽으므로,
   "마지막으로 뒤집은 뒤 WARNING_BLINK_MS가 지났는가?"만 확인하고 빠져나옵니다. */
void updateBlinkOutputs(uint32_t now) {
  if ((now - warningBlinkChangedAt) < WARNING_BLINK_MS) return;
  warningBlinkChangedAt = now;
  applyBlinkOutputs(!warningBlinkOn);  // ! 는 반대로 뒤집기(켜짐↔꺼짐)
}

// setup()은 전원을 켜거나 리셋했을 때 딱 한 번 실행됩니다.
void setup() {
  // INPUT_PULLUP은 내부 저항을 붙여 평소 값을 HIGH로 올려 둡니다.
  uint8_t irMode = IR_USE_INTERNAL_PULLUP ? INPUT_PULLUP : INPUT;
  pinMode(PIN_IR_ENTRY, irMode);
  pinMode(PIN_IR_EXIT,  irMode);

  pinMode(PIN_BTN_START, INPUT_PULLUP);
  pinMode(PIN_BTN_STOP,  INPUT_PULLUP);

  pinMode(PIN_WARN_LED, OUTPUT);
  pinMode(PIN_BUZZER,   OUTPUT);

  strip.begin();                        // 스트립 통신 시작
  strip.setBrightness(STRIP_BRIGHTNESS);

  allOutputsOff();  // 켜자마자 뭔가 켜져 있으면 안 되므로 전부 끕니다

  if (SERIAL_DEBUG) {
    Serial.begin(SERIAL_BAUD);
    Serial.println(F("무단횡단 방지 시스템(IR 2개 축소판) 시작 - 대기 상태"));
    Serial.println(F("진입 IR=D2, 통과 IR=D3 / 빨간 버튼 A0=시작, 초록 버튼 A1=종료"));
  }
}

// loop()는 setup() 뒤에 끝없이 반복 실행됩니다.
void loop() {
  uint32_t now = millis();  // 한 바퀴 도는 동안 같은 시각을 쓰도록 먼저 읽어 둡니다

  // 조건문 안에서 읽으면 앞쪽이 true일 때 뒤쪽 필터가 갱신되지 않아
  // 상태가 어긋나므로, 두 센서를 먼저 모두 읽어 둡니다.
  bool entryDetected = detectedEdge(irEntry, now);
  bool exitDetected  = detectedEdge(irExit,  now);

  bool redButtonPressed   = buttonPressedEdge(btnStart, now);
  bool greenButtonPressed = buttonPressedEdge(btnStop,  now);

  // 지금 상태가 무엇이냐에 따라 같은 입력도 다르게 처리합니다.
  switch (systemState) {
    case STATE_IDLE:
      // 대기 상태에서 통과 센서 입력만 들어온 경우는 아무 일도 하지 않습니다.
      if (redButtonPressed) {
        // 빨간 버튼은 방향·센서와 무관한 강제 시작이므로 항상 우선합니다.
        enterWarning(now, F("빨간 버튼 수동 시작"));
      } else if (entryDetected) {
        // 진입과 통과가 함께 감지되면 어느 상황인지 판단할 수 없으므로
        // 켰다가 바로 끄는 동작 대신 보수적으로 시작하지 않습니다.
        if (irExit.stable) {
          logLine(F("입력 무시: 진입과 통과가 동시에 감지됨"));
        } else {
          enterWarning(now, F("진입 IR 감지"));
        }
      }
      break;

    case STATE_WARNING:
      // break는 switch를 빠져나가라는 뜻입니다.
      // 종료가 결정되면 아래 점멸 처리를 건너뛰어야 하므로 바로 나갑니다.
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
      if (entryDetected)    logLine(F("경고 유지: 진입 IR 재감지"));
      if (redButtonPressed) logLine(F("경고 유지: 빨간 버튼 재입력"));

      if (warningTimedOut(now)) {
        enterIdle(F("제한시간 경과 - 선택 기능 자동 복귀"));
        break;
      }

      updateBlinkOutputs(now);  // 여기까지 왔으면 경고 유지 중 → 점멸 계속
      break;
  }
}
