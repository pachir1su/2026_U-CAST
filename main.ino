/* ============================================================================
   무단횡단 방지 시스템 (Jaywalking Prevention System)

   천안시 청소년 도시재생 챌린지 (U-CAST) / 5팀 「건영아 잘하자」
   3차시 팀 제작 워크북 기준 1차 코드

   [해결하려는 문제]
   천안 중앙시장 주변에서 65세 이상 고령 보행자가 횡단보도까지 우회하지 않고
   가까운 곳에서 도로를 건너면서, 운전자의 급제동·추돌 사고 위험이 발생한다.

   [동작 요약]
   1) 차도 경계선(연석)을 향한 IR 센서 한 쌍이 보행자의 접근을 감지한다.
   2) 감지되면 부저 경고음 + 빨간 LED 경고등이 켜지고,
      차도 위 LED 스트립 2개가 횡단보도 방향으로 흐르듯 점등되어
      "보행자가 건너고 있음"을 운전자에게 알린다.
   3) 보행자가 길을 다 건너 반대편 IR 센서 한 쌍을 지나가면 상황 종결로 판단한다.
   4) 모든 출력이 꺼지고 대기 상태로 복귀한다.

   센서가 동작하지 않을 때를 대비해, 빨간 버튼(수동 발동) / 초록 버튼(수동 종결)로
   동일한 로직을 그대로 실행할 수 있다. 발표 시연 시 이 버튼을 사용한다.

   [이번 프로젝트에서 제외한 기능]
   - 초음파 센서 : 역할이 불분명하여 제외
   - 펜스 / 압력 센서(FSR) : 시선이 분산되어 제외

   ※ 조정이 필요한 값은 전부 아래 [설정] 구역의 상수로 빼두었습니다.
      부품을 실제로 받은 뒤 이 구역의 숫자만 바꾸면 됩니다.
      아래쪽 로직 코드는 건드릴 필요가 없습니다.
   ========================================================================== */

#include <Adafruit_NeoPixel.h>

/* ---------------------------------------------------------------------------
   [설정 1] 핀 번호
   3차시 워크북 "8. 코드 기반 핀 연결표"를 기준으로 정리했습니다.
   실물 배선을 바꾸면 여기 숫자만 바꾸세요.
   --------------------------------------------------------------------------- */
const uint8_t PIN_IR_ENTRY_A = 2;   // 진입 감지용 IR 센서 1 (차도 경계선 쪽)
const uint8_t PIN_IR_ENTRY_B = 3;   // 진입 감지용 IR 센서 2 (차도 경계선 쪽)
const uint8_t PIN_IR_EXIT_A  = 4;   // 통과 감지용 IR 센서 1 (반대편 인도 쪽)
const uint8_t PIN_IR_EXIT_B  = 5;   // 통과 감지용 IR 센서 2 (반대편 인도 쪽)
const uint8_t PIN_STRIP_1    = 6;   // LED 스트립 1 데이터 핀 (차도)
const uint8_t PIN_STRIP_2    = 7;   // LED 스트립 2 데이터 핀 (차도)
const uint8_t PIN_RED_LED    = 8;   // 빨간색 LED (운전자 경고등, 220~330Ω 저항)
const uint8_t PIN_BUZZER     = 9;   // 부저 (경고음)
const uint8_t PIN_BTN_RED    = 10;  // 빨간색 버튼 - 수동 발동 (무단횡단 발생 가정)
const uint8_t PIN_BTN_GREEN  = 11;  // 초록색 버튼 - 수동 종결 (다 건넜다고 가정)

/* ---------------------------------------------------------------------------
   [설정 2] LED 스트립 (네오픽셀)
   스트립 2개 모두 차도 위에 설치하여, 보행자가 건너는 중임을 운전자에게 알립니다.
   ※ 픽셀 수는 부품을 받은 뒤 실제 개수로 바꾸세요.
   --------------------------------------------------------------------------- */
const uint16_t STRIP1_PIXELS = 10;    // 스트립 1의 LED(픽셀) 개수  ← 실물 기준으로 수정
const uint16_t STRIP2_PIXELS = 10;    // 스트립 2의 LED(픽셀) 개수  ← 실물 기준으로 수정

const uint8_t  STRIP_BRIGHTNESS = 120;  // 전체 밝기 0~255 (외부 전원 용량에 맞춰 조정)

const uint8_t  WARN_COLOR_R = 255;      // 경고 색상 (기본: 빨간색)
const uint8_t  WARN_COLOR_G = 0;
const uint8_t  WARN_COLOR_B = 0;

const uint8_t  COMET_LENGTH  = 4;       // 흐르는 불빛(꼬리)의 길이. 클수록 길게 늘어짐
const uint16_t ANIM_STEP_MS  = 60;      // 한 칸 이동에 걸리는 시간(ms). 작을수록 빠름

// 스트립을 반대 방향으로 붙였을 때 true 로 바꾸면 흐르는 방향이 뒤집힙니다.
const bool STRIP1_REVERSE = false;
const bool STRIP2_REVERSE = false;

/* ---------------------------------------------------------------------------
   [설정 3] IR 센서 판정 기준
   --------------------------------------------------------------------------- */
// 대부분의 IR 장애물 감지 모듈은 물체를 감지했을 때 LOW 를 출력합니다.
// 센서 모듈이 반대로 동작하면 false 로 바꾸세요.
const bool IR_ACTIVE_LOW = true;

// 한 쌍의 센서 중 "둘 다" 감지되어야 인정할지, "하나라도" 감지되면 인정할지.
// true  = 둘 다 감지 (오작동 적음 / 놓칠 수 있음)
// false = 하나라도 감지 (잘 잡힘 / 오작동 늘 수 있음)  ← 기본값
const bool ENTRY_REQUIRE_BOTH = false;
const bool EXIT_REQUIRE_BOTH  = false;

// 센서 신호가 이 시간(ms) 이상 유지되어야 진짜 감지로 인정합니다. (노이즈 제거)
// 반응이 느리면 값을 줄이고, 헛감지가 잦으면 값을 늘리세요.
const uint16_t ENTRY_CONFIRM_MS = 120;
const uint16_t EXIT_CONFIRM_MS  = 200;

/* ---------------------------------------------------------------------------
   [설정 4] 경고 동작
   --------------------------------------------------------------------------- */
// 경고가 시작된 뒤 최소 이 시간(ms) 동안은 종료 신호를 무시합니다.
// 진입 센서가 통과 센서까지 같이 건드려서 곧바로 꺼지는 것을 막아줍니다.
const uint16_t WARNING_MIN_MS = 3000;

// 종료 신호가 끝내 들어오지 않을 때 자동으로 대기 상태로 돌아가는 시간(ms).
// 0 으로 두면 자동 복귀 없이 계속 경고합니다.
const uint32_t WARNING_TIMEOUT_MS = 60000UL;

// 부저 경고음: 삐- 쉬고 삐- 하는 간헐음
const uint16_t BUZZER_FREQ_HZ = 2000;   // 경고음 높낮이(Hz). 0 이면 부저를 쓰지 않음
const uint16_t BUZZER_ON_MS   = 200;    // 소리 나는 시간
const uint16_t BUZZER_OFF_MS  = 200;    // 쉬는 시간

// 빨간색 LED 경고등
const bool     RED_LED_BLINK    = true; // false 로 두면 깜빡이지 않고 계속 켜짐
const uint16_t RED_LED_BLINK_MS = 300;  // 깜빡이는 간격(ms)

/* ---------------------------------------------------------------------------
   [설정 5] 버튼 (수동 조작)
   --------------------------------------------------------------------------- */
// INPUT_PULLUP 으로 연결하므로 눌렀을 때 LOW 입니다.
// 버튼 한쪽 다리는 아두이노 핀, 반대쪽 다리는 GND 에 연결합니다.
const bool     BUTTON_ACTIVE_LOW  = true;
const uint16_t BUTTON_DEBOUNCE_MS = 40;   // 버튼 채터링 제거 시간(ms)

/* ---------------------------------------------------------------------------
   [설정 6] 기타
   --------------------------------------------------------------------------- */
const bool     SERIAL_DEBUG    = true;   // 시리얼 모니터로 상태 출력
const long     SERIAL_BAUD     = 9600;
const uint16_t LOOP_INTERVAL_MS = 10;    // 루프 주기(ms). delay 대신 사용

/* ===========================================================================
   ↓↓↓ 아래부터는 동작 로직입니다. 기준값만 바꿀 때는 수정하지 않아도 됩니다. ↓↓↓
   =========================================================================== */

Adafruit_NeoPixel strip1(STRIP1_PIXELS, PIN_STRIP_1, NEO_GRB + NEO_KHZ800);
Adafruit_NeoPixel strip2(STRIP2_PIXELS, PIN_STRIP_2, NEO_GRB + NEO_KHZ800);

// 시스템 상태
enum SystemState {
  STATE_IDLE,     // 대기: 보행자가 인도 위를 정상 이동하는 중
  STATE_WARNING   // 경고: 무단횡단 감지, 경고 시스템 작동 중
};

SystemState systemState = STATE_IDLE;
uint32_t warningStartedAt = 0;   // 경고가 시작된 시각

/* --- 센서 신호를 일정 시간 유지될 때만 인정하는 필터 --- */
struct HoldFilter {
  bool confirmed;     // 확정된 상태
  bool lastRaw;       // 마지막으로 읽은 원본 신호
  uint32_t changedAt; // 원본 신호가 바뀐 시각
};

HoldFilter entryFilter = {false, false, 0};
HoldFilter exitFilter  = {false, false, 0};

// 원본 신호(raw)가 confirmMs 이상 그대로 유지되면 확정 상태로 반영한다.
bool updateHoldFilter(HoldFilter &f, bool raw, uint16_t confirmMs, uint32_t now) {
  if (raw != f.lastRaw) {
    f.lastRaw = raw;
    f.changedAt = now;
  } else if (raw != f.confirmed && (now - f.changedAt) >= confirmMs) {
    f.confirmed = raw;
  }
  return f.confirmed;
}

/* --- 버튼 눌림(누르는 순간)을 잡아내는 디바운스 --- */
struct Button {
  uint8_t pin;
  bool stable;        // 확정된 눌림 상태
  bool lastRaw;
  uint32_t changedAt;
};

Button btnRed   = {PIN_BTN_RED,   false, false, 0};
Button btnGreen = {PIN_BTN_GREEN, false, false, 0};

// 버튼을 "새로 누른 순간"에만 true 를 돌려준다. (누르고 있는 동안 계속 true 아님)
bool buttonPressedEdge(Button &b, uint32_t now) {
  bool raw = (digitalRead(b.pin) == LOW);
  if (!BUTTON_ACTIVE_LOW) raw = !raw;

  if (raw != b.lastRaw) {
    b.lastRaw = raw;
    b.changedAt = now;
    return false;
  }
  if (raw != b.stable && (now - b.changedAt) >= BUTTON_DEBOUNCE_MS) {
    b.stable = raw;
    return raw;   // 눌리는 쪽으로 확정된 순간만 알림
  }
  return false;
}

// IR 센서 한 개 읽기 (감지되면 true)
bool readIr(uint8_t pin) {
  bool low = (digitalRead(pin) == LOW);
  return IR_ACTIVE_LOW ? low : !low;
}

// IR 센서 한 쌍 읽기
bool readIrPair(uint8_t pinA, uint8_t pinB, bool requireBoth) {
  bool a = readIr(pinA);
  bool b = readIr(pinB);
  return requireBoth ? (a && b) : (a || b);
}

void logLine(const __FlashStringHelper *msg) {
  if (SERIAL_DEBUG) Serial.println(msg);
}

/* --- LED 스트립: 횡단보도 방향으로 흐르는 불빛(꼬리 있는 혜성 모양) --- */
void renderStrip(Adafruit_NeoPixel &strip, uint16_t pixels, bool reverse, uint16_t head) {
  strip.clear();
  for (uint8_t tail = 0; tail < COMET_LENGTH; tail++) {
    // 꼬리로 갈수록 어두워지도록 밝기를 나눈다.
    uint16_t divider = tail + 1;
    uint8_t r = WARN_COLOR_R / divider;
    uint8_t g = WARN_COLOR_G / divider;
    uint8_t b = WARN_COLOR_B / divider;

    int16_t index = (int16_t)head - tail;
    while (index < 0) index += pixels;
    index %= pixels;

    if (reverse) index = pixels - 1 - index;
    strip.setPixelColor(index, strip.Color(r, g, b));
  }
  strip.show();
}

void updateStrips(uint32_t now) {
  static uint16_t head = 0;
  static uint32_t lastStep = 0;

  if (now - lastStep < ANIM_STEP_MS) return;
  lastStep = now;

  if (STRIP1_PIXELS > 0) renderStrip(strip1, STRIP1_PIXELS, STRIP1_REVERSE, head % STRIP1_PIXELS);
  if (STRIP2_PIXELS > 0) renderStrip(strip2, STRIP2_PIXELS, STRIP2_REVERSE, head % STRIP2_PIXELS);

  head++;
}

/* --- 부저: 삐- 쉬고 삐- 하는 간헐음 --- */
void updateBuzzer(uint32_t now) {
  static bool soundOn = false;
  static uint32_t lastToggle = 0;

  if (BUZZER_FREQ_HZ == 0) return;

  uint16_t interval = soundOn ? BUZZER_ON_MS : BUZZER_OFF_MS;
  if (now - lastToggle < interval) return;
  lastToggle = now;

  soundOn = !soundOn;
  if (soundOn) tone(PIN_BUZZER, BUZZER_FREQ_HZ);
  else         noTone(PIN_BUZZER);
}

/* --- 빨간색 LED 경고등 --- */
void updateRedLed(uint32_t now) {
  static bool ledOn = false;
  static uint32_t lastToggle = 0;

  if (!RED_LED_BLINK) {
    digitalWrite(PIN_RED_LED, HIGH);
    return;
  }
  if (now - lastToggle < RED_LED_BLINK_MS) return;
  lastToggle = now;

  ledOn = !ledOn;
  digitalWrite(PIN_RED_LED, ledOn ? HIGH : LOW);
}

// 모든 출력 장치를 끈다.
void allOutputsOff() {
  digitalWrite(PIN_RED_LED, LOW);
  noTone(PIN_BUZZER);
  strip1.clear();
  strip1.show();
  strip2.clear();
  strip2.show();
}

void enterWarning(uint32_t now) {
  systemState = STATE_WARNING;
  warningStartedAt = now;
  logLine(F("상태: 무단횡단 감지, 경고 시스템 작동"));
}

void enterIdle() {
  systemState = STATE_IDLE;
  allOutputsOff();
  logLine(F("상태: 상황 종결, 대기 모드 복귀"));
}

void setup() {
  pinMode(PIN_IR_ENTRY_A, INPUT);
  pinMode(PIN_IR_ENTRY_B, INPUT);
  pinMode(PIN_IR_EXIT_A,  INPUT);
  pinMode(PIN_IR_EXIT_B,  INPUT);

  pinMode(PIN_BTN_RED,   INPUT_PULLUP);
  pinMode(PIN_BTN_GREEN, INPUT_PULLUP);

  pinMode(PIN_BUZZER,  OUTPUT);
  pinMode(PIN_RED_LED, OUTPUT);

  strip1.begin();
  strip1.setBrightness(STRIP_BRIGHTNESS);
  strip2.begin();
  strip2.setBrightness(STRIP_BRIGHTNESS);

  allOutputsOff();

  if (SERIAL_DEBUG) {
    Serial.begin(SERIAL_BAUD);
    Serial.println(F("무단횡단 방지 시스템 시작 - 대기 모드"));
  }
}

void loop() {
  uint32_t now = millis();

  // --- 입력 읽기 ---------------------------------------------------------
  bool entryRaw = readIrPair(PIN_IR_ENTRY_A, PIN_IR_ENTRY_B, ENTRY_REQUIRE_BOTH);
  bool exitRaw  = readIrPair(PIN_IR_EXIT_A,  PIN_IR_EXIT_B,  EXIT_REQUIRE_BOTH);

  bool entryDetected = updateHoldFilter(entryFilter, entryRaw, ENTRY_CONFIRM_MS, now);
  bool exitDetected  = updateHoldFilter(exitFilter,  exitRaw,  EXIT_CONFIRM_MS,  now);

  bool redPressed   = buttonPressedEdge(btnRed,   now);  // 수동 발동
  bool greenPressed = buttonPressedEdge(btnGreen, now);  // 수동 종결

  // 자동(IR 센서) 또는 수동(버튼) 어느 쪽이든 같은 로직으로 처리한다.
  bool startSignal = entryDetected || redPressed;
  bool clearSignal = exitDetected  || greenPressed;

  // --- 상태 전이 ---------------------------------------------------------
  switch (systemState) {
    case STATE_IDLE:
      if (startSignal) enterWarning(now);
      break;

    case STATE_WARNING: {
      uint32_t elapsed = now - warningStartedAt;

      // 최소 유지 시간이 지난 뒤에만 종료 신호를 받아들인다.
      if (elapsed >= WARNING_MIN_MS && clearSignal) {
        enterIdle();
        break;
      }
      // 종료 신호가 오지 않아도 일정 시간이 지나면 자동으로 복귀한다.
      if (WARNING_TIMEOUT_MS > 0 && elapsed >= WARNING_TIMEOUT_MS) {
        logLine(F("상태: 종료 신호 없음, 자동 복귀"));
        enterIdle();
        break;
      }
      break;
    }
  }

  // --- 출력 갱신 ---------------------------------------------------------
  if (systemState == STATE_WARNING) {
    updateRedLed(now);
    updateBuzzer(now);
    updateStrips(now);
  }

  delay(LOOP_INTERVAL_MS);
}
