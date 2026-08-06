/* ============================================================================
   무단횡단 방지 시스템 — IR 센서 2개 축소판 스케치

   천안시 청소년 도시재생 챌린지 (U-CAST) / 5팀 「건영아 잘하자」

   ────────────────────────────────────────────────────────────────────────────
   [이 파일의 역할]
   최종 발표용 회로는 IR 센서를 4개(진입 2 + 통과 2) 씁니다. 그 기준 스케치는
   저장소 루트의 `main.ino`이며, 이 파일은 그것을 대체하지 않습니다.

   지금은 IR 센서가 2개뿐이고 버튼도 아직 없으며 스트립도 1줄뿐이므로,
   **가진 부품만으로 조립하고 동작을 확인**하려고 만든 축소판입니다.
   (최종 구성은 IR 4개 + 스트립 2줄 + 버튼 2개입니다.)

   [지금 배선 — 실물 브레드보드 기준]

     왼쪽 연석 IR   OUT → D8
     오른쪽 연석 IR OUT → D10
     빨간 경고 LED     → D3  (220~330Ω 직렬)
     네오픽셀 스트립 DIN → D4  (330~470Ω 직렬 권장)
     부저          (+) → D5
     빨간 버튼(선택)    → A0  ↔ GND
     초록 버튼(선택)    → A1  ↔ GND

   각 IR 모듈은 `VCC` → 5V, `GND` → GND도 함께 연결합니다.
   A0·A1은 디지털 입력으로 사용합니다(우노에서 가능합니다).
   D0·D1은 시리얼 통신에 쓰이므로 아무것도 연결하지 않습니다.

   [센서를 4개로 늘릴 때]
   비어 있는 **D9(왼쪽 IR 2)** 와 **D11(오른쪽 IR 2)** 에 새 센서를 꽂기만 하면
   됩니다. 기존 선은 하나도 옮기지 않고 `main.ino`를 업로드하면 최종 구성이 됩니다.
   (두 번째 스트립 핀은 `main.ino`의 `PIN_STRIP_2`에서 직접 정합니다.)

   ────────────────────────────────────────────────────────────────────────────
   ⚠ [중요] IR 센서 두 개를 서로 마주보게 두지 마세요

   이 프로젝트가 쓰는 FC-51 / HW-201 계열 모듈은 **한 보드에 송신 LED(투명)와
   수신 LED(검정)가 같이 붙어 있는 "반사형 장애물 감지" 모듈**입니다.
   앞으로 쏜 적외선이 물체에 맞고 **되돌아오는 것**을 감지합니다.
   빔이 끊기는 것을 보는 송수신 분리형(브레이크빔)이 아닙니다.

   두 개를 마주보게 두면 한쪽의 송신 LED가 반대쪽 수신부에 그대로 꽂혀서,
   손을 대지 않아도 **계속 감지 상태로 붙어버립니다.** 그러면 아래 모호 입력
   보호(IR_AMBIGUITY_GUARD)에 걸려 아무 일도 일어나지 않습니다.

     올바름:  도로를 사이에 둔 양쪽 연석에, 둘 다 보행자가 지나갈 쪽을 향해 설치
     잘못됨:  두 센서를 도로 건너 서로 마주보게 설치 (서로 간섭해서 안 됨)

   센서 위의 파란 가변저항으로 감지 거리를 조절합니다(대략 2~30cm).
   아무것도 없는데 감지된다고 나오면 거리를 줄이는 쪽으로 돌리세요.
   ────────────────────────────────────────────────────────────────────────────

   [동작] — 양방향

   어느 쪽에서 출발하든 똑같이 동작합니다.

     [대기]  왼쪽(D8)이 먼저 감지    →  [경고 중]  (도착 쪽 = 오른쪽으로 기억)
     [대기]  오른쪽(D10)이 먼저 감지 →  [경고 중]  (도착 쪽 = 왼쪽으로 기억)

     [경고 중]  기억해 둔 반대쪽이 감지 →  [대기]
     [경고 중]  출발한 쪽이 다시 감지   →  경고 유지

   경고 중에는 스트립(D4)이 전체 빨간색으로 켜지고, 빨간 LED(D3)가 점멸하며,
   부저(D5)는 800Hz↔1800Hz를 오르내리는 사이렌 소리를 냅니다(각 구간 0.5초).
   버튼이 없으므로 도착 신호가 없어도 WARNING_TIMEOUT_MS 뒤에 자동으로 꺼집니다.

   상태 전이 규칙은 `main.ino`와 같습니다.
   **알고리즘을 바꿀 때는 두 파일을 함께 고쳐야 합니다.**

   ※ 조정값은 아래 [설정] 구역의 상수만 수정합니다.
   ※ 이 파일은 반드시 `test/` 폴더 안에 두어야 합니다. Arduino IDE는 한 스케치
     폴더의 모든 `.ino`를 합쳐 컴파일하므로, `main.ino` 옆에 두면 `setup()`과
     `loop()`가 중복 정의되어 빌드가 실패합니다.
   ========================================================================== */

/* 네오픽셀(WS2812) 스트립 제어 라이브러리입니다.
   Arduino IDE의 라이브러리 관리에서 "Adafruit NeoPixel"을 설치해야 합니다. */
#include <Adafruit_NeoPixel.h>

/* ---------------------------------------------------------------------------
   [설정 1] 핀 번호 — 지금 브레드보드에 꽂혀 있는 그대로입니다.
   --------------------------------------------------------------------------- */
const uint8_t PIN_IR_LEFT   = 8;   // 왼쪽 연석 IR 센서 OUT
const uint8_t PIN_IR_RIGHT  = 9;  // 오른쪽 연석 IR 센서 OUT
const uint8_t PIN_WARN_LED  = 3;   // 운전자 경고용 빨간 LED
const uint8_t PIN_STRIP     = 4;   // 네오픽셀 LED 스트립 DIN (1줄)
const uint8_t PIN_BUZZER    = 5;   // 경고용 부저
const uint8_t PIN_BTN_START = A0;  // 빨간 버튼: 경고 수동 시작 (지금은 미연결)
const uint8_t PIN_BTN_STOP  = A1;  // 초록 버튼: 경고 수동 종료 (지금은 미연결)

// D9·D11은 센서를 4개로 늘릴 때 쓸 자리라 비워 둡니다.
// 두 번째 스트립도 아직 없습니다(최종 구성에서 `main.ino`의 PIN_STRIP_2 사용).

/* ---------------------------------------------------------------------------
   [설정 2] 지금 없는 부품
   --------------------------------------------------------------------------- */

// 버튼을 아직 안 달았으므로 false입니다. 버튼을 A0·A1에 연결하면 true로 바꾸세요.
// false일 때는 버튼 핀을 아예 읽지 않아, 뜬 핀 때문에 오작동하지 않습니다.
const bool USE_BUTTONS = false;

/* ---------------------------------------------------------------------------
   [설정 3] 네오픽셀 스트립
   --------------------------------------------------------------------------- */

// 스트립에 박혀 있는 낱개 LED(픽셀) 개수입니다.
const uint16_t STRIP_PIXELS = 10;

// 밝기(0~255). 전원이 부족해 색이 흔들리거나 보드가 재부팅되면 낮춥니다.
const uint8_t STRIP_BRIGHTNESS = 120;

// 경고 색상을 빨강·초록·파랑 세기(각 0~255)로 지정합니다.
const uint8_t WARN_COLOR_R = 255;
const uint8_t WARN_COLOR_G = 0;
const uint8_t WARN_COLOR_B = 0;

/* ---------------------------------------------------------------------------
   [설정 4] 경고 출력 주기
   --------------------------------------------------------------------------- */
// 아래 점멸 간격은 **빨간 경고 LED**에만 쓰입니다. 부저는 사이렌 설정을 따릅니다.
const uint16_t WARNING_BLINK_MS = 300;      // 0.3초마다 켜짐↔꺼짐

/* 부저는 `tone()`으로 음 높이를 바꾸는 **수동(passive) 부저**를 기준으로 합니다.
   경고 중에는 소리를 끊지 않고 주파수만 계속 바꿉니다.
     SIREN_MIN_HZ → SIREN_MAX_HZ 로 SIREN_SWEEP_MS 동안 상승
     SIREN_MAX_HZ → SIREN_MIN_HZ 로 SIREN_SWEEP_MS 동안 하강  (반복)
   `main.ino`와 값·동작이 같아야 합니다. */
const uint16_t SIREN_MIN_HZ    = 800;   // 사이렌 최저 주파수
const uint16_t SIREN_MAX_HZ    = 1800;  // 사이렌 최고 주파수
const uint16_t SIREN_SWEEP_MS  = 500;   // 상승 한 번 / 하강 한 번에 걸리는 시간
const uint16_t SIREN_UPDATE_MS = 25;    // 주파수를 다시 계산하는 간격

/* ---------------------------------------------------------------------------
   [설정 5] IR 센서 판정
   --------------------------------------------------------------------------- */

// FC-51 / HW-201 계열은 물체를 감지하면 OUT이 LOW(0V)가 되고 보드의 빨간 LED가
// 켜집니다. 손을 대지 않았는데 계속 감지된다고 나오면 먼저 감도 나사를 조절하고,
// 그래도 반대로 동작하면 이 값을 false로 바꿉니다.
const bool IR_ACTIVE_LOW = true;

// 이 모듈은 보드에 풀업이 이미 달려 있어 false로 두어도 됩니다.
// 값이 제멋대로 떠다니면 true로 되돌리세요.
const bool IR_USE_INTERNAL_PULLUP = false;

// 신호가 이 시간(ms) 이상 유지되어야 감지로 확정합니다(짧은 노이즈 무시).
// 손을 빠르게 휘저어도 안 잡히면 이 값을 40~60으로 줄이세요.
const uint16_t IR_CONFIRM_MS = 60;

/* 양쪽이 동시에 감지 중이면 누가 어디서 출발했는지 알 수 없으므로 경고를 시작하지
   않는 보호 장치입니다.

   센서를 마주보게 두었거나 두 센서가 너무 가까워서 계속 이 로그만 나온다면,
   먼저 **센서 배치를 고치세요.** 배치를 못 고치는 상황에서 일단 시연만 해야
   한다면 이 값을 false로 바꾸면 먼저 들어온 쪽으로 무조건 시작합니다. */
const bool IR_AMBIGUITY_GUARD = true;

// 두 IR의 현재 값을 이 간격(ms)으로 시리얼 모니터에 찍습니다.
// 배선과 감도를 맞출 때 이 출력을 보면서 나사를 돌리면 됩니다.
// 0을 넣으면 주기 출력을 끄고 상태 변화 로그만 남깁니다.
// (#ifndef은 회귀 테스트가 이 값을 0으로 바꿔 "로그 끔" 동작까지 검증하기 위한
//  것입니다. Arduino IDE에서는 항상 아래 400이 그대로 쓰입니다.)
#ifndef IR_LOG_INTERVAL_MS
const uint16_t IR_LOG_INTERVAL_MS = 400;
#endif

/* ---------------------------------------------------------------------------
   [설정 6] 안전 복귀
   버튼이 없어서 손으로 끌 방법이 없으므로, 통과 IR을 못 잡아도 이 시간이 지나면
   자동으로 경고를 끄고 대기로 돌아갑니다. 0을 넣으면 자동 종료를 쓰지 않습니다.
   (`main.ino`는 버튼이 있으므로 기본값이 0입니다.)
   --------------------------------------------------------------------------- */
const uint32_t WARNING_TIMEOUT_MS = 10000UL;  // 10초

/* ---------------------------------------------------------------------------
   [설정 7] 부팅 자가진단
   전원을 켜면 스트립·LED·부저를 한 번씩 켜 봅니다. 센서를 만지기 전에 출력 배선이
   맞는지부터 확인할 수 있습니다. 확인이 끝났으면 false로 꺼도 됩니다.
   --------------------------------------------------------------------------- */
const bool STARTUP_SELFTEST = true;
const uint16_t SELFTEST_ON_MS = 700;   // 켜 두는 시간
const uint16_t SELFTEST_END_MS = 1400; // 이 시각 이후 정상 동작 시작

/* ---------------------------------------------------------------------------
   [설정 8] 버튼·시리얼
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

/* 양방향 처리의 핵심입니다. 보행자가 **어느 쪽에서 출발했는지**를 기억해 두었다가,
   그 반대편 센서가 감지될 때만 "다 건넜다"고 판단합니다.
   SIDE_NONE은 빨간 버튼으로 시작해서 출발 쪽을 모르는 경우입니다. */
enum CrossingSide {
  SIDE_NONE,
  SIDE_LEFT,
  SIDE_RIGHT
};

SystemState systemState = STATE_IDLE;
CrossingSide startedFrom = SIDE_NONE;   // 이번 횡단이 시작된 쪽

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

HoldFilter irLeft  = {PIN_IR_LEFT,  false, false, 0};
HoldFilter irRight = {PIN_IR_RIGHT, false, false, 0};

// 버튼도 채터링을 걸러야 하므로 같은 모양의 묶음을 씁니다.
struct Button {
  uint8_t pin;
  bool stable;
  bool lastRaw;
  uint32_t changedAt;
};

Button btnStart = {PIN_BTN_START, false, false, 0};  // 빨간 버튼
Button btnStop  = {PIN_BTN_STOP,  false, false, 0};  // 초록 버튼

bool warningBlinkOn = false;         // 지금 빨간 경고 LED가 켜진 상태인지
uint32_t warningBlinkChangedAt = 0;  // 점멸 상태를 마지막으로 뒤집은 시각

bool buzzerOn = false;               // 지금 부저가 울리는 중인지
bool sirenRising = true;             // 지금 구간이 상승(true)인지 하강(false)인지
uint16_t sirenHz = 0;                // 지금 내보내고 있는 주파수(0 = 정지)
uint32_t sirenSweepStartedAt = 0;    // 지금 구간이 시작된 시각
uint32_t sirenUpdatedAt = 0;         // 주파수를 마지막으로 갱신한 시각

bool selftestDone = false;           // 부팅 자가진단이 끝났는지
uint32_t irLoggedAt = 0;             // IR 값을 마지막으로 찍은 시각

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
// 버튼을 안 달았으면(USE_BUTTONS = false) 핀을 읽지 않고 항상 false를 돌려줍니다.
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

// 운전자 경고용 빨간 LED를 켜거나 끕니다.
// 부저는 사이렌이라 켜짐/꺼짐이 아니라 주파수가 바뀌므로 따로 관리합니다.
void applyBlinkOutputs(bool on) {
  warningBlinkOn = on;
  digitalWrite(PIN_WARN_LED, on ? HIGH : LOW);
}

/* 지금 구간에서 elapsed(ms)만큼 지났을 때의 주파수를 계산합니다.
   우노에는 부동소수점 연산 장치가 없으므로 정수 연산만 씁니다.
     상승 구간: MIN + (MAX-MIN) * 지난시간 / 구간길이
     하강 구간: MAX - (MAX-MIN) * 지난시간 / 구간길이 */
uint16_t sirenFrequencyAt(uint32_t elapsed) {
  if (elapsed > SIREN_SWEEP_MS) elapsed = SIREN_SWEEP_MS;

  uint32_t span = (uint32_t)SIREN_MAX_HZ - (uint32_t)SIREN_MIN_HZ;
  uint32_t moved = (span * elapsed) / SIREN_SWEEP_MS;

  return sirenRising ? (uint16_t)(SIREN_MIN_HZ + moved)
                     : (uint16_t)(SIREN_MAX_HZ - moved);
}

// 경고 시작 시 사이렌을 최저 주파수부터 올리기 시작합니다.
void startBuzzer(uint32_t now) {
  buzzerOn = true;
  sirenRising = true;
  sirenSweepStartedAt = now;
  sirenUpdatedAt = now;
  sirenHz = SIREN_MIN_HZ;
  tone(PIN_BUZZER, sirenHz);
}

// 경고가 끝나면 즉시 소리를 멈춥니다.
void stopBuzzer() {
  buzzerOn = false;
  sirenHz = 0;
  noTone(PIN_BUZZER);
}

/* 사이렌은 소리를 끊지 않고 주파수만 바꿉니다. delay()를 쓰지 않고 "마지막으로
   갱신한 뒤 얼마나 지났는가"만 확인하므로, 사이렌 중에도 센서와 버튼을 계속
   읽습니다. 시각 계산은 모두 뺄셈이라 millis()가 되돌아가도 어긋나지 않습니다. */
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
  applyBlinkOutputs(false);
  stopBuzzer();
  applyStrip(false);
}

// 대기 → 경고로 넘어갈 때 해야 할 일을 모아 둔 함수입니다.
void enterWarning(uint32_t now, CrossingSide from, const __FlashStringHelper *reason) {
  systemState = STATE_WARNING;
  startedFrom = from;
  warningBlinkChangedAt = now;

  if (WARNING_TIMEOUT_MS > 0) {
    warningTimeoutAt = now + WARNING_TIMEOUT_MS;
    if (warningTimeoutAt == 0) warningTimeoutAt = 1;  // 0은 "사용 안 함"이므로 피합니다
  } else {
    warningTimeoutAt = 0;
  }

  applyStrip(true);
  applyBlinkOutputs(true);
  startBuzzer(now);

  if (SERIAL_DEBUG) {
    Serial.print(F("상태: 경고 시작 - "));
    Serial.println(reason);
  }
}

// 경고 → 대기로 돌아갈 때의 처리입니다.
void enterIdle(const __FlashStringHelper *reason) {
  systemState = STATE_IDLE;
  startedFrom = SIDE_NONE;  // 다음 횡단을 위해 출발 쪽 기억을 지웁니다
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

/* 센서 하나의 원시 핀값과 판정을 `D8=1 없음` 형태로 찍습니다.
   `핀값`은 digitalRead 결과 그대로(0 = LOW, 1 = HIGH),
   `감지/없음`은 IR_ACTIVE_LOW와 확정 시간(IR_CONFIRM_MS)까지 적용한 판정입니다. */
void logIrSensor(const __FlashStringHelper *label, const HoldFilter &filter) {
  Serial.print(label);
  Serial.print(digitalRead(filter.pin));
  Serial.print(filter.stable ? F(" 감지  ") : F(" 없음  "));
}

/* 지금 값들을 IR_LOG_INTERVAL_MS 간격으로 한 줄로 찍습니다(`main.ino`와 같은 형식).
   loop()마다 찍으면 초당 수천 줄이 나오므로 간격을 둡니다.
   상태 전환과 오류(모호 입력 등)는 이것과 별개로 발생 즉시 따로 찍힙니다.

   손을 안 댔는데 판정이 계속 `감지`면 센서 배치나 감도 나사를 손봐야 합니다. */
void logIrValues(uint32_t now) {
  if (!SERIAL_DEBUG || IR_LOG_INTERVAL_MS == 0) return;
  if ((now - irLoggedAt) < IR_LOG_INTERVAL_MS) return;
  irLoggedAt = now;

  logIrSensor(F("D8="),  irLeft);
  logIrSensor(F("D10="), irRight);

  Serial.print(systemState == STATE_WARNING ? F("STATE_WARNING") : F("STATE_IDLE"));

  Serial.print(F(" "));
  if (startedFrom == SIDE_LEFT) {
    Serial.print(F("SIDE_LEFT"));
  } else if (startedFrom == SIDE_RIGHT) {
    Serial.print(F("SIDE_RIGHT"));
  } else {
    Serial.print(F("SIDE_NONE"));
  }

  Serial.print(systemState == STATE_WARNING ? F(" 스트립=빨강") : F(" 스트립=꺼짐"));
  Serial.print(warningBlinkOn ? F(" LED=켜짐") : F(" LED=꺼짐"));

  Serial.print(F(" 부저="));
  if (buzzerOn) {
    Serial.print((int)sirenHz);
    Serial.print(F("Hz"));
    Serial.print(sirenRising ? F("(상승)") : F("(하강)"));
  } else {
    Serial.print(F("꺼짐"));
  }

  // 이 축소판은 버튼이 없어 자동 복귀가 기본으로 켜져 있습니다.
  Serial.print(F(" 자동복귀="));
  if (WARNING_TIMEOUT_MS == 0) {
    Serial.println(F("사용안함"));
  } else if (warningTimeoutAt == 0) {
    Serial.println(F("대기중"));
  } else {
    int32_t remain = (int32_t)(warningTimeoutAt - now);
    if (remain < 0) remain = 0;
    Serial.print((int)(remain / 1000));
    Serial.println(F("초 남음"));
  }
}

/* 부팅 자가진단입니다. delay()를 쓰지 않고 millis() 시각만 보고 단계를 넘깁니다.
   아직 진단 중이면 true를 돌려주고, loop()는 거기서 바로 빠져나갑니다. */
bool runSelftest(uint32_t now) {
  if (!STARTUP_SELFTEST || selftestDone) return false;

  if (now < SELFTEST_ON_MS) {
    // 1단계: 스트립·LED·부저를 전부 켜서 배선이 맞는지 눈과 귀로 확인
    // 부저도 실제 경고와 같은 사이렌으로 울려야 소리까지 확인할 수 있습니다.
    applyStrip(true);
    applyBlinkOutputs(true);
    if (!buzzerOn) startBuzzer(now);
    updateBuzzerSiren(now);
    return true;
  }

  if (now < SELFTEST_END_MS) {
    // 2단계: 전부 끄고 잠시 대기
    allOutputsOff();
    return true;
  }

  // 3단계: 진단 종료. 여기서부터 센서를 읽기 시작합니다.
  selftestDone = true;
  allOutputsOff();
  logLine(F("자가진단 완료 - 지금부터 IR 센서를 읽습니다"));
  return false;
}

// setup()은 전원을 켜거나 리셋했을 때 딱 한 번 실행됩니다.
void setup() {
  // INPUT_PULLUP은 내부 저항을 붙여 평소 값을 HIGH로 올려 둡니다.
  uint8_t irMode = IR_USE_INTERNAL_PULLUP ? INPUT_PULLUP : INPUT;
  pinMode(PIN_IR_LEFT,  irMode);
  pinMode(PIN_IR_RIGHT, irMode);

  // 버튼을 안 달았으면 핀 설정도 하지 않습니다.
  if (USE_BUTTONS) {
    pinMode(PIN_BTN_START, INPUT_PULLUP);
    pinMode(PIN_BTN_STOP,  INPUT_PULLUP);
  }

  pinMode(PIN_WARN_LED, OUTPUT);
  pinMode(PIN_BUZZER,   OUTPUT);

  strip.begin();                        // 스트립 통신 시작
  strip.setBrightness(STRIP_BRIGHTNESS);

  allOutputsOff();  // 켜자마자 뭔가 켜져 있으면 안 되므로 전부 끕니다

  if (SERIAL_DEBUG) {
    Serial.begin(SERIAL_BAUD);
    Serial.println(F("무단횡단 방지 시스템(IR 2개 축소판) 시작 - 양방향"));
    Serial.println(F("왼쪽 IR=D8, 오른쪽 IR=D10, LED=D3, 스트립=D4, 부저=D5"));
    if (!USE_BUTTONS) {
      Serial.println(F("버튼 없음 - 반대편 도착 또는 자동 복귀로만 경고가 꺼집니다"));
    }
    Serial.println(F("두 센서를 마주보게 두지 마세요. 도로 양쪽 연석에 같은 방향으로 두세요."));
  }
}

// loop()는 setup() 뒤에 끝없이 반복 실행됩니다.
void loop() {
  uint32_t now = millis();  // 한 바퀴 도는 동안 같은 시각을 쓰도록 먼저 읽어 둡니다

  // 부팅 자가진단이 진행 중이면 센서를 읽지 않고 바로 빠져나갑니다.
  if (runSelftest(now)) return;

  logIrValues(now);

  // 조건문 안에서 읽으면 앞쪽이 true일 때 뒤쪽 필터가 갱신되지 않아
  // 상태가 어긋나므로, 두 센서를 먼저 모두 읽어 둡니다.
  bool leftEvent  = detectedEdge(irLeft,  now);
  bool rightEvent = detectedEdge(irRight, now);

  bool redButtonPressed   = buttonPressedEdge(btnStart, now);
  bool greenButtonPressed = buttonPressedEdge(btnStop,  now);

  // 지금 상태가 무엇이냐에 따라 같은 입력도 다르게 처리합니다.
  switch (systemState) {
    case STATE_IDLE:
      if (redButtonPressed) {
        // 빨간 버튼은 방향·센서와 무관한 강제 시작이므로 항상 우선합니다.
        // 출발 쪽을 모르므로 SIDE_NONE으로 두고, 어느 쪽 센서로든 종료되게 합니다.
        enterWarning(now, SIDE_NONE, F("빨간 버튼 수동 시작"));
      } else if (leftEvent && rightEvent) {
        // 같은 순간에 양쪽이 함께 잡히면 누가 어디서 출발했는지 알 수 없습니다.
        logLine(F("입력 무시: 양쪽이 동시에 감지됨 - 센서 배치를 확인하세요"));
      } else if (leftEvent) {
        // 반대쪽이 계속 감지 상태로 붙어 있으면 도착 판정을 믿을 수 없습니다.
        if (IR_AMBIGUITY_GUARD && irRight.stable) {
          logLine(F("입력 무시: 오른쪽이 계속 감지 중 - 센서 배치를 확인하세요"));
        } else {
          enterWarning(now, SIDE_LEFT, F("왼쪽에서 차도 진입"));
        }
      } else if (rightEvent) {
        if (IR_AMBIGUITY_GUARD && irLeft.stable) {
          logLine(F("입력 무시: 왼쪽이 계속 감지 중 - 센서 배치를 확인하세요"));
        } else {
          enterWarning(now, SIDE_RIGHT, F("오른쪽에서 차도 진입"));
        }
      }
      break;

    case STATE_WARNING: {
      /* 출발한 쪽의 **반대편**이 감지될 때만 다 건넜다고 봅니다.
         버튼으로 시작해서 출발 쪽을 모르면(SIDE_NONE) 어느 쪽이든 종료로 봅니다. */
      bool arrived;
      if (startedFrom == SIDE_LEFT) {
        arrived = rightEvent;
      } else if (startedFrom == SIDE_RIGHT) {
        arrived = leftEvent;
      } else {
        arrived = leftEvent || rightEvent;
      }

      // break는 switch를 빠져나가라는 뜻입니다.
      // 종료가 결정되면 아래 점멸 처리를 건너뛰어야 하므로 바로 나갑니다.
      if (arrived) {
        enterIdle(F("반대편 도착 - 횡단 완료"));
        break;
      }

      if (greenButtonPressed) {
        // 초록 버튼은 방향·센서와 무관한 강제 종료입니다.
        enterIdle(F("초록 버튼 수동 종료"));
        break;
      }

      // 출발한 쪽이 다시 감지되는 것은 같은 사람이 센서 앞에 머무는 경우이므로
      // 상태를 그대로 유지합니다. 빨간 버튼 재입력도 마찬가지입니다.
      if ((startedFrom == SIDE_LEFT && leftEvent) ||
          (startedFrom == SIDE_RIGHT && rightEvent)) {
        logLine(F("경고 유지: 출발한 쪽 재감지"));
      }
      if (redButtonPressed) logLine(F("경고 유지: 빨간 버튼 재입력"));

      if (warningTimedOut(now)) {
        enterIdle(F("제한시간 경과 - 자동 복귀"));
        break;
      }

      // 여기까지 왔으면 경고 유지 중 → LED 점멸과 부저 사이렌을 계속 돌립니다.
      updateBlinkOutputs(now);
      updateBuzzerSiren(now);
      break;
    }
  }
}
