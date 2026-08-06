/* ============================================================================
   무단횡단 방지 시스템 (Jaywalking Warning System)

   천안시 청소년 도시재생 챌린지 (U-CAST) / 5팀 「건영아 잘하자」

   ────────────────────────────────────────────────────────────────────────────
   이 파일은 발표에 쓸 **최종 실물 스케치**입니다.
   IR 센서 4개 + LED 스트립 2줄 + 버튼 2개를 모두 연결한 상태를 기준으로 합니다.
   지금 부품이 덜 모여 미리 조립해 보는 중이라면 `test/test.ino`를 쓰세요.

   `test/test.ino`가 쓰는 D8(왼쪽)·D10(오른쪽) 배선은 그대로 두고, 비어 있는
   D9·D11에 센서를 꽂은 뒤 이 파일을 업로드하면 최종 구성이 됩니다.
   ────────────────────────────────────────────────────────────────────────────

   [동작 요약] — 양방향

   도로를 사이에 두고 **양쪽 연석**에 IR 센서를 2개씩 놓습니다.
   보행자가 어느 쪽에서 출발하든 똑같이 동작합니다.

     [대기]  왼쪽이 먼저 감지   →  [경고 중]  (도착 쪽 = 오른쪽으로 기억)
     [대기]  오른쪽이 먼저 감지 →  [경고 중]  (도착 쪽 = 왼쪽으로 기억)

     [경고 중]  기억해 둔 반대쪽이 감지  →  [대기]  (다 건넜다고 판단)
     [경고 중]  출발한 쪽이 다시 감지    →  경고 유지 (같은 사람이므로 무시)

   경고 중에는 LED 스트립 2줄이 전체 빨간색으로 켜지고,
   운전자 경고용 빨간 LED와 부저가 같은 주기로 점멸합니다.

   빨간 버튼 = 경고 수동 시작, 초록 버튼 = 경고 수동 종료.
   버튼으로 시작한 경고는 출발 쪽이 없으므로 **어느 쪽 센서로도** 종료됩니다.

   ────────────────────────────────────────────────────────────────────────────
   ⚠ [중요] IR 센서를 서로 마주보게 두지 마세요

   이 프로젝트가 쓰는 FC-51 / HW-201 계열 모듈은 송신 LED(투명)와 수신 LED(검정)가
   한 보드에 같이 붙은 **반사형 장애물 감지** 모듈입니다. 앞으로 쏜 적외선이 물체에
   맞고 되돌아오는 것을 감지하며, 빔이 끊기는 것을 보는 송수신 분리형이 아닙니다.

   마주보게 두면 서로의 적외선이 상대 수신부에 직접 들어가 손이 없어도 계속 감지
   상태로 붙어버립니다. 그러면 모호 입력 보호에 걸려 아무 일도 일어나지 않습니다.

     올바름:  도로를 사이에 두고 양쪽 연석에, 모두 보행자가 지나갈 쪽을 향해 설치
     잘못됨:  두 센서를 도로 건너 서로 마주보게 설치

   모듈 위의 파란 나사로 감지 거리(약 2~30cm)를 조절합니다.
   ────────────────────────────────────────────────────────────────────────────

   [이번 MVP에서 제외] — 초기 검토 후 제외한 기능
   - 횡단보도 방향 유도와 흐르는 방향성 LED 애니메이션
   - 펜스, 압력센서(FSR), 초음파센서

   ※ 조정값은 아래 [설정] 구역의 상수만 수정합니다.
   ========================================================================== */

/* 네오픽셀(WS2812) LED 스트립을 제어하는 외부 라이브러리입니다.
   Arduino IDE의 [스케치] → [라이브러리 포함하기] → [라이브러리 관리]에서
   "Adafruit NeoPixel"을 검색해 설치해야 이 줄이 통과합니다.
   이 프로젝트가 쓰는 외부 라이브러리는 이것 하나뿐입니다. */
#include <Adafruit_NeoPixel.h>

/* ---------------------------------------------------------------------------
   [설정 1] 핀 번호

   `const uint8_t`는 "한 번 정하면 바뀌지 않는(const) 0~255 범위의 정수(uint8_t)"
   라는 뜻입니다. 핀 번호처럼 절대 변하면 안 되는 값에 씁니다.
   배선을 바꿨다면 아래 숫자만 고치면 되고, 아래쪽 로직은 손대지 않습니다.

   왼쪽/오른쪽은 도로를 사이에 둔 **두 연석**을 뜻합니다. 어느 쪽을 왼쪽이라
   부르든 상관없고, 두 묶음이 서로 반대편 연석이기만 하면 됩니다.
   --------------------------------------------------------------------------- */
const uint8_t PIN_IR_LEFT_1  = 8;   // 왼쪽 연석 IR 센서 1
const uint8_t PIN_IR_LEFT_2  = 9;   // 왼쪽 연석 IR 센서 2
const uint8_t PIN_IR_RIGHT_1 = 10;  // 오른쪽 연석 IR 센서 1
const uint8_t PIN_IR_RIGHT_2 = 11;  // 오른쪽 연석 IR 센서 2

const uint8_t PIN_WARN_LED   = 3;   // 운전자 경고용 빨간 LED(표지판 LED)
const uint8_t PIN_STRIP_1    = 4;   // 네오픽셀 LED 스트립 1 DIN
const uint8_t PIN_BUZZER     = 5;   // 경고용 부저

const uint8_t PIN_BTN_START  = A0;  // 빨간 버튼: 경고 수동 시작
const uint8_t PIN_BTN_STOP   = A1;  // 초록 버튼: 경고 수동 종료

/* ★ 두 번째 LED 스트립의 DIN 핀은 아직 정하지 않았습니다.
     쓸 핀 번호를 아래 0 자리에 적으세요.

     지금 비어 있는 핀:  D2, D6, D7, D12, D13, A2, A3, A4, A5
     (D0·D1은 시리얼 통신용이라 쓰지 않습니다.)

     0으로 두면 두 번째 스트립을 **연결하지 않은 것으로 보고 건너뜁니다.**
     스트립 1줄만 꽂아도 그대로 동작하며, 나중에 핀 번호만 적으면 2줄이 됩니다. */
const uint8_t PIN_STRIP_2 = 0;   // ← 여기에 핀 번호를 적으세요 (0 = 아직 미정)

// 위 값이 0이 아니면 두 번째 스트립을 사용합니다. 직접 고치지 마세요.
const bool STRIP_2_CONNECTED = (PIN_STRIP_2 != 0);

/* ---------------------------------------------------------------------------
   [설정 2] 네오픽셀 스트립
   경고 상태에서 두 스트립 전체가 빨간색으로 켜지고, 방향 애니메이션은 없습니다.
   --------------------------------------------------------------------------- */

// 각 스트립에 박혀 있는 낱개 LED(픽셀) 개수입니다.
// 이 값이 실물보다 작으면 뒷부분이 안 켜지고, 크면 없는 픽셀에 색을 보내 낭비됩니다.
// 두 스트립의 LED 개수가 다르면 아래 두 값을 각각 다르게 적습니다.
const uint16_t STRIP_1_PIXELS = 10;
const uint16_t STRIP_2_PIXELS = 10;

// 밝기(0~255). 값이 클수록 밝지만 전류를 더 많이 먹습니다.
// 전원이 부족해 색이 흔들리거나 보드가 재부팅되면 이 값을 낮춥니다.
const uint8_t STRIP_BRIGHTNESS = 120;

// 경고 색상을 빨강·초록·파랑 세기(각 0~255)로 나눠 지정합니다.
// (255, 0, 0)은 빨강만 최대 = 빨간색입니다.
const uint8_t WARN_COLOR_R = 255;
const uint8_t WARN_COLOR_G = 0;
const uint8_t WARN_COLOR_B = 0;

/* ---------------------------------------------------------------------------
   [설정 3] 경고 출력 주기
   빨간 경고 LED와 부저가 이 간격으로 함께 점멸합니다.
   --------------------------------------------------------------------------- */

// 300이면 0.3초마다 켜짐↔꺼짐이 바뀝니다(1000 = 1초).
const uint16_t WARNING_BLINK_MS = 300;

// 부저 음의 높이(Hz). 숫자가 클수록 높은 소리입니다.
// 소리 높이를 바꾸려면 `tone()`을 쓸 수 있는 수동(passive) 부저여야 합니다.
const uint16_t BUZZER_FREQUENCY_HZ = 1000;

/* ---------------------------------------------------------------------------
   [설정 4] IR 센서 판정
   --------------------------------------------------------------------------- */

// FC-51 / HW-201 계열은 물체를 감지하면 OUT이 LOW(0V)가 되고 보드의 빨간 LED가
// 켜집니다. 손을 대지 않았는데 계속 감지된다고 나오면 먼저 모듈 위의 파란 나사로
// 감지 거리를 줄이고, 그래도 반대로 동작하면 이 값을 false로 바꿉니다.
const bool IR_ACTIVE_LOW = true;

// 이 모듈은 보드에 풀업이 이미 달려 있어 false로 두어도 됩니다.
// 값이 제멋대로 떠다니면 true로 되돌리세요.
const bool IR_USE_INTERNAL_PULLUP = false;

// 신호가 이 시간(ms) 이상 유지되어야 "진짜 감지"로 확정합니다.
// 짧은 노이즈나 스쳐 지나감을 무시하기 위한 값입니다.
// 잠깐 스쳐도 발동하면 늘리고, 빠르게 지나갈 때 안 잡히면 줄입니다.
const uint16_t IR_CONFIRM_MS = 60;

/* 양쪽이 동시에 감지 중이면 누가 어디서 출발했는지 알 수 없으므로 경고를 시작하지
   않는 보호 장치입니다.

   센서를 마주보게 두었거나 양쪽이 서로를 보고 있어서 이 로그만 계속 나온다면,
   먼저 **센서 배치를 고치세요.** 배치를 못 고치는 상황에서 일단 시연만 해야
   한다면 이 값을 false로 바꾸면 먼저 들어온 쪽으로 무조건 시작합니다. */
const bool IR_AMBIGUITY_GUARD = true;

/* ---------------------------------------------------------------------------
   [설정 5] 안전 복귀(선택 기능)
   기본값 0은 자동 종료를 사용하지 않습니다. 버튼이 있으므로 도착 신호가 없어도
   초록 버튼으로 끌 수 있기 때문입니다.
   0보다 큰 값을 넣으면 그 시간이 지난 뒤 경고를 자동으로 끄고 대기로 돌아갑니다.
   (예: 30000UL을 넣으면 30초 뒤 자동 종료. UL은 "큰 양수"라는 표시입니다.)
   --------------------------------------------------------------------------- */
const uint32_t WARNING_TIMEOUT_MS = 0UL;

/* ---------------------------------------------------------------------------
   [설정 6] 버튼·시리얼
   --------------------------------------------------------------------------- */

// 버튼 한쪽을 GND에 연결하고 INPUT_PULLUP을 쓰므로, 누르면 LOW가 됩니다.
const bool BUTTON_ACTIVE_LOW = true;

// 버튼은 눌리는 순간 접점이 미세하게 떨려(채터링) 여러 번 눌린 것처럼 보입니다.
// 이 시간(ms) 동안 값이 유지돼야 진짜 입력으로 인정합니다.
const uint16_t BUTTON_DEBOUNCE_MS = 40;

// true로 두면 시리얼 모니터에 상태 변화 로그가 찍힙니다(문제 해결에 필요).
const bool SERIAL_DEBUG = true;
const long SERIAL_BAUD = 9600;  // 시리얼 모니터도 같은 9600으로 맞춰야 합니다

/* ===========================================================================
   아래는 동작 로직입니다. 조정값만 바꿀 때는 여기를 건드리지 않아도 됩니다.
   =========================================================================== */

/* 스트립을 다루는 객체를 스트립마다 하나씩 만듭니다.
   (픽셀 수, 연결한 핀, 스트립 종류) 순서로 알려 줍니다.
   NEO_GRB는 색 데이터가 초록-빨강-파랑 순서로 들어가는 흔한 방식,
   NEO_KHZ800은 통신 속도입니다. 대부분의 WS2812 스트립이 이 조합입니다.

   두 번째 스트립은 핀을 아직 안 정했어도 객체는 만들어 둡니다. 다만
   STRIP_2_CONNECTED가 false면 begin()도 show()도 부르지 않으므로
   어떤 핀도 건드리지 않습니다. */
Adafruit_NeoPixel strip1(STRIP_1_PIXELS, PIN_STRIP_1, NEO_GRB + NEO_KHZ800);
Adafruit_NeoPixel strip2(STRIP_2_PIXELS, PIN_STRIP_2, NEO_GRB + NEO_KHZ800);

/* 이 장치가 가질 수 있는 상태는 딱 두 가지입니다.
   enum은 "정해진 이름 중 하나만 담는 값"을 만드는 문법으로,
   0/1 같은 숫자보다 읽기 쉬워서 상태 표현에 씁니다. */
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

SystemState systemState = STATE_IDLE;   // 전원을 켜면 항상 대기로 시작합니다
CrossingSide startedFrom = SIDE_NONE;   // 이번 횡단이 시작된 쪽

// 자동 종료를 쓸 때 "언제 꺼야 하는지" 저장하는 시각입니다.
// WARNING_TIMEOUT_MS가 0이면 항상 0으로 유지되며 자동 종료를 사용하지 않습니다.
uint32_t warningTimeoutAt = 0;

/* IR 신호가 잠깐 튄 것인지, 진짜로 사람이 지나간 것인지 가려내는 필터입니다.
   struct는 서로 관련 있는 값 여러 개를 하나로 묶는 문법입니다.
   센서 4개가 각자 자기 상태를 따로 기억해야 하므로 묶어서 4벌 만듭니다. */
struct HoldFilter {
  uint8_t pin;         // 이 필터가 담당하는 핀 번호
  bool stable;         // 확정된 상태(true = 감지 중)
  bool lastRaw;        // 직전에 읽은 날것 그대로의 값
  uint32_t changedAt;  // 날것 값이 마지막으로 바뀐 시각
};

HoldFilter irLeft1  = {PIN_IR_LEFT_1,  false, false, 0};
HoldFilter irLeft2  = {PIN_IR_LEFT_2,  false, false, 0};
HoldFilter irRight1 = {PIN_IR_RIGHT_1, false, false, 0};
HoldFilter irRight2 = {PIN_IR_RIGHT_2, false, false, 0};

// 버튼도 채터링을 걸러야 하므로 같은 모양의 묶음을 씁니다.
struct Button {
  uint8_t pin;
  bool stable;
  bool lastRaw;
  uint32_t changedAt;
};

Button btnStart = {PIN_BTN_START, false, false, 0};  // 빨간 버튼
Button btnStop  = {PIN_BTN_STOP,  false, false, 0};  // 초록 버튼

bool warningBlinkOn = false;          // 지금 점멸 출력이 켜진 상태인지
uint32_t warningBlinkChangedAt = 0;   // 점멸 상태를 마지막으로 뒤집은 시각

// SERIAL_DEBUG가 false면 아무것도 출력하지 않도록 한 곳에 모아 둡니다.
// __FlashStringHelper와 F()는 문자열을 RAM 대신 플래시 메모리에 두는 방법입니다.
// 우노는 RAM이 2KB뿐이라 한글 로그를 많이 쓰면 금방 부족해집니다.
void logLine(const __FlashStringHelper *message) {
  if (SERIAL_DEBUG) Serial.println(message);
}

// 핀을 읽어 "지금 감지 중인가?"를 true/false로 바꿔 줍니다.
// 모듈이 감지 시 LOW를 내는지 HIGH를 내는지에 따라 뒤집어야 하므로
// 그 판단을 IR_ACTIVE_LOW 한 곳에서만 처리합니다.
bool readIrDetected(uint8_t pin) {
  bool low = (digitalRead(pin) == LOW);
  return IR_ACTIVE_LOW ? low : !low;
}

/* 센서가 "감지 없음 → 감지"로 바뀌는 순간에만 딱 한 번 true를 반환합니다.
   이걸 엣지(edge, 신호가 바뀌는 모서리) 처리라고 부릅니다.

   왜 필요한가: loop()는 1초에 수천 번 돕니다. 그냥 값을 읽어 쓰면
   손을 1초만 대고 있어도 "감지!"가 수천 번 발생합니다.
   그래서 (1) 값이 IR_CONFIRM_MS 이상 유지돼야 확정하고,
          (2) 확정 상태가 실제로 바뀐 그 순간에만 이벤트를 냅니다.

   HoldFilter &filter의 `&`는 "복사본 말고 원본을 직접 고치겠다"는 뜻입니다.
   이게 없으면 필터가 기억한 내용이 함수를 나가는 순간 사라집니다. */
bool detectedEdge(HoldFilter &filter, uint32_t now) {
  bool raw = readIrDetected(filter.pin);

  // 값이 방금 바뀌었다면, 바뀐 시각을 기록하고 확정 시간을 처음부터 다시 셉니다.
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

/* 스트립 하나를 경고 색으로 켜거나(on = true) 완전히 끕니다(on = false).

   setPixelColor()는 "몇 번째 픽셀을 무슨 색으로 할지" 메모리에만 적어 둡니다.
   실제로 스트립에 데이터를 쏘는 것은 마지막의 show() 한 번입니다.
   그래서 반복문으로 전부 정해 놓고 show()를 한 번만 부릅니다. */
void setStripColor(Adafruit_NeoPixel &strip, uint16_t pixels, bool on) {
  uint32_t color = on ? strip.Color(WARN_COLOR_R, WARN_COLOR_G, WARN_COLOR_B) : 0;
  for (uint16_t i = 0; i < pixels; i++) {
    strip.setPixelColor(i, color);
  }
  strip.show();
}

// 연결된 스트립을 모두 경고 색으로 켜거나 완전히 끕니다.
// 두 번째 스트립은 핀 번호를 정했을 때만 건드립니다.
void applyStrips(bool on) {
  setStripColor(strip1, STRIP_1_PIXELS, on);
  if (STRIP_2_CONNECTED) {
    setStripColor(strip2, STRIP_2_PIXELS, on);
  }
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

// 모든 출력을 한 번에 끕니다. 끄는 곳이 여러 군데라 함수로 묶어 둡니다.
void allOutputsOff() {
  applyBlinkOutputs(false);
  applyStrips(false);
}

// 대기 → 경고로 넘어갈 때 해야 할 일을 전부 모아 둔 함수입니다.
// from은 보행자가 출발한 쪽, reason은 시리얼 로그에 남길 설명입니다.
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

  applyStrips(true);
  applyBlinkOutputs(true);

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

/* 자동 종료 시각이 지났는지 확인합니다.

   millis()는 전원을 켠 뒤 흐른 밀리초인데, 약 49.7일이 지나면 0으로 되돌아갑니다.
   그때 `now >= warningTimeoutAt` 같은 단순 비교는 틀린 답을 냅니다.
   뺄셈 결과를 부호 있는 정수로 보면 되돌아가는 순간에도 정확히 비교됩니다. */
bool warningTimedOut(uint32_t now) {
  if (warningTimeoutAt == 0) return false;  // 0 = 자동 종료 사용 안 함
  return ((int32_t)(now - warningTimeoutAt) >= 0);
}

/* 점멸을 담당합니다. delay()를 쓰면 기다리는 동안 센서와 버튼을 못 읽으므로,
   "마지막으로 뒤집은 뒤 WARNING_BLINK_MS가 지났는가?"만 확인하고 바로 빠져나옵니다.
   이렇게 하면 점멸 중에도 loop()가 멈추지 않고 계속 입력을 받습니다. */
void updateBlinkOutputs(uint32_t now) {
  if ((now - warningBlinkChangedAt) < WARNING_BLINK_MS) return;
  warningBlinkChangedAt = now;
  applyBlinkOutputs(!warningBlinkOn);  // ! 는 반대로 뒤집기(켜짐↔꺼짐)
}

// setup()은 전원을 켜거나 리셋했을 때 딱 한 번 실행됩니다.
// 여기서 각 핀을 입력으로 쓸지 출력으로 쓸지 정합니다.
void setup() {
  // INPUT_PULLUP은 아두이노 내부 저항을 붙여 평소 값을 HIGH로 올려 둡니다.
  // 그래서 외부 풀업 저항 없이도 값이 떠다니지 않습니다.
  uint8_t irMode = IR_USE_INTERNAL_PULLUP ? INPUT_PULLUP : INPUT;
  pinMode(PIN_IR_LEFT_1,  irMode);
  pinMode(PIN_IR_LEFT_2,  irMode);
  pinMode(PIN_IR_RIGHT_1, irMode);
  pinMode(PIN_IR_RIGHT_2, irMode);

  pinMode(PIN_BTN_START, INPUT_PULLUP);
  pinMode(PIN_BTN_STOP,  INPUT_PULLUP);

  pinMode(PIN_WARN_LED, OUTPUT);
  pinMode(PIN_BUZZER,   OUTPUT);

  strip1.begin();                        // 스트립 1 통신 시작
  strip1.setBrightness(STRIP_BRIGHTNESS);

  // 두 번째 스트립은 핀 번호를 정했을 때만 시작합니다.
  // 정하지 않았으면 어떤 핀도 출력으로 바꾸지 않습니다.
  if (STRIP_2_CONNECTED) {
    strip2.begin();
    strip2.setBrightness(STRIP_BRIGHTNESS);
  }

  allOutputsOff();  // 켜자마자 뭔가 켜져 있으면 안 되므로 전부 끕니다

  if (SERIAL_DEBUG) {
    Serial.begin(SERIAL_BAUD);
    Serial.println(F("무단횡단 방지 시스템 시작 - 대기 상태 (양방향)"));
    Serial.println(F("왼쪽 IR=D8·D9, 오른쪽 IR=D10·D11, LED=D3, 스트립1=D4, 부저=D5"));
    Serial.println(F("빨간 버튼(A0)=경고 수동 시작, 초록 버튼(A1)=경고 수동 종료"));
    if (!STRIP_2_CONNECTED) {
      Serial.println(F("주의: PIN_STRIP_2가 0입니다. 두 번째 스트립을 건너뜁니다."));
    }
  }
}

// loop()는 setup() 뒤에 끝없이 반복 실행됩니다.
// 이 안에서는 절대 delay()로 멈추지 않고, 매번 "지금 시각"을 기준으로 판단합니다.
void loop() {
  uint32_t now = millis();  // 한 바퀴 도는 동안 같은 시각을 쓰도록 먼저 읽어 둡니다

  // 센서 4개를 모두 읽습니다. 조건문 안에서 읽으면 앞쪽이 true일 때
  // 뒤쪽 필터가 갱신되지 않아 상태가 어긋나므로, 먼저 전부 읽어 둡니다.
  bool leftEvent1  = detectedEdge(irLeft1,  now);
  bool leftEvent2  = detectedEdge(irLeft2,  now);
  bool rightEvent1 = detectedEdge(irRight1, now);
  bool rightEvent2 = detectedEdge(irRight2, now);

  // || 는 "또는"입니다. 같은 연석의 센서 둘 중 하나만 감지돼도 그쪽으로 봅니다.
  bool leftEvent  = leftEvent1  || leftEvent2;
  bool rightEvent = rightEvent1 || rightEvent2;

  // 지금 그쪽 센서가 계속 감지 상태로 붙어 있는지(모호 입력 판정용).
  bool leftStable  = irLeft1.stable  || irLeft2.stable;
  bool rightStable = irRight1.stable || irRight2.stable;

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
        if (IR_AMBIGUITY_GUARD && rightStable) {
          logLine(F("입력 무시: 오른쪽이 계속 감지 중 - 센서 배치를 확인하세요"));
        } else {
          enterWarning(now, SIDE_LEFT, F("왼쪽에서 차도 진입"));
        }
      } else if (rightEvent) {
        if (IR_AMBIGUITY_GUARD && leftStable) {
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
        enterIdle(F("제한시간 경과 - 선택 기능 자동 복귀"));
        break;
      }

      updateBlinkOutputs(now);  // 여기까지 왔으면 경고 유지 중 → 점멸 계속
      break;
    }
  }
}
