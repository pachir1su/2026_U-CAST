/* ============================================================================
   무단횡단 방지 시스템 (Jaywalking Warning System)

   천안시 청소년 도시재생 챌린지 (U-CAST) / 5팀 「건영아 잘하자」

   ────────────────────────────────────────────────────────────────────────────
   이 파일은 발표에 쓸 **최종 실물 스케치**입니다.
   IR 센서 4개를 모두 연결한 상태를 기준으로 합니다.
   지금 IR 센서가 2개뿐이라 미리 조립해 보는 중이라면 `test/test.ino`를 쓰세요.
   ────────────────────────────────────────────────────────────────────────────

   [동작 요약] — 3차시 워크북 5·6·7·8장 기준
   1) 차도 진입 구간에 진입 감지 IR 센서 2개(D2, D3)를 설치한다.
   2) 도로 반대편 통과 지점에 통과(도착) 감지 IR 센서 2개(D4, D5)를 설치한다.
   3) 대기 상태에서 진입 IR 중 하나가 감지되거나 빨간 버튼을 누르면 경고를 시작한다.
   4) 경고 중에는 네오픽셀 스트립 1줄이 전체 빨간색으로 켜지고,
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

/* 네오픽셀(WS2812) LED 스트립을 제어하는 외부 라이브러리입니다.
   Arduino IDE의 [스케치] → [라이브러리 포함하기] → [라이브러리 관리]에서
   "Adafruit NeoPixel"을 검색해 설치해야 이 줄이 통과합니다.
   이 프로젝트가 쓰는 외부 라이브러리는 이것 하나뿐입니다. */
#include <Adafruit_NeoPixel.h>

/* ---------------------------------------------------------------------------
   [설정 1] 핀 번호
   워크북 8장 「코드 기반 핀 연결표」와 같은 배정입니다.

   `const uint8_t`는 "한 번 정하면 바뀌지 않는(const) 0~255 범위의 정수(uint8_t)"
   라는 뜻입니다. 핀 번호처럼 절대 변하면 안 되는 값에 씁니다.
   배선을 바꿨다면 아래 숫자만 고치면 되고, 아래쪽 로직은 손대지 않습니다.
   --------------------------------------------------------------------------- */
const uint8_t PIN_IR_ENTRY_1 = 2;   // 진입 감지 IR 센서 1
const uint8_t PIN_IR_ENTRY_2 = 3;   // 진입 감지 IR 센서 2
const uint8_t PIN_IR_EXIT_1  = 4;   // 통과(도착) 감지 IR 센서 1
const uint8_t PIN_IR_EXIT_2  = 5;   // 통과(도착) 감지 IR 센서 2
const uint8_t PIN_STRIP      = 6;   // 네오픽셀 LED 스트립 DIN (1줄)
const uint8_t PIN_WARN_LED   = 8;   // 운전자 경고용 빨간 LED(표지판 LED)
const uint8_t PIN_BUZZER     = 9;   // 경고용 부저
const uint8_t PIN_BTN_START  = A0;  // 빨간 버튼: 경고 수동 시작
const uint8_t PIN_BTN_STOP   = A1;  // 초록 버튼: 경고 수동 종료

// D7은 스트립을 1줄로 확정하면서 비었습니다. 지금은 아무것도 연결하지 않습니다.

/* ---------------------------------------------------------------------------
   [설정 2] 네오픽셀 스트립
   스트립은 1줄만 씁니다. 경고 상태에서 전체가 빨간색으로 켜지고,
   흐르는 방향 애니메이션은 없습니다.
   --------------------------------------------------------------------------- */

// 스트립에 박혀 있는 낱개 LED(픽셀) 개수입니다.
// 이 값이 실물보다 작으면 뒷부분이 안 켜지고, 크면 없는 픽셀에 색을 보내 낭비됩니다.
const uint16_t STRIP_PIXELS = 10;

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

// 많은 디지털 IR 모듈은 물체를 감지하면 LOW(0V)를 출력합니다.
// 손을 대지 않았는데 계속 감지된다고 나오면 이 값을 false로 바꿔 보세요.
// (실물 확인 전까지는 미확정값입니다.)
const bool IR_ACTIVE_LOW = true;

// 모듈 출력이 오픈 컬렉터 방식이면 내부 풀업이 필요할 수 있습니다.
// 값이 제멋대로 떠다니면 true로 두세요.
const bool IR_USE_INTERNAL_PULLUP = true;

// 신호가 이 시간(ms) 이상 유지되어야 "진짜 감지"로 확정합니다.
// 짧은 노이즈나 스쳐 지나감을 무시하기 위한 값입니다.
// 잠깐 스쳐도 발동하면 늘리고, 반응이 굼뜨면 줄입니다.
const uint16_t IR_CONFIRM_MS = 80;

/* ---------------------------------------------------------------------------
   [설정 5] 안전 복귀(선택 기능)
   워크북 원본에는 없는 선택 설정입니다. 기본값 0은 자동 종료를 사용하지 않습니다.
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

/* 스트립을 다루는 객체를 하나 만듭니다.
   (픽셀 수, 연결한 핀, 스트립 종류) 순서로 알려 줍니다.
   NEO_GRB는 색 데이터가 초록-빨강-파랑 순서로 들어가는 흔한 방식,
   NEO_KHZ800은 통신 속도입니다. 대부분의 WS2812 스트립이 이 조합입니다. */
Adafruit_NeoPixel strip(STRIP_PIXELS, PIN_STRIP, NEO_GRB + NEO_KHZ800);

/* 이 장치가 가질 수 있는 상태는 딱 두 가지입니다.
   enum은 "정해진 이름 중 하나만 담는 값"을 만드는 문법으로,
   0/1 같은 숫자보다 읽기 쉬워서 상태 표현에 씁니다. */
enum SystemState {
  STATE_IDLE,     // 대기: 모든 출력 꺼짐
  STATE_WARNING   // 경고 중: 스트립·빨간 LED·부저 작동
};

SystemState systemState = STATE_IDLE;  // 전원을 켜면 항상 대기로 시작합니다

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

HoldFilter irEntry1 = {PIN_IR_ENTRY_1, false, false, 0};
HoldFilter irEntry2 = {PIN_IR_ENTRY_2, false, false, 0};
HoldFilter irExit1  = {PIN_IR_EXIT_1,  false, false, 0};
HoldFilter irExit2  = {PIN_IR_EXIT_2,  false, false, 0};

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

/* 스트립 전체를 경고 색으로 켜거나(on = true) 완전히 끕니다(on = false).

   setPixelColor()는 "몇 번째 픽셀을 무슨 색으로 할지" 메모리에만 적어 둡니다.
   실제로 스트립에 데이터를 쏘는 것은 마지막의 show() 한 번입니다.
   그래서 반복문으로 전부 정해 놓고 show()를 한 번만 부릅니다. */
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

// 모든 출력을 한 번에 끕니다. 끄는 곳이 여러 군데라 함수로 묶어 둡니다.
void allOutputsOff() {
  applyBlinkOutputs(false);
  applyStrip(false);
}

// 대기 → 경고로 넘어갈 때 해야 할 일을 전부 모아 둔 함수입니다.
// reason은 시리얼 로그에 "왜 시작했는지"를 남기기 위한 설명입니다.
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
  pinMode(PIN_IR_ENTRY_1, irMode);
  pinMode(PIN_IR_ENTRY_2, irMode);
  pinMode(PIN_IR_EXIT_1,  irMode);
  pinMode(PIN_IR_EXIT_2,  irMode);

  pinMode(PIN_BTN_START, INPUT_PULLUP);
  pinMode(PIN_BTN_STOP,  INPUT_PULLUP);

  pinMode(PIN_WARN_LED, OUTPUT);
  pinMode(PIN_BUZZER,   OUTPUT);

  strip.begin();                        // 스트립 통신 시작
  strip.setBrightness(STRIP_BRIGHTNESS);

  allOutputsOff();  // 켜자마자 뭔가 켜져 있으면 안 되므로 전부 끕니다

  if (SERIAL_DEBUG) {
    Serial.begin(SERIAL_BAUD);
    Serial.println(F("무단횡단 방지 시스템 시작 - 대기 상태"));
    Serial.println(F("빨간 버튼=경고 수동 시작, 초록 버튼=경고 수동 종료"));
  }
}

// loop()는 setup() 뒤에 끝없이 반복 실행됩니다.
// 이 안에서는 절대 delay()로 멈추지 않고, 매번 "지금 시각"을 기준으로 판단합니다.
void loop() {
  uint32_t now = millis();  // 한 바퀴 도는 동안 같은 시각을 쓰도록 먼저 읽어 둡니다

  // 센서 4개를 모두 읽습니다. 조건문 안에서 읽으면 앞쪽이 true일 때
  // 뒤쪽 필터가 갱신되지 않아 상태가 어긋나므로, 먼저 전부 읽어 둡니다.
  bool entryEvent1 = detectedEdge(irEntry1, now);
  bool entryEvent2 = detectedEdge(irEntry2, now);
  bool exitEvent1  = detectedEdge(irExit1,  now);
  bool exitEvent2  = detectedEdge(irExit2,  now);

  // || 는 "또는"입니다. 둘 중 하나만 감지돼도 진입/통과로 봅니다.
  bool entryDetected = entryEvent1 || entryEvent2;
  bool exitDetected  = exitEvent1  || exitEvent2;

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
        if (irExit1.stable || irExit2.stable) {
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
      // 이미 경고 중인데 또 켜 봐야 달라지는 게 없기 때문입니다.
      if (entryDetected)   logLine(F("경고 유지: 진입 IR 재감지"));
      if (redButtonPressed) logLine(F("경고 유지: 빨간 버튼 재입력"));

      if (warningTimedOut(now)) {
        enterIdle(F("제한시간 경과 - 선택 기능 자동 복귀"));
        break;
      }

      updateBlinkOutputs(now);  // 여기까지 왔으면 경고 유지 중 → 점멸 계속
      break;
  }
}
