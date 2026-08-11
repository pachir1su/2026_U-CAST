#include <Adafruit_NeoPixel.h>

// 센서 (디지털, 감지되면 LOW) - 하나는 연석 위, 하나는 도로
const int SENSOR_CURB = 10;
const int SENSOR_ROAD = 11;

// 신호등 LED
const int TRAFFIC_RED = 2;
const int TRAFFIC_YELLOW = 3;
const int TRAFFIC_GREEN = 4;

// 무단횡단 경고 출력
const int WARN_LED = 5;
const int NEOPIXEL_PIN = 6;
const int NUM_PIXELS = 12;
const int BUZZER_PIN = 9;
const bool USE_BUZZER = true;

// 버튼
const int BUTTON_PIN = 12;
const int SIM_BUTTON_PIN = 13;

Adafruit_NeoPixel strip(NUM_PIXELS, NEOPIXEL_PIN, NEO_GRB + NEO_KHZ800);

const int DETECT_THRESHOLD = 1;
const unsigned long CROSS_TIMEOUT = 2000;
const unsigned long WARNING_DURATION = 3000;  // 다시 3초로 복원

// ===== 피에조(수동) 부저 사이렌 설정 - 원래 값으로 복원 =====
const uint16_t SIREN_MIN_HZ    = 800;
const uint16_t SIREN_MAX_HZ    = 1800;
const uint16_t SIREN_SWEEP_MS  = 500;
const uint16_t SIREN_UPDATE_MS = 25;

enum LightState { LIGHT_RED, LIGHT_YELLOW, LIGHT_GREEN };
LightState currentLight = LIGHT_RED;

bool curbWasOver = false;
bool roadWasOver = false;

bool waitingCurbToRoad = false;
unsigned long curbDetectedTime = 0;

bool warningOn = false;
unsigned long warningStartTime = 0;

bool buttonWasPressed = false;
bool simButtonWasPressed = false;

// ===== 부저 사이렌 상태 변수 =====
bool buzzerOn = false;
bool sirenRising = true;
uint16_t sirenHz = 0;
unsigned long sirenSweepStartedAt = 0;
unsigned long sirenUpdatedAt = 0;

// ===== 네오픽셀을 한 프레임 늦춰서 예약 처리 (핵심 수정 유지) =====
bool needStripUpdate = false;
uint8_t pendingR = 0, pendingG = 0, pendingB = 0;

void setStripColor(uint8_t r, uint8_t g, uint8_t b) {
  for (int i = 0; i < NUM_PIXELS; i++) {
    strip.setPixelColor(i, strip.Color(r, g, b));
  }
  strip.show();
}

void applyTrafficLight() {
  digitalWrite(TRAFFIC_RED, currentLight == LIGHT_RED ? HIGH : LOW);
  digitalWrite(TRAFFIC_YELLOW, currentLight == LIGHT_YELLOW ? HIGH : LOW);
  digitalWrite(TRAFFIC_GREEN, currentLight == LIGHT_GREEN ? HIGH : LOW);
}

void startBuzzer(unsigned long now) {
  if (!USE_BUZZER) return;
  buzzerOn = true;
  sirenRising = true;
  sirenSweepStartedAt = now;
  sirenUpdatedAt = now;
  sirenHz = SIREN_MIN_HZ;
  tone(BUZZER_PIN, sirenHz);
}

void stopBuzzer() {
  buzzerOn = false;
  sirenHz = 0;
  noTone(BUZZER_PIN);
}

void updateBuzzerSiren(unsigned long now) {
  if (!buzzerOn) return;
  if ((now - sirenUpdatedAt) < SIREN_UPDATE_MS) return;
  sirenUpdatedAt = now;

  while ((now - sirenSweepStartedAt) >= SIREN_SWEEP_MS) {
    sirenSweepStartedAt += SIREN_SWEEP_MS;
    sirenRising = !sirenRising;
  }

  unsigned long elapsed = now - sirenSweepStartedAt;
  if (elapsed > SIREN_SWEEP_MS) elapsed = SIREN_SWEEP_MS;

  uint32_t span = (uint32_t)SIREN_MAX_HZ - (uint32_t)SIREN_MIN_HZ;
  uint32_t moved = (span * elapsed) / SIREN_SWEEP_MS;

  sirenHz = sirenRising ? (uint16_t)(SIREN_MIN_HZ + moved)
                        : (uint16_t)(SIREN_MAX_HZ - moved);
  tone(BUZZER_PIN, sirenHz);
}

void turnWarningOn() {
  digitalWrite(WARN_LED, HIGH);
  startBuzzer(millis());   // 소리부터 먼저
  warningStartTime = millis();
  warningOn = true;

  // 네오픽셀은 다음 루프에서 처리 (부저와 타이밍 안 겹치게)
  needStripUpdate = true;
  pendingR = 255; pendingG = 0; pendingB = 0;

  Serial.print("경고 시작 at ");
  Serial.println(millis());
}

void turnWarningOff() {
  digitalWrite(WARN_LED, LOW);
  stopBuzzer();
  warningOn = false;

  needStripUpdate = true;
  pendingR = 0; pendingG = 0; pendingB = 0;

  Serial.print("경고 종료 at ");
  Serial.println(millis());
}

void handleCrossingDetected(const char* sourceLabel) {
  if (warningOn) return;

  if (currentLight == LIGHT_RED) {
    turnWarningOn();
    Serial.print("무단횡단 감지! (빨간불) - 출처: ");
    Serial.println(sourceLabel);
  } else {
    Serial.print("정상 횡단 (신호 준수) - 출처: ");
    Serial.println(sourceLabel);
  }
}

void setup() {
  pinMode(TRAFFIC_RED, OUTPUT);
  pinMode(TRAFFIC_YELLOW, OUTPUT);
  pinMode(TRAFFIC_GREEN, OUTPUT);

  pinMode(BUTTON_PIN, INPUT_PULLUP);
  pinMode(SIM_BUTTON_PIN, INPUT_PULLUP);

  pinMode(WARN_LED, OUTPUT);
  digitalWrite(WARN_LED, LOW);

  pinMode(BUZZER_PIN, OUTPUT);

  strip.begin();
  strip.show();

  applyTrafficLight();

  Serial.begin(9600);
}

void loop() {
  unsigned long now = millis();

  // ===== 버튼으로 신호등 전환 =====
  bool buttonPressed = (digitalRead(BUTTON_PIN) == LOW);

  if (buttonPressed && !buttonWasPressed) {
    if (currentLight == LIGHT_RED) {
      currentLight = LIGHT_YELLOW;
    } else if (currentLight == LIGHT_YELLOW) {
      currentLight = LIGHT_GREEN;
    } else {
      currentLight = LIGHT_RED;
    }
    applyTrafficLight();
  }
  buttonWasPressed = buttonPressed;

  // ===== 순차 감지 =====
  int curbValue = digitalRead(SENSOR_CURB);
  int roadValue = digitalRead(SENSOR_ROAD);

  bool curbOver = curbValue < DETECT_THRESHOLD;
  bool roadOver = roadValue < DETECT_THRESHOLD;

  if (curbOver && !curbWasOver) {
    waitingCurbToRoad = true;
    curbDetectedTime = now;
  }

  if (roadOver && !roadWasOver) {
    if (waitingCurbToRoad && (now - curbDetectedTime <= CROSS_TIMEOUT)) {
      handleCrossingDetected("연석->도로 순차 감지");
      waitingCurbToRoad = false;
    }
  }

  if (waitingCurbToRoad && (now - curbDetectedTime > CROSS_TIMEOUT)) {
    waitingCurbToRoad = false;
  }

  curbWasOver = curbOver;
  roadWasOver = roadOver;

  // ===== 시뮬레이션 버튼 =====
  bool simButtonPressed = (digitalRead(SIM_BUTTON_PIN) == LOW);

  if (simButtonPressed && !simButtonWasPressed) {
    handleCrossingDetected("시뮬 버튼");
  }
  simButtonWasPressed = simButtonPressed;

  // ===== 경고 자동 종료 =====
  if (warningOn && now - warningStartTime >= WARNING_DURATION) {
    turnWarningOff();
  }

  // ===== 부저 사이렌 계속 갱신 =====
  updateBuzzerSiren(now);

  // ===== 예약된 네오픽셀 업데이트를 부저 처리 이후에 실행 =====
  if (needStripUpdate) {
    needStripUpdate = false;
    setStripColor(pendingR, pendingG, pendingB);
  }

  delay(5);
}

