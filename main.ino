#include <Adafruit_NeoPixel.h>

// 1) 회로 연결 핀 정의
const int irSensorPin = 2;     // 적외선 센서(IR) - 접근 감지
const int irEndPin = 3;        // 반대편 IR 센서 - 통과 감지
const int redButtonPin = 4;    // 수동 빨간색 버튼 (무단횡단 발생)
const int greenButtonPin = 5;  // 수동 초록색 버튼 (상황 종결)
const int stripPin = 6;        // LED 스트립(네오픽셀) 핀
const int buzzerPin = 8;       // 부저 핀
const int redLedPin = 9;       // 빨간색 LED 핀

const int NUMPIXELS = 10;      // LED 스트립 픽셀 수
Adafruit_NeoPixel strip(NUMPIXELS, stripPin, NEO_GRB + NEO_KHZ800);

bool isWarningActive = false;  // 경고 상태 변수

void setup() {
  pinMode(irSensorPin, INPUT);
  pinMode(irEndPin, INPUT);
  pinMode(redButtonPin, INPUT_PULLUP);
  pinMode(greenButtonPin, INPUT_PULLUP);
  pinMode(buzzerPin, OUTPUT);
  pinMode(redLedPin, OUTPUT);
  
  strip.begin();
  strip.show(); // 초기화 (모두 끄기)
  Serial.begin(9600);
}

void loop() {
  // 자동 센서 감지 상태 및 수동 버튼 입력 읽기
  bool irDetected = (digitalRead(irSensorPin) == LOW);
  bool redBtnPressed = (digitalRead(redButtonPin) == LOW); // 수동 작동
  
  bool irEndDetected = (digitalRead(irEndPin) == LOW);
  bool greenBtnPressed = (digitalRead(greenButtonPin) == LOW); // 수동 작동

  // 감지 조건 (자동 IR 센서 또는 수동 빨간 버튼)
  bool motionDetected = irDetected || redBtnPressed;
  
  // 종료 조건 (자동 반대편 IR 센서 또는 수동 초록 버튼)
  bool clearDetected = irEndDetected || greenBtnPressed;

  // 대기 상태에서 무단횡단 감지 시 경고 시작
  if (!isWarningActive && motionDetected) {
    isWarningActive = true;
    Serial.println("상태: 무단횡단 감지, 경고 시스템 작동");
  }

  // 경고 작동 상태
  if (isWarningActive) {
    digitalWrite(redLedPin, HIGH); // 운전자용 빨간색 LED 켜기
    tone(buzzerPin, 1000);         // 부저 경고음 울리기
    
    // LED 스트립 점등 (길 유도)
    for(int i=0; i<NUMPIXELS; i++) {
      strip.setPixelColor(i, strip.Color(255, 0, 0));
    }
    strip.show();

    // 조건이 사라지면(자동 통과 또는 초록 버튼 입력) 원래 상태로 복귀
    if (clearDetected) {
      isWarningActive = false;
      digitalWrite(redLedPin, LOW);
      noTone(buzzerPin);
      strip.clear();
      strip.show();
      Serial.println("상태: 상황 종결, 대기 모드 복귀");
    }
  } 
  else {
    digitalWrite(redLedPin, LOW);
    noTone(buzzerPin);
    strip.clear();
    strip.show();
  }
  
  delay(100);
}
