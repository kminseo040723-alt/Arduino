#include "Adafruit_VL53L0X.h"

// VL53L0X 센서 객체 선언
Adafruit_VL53L0X lox = Adafruit_VL53L0X();

// [직관적인 튜닝] 몇 mm 앞에서 자동차를 멈추게 할지 설정합니다.
// 예: 150mm = 15cm 앞에서 정지
const int stopDistance_mm = 150; 

void setup() {
  Serial.begin(9600);

  // 시리얼 모니터가 켜질 때까지 대기
  while (!Serial) { delay(1); }

  Serial.println("=========================================");
  Serial.println("   VL53L0X ToF 레이저 거리 측정 시작   ");
  Serial.println("=========================================");

  // 센서 초기화
  if (!lox.begin()) {
    Serial.println(F("❌ VL53L0X 센서를 찾지 못했습니다. 선 연결을 확인하세요!"));
    while (1);
  }
  Serial.println(F("⭕ 센서 활성화 완료. 측정 플로우를 시작합니다."));
}

void loop() {
  VL53L0X_RangingMeasurementData_t measure;
    
  // 센서로부터 거리 측정 데이터 가져오기
  lox.rangingTest(&measure, false); 

  // 측정 상태가 정상(Phase == 0)일 때만 작동
  if (measure.RangeStatus != 4) {  
    int currentDistance = measure.RangeMilliMeter; // 실시간 거리 측정값 (mm 단위)

    // 시리얼 모니터에 mm 단위로 거리 출력
    Serial.print("실시간 전방 거리: ");
    Serial.print(currentDistance);
    Serial.print(" mm | ");

    // 측정된 실제 거리를 바탕으로 판단
    if (currentDistance <= stopDistance_mm) {
      // 설정한 정지 거리(150mm)보다 가까워진 경우
      Serial.println("[❌ STOP] 장애물 근접! 구동축 모터 정지");
      //여기에 모터 정지 함수 추가 (예: stopMotors();)
    } else {
      // 안전 거리가 확보된 경우
      Serial.println("[▶️ FORWARD] 전방 클리어. 구동축 모터 전진");
      //여기에 모터 전진 함수 추가 (예: moveForward();)
    }

  } else {
    // 센서 범위를 벗어났거나 측정이 불가능할 때
    Serial.println(" [⚠️ 오류] 측정 범위를 벗어났습니다. ");
  }

  // 0.15초 간격으로 거리 측정 업데이트
  delay(150); 
}