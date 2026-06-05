// ==========================================
// 모터 1 (첫 번째 BTS7960) 핀 설정
// ==========================================
int m1_RPWM = 3;   // 모터 1 전진 속도 제어 (반드시 ~가 있는 PWM 핀)
int m1_LPWM = 5;   // 모터 1 후진 속도 제어 (반드시 ~가 있는 PWM 핀)


// ==========================================
// 모터 2 (두 번째 BTS7960) 핀 설정
// ==========================================
int m2_RPWM = 6;   // 모터 2 전진 속도 제어 (반드시 ~가 있는 PWM 핀)
int m2_LPWM = 9;  // 모터 2 후진 속도 제어 (반드시 ~가 있는 PWM 핀)


void setup() {
  // 모터 1 핀 출력 설정
  pinMode(m1_RPWM, OUTPUT);
  pinMode(m1_LPWM, OUTPUT);

  
  // 모터 2 핀 출력 설정
  pinMode(m2_RPWM, OUTPUT);
  pinMode(m2_LPWM, OUTPUT);
 
}

void loop() {
  // 1. 두 모터를 동시에 전진 (최대 속도: 255)
  moveupward(150);
  delay(2000); 
}

// ==========================================
// 모터 제어 사용자 정의 함수
// ==========================================

// 두 모터 동시 전진 함수
void moveupward(int speed) {
  // 전진 핀(RPWM)에는 속도를, 후진 핀(LPWM)에는 0을 입력
  analogWrite(m1_RPWM, 0);
  analogWrite(m1_LPWM, speed);
  
  analogWrite(m2_RPWM, 0);
  analogWrite(m2_LPWM, speed);
}

// 두 모터 동시 후진 함수
void moveBackward(int speed) {
  // 전진 핀(RPWM)에는 0을, 후진 핀(LPWM)에는 속도를 입력
  analogWrite(m1_RPWM, 0);
  analogWrite(m1_LPWM, speed);
  
  analogWrite(m2_RPWM, 0);
  analogWrite(m2_LPWM, speed);
}

// 두 모터 동시 정지 함수
void stopMotors() {
  // RPWM과 LPWM 모두에 0을 주어 모터 정지
  analogWrite(m1_RPWM, 0);
  analogWrite(m1_LPWM, 0);
  
  analogWrite(m2_RPWM, 0);
  analogWrite(m2_LPWM, 0);
}