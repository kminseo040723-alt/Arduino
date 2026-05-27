// BTS7960(기어드 모터_수직이동)
const int R_EN[] = {7, 12};
const int L_EN[] = {8, 13};
const int R_PWM[] = {5, 10};
const int L_PWM[] = {6, 11};

const int Motor_Num = 2;

int currentSpeed[Motor_Num] = {0, 0};

const int Intermediary_Speed = 175;
const int Start_Speed = 70;
const int Speed_Increment = 5;
const int Max_Speed = 255;

// 기어드모터 핀 초기화
void setup() {
  for (int i = 0; i < Motor_Num; i++) {
    pinMode(R_EN[i], OUTPUT);
    pinMode(L_EN[i], OUTPUT);
    pinMode(R_PWM[i], OUTPUT);
    pinMode(L_PWM[i], OUTPUT);

    digitalWrite(R_EN[i], LOW);
    digitalWrite(L_EN[i], LOW);

    analogWrite(R_PWM[i], 0);
    analogWrite(L_PWM[i], 0);
  }
}

void loop () {
  Climb_Up();
}
// 기어드모터 2개 상승
void Climb_Up() {
  for (int i = 0; i < Motor_Num; i++) {
    Set_Motor(i);
  }
}

// 기어드모터 1개 속도 설정
void Set_Motor(int motor) {
  digitalWrite(R_EN[motor], HIGH);
  digitalWrite(L_EN[motor], HIGH);

  currentSpeed[motor] = Intermediary_Speed;

  analogWrite(R_PWM[motor], currentSpeed[motor]);
  analogWrite(L_PWM[motor], 0);
}

// 기어드모터 전체 정지
void Stop_Motors() {
  for (int i = 0; i < Motor_Num; i++) {
    analogWrite(R_PWM[i], 0);
    analogWrite(L_PWM[i], 0);

    digitalWrite(R_EN[i], LOW);
    digitalWrite(L_EN[i], LOW);

    currentSpeed[i] = 0;
  }
}