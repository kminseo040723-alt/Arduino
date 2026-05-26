

enum RobotState {
  Start,
  Stop_5s,
  Gripper_Open,   
  Gripper_Close, 
  Climbing, 
  Harvesting,       
  IDLE                
};
  RobotState currentState = Start;

void changeState(RobotState newState) {
  currentState = newState;
}
//시작 시간 설정
unsigned long StartTime = 0;
unsigned long Go_Up_Time = 0;

//bool 설정
bool Gripper_Open_Started = false;
bool Gripper_Close_Started = false;
bool Go_Up_Started = false;

//초기 속도 설정
int gripperStep = 0;

//HM0557(스텝모터)
const int STR = 3;   // STEP/PULSE
const int DIR = 4;   // 방향

const int stepsPerRevolution = 9000;

//BTS7960(기어드 모터_수직이동)
const int R_EN[] = {7,12};   //오른쪽 수평 이동 코드와 핀 번호 겹침
const int L_EN[] = {8,13};   //왼쪽
const int R_PWM[] = {5,10};  
const int L_PWM[] = {6,11}; 


const int Motor_Num = 2;
int currentSpeed[Motor_Num] = {0, 0}; //현재 속도 저장 배열

const int Intermediary_Speed = 200; 
const int Start_Speed = 50; 
const int Speed_Increment = 5; 
const int Max_Speed = 255; 

void setup() {

  //HM0557(스텝모터)
  pinMode(STR, OUTPUT);
  pinMode(DIR, OUTPUT);
  
  //BTS7960(기어드 모터)
  for (int i =0; i<Motor_Num; i++) {
    pinMode(R_EN[i], OUTPUT);
    pinMode(L_EN[i], OUTPUT);   
    pinMode(R_PWM[i], OUTPUT);
    pinMode(L_PWM[i], OUTPUT);
    
    digitalWrite(R_EN[i], LOW);
    digitalWrite(L_EN[i], LOW);
    }
}
    
void loop() {
 switch (currentState) {
  case Start:
    StartTime = millis();
    changeState(Stop_5s);
    break;
        
  case Stop_5s:
    if(millis() - StartTime >= 5000) {
    changeState(Gripper_Open);
    }
    break;
    
    case Gripper_Open:
    if (!Gripper_Open_Started) {
      gripperStep = 0; // 그리퍼 단계 초기화
      digitalWrite(DIR, HIGH);
      Gripper_Open_Started = true;
    }
    Open_Gripper();
    break;
    
    case Gripper_Close:
    if (!Gripper_Close_Started) {
      gripperStep = 0; // 그리퍼 단계 초기화
      digitalWrite(DIR, LOW);
      Gripper_Close_Started = true;
    }
      Close_Gripper();
    break;
    
    case Climbing:
    Go_Up();
    break;
  
    case Harvesting://<-------------------------------------------------------------------추가 필요
    Harvest();
    delay(500);
    changeState(IDLE);
    break;
  
    case IDLE:
    Stop_Motors();
    break;
  }
}

void Open_Gripper() {
  if (gripperStep < stepsPerRevolution) {
    digitalWrite(STR, HIGH);
    delayMicroseconds(100);
    digitalWrite(STR, LOW);
    delayMicroseconds(100);

    gripperStep++;
  } else {
    Gripper_Open_Started = false;
    changeState(Gripper_Close);
    gripperStep = 0; // 그리퍼 단계 초기화
  }
}


void Close_Gripper () {
  if (gripperStep < stepsPerRevolution * 12 / 10) {
    digitalWrite(STR, HIGH);
    delayMicroseconds(100);
    digitalWrite(STR, LOW);
    delayMicroseconds(100);
    gripperStep++;
  } else {
    Gripper_Close_Started = false;
    changeState(Climbing);
    gripperStep = 0; // 그리퍼 단계 초기화
  }
}


void Go_Up() {
  if (!Go_Up_Started) {
    Go_Up_Time = millis();
    Go_Up_Started = true;
  }
  if (millis() - Go_Up_Time <= 50000) {
      Climb_Up();
  } else {
    Stop_Motors();
    Go_Up_Started = false;
    changeState(Harvesting);
  }
  delay(50);
} 

          
void Climb_Up() {
  for (int i=0; i<Motor_Num; i++) {
    Set_Motor (i);
  }
}

void Set_Motor (int motor) {
  
  digitalWrite(R_EN[motor], HIGH);
  digitalWrite(L_EN[motor], HIGH);

  if (currentSpeed[motor] < Intermediary_Speed) {
    if (currentSpeed[motor] == 0) {
      currentSpeed[motor] = Start_Speed;
    } else {
      currentSpeed[motor] += Speed_Increment; //속도 증가, 필요에 따라 조정
    }
  } else if (currentSpeed[motor] >= Intermediary_Speed) {
    currentSpeed[motor] = Intermediary_Speed; 
  } else {
    currentSpeed[motor] = Start_Speed; 
  }
  analogWrite(R_PWM[motor], currentSpeed[motor]);
  analogWrite(L_PWM[motor], LOW);
}


void Stop_Motors() {
  for (int i = 0; i <Motor_Num; i++) {
    analogWrite(R_PWM[i], 0);
    analogWrite(L_PWM[i], 0);
    digitalWrite(R_EN[i], LOW);
    digitalWrite(L_EN[i], LOW);
    currentSpeed[i] = 0;
  }
}

void Harvest() {
  //수확 동작 구현
}

