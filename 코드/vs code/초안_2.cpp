#include <Adafruit_VL53L0X.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>

enum RobotState {
  Start,
  Stop_5s,
  Horizontal_Move,
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
unsigned long Horizontal_Move_Time = 0;

//TB6612FNG(기어드 모터_수평이동)
const int PWMA = 5;
const int AIN1 = 2;
const int AIN2 = 3;

const int STBY = 4; 

bool Horizontal_Move_Started = false;
int Horizontal_Speed = 0;

//HM0557(스텝모터)
const int STR = 8;   // STEP/PULSE
const int DIR = 7;   // 방향

const int stepsPerRevolution = 9000;

//BTS7960(기어드 모터_수직이동)
const int R_EN[] = {2,7};   //오른쪽 수평 이동 코드와 핀 번호 겹침
const int L_EN[] = {3,8};   //왼쪽
const int R_PWM[] = {5,9};  
const int L_PWM[] = {6,10}; 

bool Go_Up_Started = false;
unsigned long Go_Up_Time = 0;

const int Motor_Num = 2;

const int Intermediary_Speed = 200; 
const int Start_Speed = 50; 
const int Speed_Increment = 5; 
const int Max_Speed = 255; 
int currentSpeed[Motor_Num] = {0, 0}; //현재 속도 저장 배열

void setup() {
  //TB6612FNG(기어드 모터)
  pinMode(STBY, OUTPUT);
  pinMode(PWMA, OUTPUT);
  pinMode(AIN1, OUTPUT);    
  pinMode(AIN2, OUTPUT);
  digitalWrite(STBY, HIGH); // Standby 해제
  analogWrite(PWMA, 0); // 속도 초기화

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
    changeState(Horizontal_Move);
    }
    break;
          
  case Horizontal_Move: 
    if (!Horizontal_Move_Started) {
      Horizontal_Move_Time = millis();
      Horizontal_Move_Started = true;
      digitalWrite(AIN1, HIGH);
      digitalWrite(AIN2, LOW);
    }
    unsigned long elapsedTime = millis() - Horizontal_Move_Time;
    if (elapsedTime <= 3000) {
      Move_Horizontal();
    } else if (elapsedTime <= 5000) {
      Stop_Horizontal_Move();
    } else  {
      analogWrite(PWMA, 0); 
      Turn_Off_Horizontal_Motor();
      
      Horizontal_Speed = 0; 
      Horizontal_Move_Started = false; 
      changeState(Gripper_Open);
    }
    delay(50);
    break;
    
    case Gripper_Open:
    Open_Gripper();
    delay(500);
    changeState(Gripper_Close);
    break;
    
    case Gripper_Close:
    Close_Gripper();
    delay(500);
    changeState(Climbing);
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
        
void Move_Horizontal() {
  if (Horizontal_Speed ==0) {
    Horizontal_Speed = Start_Speed;
  } else if (Horizontal_Speed >= Start_Speed && Horizontal_Speed < Intermediary_Speed) {
    Horizontal_Speed += Speed_Increment; 
  }  else if (Horizontal_Speed >= Intermediary_Speed) {
    Horizontal_Speed = Intermediary_Speed;
  }
  analogWrite(PWMA, Horizontal_Speed);
}

void Stop_Horizontal_Move() {
  int Calculate_Speed = Horizontal_Speed - Speed_Increment;
  if (Calculate_Speed <= 0) {
    Horizontal_Speed = 0;
  }  else {
    Horizontal_Speed= Calculate_Speed; 
  } 
  analogWrite(PWMA, Horizontal_Speed);

  if (Horizontal_Speed == 0) {
    Turn_Off_Horizontal_Motor();
  }
}

void Turn_Off_Horizontal_Motor() {
  digitalWrite(AIN1, LOW);
  digitalWrite(AIN2, LOW);
}

void Open_Gripper() {
  digitalWrite(DIR, HIGH);
  for(int i=0; i<stepsPerRevolution; i++) {
    digitalWrite(STR, HIGH);
    delayMicroseconds(100);
    
    digitalWrite(STR, LOW);
    delayMicroseconds(100);
  }
}


void Close_Gripper (){
  digitalWrite(DIR, LOW);
  for(int i=0; i< stepsPerRevolution* 12 / 10; i++) {
    digitalWrite(STR, HIGH);
    delayMicroseconds(100);
    
    digitalWrite(STR, LOW);
    delayMicroseconds(100);
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

