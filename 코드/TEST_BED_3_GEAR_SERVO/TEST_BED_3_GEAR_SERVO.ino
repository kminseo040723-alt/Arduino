#include <Servo.h>

enum RobotState {
  Start,
  Stop_2s,
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
bool Go_Up_Started = false;

//BTS7960(기어드 모터_수직이동)
const int R_EN[] = {7,12};   //오른쪽 수평 이동 코드와 핀 번호 겹침
const int L_EN[] = {8,2};   //왼쪽
const int R_PWM[] = {6,3};  
const int L_PWM[] = {5,11}; 


const int Motor_Num = 2;
int currentSpeed[Motor_Num] = {0, 0}; //현재 속도 저장 배열

const int Intermediary_Speed = 200; 
const int Start_Speed = 70; 
const int Speed_Increment = 5; 
const int Max_Speed = 255; 

//서보모터
Servo servos[1];

const int Servo_Pin[] = {1};
const int Servo_Num = 1; 
const int Total_Rotations = 3;
const int Initial_Angle = 10;
const int Max_Angle = 170;
const int Angle_Increment = 10;

int angle = Initial_Angle;
int Rotation = 0;
int Direction = 1;

void setup() {
  
  //BTS7960(기어드 모터)
  for (int i =0; i<Motor_Num; i++) {
    pinMode(R_EN[i], OUTPUT);
    pinMode(L_EN[i], OUTPUT);   
    pinMode(R_PWM[i], OUTPUT);
    pinMode(L_PWM[i], OUTPUT);
    
    digitalWrite(R_EN[i], LOW);
    digitalWrite(L_EN[i], LOW);
    }
    //Servo
    for (int i = 0; i < Servo_Num; i++) {
        servos[i].attach(Servo_Pin[i]);
    }

}
    
void loop() {
 switch (currentState) {
  case Start:
    StartTime = millis();
    changeState(Stop_2s);
    break;
        
  case Stop_2s:
    if(millis() - StartTime >= 2000) {
    changeState(Climbing);
    }
    break;
    
    case Climbing:
    Go_Up();
    break;
  
    case Harvesting://<-------------------------------------------------------------------추가 필요
    Harvest();
    break;
  
    case IDLE:
    Stop_Motors();
    while (1);
    break;
  }
}



void Go_Up() {
  if (!Go_Up_Started) {
    Go_Up_Time = millis();
    Go_Up_Started = true;
  }
  if (millis() - Go_Up_Time <= 10000) {
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

  currentSpeed[motor] = Intermediary_Speed;
  analogWrite(R_PWM[motor], LOW);
  analogWrite(L_PWM[motor], currentSpeed[motor]);
}

void Harvest () {
if (Rotation < Total_Rotations) {
    if (Direction == 1) {
        Rotating_1();
    } else if (Direction == -1) {
        Rotating_2();
    }
 } else {
    for (int i = 0; i < Servo_Num; i++) {
        servos[i].detach();
    }
    Rotation = 0;
    Direction = 1;
    changeState(IDLE);
    }
}
  
void Write_Servos(int targetAngle) {
  for (int i = 0; i < Servo_Num; i++) {
    servos[i].write(targetAngle);
  }
}
 void Rotating_1() {
    if (angle < Max_Angle) {
      angle += Angle_Increment;
    } else if (angle >= Max_Angle) {
      angle = Max_Angle;
      Direction = -1;
    }
    Write_Servos(angle);
    delay (50);
}
void Rotating_2() {
    if (angle > Initial_Angle) {
      angle -= Angle_Increment;
    } else if (angle <= Initial_Angle) {
      angle = Initial_Angle;
      Direction = 1;
      Rotation++;
    }
    Write_Servos(angle);
        delay (50);

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

