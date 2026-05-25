#include <Adafruit_VL53L0X.h>
#include <Wire.h>
#include <Adafruit_MPU6050.h>
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

const int Motor_Num = 2;

const int Intermediary_Speed = 200; 
const int Start_Speed = 50; 
const int Speed_Increment = 5; 
const int Max_Speed = 255; 
int currentSpeed[Motor_Num] = {0, 0}; //현재 속도 저장 배열

//VL53L0X
Adafruit_VL53L0X lox = Adafruit_VL53L0X();

const int Target_Distance = 800; //80cm
int distance = 0;
int failCount = 0;
const int Max_Fail_Count = 10;

//MPU6050
Adafruit_MPU6050 mpu;
int16_t Ax1, Ay1, Az1, Gx1, Gy1, Gz1;

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

  //VL53L0X
  Serial.begin(115200);
  Wire.begin();
    
  if (!lox.begin()) {
   Serial.println("VL53L0X not found");
   while(1);
  }
  //MPU6050
  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    while (1) {
      delay(10);
    }
  }
  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
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
  if (Read_Distance(distance)) {
    failCount = 0;
    if (!Read_MPU(Ax1, Ay1, Az1, Gx1, Gy1, Gz1)) {
      Serial.println("Failed to read MPU6050 data");
      Stop_Motors();
      changeState(IDLE);
      return;
    }
    
    Serial.println(distance);
    
    if (distance >= Target_Distance) {
      Stop_Motors();
      changeState(Harvesting);
    }else {
      Climb_Up();
    } 
  } else {
    Stop_Motors ();
    failCount++;
    if (failCount >= Max_Fail_Count) {
      changeState(IDLE);
      failCount = 0;
    }
  }
  delay(50);
} 

bool Read_Distance(int &distance) {
  VL53L0X_RangingMeasurementData_t measure;
  lox.rangingTest(&measure, false);
  
  if (measure.RangeStatus !=4) {
    distance = measure.RangeMilliMeter;
    return true;
  } else {
    Serial.println("Sensor error");
    return false;
  }
}

bool Read_MPU(int16_t &Ax1, int16_t &Ay1, int16_t &Az1, int16_t &Gx1, int16_t &Gy1, int16_t &Gz1) {  
  
  sensors_event_t accel, gyro, temp;
  mpu.getEvent(&accel, &gyro, &temp);
  
  Ax1 = accel.acceleration.x * 100;///cm*s^2
  Ay1 = accel.acceleration.y * 100;
  Az1 = accel.acceleration.z * 100;

  Gx1 = gyro.gyro.x * 1000;//rad/s
  Gy1 = gyro.gyro.y * 1000;
  Gz1 = gyro.gyro.z * 1000;
          
  Serial.print("Accel X: ");
  Serial.print(Ax1);
  Serial.print(" Y: ");
  Serial.print(Ay1);
  Serial.print(" Z: ");
  Serial.println(Az1);
  
  Serial.print("Gyro X: ");
  Serial.print(Gx1);
  Serial.print(" Y: ");
  Serial.print(Gy1);
  Serial.print(" Z: ");
  Serial.println(Gz1);
  return true;
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
  } else if (currentSpeed[motor] >= Intermediary_Speed && currentSpeed[motor] <= Max_Speed) {
    if (motor ==0) {
      currentSpeed[motor] = map(Ax1, 0, Target_Distance, Intermediary_Speed, Max_Speed);//<-----------------------------------speed조절 함수 추가 필수
    } else {
      currentSpeed[motor] = map(Ay1, 0, Target_Distance, Intermediary_Speed, Max_Speed);
    }
  } else if (currentSpeed[motor] > Max_Speed) {
    currentSpeed[motor] = Max_Speed; 
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
  }
}

void Harvest() {
  // digitalWrite(R_EN_1, LOW);
  // digitalWrite(L_EN_1, LOW);
  // analogWrite(R_PWM_1, 0);
  // analogWrite(L_PWM_1, 0);
  
  // for(int i=0; i< 0.8*stepsPerRevolution; i++) {
  //   digitalWrite(STR, HIGH);
  //   delayMicroseconds(100); 

  //   digitalWrite(STR, LOW);
  //   delayMicroseconds(100);
  // }
}

