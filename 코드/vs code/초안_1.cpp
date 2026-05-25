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


//시작 시간 설정
unsigned long StartTime = 0;

//TB6612FNG(기어드 모터_수평이동)
// const int PWMA;
// const int AIN1 = 2;
// const int AIN2 = 3;

//HM0557(스텝모터)
const int STR = 8;   // STEP/PULSE
const int DIR = 7;   // 방향

const int stepsPerRevolution = 9000;

//BTS7960(기어드 모터_수직이동)
const int R_EN[] = {2,7};   //오른쪽
const int L_EN[] = {3,8};   //왼쪽
const int R_PWM[] = {5,9};  
const int L_PWM[] = {6,10}; 

const int Motor_Num = 2;


//VL53L0X
Adafruit_VL53L0X lox = Adafruit_VL53L0X();

const int Target_Distance = 800; //80cm
int failCount = 0;
const int Max_Fail_Count = 10;

//MPU6050
Adafruit_MPU6050 mpu;
int16_t Ax1, Ay1, Az1, Gx1, Gy1, Gz1;

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
    currentState = Stop_5s;
    break;
        
  case Stop_5s:
    if(millis() - StartTime >= 5000) {
    currentState = Horizontal_Move;
    }
    break;
          
  case Horizontal_Move: //<-------------------------------------------------------------------추가 필요
    Move_Horizontal();
    delay(500);
    currentState = Gripper_Open;
    break;
    
  case Gripper_Open:
    Open_Gripper();
    delay(500);
    currentState = Gripper_Close;
    break;
    
  case Gripper_Close:
    Close_Gripper();
    delay(500);
    currentState = Climbing;
    break;
    
  case Climbing:
    Go_Up();
    break;
  
  case Harvesting://<-------------------------------------------------------------------추가 필요
    Harvest();
    delay(500);
    currentState = IDLE;
    break;
    
  case IDLE:
  Stop_Motors();
  break;
}
}
        
void Move_Horizontal() {
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
  int distance;
  if (Read_Distance(distance)) {
    failCount = 0;
    if (!Read_MPU(Ax1, Ay1, Az1, Gx1, Gy1, Gz1)) {
      Serial.println("Failed to read MPU6050 data");
      return;
    }

    Serial.println(distance);

    if (distance >= Target_Distance) {
      Stop_Motors();
      currentState = Harvesting;
      }else {
      Climb_Up();
    } 
  } else {
    Stop_Motors ();
    failCount++;
    if (failCount >= Max_Fail_Count) {
      currentState = IDLE;
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
    Serial.println(distance);
    return true;
  } else {
    Serial.println("Sensor error");
    return false;
  }
}

bool Read_MPU(int16_t &Ax1, int16_t &Ay1, int16_t &Az1, int16_t &Gx1, int16_t &Gy1, int16_t &Gz1) {  
  
  Wire.beginTransmission(MPU_addr);
  Wire.write(0x3B);
  Wire.endTransmission(false);

  if (Wire.requestFrom(MPU_addr, 14, true) != 14) {
  Serial.println("MPU6050 read failed");
  return false;
  }
  Ax1=Wire.read()<<8|Wire.read(); // 1byte = 8bit
  Ay1=Wire.read()<<8|Wire.read();
  Az1=Wire.read()<<8|Wire.read();
  Wire.read(); 
  Wire.read(); 
  Gx1=Wire.read()<<8|Wire.read();
  Gy1=Wire.read()<<8|Wire.read();
  Gz1=Wire.read()<<8|Wire.read();
          
  Serial.print("Accelerometer: ");
  Serial.print("X = "); Serial.print(Ax1);
  Serial.print(" | Y = "); Serial.print(Ay1);
  Serial.print(" | Z = "); Serial.println(Az1);
          
  Serial.print("Gyroscope: ");
  Serial.print("X = "); Serial.print(Gx1);
  Serial.print(" | Y = "); Serial.print(Gy1);
  Serial.print(" | Z = "); Serial.println(Gz1);
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

  int speed;

  if (motor == 0) {
    speed = map(800, 0, Target_Distance, 255, 0); //<-----------------------------------speed조절 함수 추가 필수
    analogWrite(R_PWM[motor], speed);
    analogWrite(L_PWM[motor], 0);
  } else {
    speed = map(800, 0, Target_Distance, 255, 0);
    analogWrite(R_PWM[motor], speed);
    analogWrite(L_PWM[motor], 0);
  }
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

