//A4988, TBTB6612FNG ->드라이버
//VL53L0X ->센서
//수평 이동 모터 & 텔레스코프 모터 필요
//센서 집어넣으면 조건 따라서 바뀌는 코드 필요

#include <Adafruit_VL53L0X.h>
#include <Wire.h>
#include <MPU6050.h>

enum RobotState {
  GRIPPER_CLOSE_1,
  //HORIZONTAL_MOVE    
  GRIPPER_OPEN,   
  GRIPPER_CLOSE_2, 
  CLIMBING,  
  //HARVESTING         
  IDLE                
};

RobotState currentState = GRIPPER_CLOSE_1;

///////////
///A4988///
///////////
const int dir_pin = 2;
const int step_pin = 3;

const int stepsPerRevolution = 400;

/////////////////
///TBTB6612FNG///
/////////////////
const int STBY = 8;
//MOTOT A
const int PWMA = 5;
const int AIN1 = 7;
const int AIN2 = 6;
//MOTOT B
const int PWMB = 11;
const int BIN1 = 9;
const int BIN2 = 10;

//VL53L0X
Adafruit_VL53L0X lox = Adafruit_VL53L0X();

const int Target_Distance = 800; //80cm


void setup() {
  //VL53L0X
  Serial.begin(115200);

  if (!lox.begin()) {
    Serial.println("VL53L0X not found");
    while(1);
  }

  ///////////
  ///A4988///
  ///////////
  pinMode(dir_pin, OUTPUT);
  pinMode(step_pin, OUTPUT);

  /////////////////
  ///TB6612FNG///
  /////////////////
  pinMode(STBY, OUTPUT);

  pinMode(PWMA, OUTPUT);
  pinMode(AIN1, OUTPUT);
  pinMode(AIN2, OUTPUT);

  pinMode(PWMB, OUTPUT);
  pinMode(BIN1, OUTPUT);
  pinMode(BIN2, OUTPUT);

  digitalWrite(STBY, HIGH);


}

void loop() {
  switch(currentState){
    case GRIPPER_CLOSE_1:
      delay(500);  // 안정화 대기
      currentState = GRIPPER_OPEN;
      break;

    case GRIPPER_OPEN:
      Open_Gripper();
      delay(500);
      currentState = GRIPPER_CLOSE_2;
      break;

    case GRIPPER_CLOSE_2:
      Close_Gripper();
      delay(500);
      currentState = CLIMBING;
      break;

    case CLIMBING:
      {
        VL53L0X_RangingMeasurementData_t measure;
        lox.rangingTest(&measure, false);
        int distance = measure.RangeMilliMeter;
        Serial.println(distance);

        if (measure.RangeStatus !=4) {
          int distance = measure.RangeMilliMeter;
          
          if (distance >= Target_Distance) {
            Stop_Motors();
            currentState = IDLE;
          }else {
            Climb_Up();
          }
        }
        delay(50);
      } 
      break;

    case IDLE:
      delay(1000);
      break;
  }
}

void Close_Gripper() {
  digitalWrite(dir_pin, HIGH);//정방향
    //한바퀴 회전
    for(int i=0; i<stepsPerRevolution; i++) {

      digitalWrite(step_pin, HIGH);
      delayMicroseconds(1000);

      digitalWrite(step_pin, LOW);
      delayMicroseconds(1000);
    }
}

void Open_Gripper() {
  digitalWrite(dir_pin, LOW);//역방향
    //한바퀴 회전
    for(int i=0; i<stepsPerRevolution; i++) {
      digitalWrite(step_pin, HIGH);
      delayMicroseconds(1000);

      digitalWrite(step_pin, LOW);
      delayMicroseconds(1000)
    }
}

void Climb_Up() {
  //정방향
  digitalWrite(AIN1, HIGH);
  digitalWrite(AIN2, LOW);
  
  analogWrite(PWMA, 200);

  digitalWrite(BIN1, HIGH);
  digitalWrite(BIN2, LOW);
  
  analogWrite(PWMB, 200);
}

void Stop_Motors() {
  analogWrite(PWMA, 0);
  analogWrite(PWMB, 0);
}

