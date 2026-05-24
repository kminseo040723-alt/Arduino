#include <Adafruit_VL53L0X.h>
#include <Wire.h>

enum RobotState {
  GRIPPER_CLOSE_1,
  GRIPPER_OPEN,   
  GRIPPER_CLOSE_2, 
  CLIMBING,        
  IDLE                
};
  RobotState currentState = GRIPPER_CLOSE_1;

///////////
///A4988///
///////////
const int dir_pin = 2;
const int step_pin = 3;
const int EN  = 4;

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
// Adafruit_VL53L0X lox = Adafruit_VL53L0X();

// const int Target_Distance = 800; //80cm

void setup() {
  //VL53L0X

  // Serial.begin(115200);

  // if (!lox.begin()) {
  //   Serial.println("VL53L0X not found");
  //   while(1);
  // }

  ///////////
  ///A4988///
  ///////////
  pinMode(dir_pin, OUTPUT);
  pinMode(step_pin, OUTPUT);
  pinMode(EN, OUTPUT);
  digitalWrite(EN, HIGH);
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
  switch (currentState) {
    case GRIPPER_CLOSE_1:
      delay(500);  // 안정화 대기
      currentState = GRIPPER_OPEN;
      break;

    case GRIPPER_OPEN:
      Move_Gripper(false);
      delay(500);
      currentState = GRIPPER_CLOSE_2;
      break;

    case GRIPPER_CLOSE_2:
      Move_Gripper(true);
      delay(500);
      currentState = CLIMBING;
      break;

    case CLIMBING:
      {
        // VL53L0X_RangingMeasurementData_t measure;
        // lox.rangingTest(&measure, false);
        
        // if (measure.RangeStatus != 4) {
        // int distance = measure.RangeMilliMeter;
        // Serial.println(distance);
        //   if (distance >= Target_Distance) {
          //   Stop_Motors();
          //   currentState = IDLE;
          // }else {
            Climb_Up();
        //   }
        // } else {
        //   Stop_Motors ();
        //   currentState = IDLE;
        // }
        // delay(50);
      } 
      break;

    case IDLE:
      Stop_Motors();
      break;
  }
}
void Move_Gripper(bool Direction) {
  digitalWrite(dir_pin, Direction ? HIGH : LOW);

for(short i=0; i<stepsPerRevolution; i++) {
    digitalWrite(step_pin, HIGH);
    delayMicroseconds(1000);

    digitalWrite(step_pin, LOW);
    delayMicroseconds(1000);

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

  digitalWrite(AIN1, LOW);
  digitalWrite(AIN2, LOW);
  digitalWrite(BIN1, LOW);
  digitalWrite(BIN2, LOW);
}
