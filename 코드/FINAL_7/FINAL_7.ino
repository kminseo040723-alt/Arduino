#include <Adafruit_VL53L0X.h>
#include <Adafruit_PWMServoDriver.h>
#include <Wire.h>

#define SERVO_MIN_PULSE_WIDTH 150
#define SERVO_MAX_PULSE_WIDTH 600
#define PCA_FREQUENCY 50

enum SystemMode {
    MODE_IDLE,
    MODE_RUNNING,
    MODE_DONE,
    MODE_FAIL
};

SystemMode CurrentMode = MODE_IDLE;

enum Signal {
    KEEP,
    NEXT,
    RETRY,
    FAIL
};

enum RobotState {
    Gripper_Open,
    Set_Injector,
    Horizontal_Move,
    Horizontal_Move_Reverse,
    Gripper_Close,
    Climb,
    Harvest
};
struct StateConfig {
    RobotState state;
    void (*onEnter)();
    Signal (*onUpdate)();
    void (*onExit)();
};

void Gripper_Open_Enter();
Signal Gripper_Open_Update();
void Stop_Gripper_Motor();

void Set_Injector_Enter();
Signal Set_Injectior_Update();
void Stop_Servo_Motor();

void Horizontal_Move_Enter();
Signal Horizontal_Move_Update();
void Stop_Horizontal_Motor();

void Horizontal_Move_Reverse_Enter();
Signal Horizontal_Move_Reverse_Update();

void Gripper_Close_Enter();
Signal Gripper_Close_Update();

void Climb_Enter();
Signal Climb_Update();
void Stop_Climbing_Motor();

void Harvest_Enter();
Signal Harvest_Update();
void Stop_Harvest_Climb_Motor();

void Stop_Motors();

struct StateConfig stateConfigs[] = {
    { Gripper_Open, Gripper_Open_Enter, Gripper_Open_Update, Stop_Gripper_Motor },
    {Set_Injector, Set_Injector_Enter, Set_Injectior_Update, Stop_Servo_Motor},
    { Horizontal_Move, Horizontal_Move_Enter, Horizontal_Move_Update, Stop_Horizontal_Motor },
    // {Horizontal_Move_Reverse, Horizontal_Move_Reverse_Enter, Horizontal_Move_Reverse_Update, Stop_Horizontal_Motor},
    { Gripper_Close, Gripper_Close_Enter, Gripper_Close_Update, Stop_Gripper_Motor },
    { Climb, Climb_Enter, Climb_Update, Stop_Climbing_Motor },
    { Harvest, Harvest_Enter, Harvest_Update, Stop_Harvest_Climb_Motor }
};

int System_Fail_Count = 0;
const int System_Max_Fail_Count = 2;
unsigned long System_Start_Time;
bool isNewSystem = true;
int currentState_Index = 0;
bool isNewState = true;
const unsigned long StartTime = 2000;

// HM0557 stepper motor
const int STR = 7;
const int DIR = 4;

// BTS7960 motor driver
const int R_PWM[] = { 5, 9, 10 };
const int L_PWM[] = { 3, 6, 11 };

const int Total_Motor_Num = 3;

// PCA9685 servo driver
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

const int Servo_CH[] = {4, 5};
const int Servo_Num = 2;

// Gripper open/close
const unsigned long Steps_Per_Revolution_Open = 13000;
const unsigned long Steps_Per_Revolution_Close = 35000;
const int Step_Delay_Open = 100;
const int Step_Delay_Close = 100;
unsigned long gripperStep = 0;

//Injector Setting& Harvest

const int Base_Angle = 90;
//Injector
int Injector_End_Angle_0;
int Injector_End_Angle_1;

const unsigned long Injector_Start = 500;
const unsigned long Injector_End = 1500;

unsigned long Injector_Time;
bool Injector_Closed;

//Harvest
int Harvest_Climb_End_Angle_0;
int Harvest_Climb_End_Angle_1;

const int Harvest_Climb_Speed = 40;

const unsigned long Harvest_Ready_Duration = 3000;
const unsigned long Harvest_Move_Duration = 4000;

unsigned long Harvest_Pause_Time;

// Horizontal movement
const int Motor_Horizontal = 2;
const int Move_Horizontal_Speed = 70;
const int Move_Horizontal_Reverse_Speed = 100;
const unsigned long Move_Horizontal_Duration = 24000;
const unsigned long Move_Horizontal_Reverse_Interval = 500;

unsigned long Move_Horizontal_Time;
unsigned long Move_Horizontal_Reverse_Time;

// Vertical movement

const int Climb_Motor_Num = 2;
const int Climb_Start_Speed = 50;
const int Climb_Min_Speed = 0;
const int Climb_Const_Speed = 180;
const int Climb_Max_Speed = 255;

const unsigned long Climb_Duration = 40900;
const unsigned long Stop_Before_Climbing_Duration = 3000;

int Climb_Speed;
int currentSpeed[Total_Motor_Num] = { 0, 0, 0 };
unsigned long Climb_Time;
unsigned long Stop_Before_Climbing;



void Move_Horizontal();
void Move_Horizontal_Reverse();

void Set_Climb_Motors(int speed);
void Stop_Climbing_Motor();

void Harvest_Climb_Slowly();

int angleToPulse(int angle) {
  angle = constrain(angle, 0, 180); 
  return map(angle, 0, 180, SERVO_MIN_PULSE_WIDTH , SERVO_MAX_PULSE_WIDTH);
}


void setup() {
    Serial.begin(115200);
    Wire.begin();

    pinMode(STR, OUTPUT);
    pinMode(DIR, OUTPUT);
    
    pwm.begin();
    pwm.setPWMFreq(PCA_FREQUENCY);

    for (int i = 0; i < Servo_Num; i++) {
    pwm.setPWM(Servo_CH[i], 0, angleToPulse(Base_Angle));
    }

    for (int i = 0; i < Total_Motor_Num; i++) {
        pinMode(R_PWM[i], OUTPUT);
        pinMode(L_PWM[i], OUTPUT);

        analogWrite(R_PWM[i], 0);
        analogWrite(L_PWM[i], 0);
    }   
}

void loop() {
    switch (CurrentMode) {
        case MODE_IDLE:
            if (isNewSystem) {
                Stop_Motors();
                System_Start_Time = millis();
                isNewSystem = false;
            }

            if (millis() - System_Start_Time >= StartTime) {
                currentState_Index = 0;
                isNewState = true;
                isNewSystem = true;
                CurrentMode = MODE_RUNNING;
            }
            break;

        case MODE_RUNNING:
            if (isNewState) {

                Serial.print("\n=== ENTER STATE : ");
                Serial.print(currentState_Index);
                Serial.print(" (");
                Serial.println(") ===");   

                if (stateConfigs[currentState_Index].onEnter != nullptr) {
                    stateConfigs[currentState_Index].onEnter();
                }

                isNewState = false;
                return;
            }

            Signal nextSignal = KEEP;
            if (stateConfigs[currentState_Index].onUpdate != nullptr) {
                nextSignal = stateConfigs[currentState_Index].onUpdate();
            }

            if (nextSignal == NEXT) {
                    Serial.print("Signal : NEXT ");
                if (stateConfigs[currentState_Index].onExit != nullptr) {
                    stateConfigs[currentState_Index].onExit();
                }
                currentState_Index++;

                if (currentState_Index >= sizeof(stateConfigs) / sizeof(StateConfig)) {
                    System_Fail_Count = 0;
                    isNewSystem = true;
                    CurrentMode = MODE_DONE;
                } else {
                    isNewState = true;
                }
            } else if (nextSignal == RETRY) {
                Serial.println("Signal : RETRY");
                isNewState = true;
            } else if (nextSignal == FAIL) {
                Serial.println("Signal : FAIL");
               
                if (stateConfigs[currentState_Index].onExit != nullptr) {
                    stateConfigs[currentState_Index].onExit();
                }
                System_Fail_Count++;

                if (System_Fail_Count < System_Max_Fail_Count) {
                    isNewSystem = true;
                    CurrentMode = MODE_IDLE;
                } else {
                    isNewSystem = true;
                    CurrentMode = MODE_FAIL;
                }
            }
            break;

        case MODE_DONE:
            if (isNewSystem) {
                Stop_Motors();
                isNewSystem = false;
            }
            while (1);
            break;

        case MODE_FAIL:
            if (isNewSystem) {
            Stop_Motors();
            Serial.println("System failed");
            isNewSystem = false;
            }
        while (1);
        break;
    }
}

void Gripper_Open_Enter() {
    gripperStep = 0;
    digitalWrite(DIR, HIGH);
}

Signal Gripper_Open_Update() {
    unsigned long Gripper_Open_Target_Step;

    Calculate_Motor_Step (Gripper_Open_Target_Step);

    if (gripperStep < Gripper_Open_Target_Step){
        digitalWrite(STR, HIGH);
        delayMicroseconds(Step_Delay_Open);
        digitalWrite(STR, LOW);
        delayMicroseconds(Step_Delay_Open);
        gripperStep++;
    } else {
        return NEXT;
    }

    return KEEP;
}

void Stop_Gripper_Motor() {
    digitalWrite(STR, LOW);
    gripperStep = 0;
}

void Set_Injector_Enter() {
    Injector_Time = millis();
    Injector_End_Angle_0 = Base_Angle - 60;
    Injector_End_Angle_1 = Base_Angle + 60;
    Injector_Closed = false;

}

Signal Set_Injectior_Update() {
    unsigned long elapsedTime = millis() - Injector_Time;

    if (!Injector_Closed && elapsedTime >= Injector_Start) {
        pwm.setPWM(Servo_CH[0], 0, angleToPulse(Injector_End_Angle_0));
        pwm.setPWM(Servo_CH[1], 0, angleToPulse(Injector_End_Angle_1));

        Injector_Closed = true;
    }

    if (Injector_Closed && elapsedTime >= Injector_Start + Injector_End) {
       for (int i = 0; i < Servo_Num; i++) {
        pwm.setPWM(Servo_CH[i], 0, angleToPulse(Base_Angle));
        }
        return NEXT;
    }
    return KEEP;
}


void Horizontal_Move_Enter() {
    Move_Horizontal_Time = millis();
    Move_Horizontal();
}

Signal Horizontal_Move_Update() {
    unsigned long elapsedTime = millis() - Move_Horizontal_Time;

    if (elapsedTime <= Move_Horizontal_Duration) {
        return KEEP;
    }
    return NEXT;
}

void Move_Horizontal() {
    currentSpeed[Motor_Horizontal] = Move_Horizontal_Speed;
    analogWrite(R_PWM[Motor_Horizontal], currentSpeed[Motor_Horizontal]);
    analogWrite(L_PWM[Motor_Horizontal], 0);
}

void Horizontal_Move_Reverse_Enter() {
    Move_Horizontal_Reverse_Time = millis();
    Move_Horizontal_Reverse();
}

Signal Horizontal_Move_Reverse_Update() {
    unsigned long elapsedTime = millis() - Move_Horizontal_Reverse_Time;

    if (elapsedTime <= Move_Horizontal_Reverse_Interval) {
        return KEEP;
    }
    return NEXT;
}

void Move_Horizontal_Reverse() {
    currentSpeed[Motor_Horizontal] = Move_Horizontal_Reverse_Speed;
    analogWrite(R_PWM[Motor_Horizontal], 0);
    analogWrite(L_PWM[Motor_Horizontal], currentSpeed[Motor_Horizontal]);
}

void Stop_Horizontal_Motor() {
    analogWrite(R_PWM[Motor_Horizontal], 0);
    analogWrite(L_PWM[Motor_Horizontal], 0);

    currentSpeed[Motor_Horizontal] = 0;
}


void Gripper_Close_Enter() {
    gripperStep = 0;
    digitalWrite(DIR, LOW);
}

Signal Gripper_Close_Update() {
    unsigned long Gripper_Close_Target_Step;

    Calculate_Motor_Step (Gripper_Close_Target_Step);

    if (gripperStep < Gripper_Close_Target_Step) {
        digitalWrite(STR, HIGH);
        delayMicroseconds(Step_Delay_Close);
        digitalWrite(STR, LOW);
        delayMicroseconds(Step_Delay_Close);
        gripperStep++;
    } else {
        return NEXT;
    }
    return KEEP;
}

void Calculate_Motor_Step (unsigned long &Step) {
    RobotState currentState = stateConfigs[currentState_Index].state;

    if (currentState==Gripper_Open) {
        Step = Steps_Per_Revolution_Open;
    } else if (currentState==Gripper_Close) {
        Step = Steps_Per_Revolution_Close;
    } 
}

void Climb_Enter() {
    Climb_Speed = Climb_Const_Speed;
    Climb_Time = millis();
    Stop_Before_Climbing = millis();
}

Signal Climb_Update() {
    if (millis()- Stop_Before_Climbing <= Stop_Before_Climbing_Duration) {
        return KEEP;
    }
    Set_Climb_Motors(Climb_Speed);
    if (millis() - Climb_Time >= Climb_Duration) {
        return NEXT;
    }
    return KEEP;
}

void Set_Climb_Motors(int speed) {
    Climb_Speed = constrain(speed, Climb_Min_Speed, Climb_Max_Speed);

    for (int i = 0; i < Climb_Motor_Num; i++) {
        currentSpeed[i] = Climb_Speed;
        analogWrite(R_PWM[i], Climb_Speed);
        analogWrite(L_PWM[i], 0);
    }
}

void Stop_Climbing_Motor() {
     for (int i = 0; i < Climb_Motor_Num; i++) {
        analogWrite(R_PWM[i], 0);
        analogWrite(L_PWM[i], 0);

        currentSpeed[i] = 0;
    }
    Climb_Speed = 0;
}

void Harvest_Enter() {
    Harvest_Pause_Time = millis();
    Harvest_Climb_End_Angle_0 = Base_Angle + 20;
    Harvest_Climb_End_Angle_1 = Base_Angle - 20;
}

Signal Harvest_Update() {
    unsigned long elapsed = millis() - Harvest_Pause_Time;

    Harvest_Climb_Slowly();

    if (elapsed >= Harvest_Ready_Duration) {

        pwm.setPWM(Servo_CH[0], 0,
                   angleToPulse(Harvest_Climb_End_Angle_0));

        pwm.setPWM(Servo_CH[1], 0,
                   angleToPulse(Harvest_Climb_End_Angle_1));
        
        delay (1000);

        for (int i = 0; i < Servo_Num; i++) {
        pwm.setPWM(Servo_CH[i], 0, angleToPulse(Base_Angle));
        }

        return NEXT;
    }

    return KEEP;
}

void Harvest_Climb_Slowly() {
    Set_Climb_Motors(Harvest_Climb_Speed);
}

void Stop_Servo_Motor() {
}

void Stop_Harvest_Climb_Motor() {
    Stop_Servo_Motor();
    Stop_Climbing_Motor();
}

void Stop_Motors() {
    Stop_Gripper_Motor();
    Stop_Horizontal_Motor();
    Stop_Climbing_Motor();
    Stop_Servo_Motor();
}
