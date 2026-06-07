#include <Adafruit_VL53L0X.h>
#include <Adafruit_PWMServoDriver.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>

#define SERVO_MIN_PULSE_WIDTH 650
#define SERVO_MAX_PULSE_WIDTH 2350
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
    FAIL,
    RECLIMB
};

enum RobotState {
    Gripper_Open,
    Horizontal_Move,
    Horizontal_Move_Reverse,
    Gripper_Close,
    Climb,
    Harvest
};

enum Scan_Distance_State {
    DISTANCE_WAITING,
    DISTANCE_READY,
    DISTANCE_FAILED
};

enum Climb_Trend_State {
    Climb_Trend_Normal,
    Climb_Trend_Decreasing,
    Climb_Trend_Holding
};

Climb_Trend_State Climb_Trend = Climb_Trend_Normal;

enum Climb_Phase_State {
    Climb_Phase_Normal,
    Climb_Phase_Grip,
    Climb_Phase_Accel
};

Climb_Phase_State Climb_Phase = Climb_Phase_Normal;
struct StateConfig {
    RobotState state;
    void (*onEnter)();
    Signal (*onUpdate)();
    void (*onExit)();
};

void Gripper_Open_Enter();
Signal Gripper_Open_Update();
void Stop_Gripper_Motor();

void Horizontal_Move_Enter();
Signal Horizontal_Move_Update();
void Stop_Horizontal_Motor();

void Horizontal_Move_Reverse_Enter();
Signal Horizontal_Move_Reverse_Update();

void Gripper_Close_Enter();
Signal Gripper_Close_Update();

void Climb_Enter();
Signal Climb_Update();
void Stop_Climbing();

void Harvest_Enter();
Signal Harvest_Update();
void Stop_Harvest_Climb_Motor();

void Stop_Motors();

struct StateConfig stateConfigs[] = {
    // { Gripper_Open, Gripper_Open_Enter, Gripper_Open_Update, Stop_Gripper_Motor },
    // { Horizontal_Move, Horizontal_Move_Enter, Horizontal_Move_Update, Stop_Horizontal_Motor },
    // {Horizontal_Move_Reverse, Horizontal_Move_Reverse_Enter, Horizontal_Move_Reverse_Update, Stop_Horizontal_Motor},
    // { Gripper_Close, Gripper_Close_Enter, Gripper_Close_Update, Stop_Gripper_Motor },
    { Climb, Climb_Enter, Climb_Update, Stop_Climbing },
    // { Harvest, Harvest_Enter, Harvest_Update, Stop_Harvest_Climb_Motor }
};

int System_Fail_Count = 0;
const int System_Max_Fail_Count = 2;
unsigned long System_Start_Time;
bool isNewSystem = true;
int currentState_Index = 0;
bool isNewState = true;
const unsigned long StartTime = 1000;
const int Climb_State_Index = 3;

// HM0557 stepper motor
const int STR = 7;
const int DIR = 8;

// BTS7960 motor driver
const int R_PWM[] = { 3, 6, 10 };
const int L_PWM[] = { 5, 9, 11 };

const int Total_Motor_Num = 3;

// PCA9685 servo driver
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

const int Servo_CH[] = { 4, 5, 6, 7};
const int Servo_Num = 4;

// Gripper open/close
const unsigned long Steps_Per_Revolution_Open = 17000;
const unsigned long Steps_Per_Revolution_Close = 20000;
const int Step_Delay_Open = 100;
const int Step_Delay_Close = 50;
unsigned long gripperStep = 0;

// Horizontal movement
const int Motor_Horizontal = 2;
const int Move_Horizontal_Speed = 180;
const int Move_Horizontal_Reverse_Speed = 100;
const unsigned long Move_Horizontal_Duration = 10000;
const unsigned long Move_Horizontal_Reverse_Interval = 1000;
unsigned long Move_Horizontal_Time;
unsigned long Move_Horizontal_Reverse_Time;

// Vertical movement

const int Climb_Motor_Num = 2;
const int Climb_Start_Speed = 50;
const int Climb_Speed_Decrement = 5;
const int Climb_Min_Speed = 0;
const int Climb_Intermediary_Speed = 180;
const int Climb_Max_Speed = 255;

const unsigned long Climb_Const_Time = 20000;
const unsigned long Climb_Distance_Decrease_Duration = 2000;
const unsigned long Climb_Decel_Interval = 200;

unsigned long Climb_Start_Time;
unsigned long Climb_Decel_Time;
int Climb_Speed;
int currentSpeed[Total_Motor_Num] = { 0, 0, 0 };
int Climb_Gripper_Step;

// Harvest
const int Harvest_Servo_Total_Rotations = 3;
const int Harvest_Servo_Start_Angle = 0;
const int Harvest_Servo_Max_Angle = 170;
const int Harvest_Servo_Angle_Increment = 10;
const int Harvest_Servo_Angle_Decrement = 10;

const int Harvest_Climb_Speed = 40;
const int Harvest_Reclimb_Distance_Drop = 300;

const unsigned long Servo_1_Interval = 100;
const unsigned long Servo_M1_Interval = 100;
const unsigned long Harvest_Ready_Duration = 1000;
int Servo_Angle = Harvest_Servo_Start_Angle;
int Servo_Rotation = 0;
int Servo_Direction = 1;
unsigned long Servo_Change_Angle_Time;
unsigned long Harvest_Pause_Time;

void Move_Horizontal();
void Move_Horizontal_Reverse();

void Constant_Climbing();
void Decelerate_Climbing();
void Set_Climb_Motors(int speed);
void Stop_Climbing_Motor();
void Harvest_Climb_Slowly();
void Write_Servo_Angle(int channel, int angle);
void Rotating_1();
void Rotating_2();
void Write_Servos(int Target_Angle);

void setup() {
    Serial.begin(115200);
    Wire.begin();

    pinMode(STR, OUTPUT);
    pinMode(DIR, OUTPUT);

    pwm.begin();
    pwm.setPWMFreq(PCA_FREQUENCY);

    Write_Servos(Harvest_Servo_Start_Angle);

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
            } else if (nextSignal == RECLIMB) {
                Serial.println("Signal : RECLIMB");
                
                if (stateConfigs[currentState_Index].onExit != nullptr) {
                    stateConfigs[currentState_Index].onExit();
                }
                currentState_Index = Climb_State_Index;
                isNewState = true;
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
       for (int i = 0; i < Climb_Motor_Num; i++) {
        analogWrite(R_PWM[i], 0);
        analogWrite(L_PWM[i], 0);
    }
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
    } else if (currentState==Climb) {
        Step = Climb_Step_Speed;
    }
}
void Climb_Enter() {
    Climb_Start_Time = millis();

    Climb_Speed = Climb_Intermediary_Speed;
    Climb_Decel_Time = millis();

    Set_Climb_Motors(Climb_Speed);
}

Signal Climb_Update() {
    unsigned long elapsed =
        millis() - Climb_Start_Time;

    if (elapsed < Climb_Const_Time) {
        Constant_Climbing();
        return KEEP;
    }

    if (elapsed <
        Climb_Constant_Time +
        Climb_Distance_Decrease_Duration) {

        Decelerate_Climbing();
        return KEEP;
    }

    Stop_Climbing();
    return NEXT;
}


void Constant_Climbing() {
    Set_Climb_Motors(Climb_Intermediary_Speed);
}

void Decelerate_Climbing() {
    if (millis() - Climb_Decel_Time < Climb_Decel_Interval) {
        return;
    }
    Climb_Decel_Time = millis();
    Set_Climb_Motors(Climb_Speed - Climb_Speed_Decrement);
}

void Set_Climb_Motors(int speed) {
    Climb_Speed = constrain(speed, Climb_Min_Speed, Climb_Max_Speed);

    for (int i = 0; i < Climb_Motor_Num; i++) {
        currentSpeed[i] = Climb_Speed;
        analogWrite(R_PWM[i], 0);
        analogWrite(L_PWM[i], Climb_Speed);
    }
}

void Stop_Climbing() {
    Stop_Climbing_Motor();
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
    Servo_Rotation = 0;
    Servo_Direction = 1;
    Servo_Change_Angle_Time = millis() - Servo_1_Interval;
    Servo_Angle = Harvest_Servo_Start_Angle;

    Harvest_Pause_Time = millis();

    Climb_Speed = Harvest_Climb_Speed;
    Write_Servos(Servo_Angle);
}

Signal Harvest_Update() {
    if (Servo_Rotation >= Harvest_Servo_Total_Rotations) {
        return NEXT;
    }

    Harvest_Climb_Slowly();
   
    if (Distance_Result == DISTANCE_READY) {
        if (Climb_Distance <= Climb_Target_Distance - Harvest_Reclimb_Distance_Drop) {
            return RECLIMB;
        }
    }

    if (millis() - Harvest_Pause_Time < Harvest_Ready_Duration) {
        return KEEP;
    }

     if (Servo_Direction == 1) {
        if (millis() - Servo_Change_Angle_Time >= Servo_1_Interval) {
            Servo_Change_Angle_Time = millis();
            Rotating_1();
        }
    } else if (Servo_Direction == -1) {
        if (millis() - Servo_Change_Angle_Time >= Servo_M1_Interval) {
            Servo_Change_Angle_Time = millis();
            Rotating_2();
        }
    }

    return KEEP;
}

void Harvest_Climb_Slowly() {
    Set_Climb_Motors(Harvest_Climb_Speed);
}

void Write_Servos(int Target_Angle) {
    for (int i = 0; i < Servo_Num; i++) {
        Write_Servo_Angle(Servo_CH[i], Target_Angle);
    }
}

void Rotating_1() {
    if (Servo_Angle < Harvest_Servo_Max_Angle) {
        Servo_Angle += Harvest_Servo_Angle_Increment;
    } else {
        Servo_Angle = Harvest_Servo_Max_Angle;
        Servo_Direction = -1;
    }

    Write_Servos(Servo_Angle);
}

void Rotating_2() {
    if (Servo_Angle > Harvest_Servo_Start_Angle) {
        Servo_Angle -= Harvest_Servo_Angle_Decrement;
    } else {
        Servo_Angle = Harvest_Servo_Start_Angle;
        Servo_Direction = 1;
        Servo_Rotation++;
    }

    Write_Servos(Servo_Angle);
}

void Write_Servo_Angle(int channel, int angle) {
    int pulseLength = map(angle, 0, 180, SERVO_MIN_PULSE_WIDTH, SERVO_MAX_PULSE_WIDTH);
    int pwmValue = pulseLength * PCA_FREQUENCY / 1000000.0 * 4096;

    pwm.setPWM(channel, 0, pwmValue);
}

void Stop_Servo_Motor() {
    for (int i = 0; i < Servo_Num; i++) {
        pwm.setPWM(Servo_CH[i], 0, 0);
    }

    Servo_Rotation = 0;
    Servo_Direction = 1;
}

void Stop_Harvest_Climb_Motor() {
    Stop_Servo_Motor();
    Stop_Climbing();
}

void Stop_Motors() {
    Stop_Gripper_Motor();
    Stop_Horizontal_Motor();
    Stop_Climbing_Motor();
    Stop_Servo_Motor();
}