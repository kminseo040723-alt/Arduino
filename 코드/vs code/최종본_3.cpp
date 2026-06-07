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
const int Climb_Speed_Increment = 5;
const int Climb_Speed_Decrement = 5;
const int Climb_Min_Speed = 0;
const int Climb_Intermediary_Speed = 180;
const int Climb_Max_Speed = 255;
const unsigned long Climb_Decel_Interval = 200;

const int Climb_Grip_Step_Increment = 500;
const int Climb_Distance_Tolerance = 5;
const unsigned long Climb_Accel_Interval = 200;
const unsigned long Climb_Distance_Decrease_Duration = 2000;
const unsigned long Climb_Same_Position_Duration = 3000;
const unsigned long Grip_Tighten_Duration = 1000;
const unsigned long Re_Accel_Duration = 2000;

const int Climb_Step_Speed = 500;
unsigned long Climb_Accel_Time;
unsigned long Climb_Decel_Time;
unsigned long Climb_Stall_Start_Time;
unsigned long Climb_Accel_Start_Time;
unsigned long Climb_Phase_Start_Time;
unsigned long Last_Grip_Step_Time;
int Climb_Speed;
int currentSpeed[Total_Motor_Num] = { 0, 0, 0 };
int Climb_Last_Distance;
int Climb_Stall_Base_Distance;
int Climb_Gripper_Step;
bool Initial_Climb_Distance = false;

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

// VL53L0X distance sensor
Adafruit_VL53L0X lox = Adafruit_VL53L0X();

const int Climb_Intermediary_Target_Distance = 500;
const int Climb_Target_Distance = 800;
const int Scan_Max_Fail_Count = 10;
const int Climb_Max_Retry_Count = 2;
const unsigned long Scan_Interval = 20;
const unsigned long Distance_Average_Duration = 500;
int Climb_Distance = 0;
int Scan_Fail_Count = 0;
int Climb_Retry_Count = 0;
int Distance_Count_Times = 0;
unsigned long Scan_Time;
unsigned long Distance_Average_Start_Time;
unsigned long Distance_Sum = 0;
bool Sensor_Enabled = false;

void Move_Horizontal();
void Move_Horizontal_Reverse();

void Constant_Climbing();
void Decelerate_Climbing();
void Set_Climb_Motors(int speed);
void Reset_Climb_Distance_Trend();
void Reset_Distance_Average();
Scan_Distance_State Read_Averaged_Distance(int& Average_Distance);
bool Accelerate_Climbing_When_Distance_Decreases_Or_Holds(int distance);
void Stop_Climbing_Motor();
void Harvest_Climb_Slowly();
bool Read_Distance(int& Scan_distance);
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

    if (!lox.begin()) {
        if (Sensor_Enabled) {
            Serial.println("VL53L0X not found");
            while (1);
        }
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
    Scan_Fail_Count = 0;
    Climb_Speed = Climb_Start_Speed;
    Climb_Accel_Time = millis();
    Climb_Decel_Time = millis();
    Scan_Time = millis() - Scan_Interval;
    Reset_Climb_Distance_Trend();
    Reset_Distance_Average();
}

Signal Climb_Update() {
    if (!Sensor_Enabled) {
    Constant_Climbing();
    return KEEP;
    }

    Scan_Distance_State Distance_Result = Read_Averaged_Distance(Climb_Distance);

    if (Distance_Result == DISTANCE_WAITING) {
        return KEEP;
    }

    if (Distance_Result == DISTANCE_READY) {
        if (Climb_Distance >= Climb_Target_Distance) {
            Climb_Retry_Count = 0;
            return NEXT;
        }
        Scan_Fail_Count = 0;
        bool Should_Accelerate_Climbing = Accelerate_Climbing_When_Distance_Decreases_Or_Holds(Climb_Distance);
        
        if(Should_Accelerate_Climbing) {
            return KEEP;
        }

        if (Climb_Distance < Climb_Intermediary_Target_Distance) {
            Constant_Climbing();
            return KEEP;
        }

        if (Climb_Distance < Climb_Target_Distance) {
            Decelerate_Climbing();
            return KEEP;
        }
    }
    if (Distance_Result == DISTANCE_FAILED) {
        Scan_Fail_Count++;
    
        if (Scan_Fail_Count >= Scan_Max_Fail_Count) {
            Scan_Fail_Count = 0;
            Climb_Retry_Count++;
    
            if (Climb_Retry_Count >= Climb_Max_Retry_Count) {
                return FAIL;
            }
            Stop_Climbing();
            return RETRY;
        }
    }

    return KEEP;
}

bool Read_Distance(int& Scan_distance) {
    VL53L0X_RangingMeasurementData_t measure;
    lox.rangingTest(&measure, false);

    if (measure.RangeStatus != 4) {
        Scan_distance = measure.RangeMilliMeter;
        return true;
    }

    Serial.println("Sensor error");
    return false;
}

void Set_Climb_Motors(int speed) {
    Climb_Speed = constrain(speed, Climb_Min_Speed, Climb_Max_Speed);

    for (int i = 0; i < Climb_Motor_Num; i++) {
        currentSpeed[i] = Climb_Speed;
        analogWrite(R_PWM[i], 0);
        analogWrite(L_PWM[i], Climb_Speed);
    }
}

void Reset_Climb_Distance_Trend() {
    Initial_Climb_Distance = false;
    Climb_Last_Distance = 0;
    Climb_Stall_Base_Distance = 0;
    Climb_Stall_Start_Time = 0;
    Climb_Accel_Start_Time = 0;
    Climb_Gripper_Step = 0;
    Climb_Trend = Climb_Trend_Normal;
    Climb_Phase = Climb_Phase_Normal;
}

void Reset_Distance_Average() {
    Scan_Time = millis() - Scan_Interval;
    Distance_Average_Start_Time = millis();
    Distance_Sum = 0;
    Distance_Count_Times = 0;
}

Scan_Distance_State Read_Averaged_Distance(int& Average_Distance) {
    if (millis() - Scan_Time < Scan_Interval) {
        return DISTANCE_WAITING;
    }

    Scan_Time = millis();

    int distance = 0;
    if (Read_Distance(distance)) {
        Distance_Sum += distance;
        Distance_Count_Times++;
    }

    if (millis() - Distance_Average_Start_Time < Distance_Average_Duration) {
        return DISTANCE_WAITING;
    }

    if (Distance_Count_Times == 0) {
        Reset_Distance_Average();
        return DISTANCE_FAILED;
    }

    Average_Distance = Distance_Sum / Distance_Count_Times;
    Serial.print("Average Distance: ");
    Serial.println(Average_Distance);

    Reset_Distance_Average();
    return DISTANCE_READY;
}

bool Whether_Climb_Normal(int distance) {
    bool decreasing = distance <= Climb_Last_Distance - Climb_Distance_Tolerance;
    bool holding    = abs(distance - Climb_Stall_Base_Distance) <= Climb_Distance_Tolerance;

    if (decreasing) {
        if (Climb_Trend != Climb_Trend_Decreasing) {
            Climb_Trend = Climb_Trend_Decreasing;
            Climb_Stall_Start_Time = millis();
            Climb_Stall_Base_Distance = distance;
        }
        Climb_Last_Distance = distance;

        if (millis() - Climb_Stall_Start_Time >= Climb_Distance_Decrease_Duration) {
            Climb_Phase = Climb_Phase_Grip;
            Climb_Phase_Start_Time = millis();
            Last_Grip_Step_Time = millis();
            Climb_Gripper_Step = 0;
            return true;
        }
        return false;
    }

    if (holding) {
        if (Climb_Trend != Climb_Trend_Holding) {
            Climb_Trend = Climb_Trend_Holding;
            Climb_Stall_Start_Time = millis();
            Climb_Stall_Base_Distance = distance;
        }
        Climb_Last_Distance = distance;

        if (millis() - Climb_Stall_Start_Time >= Climb_Same_Position_Duration) {
            Climb_Phase = Climb_Phase_Grip;
            Climb_Phase_Start_Time = millis();
            Last_Grip_Step_Time = millis();
            Climb_Gripper_Step = 0;
            return true;
        }
        return false;
    }

    Climb_Phase = Climb_Phase_Normal;
    Climb_Trend = Climb_Trend_Normal;
    Climb_Last_Distance = distance;
    Climb_Stall_Base_Distance = distance;
    Climb_Stall_Start_Time = millis();
    return false;
}

void Climb_Grip() {
    if (Climb_Gripper_Step == 0) {
    Stop_Climbing_Motor();
    digitalWrite(DIR, LOW);
    }
    if (millis() - Last_Grip_Step_Time >= 2) {
        if(Climb_Gripper_Step < Climb_Grip_Step_Increment) {
            Last_Grip_Step_Time = millis();
            digitalWrite(STR, HIGH);
            delayMicroseconds(10);
            digitalWrite(STR, LOW);
            Climb_Gripper_Step++;
        }
    } 

    if (millis() - Climb_Phase_Start_Time >= Grip_Tighten_Duration) {
        Climb_Phase = Climb_Phase_Accel;
        Climb_Accel_Start_Time = millis();
        Climb_Accel_Time = millis();
        Climb_Speed = Climb_Intermediary_Speed;
        Set_Climb_Motors(Climb_Speed);

    }
}

bool Climb_Accel(int distance) {
   if (millis() - Climb_Accel_Start_Time < Re_Accel_Duration) {
        if (millis() - Climb_Accel_Time >= Climb_Accel_Interval) {
            Climb_Accel_Time = millis();
            Set_Climb_Motors(Climb_Speed + Climb_Speed_Increment);
        }
        return true;
    }

    Climb_Phase = Climb_Phase_Normal;
    Climb_Trend = Climb_Trend_Normal;
    Climb_Last_Distance = distance;
    Climb_Stall_Base_Distance = distance;
    Climb_Decel_Time = millis();
    Climb_Stall_Start_Time = millis();
    Climb_Gripper_Step = 0;
    return false;
}

bool Accelerate_Climbing_When_Distance_Decreases_Or_Holds(int distance) {
    if (!Initial_Climb_Distance) {
        Initial_Climb_Distance = true;
        Climb_Last_Distance = distance;
        Climb_Stall_Base_Distance = distance;
        Climb_Stall_Start_Time = millis();
        Climb_Trend = Climb_Trend_Normal;
        Climb_Phase = Climb_Phase_Normal;
        return false;
    }

    if (Climb_Phase == Climb_Phase_Grip) {
        Climb_Grip();
        return true;
    }

    if (Climb_Phase == Climb_Phase_Accel) {
        return Climb_Accel(distance);
    }

    return Whether_Climb_Normal(distance);
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

void Stop_Climbing() {
    Stop_Climbing_Motor();
    Reset_Climb_Distance_Trend();
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
    Climb_Accel_Time = millis();
    Scan_Time = millis() - Scan_Interval;
    Reset_Climb_Distance_Trend();
    Reset_Distance_Average();
    Write_Servos(Servo_Angle);
}

Signal Harvest_Update() {
    if (Servo_Rotation >= Harvest_Servo_Total_Rotations) {
        return NEXT;
    }

    Harvest_Climb_Slowly();
   
    Scan_Distance_State Distance_Result = Read_Averaged_Distance(Climb_Distance);

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