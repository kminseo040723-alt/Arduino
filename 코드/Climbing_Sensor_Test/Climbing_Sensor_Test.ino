#include <Wire.h>
#include <Adafruit_VL53L0X.h>

// BTS7960 수직 이동 모터 2개
const int R_EN[]  = { 7, 12 };
const int L_EN[]  = { 8, 2 };
const int R_PWM[] = { 5, 3 };
const int L_PWM[] = { 6, 11 };

const int Motor_Num = 2;

// Climb 설정
const int Climb_Intermediary_Speed = 180;
const int Climb_Max_Speed = 255;
const int Climb_Target_Distance = 800;

int Climb_Speed;
int currentSpeed[Motor_Num] = { 0, 0 };

// VL53L0X 거리 센서
Adafruit_VL53L0X lox = Adafruit_VL53L0X();

const int Scan_Max_Fail_Count = 10;
const int Climb_Max_Retry_Count = 2;
const unsigned long Scan_Interval = 50;

int Climb_Distance;
int Scan_Fail_Count = 0;
int Climb_Retry_Count = 0;
unsigned long Scan_Time;

enum ClimbState {
    CLIMB_RUNNING,
    CLIMB_DONE,
    CLIMB_FAIL
};

ClimbState climbState = CLIMB_RUNNING;

void setup() {
    Serial.begin(115200);
    Wire.begin();

    // BTS7960 핀 설정
    for (int i = 0; i < Motor_Num; i++) {
        pinMode(R_EN[i], OUTPUT);
        pinMode(L_EN[i], OUTPUT);
        pinMode(R_PWM[i], OUTPUT);
        pinMode(L_PWM[i], OUTPUT);

        digitalWrite(R_EN[i], LOW);
        digitalWrite(L_EN[i], LOW);
        analogWrite(R_PWM[i], 0);
        analogWrite(L_PWM[i], 0);
    }

    // 거리 센서 초기화
    if (!lox.begin()) {
        Serial.println("VL53L0X not found");
        while (1);
    }

    Climb_Enter();
}

void loop() {
    if (climbState == CLIMB_RUNNING) {
        climbState = Climb_Update();
    }
    else if (climbState == CLIMB_DONE) {
        Serial.println("Climb complete");
        Stop_Climbing();
        while (1);
    }
    else if (climbState == CLIMB_FAIL) {
        Serial.println("Climb failed");
        Stop_Climbing();
        while (1);
    }
}

void Climb_Enter() {
    Scan_Fail_Count = 0;
    Climb_Speed = 0;
    Scan_Time = millis() - Scan_Interval;

    for (int i = 0; i < Motor_Num; i++) {
        digitalWrite(R_EN[i], HIGH);
        digitalWrite(L_EN[i], HIGH);
    }
}

ClimbState Climb_Update() {
    if (millis() - Scan_Time >= Scan_Interval) {
        Scan_Time = millis();

        if (Read_Distance(Climb_Distance)) {
            Scan_Fail_Count = 0;

            Serial.print("Distance: ");
            Serial.println(Climb_Distance);

            if (Climb_Distance < Climb_Target_Distance) {
                Constant_Climbing();
                return CLIMB_RUNNING;
            }
            else {
                Stop_Climbing();
                Climb_Retry_Count = 0;
                return CLIMB_DONE;
            }
        }
        else {
            Scan_Fail_Count++;

            if (Scan_Fail_Count >= Scan_Max_Fail_Count) {
                Stop_Climbing(); 
                Scan_Fail_Count = 0;
                Climb_Retry_Count++;

                if (Climb_Retry_Count >= Climb_Max_Retry_Count) {
                    return CLIMB_FAIL;
                }

                Climb_Enter();
            }
        }
    }

    return CLIMB_RUNNING;
}

bool Read_Distance(int& Scan_distance) {
    VL53L0X_RangingMeasurementData_t measure;
    lox.rangingTest(&measure, false);

    if (measure.RangeStatus != 4) {
        Scan_distance = measure.RangeMilliMeter;
        return true;
    }
    else {
        Serial.println("Sensor error");
        return false;
    }
}

void Constant_Climbing() {
    Climb_Speed = Climb_Intermediary_Speed;

    for (int i = 0; i < Motor_Num; i++) {
        Set_Motor_Speed(i, Climb_Speed);
    }
}

void Set_Motor_Speed(int motor, int Motor_Speed) {
    if (motor < 0 || motor >= Motor_Num) return;

    Motor_Speed = constrain(Motor_Speed, 0, Climb_Max_Speed);
    currentSpeed[motor] = Motor_Speed;

    analogWrite(R_PWM[motor], currentSpeed[motor]);
    analogWrite(L_PWM[motor], 0);
}

void Stop_Climbing() {
    for (int i = 0; i < Motor_Num; i++) {
        analogWrite(R_PWM[i], 0);
        analogWrite(L_PWM[i], 0);

        digitalWrite(R_EN[i], LOW);
        digitalWrite(L_EN[i], LOW);

        currentSpeed[i] = 0;
    }
}