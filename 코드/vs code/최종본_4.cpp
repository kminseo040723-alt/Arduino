
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
const unsigned long Climb_Start_Time_Delay = 1000;

const int Climb_Step_Speed = 500;
unsigned long Climb_Accel_Time;
unsigned long Climb_Decel_Time;
unsigned long Climb_Stall_Start_Time;
unsigned long Climb_Accel_Start_Time;
unsigned long Climb_Phase_Start_Time;
unsigned long Last_Grip_Step_Time;
unsigned long Climb_Start_Time;
int Climb_Speed;
int currentSpeed[Total_Motor_Num] = { 0, 0, 0 };
int Climb_Last_Distance;
int Climb_Stall_Base_Distance;
int Climb_Gripper_Step;
bool Initial_Climb_Distance = false;


void Climb_Enter() {
    Scan_Fail_Count = 0;
    Climb_Speed = Climb_Start_Speed;
    Climb_Accel_Time = millis();
    Climb_Decel_Time = millis();
    Climb_Start_Time = millis();
    Scan_Time = millis() - Scan_Interval;
    Reset_Climb_Distance_Trend();
    Reset_Distance_Average();
}

Signal Climb_Update() {
    if (millis()-Climb_Start_Time < Climb_Start_Time_Delay) {
        return KEEP;
    }
    if (!Sensor_Enabled) {
    Constant_Climbing();
    Use_Step_Motor_While_Climbing();
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
        analogWrite(R_PWM[i], Climb_Speed);
        analogWrite(L_PWM[i], 0);
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

void Use_Step_Motor_While_Climbing(); {
        unsigned long now = micros();

        if (now - Step_Last_Time >= Step_While_Climbing_Interval) {
        Step_Last_Time = micros ();
        digitalWrite(STR, HIGH);
        delayMicroseconds(Step_Close_Delay_While_Climbing);
        digitalWrite(STR, LOW);
        delayMicroseconds(Step_Close_Delay_While_Climbing);
        }
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
