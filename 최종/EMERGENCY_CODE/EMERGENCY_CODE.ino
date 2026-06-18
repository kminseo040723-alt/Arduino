//Emergency Code
const int R_PWM[] = {5, 9};
const int L_PWM[] = {3, 6};

const int Total_Motor_Num = 2;

const int Motor_Speed = 100; 

void setup () {
    for (int i = 0; i < Total_Motor_Num; i++) {
        pinMode(R_PWM[i], OUTPUT);
        pinMode(L_PWM[i], OUTPUT);

        analogWrite(R_PWM[i], 0);
        analogWrite(L_PWM[i], 0);
    }
}

void loop() {
    for (int i = 0; i < Total_Motor_Num; i++) {
        analogWrite(R_PWM[i], Motor_Speed);
        analogWrite(L_PWM[i], 0);
    }
}
