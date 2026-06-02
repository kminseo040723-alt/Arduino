const int R_PWM_1 = 5;  
const int L_PWM_1 = 6;  

void setup() {
  pinMode(R_PWM_1, OUTPUT);
  pinMode(L_PWM_1, OUTPUT);
}
void loop() {
  analogWrite(R_PWM_1, 0);
  analogWrite(L_PWM_1, 150);
  delay(1000);

  analogWrite(R_PWM_1, 0);
  analogWrite(L_PWM_1, 0);-
  delay(1000);
}