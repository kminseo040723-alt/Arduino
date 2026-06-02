const int R_EN_1 = 7;   //오른쪽
const int L_EN_1 = 8;   //왼쪽
const int R_PWM_1 = 5;  
const int L_PWM_1 = 6;  

void setup() {

  pinMode(R_EN_1, OUTPUT);
  pinMode(L_EN_1, OUTPUT);
  pinMode(R_PWM_1, OUTPUT);
  pinMode(L_PWM_1, OUTPUT);

  digitalWrite(R_EN_1, LOW);
  digitalWrite(L_EN_1, LOW);
}
void loop() {
  analogWrite(R_PWM_1, 0);
  analogWrite(L_PWM_1, 150);
  delay(1000);

  analogWrite(R_PWM_1, 0);
  analogWrite(L_PWM_1, 0);
  digitalWrite(R_EN_1, LOW);
  digitalWrite(L_EN_1, LOW);
  delay(1000);
}