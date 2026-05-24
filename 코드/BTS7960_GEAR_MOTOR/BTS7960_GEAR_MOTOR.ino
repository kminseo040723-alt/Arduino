const int R_EN_1 = 2;   //오른쪽
const int L_EN_1 = 3;   //왼쪽
const int R_PWM_1 = 5;  
const int L_PWM_1 = 6;  

void setup() {

  pinMode(R_EN_1, OUTPUT);
  pinMode(L_EN_1, OUTPUT);
  pinMode(R_PWM_1, OUTPUT);
  pinMode(L_PWM_1, OUTPUT);

  digitalWrite(R_EN_1, HIGH);
  digitalWrite(L_EN_1, HIGH);
}
void loop() {
  analogWrite(R_PWM_1, 0);
  analogWrite(L_PWM_1, 50);
  delay(1000);
}