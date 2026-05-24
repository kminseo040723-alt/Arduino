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
void setup() {
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
   digitalWrite(BIN1, HIGH);
  digitalWrite(BIN2, LOW);
  analogWrite(PWMB, 255);

  delay(3000);

  // 정지
  analogWrite(PWMB, 0);
  delay(1000);

  // 역방향
  digitalWrite(BIN1, LOW);
  digitalWrite(BIN2, HIGH);
  analogWrite(PWMB, 255);

  delay(3000);

  analogWrite(PWMB, 0);
  delay(1000);
}
