/////////////////
///TBTB6612FNG///
/////////////////
const int STBY = 9;
//MOTOT A
const int PWMA = 5;
const int AIN1 = 3;
const int AIN2 = 4;

void setup() {
  pinMode(STBY, OUTPUT);

  pinMode(PWMA, OUTPUT);
  pinMode(AIN1, OUTPUT);
  pinMode(AIN2, OUTPUT);


  digitalWrite(STBY, HIGH);

}

void loop() {
   digitalWrite(AIN1, HIGH);
  digitalWrite(AIN2, LOW);
  analogWrite(PWMA, 255);

  delay(3000);

}
