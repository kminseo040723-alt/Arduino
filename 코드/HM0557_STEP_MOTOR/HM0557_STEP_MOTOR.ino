const int STR = 3;   // STEP/PULSE
const int DIR = 2;   // 방향
// const int EN  = 9;   // Enable

const int stepsPerRevolution = 400;

void setup() {
  pinMode(STR, OUTPUT);
  pinMode(DIR, OUTPUT);
  // pinMode(EN, OUTPUT);

  // digitalWrite(EN, HIGH); 
}

void loop() {
  digitalWrite(DIR, LOW);
  

  for(int i=0; i<stepsPerRevolution; i++) {

      digitalWrite(STR, HIGH);
      delayMicroseconds(50);

      digitalWrite(STR, LOW);
      delayMicroseconds(50);
    }
 
  // digitalWrite(DIR, HIGH);
  

  // for(int i=0; i<stepsPerRevolution; i++) {

  //     digitalWrite(STR, HIGH);
  //     delayMicroseconds(50);

  //     digitalWrite(STR, LOW);
  //     delayMicroseconds(50);
  //   }
  // delay(1000);
    // while(1);
  
}