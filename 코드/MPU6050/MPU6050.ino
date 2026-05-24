#include <Wire.h>
const int MPU_addr=0x68;
int16_t Ax1, Ay1, Az1, Gx1, Gy1, Gz1;

void setup() {
  Serial.begin(115200);
  Wire.begin();
  Wire.beginTransmission(MPU_addr);
  Wire.write(0x6B);
  Wire.write(0);
  Wire.endTransmission(true);
}

void loop() {
  Read_MPU(Ax1, Ay1, Az1, Gx1, Gy1, Gz1);
  delay(50);
}

void Read_MPU(int16_t &Ax1, int16_t &Ay1, int16_t &Az1, int16_t &Gx1, int16_t &Gy1, int16_t &Gz1) {  

Wire.beginTransmission(MPU_addr);
Wire.write(0x3B);
Wire.endTransmission(false);
Wire.requestFrom(MPU_addr,14,true);
Ax1=Wire.read()<<8|Wire.read(); // 1byte = 8bit
Ay1=Wire.read()<<8|Wire.read();
Az1=Wire.read()<<8|Wire.read();
Wire.read(); 
Wire.read(); 
Gx1=Wire.read()<<8|Wire.read();
Gy1=Wire.read()<<8|Wire.read();
Gz1=Wire.read()<<8|Wire.read();

Serial.print("Accelerometer: ");
Serial.print("X = "); Serial.print(Ax1);
Serial.print(" | Y = "); Serial.print(Ay1);
Serial.print(" | Z = "); Serial.println(Az1);

Serial.print("Gyroscope: ");
Serial.print("X = "); Serial.print(Gx1);
Serial.print(" | Y = "); Serial.print(Gy1);
Serial.print(" | Z = "); Serial.println(Gz1);
}