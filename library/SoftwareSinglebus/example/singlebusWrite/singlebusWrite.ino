#include <SoftwareSinglebus.h>

SoftwareSinglebus  mySingleBus(2);

void SinglebusWriteByte(char reg, char* str, char len){
  mySingleBus.beginTransmission(0x01,WRITE);
  mySingleBus.write(reg);//register address
  while(len--)
	mySingleBus.write(*str++);//data
  mySingleBus.endTransmission();
}

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  mySingleBus.begin();
}

void loop() {
  // put your main code here, to run repeatedly:
  if(Serial.available() == 2){
    char turn_ = Serial.read();
    char spd_ = Serial.read();
    SinglebusWriteByte(0x02,&turn_,1);
    SinglebusWriteByte(0x01,&spd_,1);
  }
  delay(200);
}