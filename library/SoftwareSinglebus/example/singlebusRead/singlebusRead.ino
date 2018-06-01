#include <SoftwareSinglebus.h>

SoftwareSinglebus  mySingleBus(2);

char readdata(void){
  char data = -1;
  mySingleBus.beginTransmission(0x01,READ);
  mySingleBus.write(0x05);//register address
  mySingleBus.write(0x04);//length
  mySingleBus.endTransmission();
  mySingleBus.requestFrom(0x01,0x04);

  while(mySingleBus.available()){
    data = mySingleBus.read();  
    if(data != -1)
     Serial.println((unsigned char)data);
  }
  mySingleBus.endTransmission();

  return data;
}
void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  mySingleBus.begin();
}

void loop() {
  // put your main code here, to run repeatedly:
  char data = readdata();
  
  delay(10);
}