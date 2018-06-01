#include <SoftwareSinglebus.h>

SoftwareSinglebus  mySingleBus(2);

#define deviceID1    0x01
#define registerID  0x01

bool state_ = false;
char dataSum[10] = {0};

void serviceCB1(void* p){
  char i = 0;
  state_ = !state_;
  while(mySingleBus.available(deviceID1)){
    dataSum[i++] = mySingleBus.read(deviceID1);
  }
  Serial.print(dataSum);
  memset(dataSum,0,10);
  digitalWrite(13,state_);
}

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  
  mySingleBus.begin();
  mySingleBus.registerCB(deviceID1,serviceCB1,&mySingleBus);

  pinMode(13,OUTPUT);
}

void loop() {
  // put your main code here, to run repeatedly:
}