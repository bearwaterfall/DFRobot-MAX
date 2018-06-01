/*!
   @file MAX.ino
   @brief DFRobot's Explorer MAX
   @n [Get the module here]()
   @n This example is a factory procedure, obstacle avoidance, patrol, search light, bluetooth remote control functions.
   @n [Connection and Diagram](http://wiki.dfrobot.com.cn/index.php?title=(SKU:ROB0137)_%E6%8E%A2%E7%B4%A2%E8%80%85MAX%E6%9C%BA%E5%99%A8%E4%BA%BA))_探索者MAX机器人)

   @copyright  [DFRobot](http://www.dfrobot.com), 2017
   @copyright GNU Lesser General Public License

   @author [lijun](ju.li@dfrobot.com)
   @debuggers[shichao](sc.mingyang@dfrobot.com)
   @version  V1.1
   @date  2017-08-31
*/

#include <Adafruit_NeoPixel.h>
#include <SoftwareSinglebus.h>
#include <Metro.h>
#include "GoBLE.h"


/*-----------------电机控制端口设置-----------------*/
#define EN1  5//控制右侧电机速度   
#define EN2  6//控制左侧电机速度
#define IN1  4//控制右侧电机方向
#define IN2  7//控制左侧电机方向
#define Key  12//按键
#define PIN 16          //灯IO 

#define NUMBER 4       //共有4个灯
#define FORW 0//前进
#define BACK 1//后退
#define DEFAULTSPEED 120
#define sdaPin 3     //音乐IO
//传感器端口设置
#define singleBusPin 2     //总数DATA
const char URLM = 0x02; //超声波传感器地址
const char findLine = 0x03; //巡线传感器地址
const char Expression = 0x4; //表情板传感器地址

unsigned char buf[10] = {0}; //接收总线数据
unsigned char findLineflag;
int buttonState[6];
int Direction_R = FORW, Direction_L = FORW; //前进方向
unsigned int Distance;
unsigned int Mode = 1, Mode1 = 0; //小车模式
unsigned int Mark = 1, Mark_1 = 0; //当前状态
unsigned long lasttime = 0, Modetime = 0, Musictime = 0, RGBtime = 0,Smiliestime = 0,Watchtime = 0; //
uint8_t ExpressionCnt = 0x00;
uint8_t ExpressionColor = 0x01;
uint8_t Music = 1,Watch = 1;
uint8_t speedL=DEFAULTSPEED,speedR=DEFAULTSPEED; //定义左右轮的速度
uint16_t findLinecnt=0;
uint32_t a = 0, b = 1, c = 2, d = 3;
uint32_t e,f;
SoftwareSinglebus  mySingleBus(singleBusPin);  //打开总线
Adafruit_NeoPixel strip = Adafruit_NeoPixel(NUMBER, PIN, NEO_GRB + NEO_KHZ800);   //灯参数

///////////////**********总线读数据************/////////////////
unsigned char readdata(char hardwareAddr, char registerAddr, char len) 
{
  int data = 0;
  unsigned char readFlag = 0;
  mySingleBus.beginTransmission(hardwareAddr, READ);
  mySingleBus.write(registerAddr);//寄存器地址
  mySingleBus.write(len);  //长度
  mySingleBus.endTransmission();
  mySingleBus.requestFrom(hardwareAddr, len);
  if (mySingleBus.available() == len)
  {
    for (int i = 0; i < len; i++)
    {
      buf[i] = mySingleBus.read();
    }
    readFlag = 1;
  }
  delayMicroseconds(500);
  return (readFlag);
}

///////////////****总线写数据****/////////////////
void SinglebusWrite(char hardwareAddr, char registerAddr, int str)
{
  int i;
  mySingleBus.beginTransmission(hardwareAddr, WRITE);
  mySingleBus.write(registerAddr); //注册地址
  mySingleBus.write(str);         //数据
  mySingleBus.endTransmission();
  delayMicroseconds(500);
}

///////////////****控制电机转动子函数****/////////////////
void Motor_Control(int M1_DIR, int M1_EN, int M2_DIR, int M2_EN)
{
  //////////M1////////////////////////
  if (M1_DIR == FORW) //M1电机的方向
    digitalWrite(IN1, HIGH); //置高，设置方向向前
  else
    digitalWrite(IN1, LOW); //置低，设置方向向后
  if (M1_EN == 0)  //M1电机的速度
    analogWrite(EN1, LOW); //置低，停止
  else
    analogWrite(EN1, M1_EN); //否则，就设置相应的数值
  ///////////M2//////////////////////
  if (M2_DIR == FORW) //M2电机的方向
    digitalWrite(IN2, HIGH); //置高，方向向前
  else
    digitalWrite(IN2, LOW); //置低，方向向后
  if (M2_EN == 0) //M2电机的速度
    analogWrite(EN2, LOW); //置低，停止
  else
    analogWrite(EN2, M2_EN); //否则，就设置相应的数值
}

///////////////**********音乐函数************/////////////////
void NVCwrite(uint8_t temp)
{
  digitalWrite(sdaPin, LOW);
  delay(2);
  for (int i = 0; i < 8; i++) 
  {
    digitalWrite(sdaPin, HIGH); // 对音乐IO口置高电平
    if (temp & 1) 
    {
      delayMicroseconds(1500);
      digitalWrite(sdaPin, LOW); // 对音乐IO口置低电平
      delayMicroseconds(500);
    }
    else
    {
      delayMicroseconds(500);
      digitalWrite(sdaPin, LOW);
      delayMicroseconds(1500);
    }
    temp >>= 1;
  }
  digitalWrite(sdaPin, HIGH);
}

///////////////**********巡线函数************/////////////////
void findLineControl(unsigned char Direction) 
{
  static uint8_t findLineFlag;
  uint8_t maxSpeed = 70, minSpeed =50; //定义左右两个轮子的最大值和最小值                                                                                                                                
  uint8_t slow = 1, fast = minSpeed;

  if (Direction)
  {
    findLinecnt = 0;
  }
  else
  {
    findLinecnt++;
    if (findLinecnt > 100)
    {
      findLinecnt = 101;
      Motor_Control(FORW, 0, FORW, 0);
      return;
    }
  }

  switch (Direction)         //判断max方向与所巡路径的相对值，从而实行相应的方向调整
  {
    case 0x06:     //->left
      speedR = DEFAULTSPEED + 50 ;                                                                                                                                   
      speedL = DEFAULTSPEED;
      findLineFlag = Direction;
      break;
    case 0x04:
      speedR = DEFAULTSPEED + 50 ;
      speedL = DEFAULTSPEED - 30 ;
      findLineFlag = Direction;
      break;
    case 0x03:     //->right
      speedR = DEFAULTSPEED;
      speedL = DEFAULTSPEED + 50;
      findLineFlag = Direction;
      break;
    case 0x01:
      speedR = DEFAULTSPEED - 30 ;
      speedL = DEFAULTSPEED + 50 ;
      findLineFlag = Direction;
      break;
    case 0x02:
      speedR = DEFAULTSPEED;
      speedL = DEFAULTSPEED;
      findLineFlag = Direction;
      break;
    case 0x00:   //no return
      if (Mark == 1)
      {
        NVCwrite(9);
        Mark == 2;
        Musictime = millis();
      }

       if (0x01 & findLineFlag) //调节MAX动作时运动速度，可以使运行更平稳
      {
        if (speedR > DEFAULTSPEED)
        {
          speedR -= fast;
        }
        else if (speedL < maxSpeed)
        {
          speedL += fast;
        }
        else if (speedR > minSpeed)
        {
          speedR -= fast;
        }
      }
      else
      {
        if (speedL > DEFAULTSPEED)
        {
          speedL -= fast;
        }
        else if (speedR < maxSpeed)
        {
          speedR += fast;
        }
        else if (speedL > minSpeed)
        {
          speedL -= fast;
        }
      }

      if (millis() - 500 > Musictime)
      {
        NVCwrite(15);
      }
      break;
    default: break;
  }
  Motor_Control(FORW, speedL, FORW, speedR);
}

///////////////////****RGB灯光模式函数****////////////////////////
void RGB_Common(uint32_t a, uint32_t b, uint32_t c)//RGB灯四个灯亮同一个颜色
{
  strip.setPixelColor(0, strip.Color( a, b, c) ); //0,指第1个RGB灯，后面的值是R，G，B其值为0-255
  strip.setPixelColor(1, strip.Color( a, b, c) ); 
  strip.setPixelColor(2, strip.Color( a, b, c) );
  strip.setPixelColor(3, strip.Color( a, b, c) );
  strip.show();
}

void RGB_two(uint32_t x, uint32_t y, uint32_t z, uint32_t w,uint32_t a, uint32_t b, uint32_t c)//RGB灯2个灯亮同一个颜色,2个灯灭
{
  strip.setPixelColor(x, strip.Color( a, b, c) ); //0,指第1个RGB灯，后面的值是R，G，B其值为0-255
  strip.setPixelColor(y, strip.Color( a, b, c) ); 
  strip.setPixelColor(z, strip.Color( 0, 0, 0) );
  strip.setPixelColor(w, strip.Color( 0, 0, 0) );
  strip.show();
}

void RGB(uint32_t a, uint32_t b, uint32_t c, uint32_t d)//4个RGB灯亮红绿蓝白四个颜色
{
  strip.setPixelColor(a, strip.Color( 255, 0, 0) ); //a,指第a+1位上的RGB灯，从0开始；后面的值是R，G，B其值为0-255
  strip.setPixelColor(b, strip.Color( 0, 255, 0) ); //
  strip.setPixelColor(c, strip.Color( 0, 0, 255) );
  strip.setPixelColor(d, strip.Color( 250, 245, 245) );
  strip.show();
}
void RGBx(uint32_t a, uint32_t b, uint32_t c, uint32_t d)//4个RGB灯亮橙蓝深蓝四个颜色
{
  strip.setPixelColor(a, strip.Color(  200, 73, 0) ); 
  strip.setPixelColor(b, strip.Color( 30, 144, 255) ); 
  strip.setPixelColor(c, strip.Color( 153,51, 250) );
  strip.setPixelColor(d, strip.Color( 51, 161,201) );
  strip.show();
}


void RGBy(uint32_t a, uint32_t b, uint32_t c, uint32_t d)//4个RGB灯亮橙蓝深蓝四个颜色
{
  strip.setPixelColor(a, strip.Color(  200, 73, 0) ); 
  strip.setPixelColor(b, strip.Color( 30, 144, 255) ); 
  strip.setPixelColor(c, strip.Color( 153,51, 250) );
  strip.setPixelColor(d, strip.Color( 0, 201,201) );
  strip.show();
}
void RGB1(uint32_t a, uint32_t b, uint32_t c, uint32_t d)//4个RGB蓝白两种颜色
{
  strip.setPixelColor(a, strip.Color( 255, 245, 245 )); 
  strip.setPixelColor(b, strip.Color( 0, 255, 0) );
  strip.setPixelColor(c, strip.Color( 0, 255, 0) );
  strip.setPixelColor(d, strip.Color( 250, 245, 245) );
  strip.show();
}
void RGB2(uint32_t a, uint32_t b, uint32_t c, uint32_t d )//4个RGB中一个橙色三个青色
  {strip.setPixelColor(a, strip.Color( 255, 128, 0) ); 
  strip.setPixelColor(b, strip.Color( 0, 255, 255) ); 
  strip.setPixelColor(c, strip.Color( 0, 255, 255) );
  strip.setPixelColor(d, strip.Color( 0, 255, 255) );
  strip.show();
}
void RGB3(uint32_t a, uint32_t b, uint32_t c, uint32_t d, uint32_t e, uint32_t f)//4个RGB灯亮红绿蓝白紫罗兰五个颜色
{
  strip.setPixelColor(a, strip.Color(250, 245, 245 )); 
  strip.setPixelColor(b, strip.Color( 255, 0, 0) );
  strip.setPixelColor(c, strip.Color( 255, 0, 0) );
  strip.setPixelColor(d, strip.Color( 250, 245, 245) );
  strip.setPixelColor(e, strip.Color( 30, 144, 255) );
  strip.setPixelColor(f, strip.Color( 30, 144, 255) );
  strip.show();
}

void RGB4(uint32_t a, uint32_t b, uint32_t c, uint32_t d)//4个RGB中一个橙色，三个青色
{
  strip.setPixelColor(a, strip.Color( 255, 128, 0) ); 
  strip.setPixelColor(b, strip.Color(  0, 255, 255) ); 
  strip.setPixelColor(c, strip.Color(  0, 255, 255) );
  strip.setPixelColor(d, strip.Color(  0, 255, 255) );
  strip.show();
}

///////////////////****初始化****////////////////////////
void setup()
{
  int i;
  mySingleBus.begin();
  Goble.begin();//
  strip.begin();       //启动RGB灯
  Serial.begin(115200);    // 打开串口，设置速率为115200 bps

  SinglebusWrite(URLM, 0x05, 0x01); //启动超声波
  SinglebusWrite(Expression, 0x05, 0x01); //使能表情板
  SinglebusWrite(Expression, 0x07, 0x0A); //设置表情板亮度0x01-0xA0
  SinglebusWrite(Expression, 0x08, 0x00); //表情板内部库表情0x00-0x16
  SinglebusWrite(Expression, 0x09, 0x01); //表情板显示颜色0x01-0x07

  pinMode(sdaPin, OUTPUT);
  for (i = 4; i <= 7; i++) //设置控制电机的各端口为输出模式
    pinMode(i, OUTPUT);
  pinMode(Key, INPUT);
  pinMode(PIN, OUTPUT);

  NVCwrite(0xE7);   //音乐音量，0xE0-0xE7,由小到大;0xF2循环播放;0xFE停止播放

  SinglebusWrite(Expression, 0x08, 0x14);
  SinglebusWrite(Expression, 0x09, 0x05);
  RGB_Common(255, 0, 0); //红色
  NVCwrite(12);//播放编号12音乐
  delay(500);
  RGB_Common(0, 255, 0); //绿色
  delay(500);
  RGB_Common(0, 0, 255); //蓝色
  delay(1000);
  RGB_Common(0, 0, 0); //关灯
  NVCwrite(12);
  delay(1000);
  lasttime = millis();
}

void loop()
{
  int a1 = 1,a2 = 1,a3 = 1,a4 = 1,a5 = 1,a6 = 1,a7 = 1;
  if (millis() - 150 > Modetime)//150ms检测一次是否按键按下改变模式
  {
    Modetime = millis();
    if (digitalRead(Key) == LOW)
      if (digitalRead(Key) == LOW)
        Mode += 1;
    if (Mode > 4)
      Mode = 1;

    if (Mode1 != Mode) //检测到按键按下
    {
      SinglebusWrite(findLine, 0x14, 0x00); //关巡线传感器LED灯
      Motor_Control(FORW, 0, FORW, 0); //停止
      Mode1 = Mode;
    }

    while (digitalRead(Key) == LOW);
  }

  //1.避障程序模式
  if (Mode == 1)
  {
    //RGB_Common(255, 0, 0); //红色
    RGB_two(a,b,c,d,255,0,0);//红色
    if (millis() - 150 > RGBtime)//150ms循环一次，灯光逆时针旋转
    {
      if (a == 3) a = 0;
      else a += 1;
      if (b == 3) b = 0;
      else b += 1;
      if (c == 3) c = 0;
      else c += 1;
      if (d == 3) d = 0;
      else d += 1;
      RGBtime = millis();
    }

    if(millis() - 1000 > Watchtime)
    {
      if(Watch == 4)
        Watch = 1;
      else
        Watch +=1;
      
      Watchtime = millis();
    }
    
    readdata(URLM, 0x08, 0x02);//读取超声波的值
    Distance = (unsigned int)(buf[0] << 8) + buf[1]; //读取的值赋给Distance
     Serial.println( Distance);
    if (Distance < 105 && Distance > 95 && a1 == 1)
    {
      NVCwrite(1);//播放编号1音乐
      a1=0; a2=1; a3=1; a4=1; a5=1; a6==1; a7==1;
    }

    else if (Distance < 95 && Distance > 85 && a2 == 1)
    {
      NVCwrite(2);//播放编号2音乐
     a1=1; a2=0; a3=1; a4=1; a5=1; a6==1; a7==1;
    }

    else if (Distance < 85 && Distance > 75 && a3 == 1)
    {
      NVCwrite(3);//播放编号3音乐
      a1=1; a2=1; a3=0; a4=1; a5=1; a6==1; a7==1;
    }

    else if (Distance < 75 && Distance > 65 && a4 == 1)
    {
      NVCwrite(4);//播放编号4音乐
      a1=1; a2=1; a3=1; a4=0; a5=1; a6==1; a7==1;
    }

    else if (Distance < 65 && Distance > 55 && a5 == 1)
    {
      NVCwrite(5);//播放编号5音乐
      a1=1; a2=1; a3=1; a4=1; a5=0; a6==1; a7==1;
    }

    else if (Distance < 55 && Distance > 45 && a6 == 1)
    {
      NVCwrite(6);//播放编号6音乐
     a1=1; a2=1; a3=1; a4=1; a5=1; a6==0; a7==1;
    }

   else if (Distance <45 && Distance > 35 && a7 == 1)
    {
      NVCwrite(7);//播放编号7音乐
      a1=1; a2=1; a3=1; a4=1; a5=1; a6==1; a7==0;
    }

    else if (Distance < 35 && Distance > 15)
    {
      NVCwrite(9);
      if(Watch == 1)
       {  Motor_Control(FORW, 140, BACK, 140);delay(300); }//右转                                                                                                   
      else if(Watch == 2)
       { Motor_Control(BACK, 140, FORW, 140);delay(300);} //左转
      else if(Watch == 3)
       {Motor_Control(BACK, 60, BACK,140);  //后退左偏转
       delay(600);
        }
      else if(Watch == 4)
       { Motor_Control(BACK, 140, BACK,60); //后退右偏转
       delay(600);
        }
    }
    
    else if(Distance = 15 && Distance < 15)
     {  
  Motor_Control(FORW, 100, BACK,100);       //原地掉头                                                      
    delay(1200);
    }
    else
     { for(int j = 50;j < 120;j++) { Motor_Control(FORW, j, FORW, j);} //前进
     }

    if (millis() - 1000 > Smiliestime)//1s改变一次表情和色彩
    {
       SinglebusWrite(Expression, 0x08, ExpressionCnt);
      SinglebusWrite(Expression, 0x09, ExpressionColor);

      if (ExpressionCnt == 0x16)
        ExpressionCnt = 0x00;
      else
        ExpressionCnt += 1;

      if (ExpressionColor == 0x07)
        ExpressionColor = 0x01;
      else
        ExpressionColor += 1;

      Smiliestime = millis();
    }
  }
  

  //2.巡线程序模式
  else if (Mode == 2)
  {
    //RGB_Common(0, 255, 0); //绿色
    RGB_two(a,b,c,d,0,255,0); //绿色
    if (millis() - 150 > RGBtime)//150ms循环一次，灯光逆时针旋转
    {
      if (a == 3) a = 0;
      else a += 1;
      if (b == 3) b = 0;
      else b += 1;
      if (c == 3) c = 0;
      else c += 1;
      if (d == 3) d = 0;
      else d += 1;
      RGBtime = millis();
    }
    SinglebusWrite(findLine, 0x14, 0x01); //开巡线传感器LED灯
    //SinglebusWrite(findLine,0x14,0x00);  //关巡线传感器LED灯

    if (millis() - 1000 > Smiliestime)//循环表情和表情显示彩色
    {
      SinglebusWrite(Expression, 0x08, ExpressionCnt);
      SinglebusWrite(Expression, 0x09, ExpressionColor);

      if (ExpressionCnt == 0x16)
        ExpressionCnt = 0x00;
      else
        ExpressionCnt += 1;

      if (ExpressionColor == 0x07)
        ExpressionColor = 0x01;
      else
        ExpressionColor += 1;

      Smiliestime = millis();
    }

    if (readdata(findLine, 0x13, 0x01))//读取巡线传感器的值
    {
      if (findLineflag == buf[0])
      {
        findLineControl(findLineflag);

        if (millis() - 500 > Musictime)//循环播放1号到8号音乐，500ms改变一次
        {
          NVCwrite(Music);

          if (Music == 8)
            Music = 1;
          else
            Music += 1;

          Musictime = millis();
          Mark = 1;
        }
      }
      findLineflag = buf[0];//将巡线值赋给findLineflag
    }
  }

  //3.追光程序模式
  else if (Mode == 3)
  {
    RGB_two(a,b,c,d,30,144,255);//道奇蓝色
    if (millis() - 150 > RGBtime)//150ms循环一次，灯光逆时针旋转
    {
      if (a == 3) a = 0;
      else a += 1;
      if (b == 3) b = 0;
      else b += 1;
      if (c == 3) c = 0;
      else c += 1;
      if (d == 3) d = 0;
      else d += 1;
      RGBtime = millis();
    }
    readdata(URLM, 0x0B, 0x04);//读取光敏传感器数据
    int rightLed = (int)(buf[0] << 8) + buf[1];//右光敏传感器数据
    int leftLed = (int)(buf[2] << 8) + buf[3];//左光敏传感器数据

    if (leftLed - rightLed > 130)//光线在右，向右移动                                                                                                
    {
      Motor_Control(FORW, 150, FORW, 50);
      SinglebusWrite(Expression, 0x08, 0x01);
      SinglebusWrite(Expression, 0x09, 0x02);
      NVCwrite(13);
    }

    else if (rightLed - leftLed > 130)//光线在左，向左移动
    {
      Motor_Control(FORW, 50, FORW, 150);
      SinglebusWrite(Expression, 0x08, 0x02);
      SinglebusWrite(Expression, 0x09, 0x02);
      NVCwrite(13);
    }

    else if (rightLed + leftLed < 1000)//光线在前方，向前移动
    {
      Motor_Control(FORW, 100, FORW, 100);
      SinglebusWrite(Expression, 0x08, 0x14);
      SinglebusWrite(Expression, 0x09, 0x05);
    }

    else//未找到光线
    {
      Motor_Control(FORW, 0, FORW, 0);
      SinglebusWrite(Expression, 0x08, 0x00);
      SinglebusWrite(Expression, 0x09, 0x01);
    }
  }
  //4.蓝牙程序模式
  else if (Mode == 4)
  {
    if (Goble.available())//读取蓝牙数据
    {
      buttonState[SWITCH_UP]     = Goble.readSwitchUp();
      buttonState[SWITCH_DOWN]   = Goble.readSwitchDown();
      buttonState[SWITCH_LEFT]   = Goble.readSwitchLeft();
      buttonState[SWITCH_RIGHT]  = Goble.readSwitchRight();
      buttonState[SWITCH_SELECT] = Goble.readSwitchSelect();
      buttonState[SWITCH_START]  = Goble.readSwitchStart();
     
       if (buttonState[1] == PRESSED)  Mark_1 = 1; //向前

      else if (buttonState[2] == PRESSED) Mark_1 = 2; //向右

      else if (buttonState[3] == PRESSED) Mark_1 = 3; //向后

      else if (buttonState[4] == PRESSED) Mark_1 = 4;//向左

      else if (buttonState[5] == PRESSED) Mark_1 = 5; //停止

      else if (buttonState[6] == PRESSED) Mark_1 = 6;//原地转动
    }

    switch (Mark_1)
     {
       case 1:
        { RGB1(a, b,c,d); //RGB模式为正面两白灯，尾部两绿灯
          a=2;b=3;c=0;d=1;//RGB灯分配的位置
       
          for(int j = 60;j < 150;j++)Motor_Control(FORW, j, FORW, j);}//前进
          break;  
      case 2:
        { RGB2(a, b,c,d);//RGB模式为右前方为橙色灯，其余为兰色
          a=1;b=2;c=0;d=3;
         for(int j = 100;j < 150;j++)Motor_Control(FORW, j, FORW, 40);}//右转
         break;
      case 3:
        { RGB3(a, b,c,d,e,f);//RGB模式为正面两白灯，尾部两红灯灯
         a=1;b=0;c=3;d=2;
         if (millis() - 200 > RGBtime)//150ms循环一次，灯光逆时针旋转
        {
        if (b == 3) b = 0;
        else b += 3;
        if (c == 3)c = 0;
        else c += 3;
        if (e == 3) e = 0;
        else e += 3;
        if (f == 3) f = 0;
        else f += 3;
      RGBtime = millis();}
           for(int j = 90;j < 130;j++)Motor_Control(BACK, j, BACK, j);//后退
        }
        break;
      case 4:  
        { RGB4(a, b,c,d); //RGB模式为左前方为橙色灯，其余为兰色
          a=2;b=0;c=1;d=3;      
           for(int j = 100;j < 150;j++)Motor_Control(FORW, 40, FORW, j);}//左转
        break;
      case 5:
       Motor_Control(FORW, 0, FORW, 0);//停止
       RGB(a, b,c,d);
       if (millis() - 300 > RGBtime)//150ms循环一次，灯光逆时针旋转
     {
      if (a == 3) a = 0;
      else a += 1;
      if (b == 3) b = 0;
      else b += 1;
      if (c == 3) c = 0;
      else c += 1;
      if (d == 3) d = 0;
      else d += 1;
      RGBtime = millis();
    }break;
       case 6:
       { RGBx(a, b,c,d);  
       if (millis() - 200> RGBtime)//150ms循环一次，灯光逆时针旋转
    {
      if (a == 0) a = 3;
      else a -= 1;
      if (b == 0) b = 3;
      else b-= 1;
      if (c == 0) c = 3;
      else c -= 1;
      if (d == 0) d = 3;
      else d -= 1;
      RGBtime = millis();
    } 
      Motor_Control(FORW, 100,  BACK, 100);}  break;//原地旋转
      default:
  {  RGBy(a, b,c,d); a=0;b=1;c=2;d=3;}
       break;
        }
       }
     }

