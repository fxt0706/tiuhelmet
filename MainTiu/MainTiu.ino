/* 
 *  模块化使用
   温度气压海拔  SDA 2 SCL 3     气体 A0     倒地报警 SDA 2 SCL 3       磁场 7     屏幕 14 16      软串口   8 9 
 */
#include <SoftwareSerial.h>
#include <Wire.h>

#include "MQ135.h"
#include <Adafruit_BMP085.h>
#include <MPU6050.h>
#include <I2Cdev.h>

#define GAS         A0

#define SDA         2
#define SCL         3             
#define HALL        7
#define RING        6

MQ135 gasSensor = MQ135(GAS);     //RZero 校准
Adafruit_BMP085 bmp;              //bmp.begin()测试
MPU6050 accelgyro;

SoftwareSerial conSerial(8,9);   //RX,TX
SoftwareSerial scrSerial(14,16);

/////////////////////////////////////////////
int16_t ax,ay,az;
int16_t gx,gy,gz;

float C_Z = -1343.91;//Z轴零点偏移量
float C_Gyro = -99.90;//陀螺仪零点偏移量
float Z_Min = -17873.76;//最小极值
float Z_Max = 15186.91;//最大极
float T_Z = 3;//Z轴角度补偿时间常数
float R_Z = 180/(Z_Max - Z_Min);//Z轴比
float R_Gyro = 0.081;//陀螺仪比例
unsigned long T_Now =0;//系统当前时间
unsigned long T_Last;//上次时间
float Angle_G,Angle_AG,Angle_GG;
float lastAngle_AG=0;
float GYRO;
///////////////////////////////////////

float ppm;
float temperature;
float pressure;
int beatValue = 0;
char bump = 'A';
int hall;

void setup() 
{
    delay(1000);
    conSerial.begin(9600);
    scrSerial.begin(9600);
    Serial.begin(9600);
    Serial1.begin(9600);
    bmp.begin();
    accelgyro.initialize();
    pinMode(RING,OUTPUT);
    digitalWrite(RING,LOW);
}

void loop() 
{
    delay(10);
    doGas();
    delay(10);
    doTemper();
    delay(10);
    doPressure();
    delay(10);
    doHall();
    delay(10);
    doBeat();
    delay(10);
    doBump();
    delay(10);
    sendCon();
    if(bump == 'B')
       digitalWrite(RING,HIGH);
    else
       digitalWrite(RING,LOW);
}


//////////////////////模块性//////////////////////////

void doGas()
{ 
    ppm = gasSensor.getPPM();
    //Serial.print("the gas PPM is ");
    //Serial.println(ppm);

    scrSerial.print("air.txt=\"");
    scrSerial.print(ppm);
    scrSerial.print("\"");
    doWrite();
}

void doTemper()
{
    temperature = bmp.readTemperature();
    //Serial.print("Temperature = ");
    //Serial.print(temperature);
    //Serial.println(" *C");

    scrSerial.print("temp.txt=\"");
    scrSerial.print(temperature);
    scrSerial.print("\"");
    doWrite();
}

void doPressure()
{
    pressure = bmp.readPressure();
    //Serial.print("Pressure = ");
    //Serial.print(pressure/100000);
    //Serial.println(" Pa");

    scrSerial.print("pre.txt=\"");
    scrSerial.print(pressure/100000);
    scrSerial.print("\"");
    doWrite();

    pressure = pressure/100000;
}

void doHall()
{
     hall = digitalRead(HALL);
     if(hall == 1 )
     {
        scrSerial.print("hall.txt=\"No\"");
        doWrite();
     }
     else
     {
        scrSerial.print("hall.txt=\"Yes\"");
        doWrite();
     }
}

void doBeat()
{
    if(Serial1.available() > 0)
    {
        int beat = beatValue;
        beatValue = int(Serial1.read());
        if(beatValue < 45)
            beatValue = beat;
    }
    Serial1.flush();
}

void doBump()
{
   accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz); //读取6050数
   float Angle_Z = (az-C_Z)*R_Z;//加速度计 角度计算 （读取值-偏移量）比例单位：°
   Angle_G = -(GYRO-C_Gyro)*R_Gyro;//陀螺仪采样 （采样值-偏移量）比例 单位：°/s
   Angle_AG = Angle_AG +(((Angle_Z -Angle_AG)*1/T_Z) + Angle_G)*0.005;
   Angle_GG = Angle_GG + Angle_G*0.005;//陀螺仪对X轴积分 得出角度。
   if(lastAngle_AG==0||Angle_AG>=lastAngle_AG)
   {
       bump = 'A';
       digitalWrite(RING,LOW);
   }
   else
   {
       bump = 'B';
       digitalWrite(RING,HIGH);
   }
   lastAngle_AG=Angle_AG;
   
}

void sendCon()
{
    conSerial.print(1);
    conSerial.print(",");
    conSerial.print(bump);
    conSerial.print(",");
    if(beatValue < 100)
    {
        conSerial.print(0);
        if(beatValue == 0)
          conSerial.print(0);
    }
    conSerial.print(beatValue);
    conSerial.print(",");
    if(int(temperature) == 0)
        conSerial.print(0);
    conSerial.print(int(temperature));
    conSerial.print(",");
    conSerial.print(hall);
    conSerial.print(",");
    conSerial.print(pressure,1);
    conSerial.print(",");
    if(ppm < 10)
      conSerial.print(0);
    conSerial.print(ppm,2);
    conSerial.print(",");
    delay(500);
    
}

//////////////////////功能性//////////////////////////
/*
float mapfloat(float x, float in_min, float in_max, float out_min, float out_max)
{
     return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}


int averageAnalogRead(int pinToRead)
{
    byte numberOfReadings = 8;
    unsigned int runningValue = 0; 

    for(int x = 0 ; x < numberOfReadings ; x++)
      runningValue += analogRead(pinToRead);
    runningValue /= numberOfReadings;

    return(runningValue);  
}

*/

void doWrite()
{
    scrSerial.write(0XFF);
    scrSerial.write(0XFF);
    scrSerial.write(0XFF);
}

