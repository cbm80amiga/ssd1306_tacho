// Fidget Spinner counter and tachometer
// 27.08.2017 Pawel A. Hernik
// source code for the video:
// https://youtu.be/42qNfPOYlR8

/*
 Parts:
 - Hall sensor 3144
 - Neodymium magnet for fidget spinner
 - OLED SSD3106
 - Arduino Pro Mini/Nano
 
 3144 pinout from front:
 1 - VCC
 2 - GND
 3 - DATA (0 when close to magnet)
*/

#define USEHW 0  // 0 - use fast softi2c, 1 - use hw implementation on A4/A5

#if USEHW==1
#include <Wire.h>
#else
//#define SDA_PORT PORTC
//#define SDA_PIN 4  // A4
//#define SCL_PORT PORTC
//#define SCL_PIN 5  // A5

#define SDA_PORT PORTB
#define SDA_PIN 3  // D11
#define SCL_PORT PORTB
#define SCL_PIN 2  // D10

#define I2C_FASTMODE 1
#include <SoftI2CMaster.h>
#endif

#include "OLED_SoftI2C.h"
#include "term8x14_font.h"

const int hallPin = 2; // pin 2 = int 0

OLEDSoftI2C oled(0x3c);
char txt[10];

// --------------------------------------------------------------------------

volatile unsigned long cntTime=0;
volatile unsigned long cnt=0;

void doCount() // interrupt callback should be as short as possible!
{
  if(digitalRead(hallPin) == LOW)
  {
    cnt++;
    cntTime = millis();
  }
}

// --------------------------------------------------------------------------

void setup()
{
  Serial.begin(9600);
  oled.init();
  oled.clrScr();
  oled.setFont(Term8x14PL);
  oled.printStr(0, 0, "Rot:");
  oled.printStr(0, 2, "RPM:");
  oled.printStr(0, 4, "Max:");
  oled.printStr(0, 6, "Time:        s");
  oled.setMinCharWd(8);
  pinMode(hallPin,INPUT_PULLUP);
  digitalWrite(hallPin,HIGH);
  attachInterrupt(digitalPinToInterrupt(hallPin), doCount, FALLING);  // hall pin on interrupt 0 = pin 2
  digitalWrite(hallPin,HIGH);
  pinMode(hallPin,INPUT_PULLUP);
  cntTime = millis();
}

// --------------------------------------------------------------------------

volatile unsigned long rpm=0,maxrpm=0;
int dispRotTime=0, rotTime=0;
unsigned long measureTime=0,curTime,startTime=0;
int dispCnt=0,measureCnt=0;

const int resetTime = 2000;
const int minRotNum = 1;  // 1 - calc after every rotation

void loop()
{
  curTime = millis();
  if(curTime-cntTime>resetTime) { // reset when less than 30RPM (dt>2s)
    cnt = measureCnt = 0;
    rpm = 0;
  }
  if(cnt==1) startTime = cntTime;
  if(cnt-measureCnt>=minRotNum) {
    rpm = 60000L*(cnt-measureCnt)/(cntTime-measureTime);
    //Serial.println(String("Cnt=")+cnt+" dcnt="+(cnt-measureCnt)+" dtime="+(cntTime-measureTime));
    measureCnt = cnt;
    measureTime = cntTime;
  }
  rotTime = (cntTime-startTime)/1000; // time in seconds
  if(cnt>1 || !dispRotTime) {  // keep previous time on the OLED until new rotation starts
    dispRotTime = rotTime;
    dispCnt = cnt;
  }
  if(rpm>maxrpm) maxrpm=rpm;
  sprintf(txt,"% 5u",dispCnt);     oled.printStr(38, 0, txt);
  sprintf(txt,"% 5u",rpm);         oled.printStr(38, 2, txt);
  sprintf(txt,"% 5u",maxrpm);      oled.printStr(38, 4, txt);
  sprintf(txt,"% 5u",dispRotTime); oled.printStr(38, 6, txt);
}

// --------------------------------------------------------------------------

