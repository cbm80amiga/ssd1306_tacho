// Fast OLED SoftI2C/HW I2C
// 2015-2016 Pawel A. Hernik

#ifndef _OLED_SoftI2C_H
#define _OLED_SoftI2C_H 

#define OLED_CMD 0x00   // Co = 0, D/C = 0
#define OLED_DAT 0x40   // Co = 0, D/C = 1

#include <Arduino.h>
#include <avr/pgmspace.h>

// ---------------------------------
class OLEDSoftI2C {
 public:
// ---------------------------------
OLEDSoftI2C()
{
  i2cAddr = 0x3c;
}
// ---------------------------------
OLEDSoftI2C(int addr)
{
  i2cAddr = addr;
}
// ---------------------------------
void setFont(const uint8_t* _font)
{
  font     = _font;
  xSize    =-pgm_read_byte(font+0);
  ySize    = pgm_read_byte(font+1);
  offs     = pgm_read_byte(font+2);
  numChars = pgm_read_byte(font+3);
  ySize8   = (ySize+7)/8;
  minCharWd = 0;
}
// ---------------------------------
void setMinCharWd(uint8_t wd) { minCharWd = wd; }
// ---------------------------------
void printStr(uint8_t x, uint8_t row, char *characters)
{
  uint8_t xpos = x;
  uint8_t ypos = row;
  while (*characters) {
    if(*characters==10) { // \n clears area to the end of line and skips to the next line
      fillWin(xpos,ypos,128-xpos,ySize8,0);
      xpos = 0;
      ypos += ySize8;
      characters++;
    } else {
      int wd = printChar(xpos, ypos, *characters++);
      xpos += wd;
      if(xpos>127) {
        xpos = 0;
        ypos += ySize8;
      }
    }
  }
}
// ---------------------------------
uint8_t printChar(uint8_t x, uint8_t row, uint8_t ch)
{
  if(ch < offs)
    return 0;

  int i, idx = 4 + (ch - offs)*(xSize*ySize8+1);
  int wd = pgm_read_byte(font + idx++);
  int wdL = 0, wdR = 1; // default spacing before and behind char
  if(minCharWd>wd) {
    wdL = (minCharWd-wd)/2;
    wdR += minCharWd-wd-wdL;
  }
  setVWin(x, row, wd+wdL+wdR, ySize8);
  wdL *= ySize8;
  wdR *= ySize8;
  wd *= ySize8;
#if USEHW==1
  Wire.beginTransmission(i2cAddr);
  Wire.write(OLED_DAT);
  for(i=0; i<wdL; i++) Wire.write(0);
  for(i=0; i<wd; i++)  Wire.write(pgm_read_byte(font+idx+i));
  for(i=0; i<wdR; i++) Wire.write(0);
  Wire.endTransmission();
#else
  i2c_start(i2cAddr<<1 | I2C_WRITE);
  i2c_write(OLED_DAT);
  for(i=0; i<wdL; i++) i2c_write(0);
  for(i=0; i<wd; i++)  i2c_write(pgm_read_byte(font+idx+i));
  for(i=0; i<wdR; i++) i2c_write(0);
  i2c_stop();
#endif
  return (wd+wdL+wdR)/ySize8;
}
// ---------------------------------
void setHWin(uint8_t colS, uint8_t rowS, uint8_t wd, uint8_t ht)
{
  writeCmd(0x20); //set horizontal addressing mode for screen clear
  writeCmd(0x00);
  
  writeCmd(0x21); //set column start and end address
  writeCmd(colS); //set column start address
  writeCmd(colS+wd-1); //set column end address
  
  writeCmd(0x22); //set row start and end address
  writeCmd(rowS); //set row start address
  writeCmd(rowS+ht-1); //set row end address
}
// ---------------------------------
void setVWin(uint8_t colS, uint8_t rowS, uint8_t wd, uint8_t ht)
{
  writeCmd(0x20); //set horizontal addressing mode for screen clear
  writeCmd(0x01);
  
  writeCmd(0x21); //set column start and end address
  writeCmd(colS); //set column start address
  writeCmd(colS+wd-1); //set column end address
  
  writeCmd(0x22); //set row start and end address
  writeCmd(rowS); //set row start address
  writeCmd(rowS+ht-1); //set row end address
}
// ---------------------------------
void writeData(byte data)
{
#if USEHW==1
  Wire.beginTransmission(i2cAddr);
  Wire.write(OLED_DAT);
  Wire.write(data);
  Wire.endTransmission();
#else
  i2c_start(i2cAddr<<1 | I2C_WRITE);
  i2c_write(OLED_DAT);
  i2c_write(data);
  i2c_stop();
#endif
}
// ---------------------------------
void writeCmd(byte command)
{
#if USEHW==1
  Wire.beginTransmission(i2cAddr);
  Wire.write(OLED_CMD);
  Wire.write(command);
  Wire.endTransmission();
#else
  i2c_start(i2cAddr<<1 | I2C_WRITE);
  i2c_write(OLED_CMD);
  i2c_write(command);
  i2c_stop();
#endif
}
// ---------------------------------
void init()
{
#if USEHW==1
  Wire.begin();
  Wire.setClock(800000); // faster I2C
#else
  Serial.println(i2c_init() ? F("SoftI2C initialization done") : F("SoftI2C initialization error. SDA or SCL are low"));
#endif
  
  writeCmd(0x8d); //enable charge pump
  writeCmd(0x14);
  delay(1);
  writeCmd(0xaf); //set display on

//  writeCmd(0xa8); //set MUX ratio
//  writeCmd(0x3f);
  writeCmd(0xd3); //set display offset
  writeCmd(0x00);
  writeCmd(0x40); //set display start line
  writeCmd(0xa1); //set segment re-map (horizontal flip) - reset value is 0xa0 (or 0xa1)
  writeCmd(0xc8); //set COM output scan direction (vertical flip) - reset value is 0xc0 (or 0xc8)
  writeCmd(0xda); //set COM pins hardware configuration
  writeCmd(0x12); //reset value is 0x12
  writeCmd(0x81); //set contrast (2-byte)
  writeCmd(0xff);
//  writeCmd(0xa4); //disable entire display on
//  writeCmd(0xa6); //set normal display
//  writeCmd(0xd5); //set oscillator frequency
//  writeCmd(0x80);
//  writeCmd(0xdb); //vcomh deselect level (brightness)
//  writeCmd(0x20);
}
// ---------------------------------
void fillWin(int x, int y, int w, int h, int val)
{
  setVWin(x,y,w,h);
#if USEHW==1
  for (int i=0; i<1024; i+=16) 
  {
    Wire.beginTransmission(i2cAddr);
    Wire.write(OLED_DAT);
    for(int i=0; i<(128-x)*h; i++) Wire.write(val);
    Wire.endTransmission();
  }
#else
  i2c_start(i2cAddr<<1 | I2C_WRITE);
  i2c_write(OLED_DAT);
  for(int i=0; i<(128-x)*h; i++) i2c_write(val);
  i2c_stop();
#endif
}
// ---------------------------------
void clrScr()
{
  setVWin(0,0,128,8);
#if USEHW==1
  for (int i=0; i<1024; i+=16) 
  {
    Wire.beginTransmission(i2cAddr);
    Wire.write(OLED_DAT);
    for(int j=0; j<4; j++) 
      {Wire.write(0); Wire.write(0); Wire.write(0); Wire.write(0);}
    Wire.endTransmission();
  }
#else
  i2c_start(i2cAddr<<1 | I2C_WRITE);
  i2c_write(OLED_DAT);
  for (int i=0; i<1024/4; i++) {i2c_write(0); i2c_write(0); i2c_write(0); i2c_write(0); }
  i2c_stop();
#endif
}
/*
void scroll()
{
  writeCmd(0x27); //set right horizontal scroll
  writeCmd(0x0);  //dummy byte
  writeCmd(0x0);  //page start address
  writeCmd(0x7);  //scroll speed
  writeCmd(0x7);  //page end address
  writeCmd(0x0);  //dummy byte
  writeCmd(0xff); //dummy byte
  writeCmd(0x2f); //start scrolling
}
*/
// ---------------------------------
void drawBitmap(const uint8_t *bmp, uint8_t colS, uint8_t rowS, uint8_t wd, uint8_t ht)
{
  setHWin(colS,rowS,wd,ht);
#if USEHW==1
  for (int i=0; i<wd*ht; i++) {
    Wire.beginTransmission(i2cAddr);
    Wire.write(OLED_DAT);
    Wire.write(pgm_read_byte(bmp+i));
    Wire.endTransmission();
  }
#else
  i2c_start(i2cAddr<<1 | I2C_WRITE);
  i2c_write(OLED_DAT);
  for (int i=0; i<wd*ht; i++) 
     i2c_write(pgm_read_byte(bmp+i));
  i2c_stop();
#endif
}
// ---------------------------------
void drawBitmap(const uint8_t *bmp, uint8_t colS, uint8_t rowS)
{
  uint8_t wd = pgm_read_byte(bmp+0);
  uint8_t ht = pgm_read_byte(bmp+1);
  drawBitmap(bmp+2, colS, rowS, wd, ht);
}
// ---------------------------------
 private:
  int i2cAddr = 0x3c;
  const uint8_t* font; 
  uint8_t xSize;
  uint8_t ySize;
  uint8_t ySize8;
  uint8_t offs;
  uint8_t numChars;
  uint8_t minCharWd;
};
#endif

