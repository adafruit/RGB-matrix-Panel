#include "RGBmatrixPanel.h"

uint8_t RGBmatrixPanel::width() {return WIDTH; }

uint8_t RGBmatrixPanel::height() {return HEIGHT; }


RGBmatrixPanel::RGBmatrixPanel(uint8_t a, uint8_t b, uint8_t c, 
			       uint8_t latch, uint8_t oe) {

  _a = a;
  _b = b;
  _c = c;
  _latch = latch;
  _oe = oe;
  //_sclk = sclk;

  pwmcounter = 0;
  scansection = 0;

  //sclkportreg =  portOutputRegister(digitalPinToPort(sclk));
  //sclkpin = digitalPinToBitMask(sclk);
 
  latportreg =  portOutputRegister(digitalPinToPort(latch));
  latpin = digitalPinToBitMask(latch);
  
  oeportreg =  portOutputRegister(digitalPinToPort(oe));
  oepin = digitalPinToBitMask(oe);

  addraportreg =  portOutputRegister(digitalPinToPort(a));
  addrapin = digitalPinToBitMask(a);
  addrbportreg =  portOutputRegister(digitalPinToPort(b));
  addrbpin = digitalPinToBitMask(b);
  addrcportreg =  portOutputRegister(digitalPinToPort(c));
  addrcpin = digitalPinToBitMask(c); 
}


void RGBmatrixPanel::begin(void) {
  pinMode(_a, OUTPUT);
  digitalWrite(_a, LOW); 
  pinMode(_b, OUTPUT);
  digitalWrite(_b, LOW); 
  pinMode(_c, OUTPUT);
  digitalWrite(_c, LOW); 
  pinMode(_latch, OUTPUT);
  digitalWrite(_latch, LOW); 
  //pinMode(_sclk, OUTPUT);
  //digitalWrite(_sclk, LOW); 
  pinMode(_oe, OUTPUT);
  digitalWrite(_oe, HIGH); 
 
  RGBMATRIX_DATADDR = 0xFF << RGBMATRIX_DATASHIFT;
  RGBMATRIX_DATAPORT = 0x0;
  RGBMATRIX_CLOCKDDR |= _BV(RGBMATRIX_CLOCKPIN);
}

uint16_t RGBmatrixPanel::Color444(uint8_t r, uint8_t g, uint8_t b) {
  uint16_t c;
  
  c = r;
  c <<= 4;
  c |= g & 0xF;
  c <<= 4;
  c |= b & 0xF;
  return c;
}

uint16_t RGBmatrixPanel::Color888(uint8_t r, uint8_t g, uint8_t b) {
  uint16_t c;
  
  c = (r >> 5);
  c <<= 4;
  c |= (g >> 5) & 0xF;
  c <<= 4;
  c |= (b >> 5) & 0xF;

  /*
  Serial.print(r, HEX); Serial.print(", ");
  Serial.print(g, HEX); Serial.print(", ");
  Serial.print(b, HEX); Serial.print("->");
  Serial.println(c, HEX);
  */

  return c;
}


void  RGBmatrixPanel::setPixel(uint8_t x, uint8_t y, uint16_t c) {
  uint16_t index;
  uint8_t old;
  uint8_t red, green, blue;
  
  // extract the 12 bits of color
  red = (c >> 8) & 0xF;
  green = (c >> 4) & 0xF;
  blue = c & 0xF;
  
  if (y < 8)
    index = y;
  else 
    index = y-8;
  index *= 32;
  index += x;
  // then multiply by 3 bytes per color (12 bit * High and Low = 24 bit = 3 byte)
  index *= 3;
  //Serial.print("$("); Serial.print(x, DEC); Serial.print(", "); Serial.print(y, DEC); Serial.print(") -> "); Serial.println(index, DEC);

  old = matrixbuff[index];
  //Serial.print(old, HEX); Serial.print(" -> ");
  if (y < 8) {
    // we're going to replace the high nybbles only
    // red first!
    matrixbuff[index] &= ~0xF0;  // mask off top 4 bits
    matrixbuff[index] |= (red << 4);
    index++;
    // then green
    matrixbuff[index] &= ~0xF0;  // mask off top 4 bits
    matrixbuff[index] |= (green << 4);
    index++;
    // finally blue
    matrixbuff[index] &= ~0xF0;  // mask off top 4 bits
    matrixbuff[index] |= (blue << 4);
  } else {
    // we're going to replace the low nybbles only
    // red first!
    matrixbuff[index] &= ~0x0F;  // mask off bottom 4 bits
    matrixbuff[index] |= red;
    index++;
    // then green
    matrixbuff[index] &= ~0x0F;  // mask off bottom 4 bits
    matrixbuff[index] |= green;
    index++;
    // finally blue
    matrixbuff[index] &= ~0x0F;  // mask off bottom 4 bits
    matrixbuff[index] |= blue;
  }
  
  // Serial.println( matrixbuff[index], HEX);
}


void  RGBmatrixPanel::updateDisplay(void) {
  writeSection(scansection, matrixbuff + (3*32*scansection));  
  scansection++;
  if (scansection == 8) { 
    scansection = 0;
    pwmcounter++;
    if (pwmcounter == PWMMAX) { pwmcounter = 0; }
  }
}


void RGBmatrixPanel::dumpMatrix(void) {
  uint8_t i=0;
  
  do {
    Serial.print("0x");
    if (matrixbuff[i] < 0xF)  Serial.print('0');
    Serial.print(matrixbuff[i], HEX);
    Serial.print(" ");
    i++;
    if (! (i %32) ) Serial.println();
  } while (i != 0);
  
    
}



void RGBmatrixPanel::writeSection(uint8_t secn, uint8_t *buffptr) {
  //  digitalWrite(_oe, HIGH);
  *oeportreg |= oepin;

  if (secn & 0x1) {
    //digitalWrite(_a, HIGH);
    *addraportreg |= addrapin;
  } else {
    //digitalWrite(_a, LOW);
    *addraportreg &= ~addrapin;
  }
  if (secn & 0x2) {
    //digitalWrite(_b, HIGH);
    *addrbportreg |= addrbpin;
  } else {
    //digitalWrite(_b, LOW);
    *addrbportreg &= ~addrbpin;
  }
  if (secn & 0x4) {
    //digitalWrite(_c, HIGH);
    *addrcportreg |= addrcpin;
  } else {
    //digitalWrite(_c, LOW);
    *addrcportreg &= ~addrcpin;
  } 
  
  //Serial.print ("\nsection #"); Serial.println(secn, DEC);
  // one section is 192 LEDs. 192 / 6 = 32 'bytes'

  uint8_t out, low, high;
  
  for (uint8_t i=0; i<32; i++) {

    out = 0;
    // red
   low = *buffptr++;
   high = low >> 4;
   low &= 0x0F;
   if (low > pwmcounter) out |= 0x20;
   if (high > pwmcounter) out |= 0x04;

   // green
   low = *buffptr++;
   high = low >> 4;
   low &= 0x0F;
   if (low > pwmcounter) out |= 0x40;
   if (high > pwmcounter) out |= 0x08;

   // blue
   low = *buffptr++;
   high = low >> 4;
   low &= 0x0F;
   if (low > pwmcounter) out |= 0x80;
   if (high > pwmcounter) out |= 0x10;

    
   //digitalWrite(CLK, LOW);
   //*sclkportreg &= ~sclkpin;
   RGBMATRIX_CLOCKPORT &= ~_BV(RGBMATRIX_CLOCKPIN);

   RGBMATRIX_DATAPORT = out;
   //Serial.print(*buffptr, HEX); Serial.print(", ");
   //digitalWrite(CLK, HIGH);
   //*sclkportreg |= sclkpin;
   RGBMATRIX_CLOCKPORT |= _BV(RGBMATRIX_CLOCKPIN);
  } 

  // latch it!
  /*
    digitalWrite(_latch, HIGH);
    digitalWrite(_latch, LOW);  
    digitalWrite(_oe, LOW);
  */

  *latportreg |= latpin;
  *latportreg &= ~latpin;
  *oeportreg &= ~oepin;
}
