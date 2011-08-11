#include <WProgram.h>
#include <util/delay.h>
#include "pins_arduino.h"
#include "wiring_private.h"

#define A A0
#define B A1
#define C A2

#define OE 9
#define CLK 8
#define LAT A3

volatile uint8_t *sclkportreg;
uint8_t sclkpin;
      
#define WIDTH 32
#define HEIGHT 16
#define NUMBYTES (WIDTH * HEIGHT / 2)
uint8_t matrixbuff[NUMBYTES];

#define BLACK 0x0
#define WHITE 0x7
#define RED 0x1
#define YELLOW 0x3
#define GREEN 0x2
#define TEAL 0x6
#define BLUE 0x4
#define VIOLET 0x5

volatile uint8_t halted = 0;
// This is the Timer 1 CTC interrupt, it goes off once a second
volatile uint8_t scansection = 0;

void setup() {
 Serial.begin(115200);
sclkportreg =  portOutputRegister(digitalPinToPort(CLK));
sclkpin = digitalPinToBitMask(CLK);
 pinMode(A, OUTPUT);
 digitalWrite(A, LOW); 
 pinMode(B, OUTPUT);
 digitalWrite(B, LOW); 
 pinMode(C, OUTPUT);
 digitalWrite(C, LOW); 
 pinMode(LAT, OUTPUT);
 digitalWrite(LAT, LOW); 
 pinMode(CLK, OUTPUT);
 digitalWrite(CLK, LOW); 
 pinMode(OE, OUTPUT);
 digitalWrite(OE, HIGH); 
 
// 6 bits out, low
  DDRD = 0xFC;
  PORTD = 0x0;
  
  for (uint16_t i=0; i<NUMBYTES; i++) {
    //matrixbuff[i] = 0x80;
    // 0x80 = top half blue
    // 0x40 = top half green
    // 0x20 = top half red
    // 0x10 = bottom half blue
    // 0x08 = bottom half green
    // 0x04 = bottom half red
  }
  for (uint16_t i=0; i<32; i++) {
    //matrixbuff[i] = 0x48;
  }
    

// 
 //matrixbuff[0] = 0xFF; 
 //matrixbuff[100] = 0xFF; 
 //matrixbuff[200] = 0xFF; 

  dumpMatrix();
 
  //Setup 1 KHz timer to refresh display using 16 Timer 1
    TCNT1 = 0;
  TCCR1A = 0;                           // CTC mode (interrupt after timer reaches OCR1A)
  TCCR1B = _BV(WGM12) | _BV(CS10) | _BV(CS12);    // CTC & clock div 1024
  OCR1A = 4;                                 
  TIMSK1 = _BV(OCIE1A);  // turn on interrupt   

}

void dumpMatrix() {
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

uint8_t color = BLACK;
void loop() {
  for (uint8_t y=0; y < 16; y++) {  
  for (uint8_t x=0; x < 32; x++) {  
    setPixel(x,y,color); 
    delay(10);
  }
  }
 color++; 
}


void setPixel(uint8_t x, uint8_t y, uint8_t c) {
  uint16_t index;
  uint8_t old;

  if (y < 8)
    index = y;
  else 
    index = y-8;
  index *= 32;
  index += x;
  Serial.print("$("); Serial.print(x, DEC); Serial.print(", "); Serial.print(y, DEC); Serial.print(") -> "); Serial.println(index, DEC);

  old = matrixbuff[index];
    Serial.print(old, HEX); Serial.print(" -> ");
  if (y >= 8) {
    old &= ~0xE0;  // mask off top 3 bits
    old |= ((c&0x7) << 5);
  } else {
    old &= ~0x1C;  // mask off bottom 3 bits
    old |= ((c&0x7) << 2);
  }
  
  matrixbuff[index] = old;
   Serial.println(old, HEX);
}

SIGNAL(TIMER1_COMPA_vect) { 
  latchit();
  
}


void latchit(void) {
  writesection(scansection, matrixbuff + (32*scansection));  
  scansection++;
  if (scansection == 8) 
    scansection = 0;
}
    
void writesection(uint8_t secn, uint8_t *buffptr) {
  digitalWrite(OE, HIGH);

  if (secn & 0x1) {
    digitalWrite(A, HIGH);
  } else {
    digitalWrite(A, LOW);
  }
  if (secn & 0x2) {
    digitalWrite(B, HIGH);
  } else {
    digitalWrite(B, LOW);
  }
  if (secn & 0x4) {
    digitalWrite(C, HIGH);
  } else {
    digitalWrite(C, LOW);
  } 
  
  //Serial.print ("\nsection #"); Serial.println(secn, DEC);
  // one section is 192 LEDs. 192 / 6 = 32 'bytes'
  for (uint8_t i=0; i<32; i++) {
   PORTD = *buffptr;
   //Serial.print(*buffptr, HEX); Serial.print(", ");
   //digitalWrite(CLK, HIGH);
   *sclkportreg |= sclkpin;
   buffptr++;

   //digitalWrite(CLK, LOW);
  *sclkportreg &= ~sclkpin;
  } 

  // latch it!
   
  digitalWrite(LAT, HIGH);
  digitalWrite(LAT, LOW);
  

  digitalWrite(OE, LOW);

  //   digitalWrite(LAT, LOW);
}


