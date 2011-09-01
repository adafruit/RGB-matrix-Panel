// example to draw all 9-bit colors! public domain

#include "RGBmatrixPanel.h"
#include <TimerOne.h>

#define A     A0
#define B     A1
#define C     A2
#define OE    A3
#define LAT   9

// The clock pin must be digital 8
// The data pins must connect to digital 2-7

RGBmatrixPanel matrix(A, B, C, LAT, OE);

// wrapper for the redrawing code, this gets called by the interrupt
void refresh() { 
  matrix.updateDisplay();
}

// m ranges from 0 to 100%
void setCPUmaxpercent(uint8_t m) {
  float time = 100;        // 100 %

  time *=  150;           // each redraw takes 150 microseconds
  time /= m;              // how long between interrupts
  Timer1.initialize(time);  // microseconds per tick
  Timer1.attachInterrupt(refresh);
}

void setup() {
  Serial.begin(9600);

  matrix.begin();

  // initialize the timer that refreshes the delay using our helper
  // 50% seems to be the minimum for 9 bit color, higher will make the display
  // look better
  setCPUmaxpercent(75);
  
  uint8_t r=0, g=0, b=0;
  for (uint8_t x=0; x < 32; x++) {      
    for (uint8_t y=0; y < 8; y++) {  
      matrix.drawPixel(x, y, matrix.Color333(r, g, b));
      r++;
      if (r == 8) {
        r = 0; g++;
      }
      if (g == 8) {
        g = 0; b++;
      }
    }
  }

  for (uint8_t x=0; x < 32; x++) {      
    for (uint8_t y=8; y < 16; y++) {  
      matrix.drawPixel(x, y, matrix.Color333(r, g, b));
      r++;
      if (r == 8) {
        r = 0; g++;
      }
      if (g == 8) {
        g = 0; b++;
      }
    }
  }
}

void loop() {
  // do nothing
}

