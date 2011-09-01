// demo for all the abilities of the RGBmatrixPanel library. public domain!

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
  
  // draw a pixel in solid white
  matrix.drawPixel(0, 0, matrix.Color333(7, 7, 7)); 
  delay(500);

  // fix the screen with green
  matrix.fillRect(0, 0, 32, 16, matrix.Color333(0, 7, 0));
  delay(500);

  // draw a box in yellow
  matrix.drawRect(0, 0, 32, 16, matrix.Color333(7, 7, 0));
  delay(500);
  
  // draw an 'X' in red
  matrix.drawLine(0, 0, 31, 15, matrix.Color333(7, 0, 0));
  matrix.drawLine(31, 0, 0, 15, matrix.Color333(7, 0, 0));
  delay(500);
  
  // draw a blue circle
  matrix.drawCircle(7, 7, 7, matrix.Color333(0, 0, 7));
  delay(500);
  
  // fill a violet circle
  matrix.fillCircle(23, 7, 7, matrix.Color333(7, 0, 7));
  delay(500);
  
  // fill the screen with 'black'
  matrix.fill(matrix.Color333(0, 0, 0));
  
  // draw some text!
  matrix.setCursor(1, 0);   // start at top left, with one pixel of spacing
  matrix.setTextSize(1);    // size 1 == 8 pixels high
  
  // print each letter with a rainbow color
  matrix.setTextColor(matrix.Color333(7,0,0));
  matrix.print('1');
  matrix.setTextColor(matrix.Color333(7,4,0)); 
  matrix.print('6');
  matrix.setTextColor(matrix.Color333(7,7,0));
  matrix.print('x');
  matrix.setTextColor(matrix.Color333(4,7,0)); 
  matrix.print('3');
  matrix.setTextColor(matrix.Color333(0,7,0));  
  matrix.print('2');
  
  matrix.setCursor(1, 9);   // next line
  matrix.setTextColor(matrix.Color333(0,7,7)); 
  matrix.print('*');
  matrix.setTextColor(matrix.Color333(0,4,7)); 
  matrix.print('R');
  matrix.setTextColor(matrix.Color333(0,0,7));
  matrix.print('G');
  matrix.setTextColor(matrix.Color333(4,0,7)); 
  matrix.print("B");
  matrix.setTextColor(matrix.Color333(7,0,4)); 
  matrix.print("*");

  // whew!
}

void loop() {
  // do nothing
}

