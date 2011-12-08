// scrolltext demo for RGBmatrixPanel library.
// Demonstrates double-buffered animation on 16x32 RGB LED matrix.

#include "RGBmatrixPanel.h"

#define A   A0
#define B   A1
#define C   A2
#define CLK 8  // MUST be on PORTB!
#define LAT A3
#define OE  9
// Last parameter = 'true' enables double-buffering, for flicker-free,
// buttery smooth animation.  Note that NOTHING WILL SHOW ON THE DISPLAY
// until the first call to swapBuffers().  This is normal.
RGBmatrixPanel matrix(A, B, C, CLK, LAT, OE, true);

char     str[]   = "Adafruit 16x32 RGB LED Matrix";
int      textX   = matrix.width(),
         textMin = sizeof(str) * -12;
long     hue     = 0;
int      ball[3][4] = {
   3,  0,  1,  1, // Initial X,Y pos & velocity for 3 bouncy balls
  17, 15,  1, -1,
  27,  4, -1,  1
};
uint16_t ballcolor[3] = {
  matrix.Color444(0,1,0),
  matrix.Color444(0,0,1),
  matrix.Color444(1,0,0)
};

void setup() {
  matrix.begin();
}

void loop() {
  byte i;

  // Clear background
  matrix.fill(0);

  // Bounce three balls around
  for(i=0; i<3; i++) {
    // Draw 'ball'
    matrix.fillCircle(ball[i][0], ball[i][1], 5, ballcolor[i]);
    // Update X, Y position
    ball[i][0] += ball[i][2];
    ball[i][1] += ball[i][3];
    // Bounce off edges
    if((ball[i][0] == 0) || (ball[i][0] == (matrix.width() - 1)))
      ball[i][2] *= -1;
    if((ball[i][1] == 0) || (ball[i][1] == (matrix.height() - 1)))
      ball[i][3] *= -1;
  }

  // Draw big scrolly text on top
  matrix.setTextSize(2);
  matrix.setCursor(textX, 1);
  matrix.setTextColor(matrix.ColorHSV(hue, 255, 255, true));
  matrix.print(str);
  // Move text left (w/wrap), increase hue
  if((--textX) < textMin) textX = matrix.width();
  hue += 7;

  // Update display
  matrix.swapBuffers(false);
}
