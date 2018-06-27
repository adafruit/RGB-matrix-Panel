// colorwheel_progmem demo for Adafruit RGBmatrixPanel library.
// Renders a nice circle of hues on our 32x32 RGB LED matrix:
// http://www.adafruit.com/products/607

// This version uses precomputed image data stored in PROGMEM
// rather than calculating each pixel.  Nearly instantaneous!  Woo!

// Written by Limor Fried/Ladyada & Phil Burgess/PaintYourDragon
// for Adafruit Industries.
// BSD license, all text above must be included in any redistribution.

#include <RGBmatrixPanel.h>
#include "image.h" // Precomputed colorwheel image is here

#define CLK  8   // USE THIS ON ARDUINO UNO, ADAFRUIT METRO M0, etc.
//#define CLK A4 // USE THIS ON METRO M4 (not M0)
//#define CLK 11 // USE THIS ON ARDUINO MEGA
#define OE   9
#define LAT 10
#define A   A0
#define B   A1
#define C   A2
#define D   A3

RGBmatrixPanel matrix(A, B, C, D, CLK, LAT, OE, false);

void setup() {
  int     i, len;
  uint8_t *ptr = matrix.backBuffer(); // Get address of matrix data

  // Copy image from PROGMEM to matrix buffer:
  memcpy_P(ptr, img, sizeof(img));

  // Start up matrix AFTER data is copied.  The RGBmatrixPanel
  // interrupt code ties up about 40% of the CPU time, so starting
  // it now allows the prior drawing code to run even faster!
  matrix.begin();
}

void loop() {
  // Do nothing -- image doesn't change
}
