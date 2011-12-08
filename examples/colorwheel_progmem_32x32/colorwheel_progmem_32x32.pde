// colorwheel_progmem demo for RGBmatrixPanel library.
// Renders a nice circle of hues on a 32x32 RGB LED matrix.
// Uses precomputed image data stored in PROGMEM rather than
// calculating each pixel.  Nearly instantaneous!  Woo!

#include "RGBmatrixPanel.h"
#include "image.h"

#define A   A0
#define B   A1
#define C   A2
#define D   A4 // Skip A3 for compatibility with 16x32 wiring
#define CLK 8  // MUST be on PORTB!
#define LAT A3
#define OE  9
RGBmatrixPanel matrix(A, B, C, D, CLK, LAT, OE, false);

void setup() {
  int     i, len;
  uint8_t *ptr = matrix.backBuffer(); // Get address of matrix data

  // Copy each byte from PROGMEM to matrix buffer:
  len = sizeof(img);
  for(i=0; i<len; i++) ptr[i] = pgm_read_byte(&img[i]);

  // Start up matrix AFTER data is copied.  The RGBmatrixPanel
  // interrupt code ties up about 40% of the CPU time, so starting
  // it now allows the prior drawing code to run even faster!
  matrix.begin();
}

void loop() {
  // do nothing
}
