/*
RGBmatrixPanel Arduino library for Adafruit 16x32 and 32x32 RGB LED
matrix panels.  Pick one up at:
  http://www.adafruit.com/products/420
  http://www.adafruit.com/products/607

This version uses a few tricks to achieve better performance and/or
lower CPU utilization:

- To control LED brightness, traditional PWM is eschewed in favor of
  Binary Code Modulation, which operates through a succession of periods
  each twice the length of the preceeding one (rather than a direct
  linear count a la PWM).  It's explained well here:

    http://www.batsocks.co.uk/readme/art_bcm_1.htm

  I was initially skeptical, but it works exceedingly well in practice!
  And this uses considerably fewer CPU cycles than software PWM.

- Although many control pins are software-configurable in the user's
  code, a couple things are tied to specific PORT registers.  It's just
  a lot faster this way -- port lookups take time.  Please see the notes
  later regarding wiring on "alternative" Arduino boards.

- A tiny bit of inline assembly language is used in the most speed-
  critical section.  The C++ compiler wasn't making optimal use of the
  instruction set in what seemed like an obvious chunk of code.  Since
  it's only a few short instructions, this loop is also "unrolled" --
  each iteration is stated explicitly, not through a control loop.

Written by Limor Fried/Ladyada & Phil Burgess/PaintYourDragon for
Adafruit Industries.
BSD license, all text above must be included in any redistribution.

    ****************

Support for Feather M0 contributed by ee-quipment.com

To support the Feather M0, direct writes to AVR ports have been replaced with
direct writes to Zero ports that work on the Feather M0. The pinouts are fixed
and do not conflict with the SPI or I2C ports. The LED_BUILTIN port is used however.

Panel Pins:      OE   LAT        B2    G2    R2    B1    G1    R1       CLK         D         C     B         A
Connector Pins:  15    14         7     6     5     3     2     1        13        12        11    10         9
Feather Pins:     7     6        20    24    22    25    23    19        14        21         9     8         5
Port Pins:      PB09  PB08  ..  PA20  PA19  PA18  PA17  PA16  PA15  ..  PA11  ..  PA07  ..  PA05  PA04  ..  PA02
Arduino IDE:     16    15         6    12    10    13    11     5         0         9        18    17        14

Schematics and a board layout are available at https://github.com/ee-quipment/RGB-matrix-Panel-Zero
A bare board can be ordered from OshPark at https://oshpark.com/shared_projects/1QNzmjwa

To set up interrupts a third-party library is used:
#include  <avdweb_SAMDtimer.h>  //http://www.avdweb.nl/arduino/libraries/samd21-timer.html

*/

#include "RGBmatrixPanel.h"

static const uint8_t gamma_lut[] = {
  0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
  0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
  0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
  0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
  0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
  0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
  0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
  0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
  0x00,0x00,0x01,0x01,0x01,0x01,0x01,0x01,
  0x01,0x01,0x01,0x01,0x01,0x01,0x01,0x01,
  0x01,0x01,0x01,0x01,0x01,0x01,0x01,0x01,
  0x01,0x01,0x01,0x01,0x01,0x01,0x01,0x01,
  0x01,0x01,0x01,0x01,0x01,0x01,0x02,0x02,
  0x02,0x02,0x02,0x02,0x02,0x02,0x02,0x02,
  0x02,0x02,0x02,0x02,0x02,0x02,0x02,0x02,
  0x02,0x02,0x02,0x02,0x02,0x03,0x03,0x03,
  0x03,0x03,0x03,0x03,0x03,0x03,0x03,0x03,
  0x03,0x03,0x03,0x03,0x03,0x03,0x03,0x04,
  0x04,0x04,0x04,0x04,0x04,0x04,0x04,0x04,
  0x04,0x04,0x04,0x04,0x04,0x04,0x05,0x05,
  0x05,0x05,0x05,0x05,0x05,0x05,0x05,0x05,
  0x05,0x05,0x05,0x06,0x06,0x06,0x06,0x06,
  0x06,0x06,0x06,0x06,0x06,0x06,0x06,0x07,
  0x07,0x07,0x07,0x07,0x07,0x07,0x07,0x07,
  0x07,0x07,0x08,0x08,0x08,0x08,0x08,0x08,
  0x08,0x08,0x08,0x08,0x09,0x09,0x09,0x09,
  0x09,0x09,0x09,0x09,0x09,0x0a,0x0a,0x0a,
  0x0a,0x0a,0x0a,0x0a,0x0a,0x0a,0x0b,0x0b,
  0x0b,0x0b,0x0b,0x0b,0x0b,0x0b,0x0c,0x0c,
  0x0c,0x0c,0x0c,0x0c,0x0c,0x0c,0x0d,0x0d,
  0x0d,0x0d,0x0d,0x0d,0x0d,0x0e,0x0e,0x0e,
  0x0e,0x0e,0x0e,0x0e,0x0f,0x0f,0x0f,0x0f
};

#ifndef _swap_int16_t
#define _swap_int16_t(a, b) { int16_t t = a; a = b; b = t; }
#endif

// A full PORT register is required for the data lines, though only the
// top 6 output bits are used.  For performance reasons, the port # cannot
// be changed via library calls, only by changing constants in the library.
// For similar reasons, the clock pin is only semi-configurable...it can
// be specified as any pin within a specific PORT register stated below.

#if defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__)
 // Arduino Mega is now tested and confirmed, with the following caveats:
 // Because digital pins 2-7 don't map to a contiguous port register,
 // the Mega requires connecting the matrix data lines to different pins.
 // Digital pins 24-29 are used for the data interface, and 22 & 23 are
 // unavailable for other outputs because the software needs to write to
 // the full PORTA register for speed.  Clock may be any pin on PORTB --
 // on the Mega, this CAN'T be pins 8 or 9 (these are on PORTH), thus the
 // wiring will need to be slightly different than the tutorial's
 // explanation on the Uno, etc.  Pins 10-13 are all fair game for the
 // clock, as are pins 50-53.
 #define DATAPORT PORTA
 #define DATADIR  DDRA
 #define SCLKPORT PORTB
#elif defined(__AVR_ATmega32U4__)
 // Arduino Leonardo: this is vestigial code an unlikely to ever be
 // finished -- DO NOT USE!!!  Unlike the Uno, digital pins 2-7 do NOT
 // map to a contiguous port register, dashing our hopes for compatible
 // wiring.  Making this work would require significant changes both to
 // the bit-shifting code in the library, and how this board is wired to
 // the LED matrix.  Bummer.
 #define DATAPORT PORTD
 #define DATADIR  DDRD
 #define SCLKPORT PORTB
#elif defined(ARDUINO_SAMD_FEATHER_M0)
 // Arduino IDE pin mapping -- hard coded, cannot be changed by user
 #define ZERO_CLK    0
 #define ZERO_LAT   15
 #define ZERO_OE    16
 #define ZERO_A     14
 #define ZERO_B     17
 #define ZERO_C     18
 #define ZERO_D      9
 #define ZERO_R1     5
 #define ZERO_G1    11
 #define ZERO_B1    13
 #define ZERO_R2    10
 #define ZERO_G2    12
 #define ZERO_B2     6
 //Row refresh timer library and interrupt forward declaration
 #include  <avdweb_SAMDtimer.h>  //http://www.avdweb.nl/arduino/libraries/samd21-timer.html
 void ISR_LEDPanelRefresh(struct tc_module *const module_inst);
 #define LED_PANEL_REFRESH_RATE    100   // Hz, up to a maximum of TBD


#else
 // Ports for "standard" boards (Arduino Uno, Duemilanove, etc.)
 #define DATAPORT PORTD
 #define DATADIR  DDRD
 #define SCLKPORT PORTB
#endif

#define nPlanes 4

// The fact that the display driver interrupt stuff is tied to the
// singular Timer1 doesn't really take well to object orientation with
// multiple RGBmatrixPanel instances.  The solution at present is to
// allow instances, but only one is active at any given time, via its
// begin() method.  The implementation is still incomplete in parts;
// the prior active panel really should be gracefully disabled, and a
// stop() method should perhaps be added...assuming multiple instances
// are even an actual need.
static RGBmatrixPanel *activePanel = NULL;

// Code common to both the 16x32 and 32x32 constructors:
void RGBmatrixPanel::init(uint8_t rows, uint8_t a, uint8_t b, uint8_t c,
  uint8_t sclk, uint8_t latch, uint8_t oe, boolean dbuf, uint8_t width) {

  nRows = rows; // Number of multiplexed rows; actual height is 2X this

  // Allocate and initialize matrix buffer:
  int buffsize  = width * nRows * 3, // x3 = 3 bytes holds 4 planes "packed"
      allocsize = (dbuf == true) ? (buffsize * 2) : buffsize;
  if(NULL == (matrixbuff[0] = (uint8_t *)malloc(allocsize))) return;
  memset(matrixbuff[0], 0, allocsize);
  // If not double-buffered, both buffers then point to the same address:
  matrixbuff[1] = (dbuf == true) ? &matrixbuff[0][buffsize] : matrixbuff[0];

  // Save pin numbers for use by begin() method later.
  _a     = a;
  _b     = b;
  _c     = c;
  _sclk  = sclk;
  _latch = latch;
  _oe    = oe;

#if !defined(ARDUINO_SAMD_FEATHER_M0)
  // Look up port registers and pin masks ahead of time,
  // avoids many slow digitalWrite() calls later.
  sclkpin   = digitalPinToBitMask(sclk);
  latport   = portOutputRegister(digitalPinToPort(latch));
  latpin    = digitalPinToBitMask(latch);
  oeport    = portOutputRegister(digitalPinToPort(oe));
  oepin     = digitalPinToBitMask(oe);
  addraport = portOutputRegister(digitalPinToPort(a));
  addrapin  = digitalPinToBitMask(a);
  addrbport = portOutputRegister(digitalPinToPort(b));
  addrbpin  = digitalPinToBitMask(b);
  addrcport = portOutputRegister(digitalPinToPort(c));
  addrcpin  = digitalPinToBitMask(c);
#endif
  plane     = nPlanes - 1;
  row       = nRows   - 1;
  swapflag  = false;
  backindex = 0;     // Array index of back buffer
}

// Constructor for 16x32 panel:
RGBmatrixPanel::RGBmatrixPanel(
  uint8_t a, uint8_t b, uint8_t c,
  uint8_t sclk, uint8_t latch, uint8_t oe, boolean dbuf) :
  Adafruit_GFX(32, 16) {

  init(8, a, b, c, sclk, latch, oe, dbuf, 32);
}

// Constructor for 32x32 or 32x64 panel:
RGBmatrixPanel::RGBmatrixPanel(
  uint8_t a, uint8_t b, uint8_t c, uint8_t d,
  uint8_t sclk, uint8_t latch, uint8_t oe, boolean dbuf, uint8_t width) :
  Adafruit_GFX(width, 32) {

  init(16, a, b, c, sclk, latch, oe, dbuf, width);

  // Init a few extra 32x32-specific elements:
  _d        = d;
#if !defined(ARDUINO_SAMD_FEATHER_M0)
  addrdport = portOutputRegister(digitalPinToPort(d));
  addrdpin  = digitalPinToBitMask(d);
#endif
}

void RGBmatrixPanel::begin(void) {

  backindex   = 0;                         // Back buffer
  buffptr     = matrixbuff[1 - backindex]; // -> front buffer
  activePanel = this;                      // For interrupt hander

#if defined(ARDUINO_SAMD_FEATHER_M0)
  pinMode(ZERO_CLK,  OUTPUT);
  pinMode(ZERO_LAT,  OUTPUT);
  pinMode(ZERO_OE,   OUTPUT);
  pinMode(ZERO_A,    OUTPUT);
  pinMode(ZERO_B,    OUTPUT);
  pinMode(ZERO_C,    OUTPUT);
  if(nRows > 8) { pinMode(ZERO_D,    OUTPUT); }
  pinMode(ZERO_R1,   OUTPUT);
  pinMode(ZERO_G1,   OUTPUT);
  pinMode(ZERO_B1,   OUTPUT);
  pinMode(ZERO_R2,   OUTPUT);
  pinMode(ZERO_G2,   OUTPUT);
  pinMode(ZERO_B2,   OUTPUT);

  unsigned interrupt_interval_us = 1000000 / (16 * nRows * LED_PANEL_REFRESH_RATE); // 16 is color depth
  static SAMDtimer timer(4, ISR_LEDPanelRefresh, interrupt_interval_us);

#else
  // Enable all comm & address pins as outputs, set default states:
  pinMode(_sclk , OUTPUT); SCLKPORT   &= ~sclkpin;  // Low
  pinMode(_latch, OUTPUT); *latport   &= ~latpin;   // Low
  pinMode(_oe   , OUTPUT); *oeport    |= oepin;     // High (disable output)
  pinMode(_a    , OUTPUT); *addraport &= ~addrapin; // Low
  pinMode(_b    , OUTPUT); *addrbport &= ~addrbpin; // Low
  pinMode(_c    , OUTPUT); *addrcport &= ~addrcpin; // Low
  if(nRows > 8) {
    pinMode(_d  , OUTPUT); *addrdport &= ~addrdpin; // Low
  }

  // The high six bits of the data port are set as outputs;
  // Might make this configurable in the future, but not yet.
  DATADIR  = B11111100;
  DATAPORT = 0;

  // Set up Timer1 for interrupt:
  TCCR1A  = _BV(WGM11); // Mode 14 (fast PWM), OC1A off
  TCCR1B  = _BV(WGM13) | _BV(WGM12) | _BV(CS10); // Mode 14, no prescale
  ICR1    = 100;
  TIMSK1 |= _BV(TOIE1); // Enable Timer1 interrupt
  sei();                // Enable global interrupts
#endif
}

// Original RGBmatrixPanel library used 3/3/3 color.  Later version used
// 4/4/4.  Then Adafruit_GFX (core library used across all Adafruit
// display devices now) standardized on 5/6/5.  The matrix still operates
// internally on 4/4/4 color, but all the graphics functions are written
// to expect 5/6/5...the matrix lib will truncate the color components as
// needed when drawing.  These next functions are mostly here for the
// benefit of older code using one of the original color formats.

// Promote 3/3/3 RGB to Adafruit_GFX 5/6/5
uint16_t RGBmatrixPanel::Color333(uint8_t r, uint8_t g, uint8_t b) {
  // RRRrrGGGgggBBBbb
  return ((r & 0x7) << 13) | ((r & 0x6) << 10) |
         ((g & 0x7) <<  8) | ((g & 0x7) <<  5) |
         ((b & 0x7) <<  2) | ((b & 0x6) >>  1);
}

// Promote 4/4/4 RGB to Adafruit_GFX 5/6/5
uint16_t RGBmatrixPanel::Color444(uint8_t r, uint8_t g, uint8_t b) {
  // RRRRrGGGGggBBBBb
  return ((r & 0xF) << 12) | ((r & 0x8) << 8) |
         ((g & 0xF) <<  7) | ((g & 0xC) << 3) |
         ((b & 0xF) <<  1) | ((b & 0x8) >> 3);
}

// Demote 8/8/8 to Adafruit_GFX 5/6/5
// If no gamma flag passed, assume linear color
uint16_t RGBmatrixPanel::Color888(uint8_t r, uint8_t g, uint8_t b) {
  return ((uint16_t)(r & 0xF8) << 8) | ((uint16_t)(g & 0xFC) << 3) | (b >> 3);
}

// 8/8/8 -> gamma -> 5/6/5
uint16_t RGBmatrixPanel::Color888(
  uint8_t r, uint8_t g, uint8_t b, boolean gflag) {
  if(gflag) { // Gamma-corrected color?
    r = pgm_read_byte(&gamma_lut[r]); // Gamma correction table maps
    g = pgm_read_byte(&gamma_lut[g]); // 8-bit input to 4-bit output
    b = pgm_read_byte(&gamma_lut[b]);
    return ((uint16_t)r << 12) | ((uint16_t)(r & 0x8) << 8) | // 4/4/4->5/6/5
           ((uint16_t)g <<  7) | ((uint16_t)(g & 0xC) << 3) |
           (          b <<  1) | (           b        >> 3);
  } // else linear (uncorrected) color
  return ((uint16_t)(r & 0xF8) << 8) | ((uint16_t)(g & 0xFC) << 3) | (b >> 3);
}

uint16_t RGBmatrixPanel::ColorHSV(
  long hue, uint8_t sat, uint8_t val, boolean gflag) {

  uint8_t  r, g, b, lo;
  uint16_t s1, v1;

  // Hue
  hue %= 1536;             // -1535 to +1535
  if(hue < 0) hue += 1536; //     0 to +1535
  lo = hue & 255;          // Low byte  = primary/secondary color mix
  switch(hue >> 8) {       // High byte = sextant of colorwheel
    case 0 : r = 255     ; g =  lo     ; b =   0     ; break; // R to Y
    case 1 : r = 255 - lo; g = 255     ; b =   0     ; break; // Y to G
    case 2 : r =   0     ; g = 255     ; b =  lo     ; break; // G to C
    case 3 : r =   0     ; g = 255 - lo; b = 255     ; break; // C to B
    case 4 : r =  lo     ; g =   0     ; b = 255     ; break; // B to M
    default: r = 255     ; g =   0     ; b = 255 - lo; break; // M to R
  }

  // Saturation: add 1 so range is 1 to 256, allowig a quick shift operation
  // on the result rather than a costly divide, while the type upgrade to int
  // avoids repeated type conversions in both directions.
  s1 = sat + 1;
  r  = 255 - (((255 - r) * s1) >> 8);
  g  = 255 - (((255 - g) * s1) >> 8);
  b  = 255 - (((255 - b) * s1) >> 8);

  // Value (brightness) & 16-bit color reduction: similar to above, add 1
  // to allow shifts, and upgrade to int makes other conversions implicit.
  v1 = val + 1;
  if(gflag) { // Gamma-corrected color?
    r = pgm_read_byte(&gamma_lut[(r * v1) >> 8]); // Gamma correction table maps
    g = pgm_read_byte(&gamma_lut[(g * v1) >> 8]); // 8-bit input to 4-bit output
    b = pgm_read_byte(&gamma_lut[(b * v1) >> 8]);
  } else { // linear (uncorrected) color
    r = (r * v1) >> 12; // 4-bit results
    g = (g * v1) >> 12;
    b = (b * v1) >> 12;
  }
  return (r << 12) | ((r & 0x8) << 8) | // 4/4/4 -> 5/6/5
         (g <<  7) | ((g & 0xC) << 3) |
         (b <<  1) | ( b        >> 3);
}

void RGBmatrixPanel::drawPixel(int16_t x, int16_t y, uint16_t c) {
  uint8_t r, g, b, bit, limit, *ptr;

  if((x < 0) || (x >= _width) || (y < 0) || (y >= _height)) return;

  switch(rotation) {
   case 1:
    _swap_int16_t(x, y);
    x = WIDTH  - 1 - x;
    break;
   case 2:
    x = WIDTH  - 1 - x;
    y = HEIGHT - 1 - y;
    break;
   case 3:
    _swap_int16_t(x, y);
    y = HEIGHT - 1 - y;
    break;
  }

  // Adafruit_GFX uses 16-bit color in 5/6/5 format, while matrix needs
  // 4/4/4.  Pluck out relevant bits while separating into R,G,B:
  r =  c >> 12;        // RRRRrggggggbbbbb
  g = (c >>  7) & 0xF; // rrrrrGGGGggbbbbb
  b = (c >>  1) & 0xF; // rrrrrggggggBBBBb

  // Loop counter stuff
  bit   = 2;
  limit = 1 << nPlanes;

  if(y < nRows) {
    // Data for the upper half of the display is stored in the lower
    // bits of each byte.
    ptr = &matrixbuff[backindex][y * WIDTH * (nPlanes - 1) + x]; // Base addr
    // Plane 0 is a tricky case -- its data is spread about,
    // stored in least two bits not used by the other planes.
    ptr[WIDTH*2] &= ~B00000011;           // Plane 0 R,G mask out in one op
    if(r & 1) ptr[WIDTH*2] |=  B00000001; // Plane 0 R: 64 bytes ahead, bit 0
    if(g & 1) ptr[WIDTH*2] |=  B00000010; // Plane 0 G: 64 bytes ahead, bit 1
    if(b & 1) ptr[WIDTH]   |=  B00000001; // Plane 0 B: 32 bytes ahead, bit 0
    else      ptr[WIDTH]   &= ~B00000001; // Plane 0 B unset; mask out
    // The remaining three image planes are more normal-ish.
    // Data is stored in the high 6 bits so it can be quickly
    // copied to the DATAPORT register w/6 output lines.
    for(; bit < limit; bit <<= 1) {
      *ptr &= ~B00011100;            // Mask out R,G,B in one op
      if(r & bit) *ptr |= B00000100; // Plane N R: bit 2
      if(g & bit) *ptr |= B00001000; // Plane N G: bit 3
      if(b & bit) *ptr |= B00010000; // Plane N B: bit 4
      ptr  += WIDTH;                 // Advance to next bit plane
    }
  } else {
    // Data for the lower half of the display is stored in the upper
    // bits, except for the plane 0 stuff, using 2 least bits.
    ptr = &matrixbuff[backindex][(y - nRows) * WIDTH * (nPlanes - 1) + x];
    *ptr &= ~B00000011;                  // Plane 0 G,B mask out in one op
    if(r & 1)  ptr[WIDTH] |=  B00000010; // Plane 0 R: 32 bytes ahead, bit 1
    else       ptr[WIDTH] &= ~B00000010; // Plane 0 R unset; mask out
    if(g & 1) *ptr        |=  B00000001; // Plane 0 G: bit 0
    if(b & 1) *ptr        |=  B00000010; // Plane 0 B: bit 0
    for(; bit < limit; bit <<= 1) {
      *ptr &= ~B11100000;            // Mask out R,G,B in one op
      if(r & bit) *ptr |= B00100000; // Plane N R: bit 5
      if(g & bit) *ptr |= B01000000; // Plane N G: bit 6
      if(b & bit) *ptr |= B10000000; // Plane N B: bit 7
      ptr  += WIDTH;                 // Advance to next bit plane
    }
  }
}

void RGBmatrixPanel::fillScreen(uint16_t c) {
  if((c == 0x0000) || (c == 0xffff)) {
    // For black or white, all bits in frame buffer will be identically
    // set or unset (regardless of weird bit packing), so it's OK to just
    // quickly memset the whole thing:
    memset(matrixbuff[backindex], c, WIDTH * nRows * 3);
  } else {
    // Otherwise, need to handle it the long way:
    Adafruit_GFX::fillScreen(c);
  }
}

// Return address of back buffer -- can then load/store data directly
uint8_t *RGBmatrixPanel::backBuffer() {
  return matrixbuff[backindex];
}

// For smooth animation -- drawing always takes place in the "back" buffer;
// this method pushes it to the "front" for display.  Passing "true", the
// updated display contents are then copied to the new back buffer and can
// be incrementally modified.  If "false", the back buffer then contains
// the old front buffer contents -- your code can either clear this or
// draw over every pixel.  (No effect if double-buffering is not enabled.)
void RGBmatrixPanel::swapBuffers(boolean copy) {
  if(matrixbuff[0] != matrixbuff[1]) {
    // To avoid 'tearing' display, actual swap takes place in the interrupt
    // handler, at the end of a complete screen refresh cycle.
    swapflag = true;                  // Set flag here, then...
    while(swapflag == true) delay(1); // wait for interrupt to clear it
    if(copy == true)
      memcpy(matrixbuff[backindex], matrixbuff[1-backindex], WIDTH * nRows * 3);
  }
}

// Dump display contents to the Serial Monitor, adding some formatting to
// simplify copy-and-paste of data as a PROGMEM-embedded image for another
// sketch.  If using multiple dumps this way, you'll need to edit the
// output to change the 'img' name for each.  Data can then be loaded
// back into the display using a pgm_read_byte() loop.
void RGBmatrixPanel::dumpMatrix(void) {

  int i, buffsize = WIDTH * nRows * 3;

  Serial.print(F("\n\n"
    "#include <avr/pgmspace.h>\n\n"
    "static const uint8_t PROGMEM img[] = {\n  "));

  for(i=0; i<buffsize; i++) {
    Serial.print(F("0x"));
    if(matrixbuff[backindex][i] < 0x10) Serial.write('0');
    Serial.print(matrixbuff[backindex][i],HEX);
    if(i < (buffsize - 1)) {
      if((i & 7) == 7) Serial.print(F(",\n  "));
      else             Serial.write(',');
    }
  }
  Serial.println(F("\n};"));
}

// -------------------- Interrupt handler stuff --------------------
#if !defined(ARDUINO_SAMD_FEATHER_M0)
ISR(TIMER1_OVF_vect, ISR_BLOCK) { // ISR_BLOCK important -- see notes later
  activePanel->updateDisplay();   // Call refresh func for active display
  TIFR1 |= TOV1;                  // Clear Timer1 interrupt flag
}

// Two constants are used in timing each successive BCM interval.
// These were found empirically, by checking the value of TCNT1 at
// certain positions in the interrupt code.
// CALLOVERHEAD is the number of CPU 'ticks' from the timer overflow
// condition (triggering the interrupt) to the first line in the
// updateDisplay() method.  It's then assumed (maybe not entirely 100%
// accurately, but close enough) that a similar amount of time will be
// needed at the opposite end, restoring regular program flow.
// LOOPTIME is the number of 'ticks' spent inside the shortest data-
// issuing loop (not actually a 'loop' because it's unrolled, but eh).
// Both numbers are rounded up slightly to allow a little wiggle room
// should different compilers produce slightly different results.
#define CALLOVERHEAD 60   // Actual value measured = 56
#define LOOPTIME     200  // Actual value measured = 188
// The "on" time for bitplane 0 (with the shortest BCM interval) can
// then be estimated as LOOPTIME + CALLOVERHEAD * 2.  Each successive
// bitplane then doubles the prior amount of time.  We can then
// estimate refresh rates from this:
// 4 bitplanes = 320 + 640 + 1280 + 2560 = 4800 ticks per row.
// 4800 ticks * 16 rows (for 32x32 matrix) = 76800 ticks/frame.
// 16M CPU ticks/sec / 76800 ticks/frame = 208.33 Hz.
// Actual frame rate will be slightly less due to work being done
// during the brief "LEDs off" interval...it's reasonable to say
// "about 200 Hz."  The 16x32 matrix only has to scan half as many
// rows...so we could either double the refresh rate (keeping the CPU
// load the same), or keep the same refresh rate but halve the CPU
// load.  We opted for the latter.
// Can also estimate CPU use: bitplanes 1-3 all use 320 ticks to
// issue data (the increasing gaps in the timing invervals are then
// available to other code), and bitplane 0 takes 920 ticks out of
// the 2560 tick interval.
// 320 * 3 + 920 = 1880 ticks spent in interrupt code, per row.
// From prior calculations, about 4800 ticks happen per row.
// CPU use = 1880 / 4800 = ~39% (actual use will be very slightly
// higher, again due to code used in the LEDs off interval).
// 16x32 matrix uses about half that CPU load.  CPU time could be
// further adjusted by padding the LOOPTIME value, but refresh rates
// will decrease proportionally, and 200 Hz is a decent target.

// The flow of the interrupt can be awkward to grasp, because data is
// being issued to the LED matrix for the *next* bitplane and/or row
// while the *current* plane/row is being shown.  As a result, the
// counter variables change between past/present/future tense in mid-
// function...hopefully tenses are sufficiently commented.

void RGBmatrixPanel::updateDisplay(void) {
  uint8_t  i, tick, tock, *ptr;
  uint16_t t, duration;

  *oeport  |= oepin;  // Disable LED output during row/plane switchover
  *latport |= latpin; // Latch data loaded during *prior* interrupt

  // Calculate time to next interrupt BEFORE incrementing plane #.
  // This is because duration is the display time for the data loaded
  // on the PRIOR interrupt.  CALLOVERHEAD is subtracted from the
  // result because that time is implicit between the timer overflow
  // (interrupt triggered) and the initial LEDs-off line at the start
  // of this method.
  t = (nRows > 8) ? LOOPTIME : (LOOPTIME * 2);
  duration = ((t + CALLOVERHEAD * 2) << plane) - CALLOVERHEAD;

  // Borrowing a technique here from Ray's Logic:
  // www.rayslogic.com/propeller/Programming/AdafruitRGB/AdafruitRGB.htm
  // This code cycles through all four planes for each scanline before
  // advancing to the next line.  While it might seem beneficial to
  // advance lines every time and interleave the planes to reduce
  // vertical scanning artifacts, in practice with this panel it causes
  // a green 'ghosting' effect on black pixels, a much worse artifact.

  if(++plane >= nPlanes) {      // Advance plane counter.  Maxed out?
    plane = 0;                  // Yes, reset to plane 0, and
    if(++row >= nRows) {        // advance row counter.  Maxed out?
      row     = 0;              // Yes, reset row counter, then...
      if(swapflag == true) {    // Swap front/back buffers if requested
        backindex = 1 - backindex;
        swapflag  = false;
      }
      buffptr = matrixbuff[1-backindex]; // Reset into front buffer
    }
  } else if(plane == 1) {
    // Plane 0 was loaded on prior interrupt invocation and is about to
    // latch now, so update the row address lines before we do that:
    if(row & 0x1)   *addraport |=  addrapin;
    else            *addraport &= ~addrapin;
    if(row & 0x2)   *addrbport |=  addrbpin;
    else            *addrbport &= ~addrbpin;
    if(row & 0x4)   *addrcport |=  addrcpin;
    else            *addrcport &= ~addrcpin;
    if(nRows > 8) {
      if(row & 0x8) *addrdport |=  addrdpin;
      else          *addrdport &= ~addrdpin;
    }
  }

  // buffptr, being 'volatile' type, doesn't take well to optimization.
  // A local register copy can speed some things up:
  ptr = (uint8_t *)buffptr;

  ICR1      = duration; // Set interval for next interrupt
  TCNT1     = 0;        // Restart interrupt timer
  *oeport  &= ~oepin;   // Re-enable output
  *latport &= ~latpin;  // Latch down

  // Record current state of SCLKPORT register, as well as a second
  // copy with the clock bit set.  This makes the innnermost data-
  // pushing loops faster, as they can just set the PORT state and
  // not have to load/modify/store bits every single time.  It's a
  // somewhat rude trick that ONLY works because the interrupt
  // handler is set ISR_BLOCK, halting any other interrupts that
  // might otherwise also be twiddling the port at the same time
  // (else this would clobber them).
  tock = SCLKPORT;
  tick = tock | sclkpin;

  if(plane > 0) { // 188 ticks from TCNT1=0 (above) to end of function

    // Planes 1-3 copy bytes directly from RAM to PORT without unpacking.
    // The least 2 bits (used for plane 0 data) are presumed masked out
    // by the port direction bits.

    // A tiny bit of inline assembly is used; compiler doesn't pick
    // up on opportunity for post-increment addressing mode.
    // 5 instruction ticks per 'pew' = 160 ticks total
    #define pew asm volatile(                 \
      "ld  __tmp_reg__, %a[ptr]+"    "\n\t"   \
      "out %[data]    , __tmp_reg__" "\n\t"   \
      "out %[clk]     , %[tick]"     "\n\t"   \
      "out %[clk]     , %[tock]"     "\n"     \
      :: [ptr]  "e" (ptr),                    \
         [data] "I" (_SFR_IO_ADDR(DATAPORT)), \
         [clk]  "I" (_SFR_IO_ADDR(SCLKPORT)), \
         [tick] "r" (tick),                   \
         [tock] "r" (tock));

    // Loop is unrolled for speed:
    pew pew pew pew pew pew pew pew
    pew pew pew pew pew pew pew pew
    pew pew pew pew pew pew pew pew
    pew pew pew pew pew pew pew pew

      if (WIDTH == 64) {
    pew pew pew pew pew pew pew pew
    pew pew pew pew pew pew pew pew
    pew pew pew pew pew pew pew pew
    pew pew pew pew pew pew pew pew
      }

    buffptr = ptr; //+= 32;

  } else { // 920 ticks from TCNT1=0 (above) to end of function

    // Planes 1-3 (handled above) formatted their data "in place,"
    // their layout matching that out the output PORT register (where
    // 6 bits correspond to output data lines), maximizing throughput
    // as no conversion or unpacking is needed.  Plane 0 then takes up
    // the slack, with all its data packed into the 2 least bits not
    // used by the other planes.  This works because the unpacking and
    // output for plane 0 is handled while plane 3 is being displayed...
    // because binary coded modulation is used (not PWM), that plane
    // has the longest display interval, so the extra work fits.
    for(i=0; i<WIDTH; i++) {
      DATAPORT =
        ( ptr[i]    << 6)         |
        ((ptr[i+WIDTH] << 4) & 0x30) |
        ((ptr[i+WIDTH*2] << 2) & 0x0C);
      SCLKPORT = tick; // Clock lo
      SCLKPORT = tock; // Clock hi
    }
  }
}

#else     // ARDUINO_SAMD_FEATHER_M0

/*******************************************************************************

  All the magic to support the Feather M0 occurs here in the interrupt handler.

  Timer 4 is configured to interrupt at a rate 16x the desired panel refresh
  rate to enable Binary Code Modulation. Unlike the AVR interrupt handler,
  the Feather M0 refresh algorithm does not change the interrupt interval.
  Instead, the interrupt will return immediately until the current plane
  has been displayed the required amount of time.
    Plane 0 is displayed for only 1 interrupt interval.
    Planes 1, 2, and 3 are displayed for 2, 4, and 8 interrupt intervals respectively.

  The original AVR refresh algorithm is kept in spirit, but is modified somewhat
  to make it more efficient for the ARM M0+ core. The original AVR comments
  are embedded in the code below.

  Max time spent in the interrupt handler is < 25 uS worst case (preparing plane 0).
  The maximum refresh rate for a 32 row panel is
      1 / (25uS x 16 rows x 16 bcm_intervals) = 156 Hz

  The screen is actually refreshed only 4 times out of the 16 bcm_intervals that
  the interrupt handler is called for each row so the worst case CPU overhead
  at the maximum refresh rate is only 25%.

 ******************************************************************************/

// Pointers to PORT Output Set/Clear registers
volatile uint32_t *setPortA = &PORT->Group[PORTA].OUTSET.reg;
volatile uint32_t *clrPortA = &PORT->Group[PORTA].OUTCLR.reg;
volatile uint32_t *setPortB = &PORT->Group[PORTB].OUTSET.reg;
volatile uint32_t *clrPortB = &PORT->Group[PORTB].OUTCLR.reg;

static const uint32_t  PIXEL_MASK   = 0x3f << 15;
static const uint32_t  CLK_MASK     = 0x01 << 11;
static const uint32_t  LAT_MASK     = 0x01 << 8;    // port B
static const uint32_t  OE_MASK      = 0x01 << 9;    // port B
static const uint32_t  ABCD_MASK    = 0x2d << 2;
static const uint32_t  A_MASK       = 0x01;
static const uint32_t  BC_MASK      = 0x06;
static const uint32_t  D_MASK       = 0x08;
static const uint32_t  PLANE0_MASK  = 0x03030303;

/*******************************************************************************
    Timer 4 interrupt handler.
******************************************************************************/
void ISR_LEDPanelRefresh(struct tc_module *const module_inst) {
  activePanel->updateDisplay();   // Call refresh func for active display
}

void RGBmatrixPanel::updateDisplay(void) {
  static uint32_t plane_dly_cnt=0;
  uint8_t  i, *ptr;

  /*******************************************************************************
      The plane variable is the plane that will be displayed next. The plane
      being displayed now is the previous plane.
   ******************************************************************************/
  if (++plane_dly_cnt < (1 << (plane+3) % 4)) { return; }
  plane_dly_cnt = 0;

  // Disable LED output during row/plane switchover
  // Latch data loaded during *prior* interrupt
  *setPortB = OE_MASK | LAT_MASK;

  // Borrowing a technique here from Ray's Logic:
  // www.rayslogic.com/propeller/Programming/AdafruitRGB/AdafruitRGB.htm
  // This code cycles through all four planes for each scanline before
  // advancing to the next line.  While it might seem beneficial to
  // advance lines every time and interleave the planes to reduce
  // vertical scanning artifacts, in practice with this panel it causes
  // a green 'ghosting' effect on black pixels, a much worse artifact.

  /*******************************************************************************
      Reordered original logic to minimize time while display is blanked.
   ******************************************************************************/
  ++plane;
  if(plane == 1) {
      // Plane 0 was loaded on prior interrupt invocation and is about to
      // latch now, so update the row address lines before we do that:
      /*******************************************************************************
          Set the ABCD pins to the row address.
          Sadly, the port address pins are not contiguous.
          The D pin is always driven. If the panel only has 16 rows then the
          D address was not configured as an output so it doesn't matter.
       ******************************************************************************/
      *clrPortA = ABCD_MASK;
      *setPortA = ((row & D_MASK) << 4) | ((row & BC_MASK) << 3) | ((row & A_MASK) << 2);
  }

  *clrPortB = LAT_MASK | OE_MASK; // Re-enable output and set the latch

  if(plane >= nPlanes) {          // Advance plane counter.  Maxed out?
    plane = 0;                    // Yes, reset to plane 0, and
    if(++row >= nRows) {          // advance row counter.  Maxed out?
      row     = 0;                // Yes, reset row counter, then...
      if(swapflag == true) {      // Swap front/back buffers if requested
        backindex = 1 - backindex;
        swapflag  = false;
      }
      buffptr = matrixbuff[1-backindex]; // Reset into front buffer
    }
  }

  /*******************************************************************************
      AVR pixel output algorithm replaced. No assembly is used, the ARM
      compiler is sufficiently (that is to say VERY) efficient.
      The LSb of the pixel is bit 2, and the LSb of the port is bit 15, so
      the pixels are shifted 13 bits to align them with the port.
   ******************************************************************************/
  // buffptr, being 'volatile' type, doesn't take well to optimization.
  // A local register copy can speed some things up:
  ptr = (uint8_t *) buffptr;
  if (plane > 0) {
      // Planes 1-3 copy bytes directly from RAM to PORT without unpacking.
      // The least 2 bits (used for plane 0 data) are masked out.
      // loop is unrolled x4
      for (int col=0; col<WIDTH; col+=4) {
          *clrPortA = CLK_MASK + PIXEL_MASK;        // clk low, pixel data zeroed
          *setPortA = (*ptr++ << 13) & PIXEL_MASK;  // write pixel data
          *setPortA = CLK_MASK;                     // clk high
          *clrPortA = CLK_MASK + PIXEL_MASK;
          *setPortA = (*ptr++ << 13) & PIXEL_MASK;
          *setPortA = CLK_MASK;
          *clrPortA = CLK_MASK + PIXEL_MASK;
          *setPortA = (*ptr++ << 13) & PIXEL_MASK;
          *setPortA = CLK_MASK;
          *clrPortA = CLK_MASK + PIXEL_MASK;
          *setPortA = (*ptr++ << 13) & PIXEL_MASK;
          *setPortA = CLK_MASK;
      }
      buffptr += WIDTH;
  }

  else {
      // Planes 1-3 (handled above) formatted their data "in place,"
      // their layout matching that out the output PORT register (where
      // 6 bits correspond to output data lines), maximizing throughput
      // as no conversion or unpacking is needed.  Plane 0 then takes up
      // the slack, with all its data packed into the 2 least bits not
      // used by the other planes.  This works because the unpacking and
      // output for plane 0 is handled while plane 3 is being displayed...
      // because binary coded modulation is used (not PWM), that plane
      // has the longest display interval, so the extra work fits.
      //
      /*******************************************************************************
          Unroll the loop x4 by reading 4 bytes from each plane at a time.
          Unrolled loop is equivalent to:

           for(i=0; i<WIDTH; i++) {
              *clrPortA = CLK_MASK + PIXEL_MASK;
              *setPortA = (((ptr[i] & 0x03) << 4) | ((ptr[i+WIDTH] & 0x03) << 2) | (ptr[i+WIDTH*2] & 0x03)) << 15;
              *setPortA = CLK_MASK;
          }

       ******************************************************************************/

      // Unroll the loop x4 by reading 4 bytes from each plane at a time.
      uint32_t plane0_g1r1, plane0_r2b1, plane0_b2g2;
      uint32_t pixel;

      for(i=0; i<WIDTH; i+=4) {
          plane0_b2g2 = ((* ((uint32_t *) (ptr+i))) & PLANE0_MASK) << 4;
          plane0_r2b1 = ((* ((uint32_t *) (ptr+i+WIDTH))) & PLANE0_MASK) << 2;
          plane0_g1r1 =  (* ((uint32_t *) (ptr+i+WIDTH+WIDTH))) & PLANE0_MASK;
          pixel = plane0_g1r1 | plane0_r2b1 | plane0_b2g2;
          *clrPortA = CLK_MASK + PIXEL_MASK;        // clk low, pixel data zeroed
          *setPortA = (pixel << 15) & PIXEL_MASK;
          *setPortA = CLK_MASK;                     // clk high
          *clrPortA = CLK_MASK + PIXEL_MASK;
          *setPortA = (pixel << 7) & PIXEL_MASK;
          *setPortA = CLK_MASK;
          *clrPortA = CLK_MASK + PIXEL_MASK;
          *setPortA = (pixel >> 1) & PIXEL_MASK;
          *setPortA = CLK_MASK;
          *clrPortA = CLK_MASK + PIXEL_MASK;
          *setPortA = (pixel >> 9) & PIXEL_MASK;
          *setPortA = CLK_MASK;
      }
  }
}
#endif

