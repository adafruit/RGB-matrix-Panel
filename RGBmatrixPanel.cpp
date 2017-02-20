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
*/

#include "RGBmatrixPanel.h"
#include "gamma.h"

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
  addrdport = portOutputRegister(digitalPinToPort(d));
  addrdpin  = digitalPinToBitMask(d);
}

void RGBmatrixPanel::begin(void) {

  backindex   = 0;                         // Back buffer
  buffptr     = matrixbuff[1 - backindex]; // -> front buffer
  activePanel = this;                      // For interrupt hander

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
    r = pgm_read_byte(&gamma[r]); // Gamma correction table maps
    g = pgm_read_byte(&gamma[g]); // 8-bit input to 4-bit output
    b = pgm_read_byte(&gamma[b]);
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
    r = pgm_read_byte(&gamma[(r * v1) >> 8]); // Gamma correction table maps
    g = pgm_read_byte(&gamma[(g * v1) >> 8]); // 8-bit input to 4-bit output
    b = pgm_read_byte(&gamma[(b * v1) >> 8]);
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

ISR(TIMER1_OVF_vect, ISR_BLOCK) { // ISR_BLOCK important -- see notes later
  activePanel->updateDisplay();   // Call refresh func for active display
  TIFR1 |= TOV1;                  // Clear Timer1 interrupt flag
}

// The BCM methodology works great for many things. It works poorly for
// light, because light isn't really linear; a light on 1% of the time is
// not "1% as bright" as a light on all the time. (This is why the gamma
// table for the BCM implementation has everything under 0x40 on an 8-bit
// scale map to zero, rather than mapping 0x10 to 1.)
//
// It takes about 200 cycles to run through the loop, and about 60 cycles
// on each side for interrupt handling. The original BCM implementation
// thus used four time intervals: 320, 640, 1280, 2560 cycles. That is
// way too bright on the low end.
//
// Additional observation: The interrupt handler time of ~120 cycles total
// is pretty large compared to a 200-cycle loop. So, we skip the first
// interrupt; we draw plane 0, then immediately draw plane 1, and only use
// timer interval for the times when plane 1, 2, or 3 are being displayed,
// which are the times when planes 2, 3, and 0 are being drawn. (Because the
// drawing code happens during the time when the previous plane's pixels are
// lit.)
//
// So instead of using the actual full loop time for the shortest interval,
// giving us close to 25% perceptual brightness, we use pretty much the
// shortest time we can, performing about two operations before turning the
// lights back off. That gets us a much dimmer "dimmest" light for a given
// pixel. Then we jump to the top of the interrupt handler and re-run it
// immediately, saving 120 cycles of interrupt handling.
//
// The second display interval is longer, but still shorter than the timer
// interrupt; we turn the display off before we return from the interrupt
// handler.
//
// That leaves us a fair bit of time for the third and fourth intervals.

// The calculations from the BCM implementation were:
// 4 bitplanes = 320 + 640 + 1280 + 2560 = 4800 ticks per row.
// 4800 ticks * 16 rows (for 32x32 matrix) = 76800 ticks/frame.
// 16M CPU ticks/sec / 76800 ticks/frame = 208.33 Hz.
// Actual frame rate will be slightly less due to work being done
// during the brief "LEDs off" interval...it's reasonable to say
// "about 200 Hz."
//
// For the not-really-BCM implementation, we've used about 520 ticks
// for the first two bitplanes. The third and fourth display intervals
// will be approximately 60 cycles longer than whatever duration we
// set the timer for. Also, we want some time between Plane 1 being
// drawn and Plane 2 being drawn, even though we'eve already turned off
// the display at that point, in case other code wants to run; 520
// ticks is maybe reasonable, 720 would be starting to push it if you're
// also using 400kHz i2c or something. So we set an interrupt timer for
// about 400 cycles, starting right before drawing Plane 1. That will
// fire about 200 cycles later, leaving at least a little time for other
// operations, and we'll be at about 720 total cycles between code
// execution and delay. So we have about another 4180 cycles available
// if we want to keep our performance comparable. Let's call that
// 800 and 3200 cycle delays, plus interrupt overhead.
//
// The BCM implementation used about 320 ticks per plane for the first
// three planes drawn, and 920 for the last one. This code trims about
// 120 cycles off that by skipping one round of interrupt handling, but
// is otherwise similar. So it should use slightly less CPU time at
// the same display refresh rate.
//

static const uint16_t PROGMEM durations[] = {
	0, 400, 800, 3200,
};

void RGBmatrixPanel::updateDisplay(void) {
  uint8_t  i, tick, tock, *ptr;
  uint16_t t, duration;

restart:
  *oeport  |= oepin;  // Disable LED output during row/plane switchover
  *latport |= latpin; // Latch data loaded during *prior* interrupt

  // use precomputed values; see discussion above.
  duration = pgm_read_word(&durations[plane]);

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

  // set interval timer, but only if we want one.
  if (duration != 0) {
  	ICR1      = duration; // Set interval for next interrupt
  	TCNT1     = 0;        // Restart interrupt timer
  }
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

  // Yes, really, two whole instructions later, plus the branch. It's still
  // quite visible.
  if (plane == 1) {
    *oeport  |= oepin;  // during draw of plane 1, plane 0 is up, make that be shorter
  }
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
    // according to previous measurements, this is about 180ish cycles later
    // than we turned off plane 0. perceptually, it's on the rough order of
    // "twice as bright".
    if (plane == 2) {
      *oeport  |= oepin;  // during draw of plane 2, plane 1 is up, shorten that interval
    }

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
  /* timer interrupt overhead makes it cost a LOT of cycles to return when the next
   * interrupt should be almost immediate, so we just skip back up once:
   */
  if (plane == 1)
    goto restart;
}

