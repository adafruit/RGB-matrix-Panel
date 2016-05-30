#if ARDUINO >= 100
 #include "Arduino.h"
#else
 #include "WProgram.h"
 #include "pins_arduino.h"
#endif
#include "Adafruit_GFX.h"

#if __cplusplus >= 201103L
 #define CONSTEXPR constexpr
#else
 #define CONSTEXPR const
#endif

class RGBmatrixPanel : public Adafruit_GFX {

 public:

  // Constructor for 16x32 panel:
  RGBmatrixPanel(uint8_t a, uint8_t b, uint8_t c,
    uint8_t sclk, uint8_t latch, uint8_t oe, boolean dbuf);

  // Constructor for 32x32 panel (adds 'd' pin):
  RGBmatrixPanel(uint8_t a, uint8_t b, uint8_t c, uint8_t d,
    uint8_t sclk, uint8_t latch, uint8_t oe, boolean dbuf, uint8_t width=32);

  void
    begin(void),
    drawPixel(int16_t x, int16_t y, uint16_t c),
    fillScreen(uint16_t c),
    updateDisplay(void),
    swapBuffers(boolean),
    dumpMatrix(void);
  uint8_t
    *backBuffer(void);
    
  uint16_t
    Color888(uint8_t r, uint8_t g, uint8_t b, boolean gflag),
    ColorHSV(long hue, uint8_t sat, uint8_t val, boolean gflag);

  // Promote 3/3/3 RGB to Adafruit_GFX 5/6/5
  static CONSTEXPR uint16_t Color333(uint8_t r, uint8_t g, uint8_t b) {
    // RRRrrGGGgggBBBbb
    return ((r & 0x7) << 13) | ((r & 0x6) << 10) |
           ((g & 0x7) <<  8) | ((g & 0x7) <<  5) |
           ((b & 0x7) <<  2) | ((b & 0x6) >>  1);
  }

  // Promote 4/4/4 RGB to Adafruit_GFX 5/6/5
  static CONSTEXPR uint16_t Color444(uint8_t r, uint8_t g, uint8_t b) {
    // RRRRrGGGGggBBBBb
    return ((r & 0xF) << 12) | ((r & 0x8) << 8) |
           ((g & 0xF) <<  7) | ((g & 0xC) << 3) |
           ((b & 0xF) <<  1) | ((b & 0x8) >> 3);
  }

  // Demote 8/8/8 to Adafruit_GFX 5/6/5
  // If no gamma flag passed, assume linear color
  static CONSTEXPR uint16_t Color888(uint8_t r, uint8_t g, uint8_t b) {
    return ((uint16_t)(r & 0xF8) << 8) | ((uint16_t)(g & 0xFC) << 3) | (b >> 3);
  };

 private:

  uint8_t         *matrixbuff[2];
  uint8_t          nRows;
  volatile uint8_t backindex;
  volatile boolean swapflag;

  // Init/alloc code common to both constructors:
  void init(uint8_t rows, uint8_t a, uint8_t b, uint8_t c,
	    uint8_t sclk, uint8_t latch, uint8_t oe, boolean dbuf, 
	    uint8_t width);

  // PORT register pointers, pin bitmasks, pin numbers:
  volatile uint8_t
    *latport, *oeport, *addraport, *addrbport, *addrcport, *addrdport;
  uint8_t
    sclkpin, latpin, oepin, addrapin, addrbpin, addrcpin, addrdpin,
    _sclk, _latch, _oe, _a, _b, _c, _d;

  // Counters/pointers for interrupt handler:
  volatile uint8_t row, plane;
  volatile uint8_t *buffptr;
};

#undef CONSTEXPR
