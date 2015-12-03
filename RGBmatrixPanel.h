#if ARDUINO >= 100
 #include "Arduino.h"
#else
 #include "WProgram.h"
 #include "pins_arduino.h"
#endif
#include "Adafruit_GFX.h"

#ifdef CORE_TEENSY
	#define CLK 19
	#define LAT 18
	#define OE	17
	#define A   23
	#define B   22
	#define C   21
	#define D   20
#endif

class RGBmatrixPanel : public Adafruit_GFX {

 public:
#ifdef CORE_TEENSY
	 RGBmatrixPanel(boolean dbuf, uint8_t width = 32);
#else
  // Constructor for 16x32 panel:
  RGBmatrixPanel(uint8_t a, uint8_t b, uint8_t c,
    uint8_t sclk, uint8_t latch, uint8_t oe, boolean dbuf);

  // Constructor for 32x32 panel (adds 'd' pin):
  RGBmatrixPanel(uint8_t a, uint8_t b, uint8_t c, uint8_t d,
    uint8_t sclk, uint8_t latch, uint8_t oe, boolean dbuf, uint8_t width=32);
#endif

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
    Color333(uint8_t r, uint8_t g, uint8_t b),
    Color444(uint8_t r, uint8_t g, uint8_t b),
    Color888(uint8_t r, uint8_t g, uint8_t b),
    Color888(uint8_t r, uint8_t g, uint8_t b, boolean gflag),
    ColorHSV(long hue, uint8_t sat, uint8_t val, boolean gflag);

 private:

  uint8_t         *matrixbuff[2];
  uint8_t          nRows;
  volatile uint8_t backindex;
  volatile boolean swapflag;

#ifdef CORE_TEENSY
  IntervalTimer drawTimer;

  // Init/alloc code common to both constructors:
  void init(uint8_t rows, boolean dbuf, uint8_t width);

#else
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
#endif

  // Counters/pointers for interrupt handler:
  volatile uint8_t row, plane;
  volatile uint8_t *buffptr;
};

