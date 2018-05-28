#if ARDUINO >= 100
 #include "Arduino.h"
#else
 #include "WProgram.h"
 #include "pins_arduino.h"
#endif
#include "Adafruit_GFX.h"

#if defined(__AVR__)
  typedef volatile uint8_t RwReg;
#endif
#if defined(__arm__)
  typedef volatile uint32_t RwReg;
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

  // Init/alloc code common to both constructors:
  void init(uint8_t rows, uint8_t a, uint8_t b, uint8_t c,
	    uint8_t sclk, uint8_t latch, uint8_t oe, boolean dbuf, 
	    uint8_t width);

  // PORT register pointers, pin bitmasks, pin numbers:
  uint8_t _sclk, _latch, _oe, _a, _b, _c, _d;
  
  RwReg *latport, *oeport, *addraport, *addrbport, *addrcport, *addrdport;
#if defined(ARDUINO_ARCH_SAMD)
  uint32_t r1_pinmask, r2_pinmask, g1_pinmask, g2_pinmask, b1_pinmask, b2_pinmask, clk_pinmask, data_pinmask;
  uint32_t sclkpin, latpin, oepin, addrapin, addrbpin, addrcpin, addrdpin;
  RwReg *outreg, *outsetreg, *outclrreg;
#elif defined(__AVR__)
  uint8_t sclkpin, latpin, oepin, addrapin, addrbpin, addrcpin, addrdpin;
#endif

  // Counters/pointers for interrupt handler:
  volatile uint8_t row, plane;
  volatile uint8_t *buffptr;
};

