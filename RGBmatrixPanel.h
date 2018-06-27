#ifndef RGBMATRIXPANEL_H
#define RGBMATRIXPANEL_H

#if ARDUINO >= 100
 #include "Arduino.h"
#else
 #include "WProgram.h"
 #include "pins_arduino.h"
#endif
#include "Adafruit_GFX.h"

#if defined(__AVR__)
  typedef uint8_t  PortType;
#elif defined(__arm__)
  typedef uint32_t PortType; // Formerly 'RwReg' but interfered w/CMCIS header
#endif

class RGBmatrixPanel : public Adafruit_GFX {

 public:

  // Constructor for 16x32 panel:
  RGBmatrixPanel(uint8_t a, uint8_t b, uint8_t c,
    uint8_t clk, uint8_t lat, uint8_t oe, boolean dbuf
#if defined(ARDUINO_ARCH_SAMD)
    ,uint8_t *pinlist=NULL
#endif
    );

  // Constructor for 32x32 panel (adds 'd' pin):
  RGBmatrixPanel(uint8_t a, uint8_t b, uint8_t c, uint8_t d,
    uint8_t clk, uint8_t lat, uint8_t oe, boolean dbuf, uint8_t width=32
#if defined(ARDUINO_ARCH_SAMD)
    ,uint8_t *pinlist=NULL
#endif
    );

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
	    uint8_t clk, uint8_t lat, uint8_t oe, boolean dbuf, 
	    uint8_t width
#if defined(ARDUINO_ARCH_SAMD)
            ,uint8_t *rgbpins
#endif
  );

  uint8_t  _clk, _lat, _oe, _a, _b, _c, _d; // Pin numbers
  PortType clkmask, latmask, oemask,        // Pin bitmasks
           addramask, addrbmask, addrcmask, addrdmask;
  // PORT register pointers (CLKPORT is hardcoded on AVR)
  volatile PortType *latport, *oeport,
                    *addraport, *addrbport, *addrcport, *addrdport;

#if defined(ARDUINO_ARCH_SAMD)
  uint8_t  rgbpins[6];                      // Pin numbers for 2x R,G,B bits
  volatile PortType *outsetreg, *outclrreg; // PORT bit set, clear registers
  PortType           rgbclkmask;            // Mask of all RGB bits + CLK
  PortType           expand[256];           // 6-to-32 bit converter table
#endif

  // Counters/pointers for interrupt handler:
  volatile uint8_t row, plane;
  volatile uint8_t *buffptr;
};

#endif // RGBMATRIXPANEL_H
