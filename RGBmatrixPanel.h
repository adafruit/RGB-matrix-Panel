#if ARDUINO >= 100
 #include "Arduino.h"
#else
 #include "WProgram.h"
 #include "pins_arduino.h"
#endif

class RGBmatrixPanel : public Print {

 public:

  // Constructor for 16x32 panel:
  RGBmatrixPanel(uint8_t a, uint8_t b, uint8_t c,
    uint8_t sclk, uint8_t latch, uint8_t oe, boolean dbuf);

  // Constructor for 32x32 panel (adds 'd' pin):
  RGBmatrixPanel(uint8_t a, uint8_t b, uint8_t c, uint8_t d,
    uint8_t sclk, uint8_t latch, uint8_t oe, boolean dbuf);

  void
    begin(void),
    drawPixel(int x, int y, uint16_t c),
    drawLine(int x0, int y0, int x1, int y1, uint16_t c),
    drawRect(int x, int y, uint8_t w, uint8_t h, uint16_t c),
    fillRect(int x, int y, uint8_t w, uint8_t h, uint16_t c),
    drawCircle(int x0, int y0, uint8_t r, uint16_t c),
    fillCircle(int x0, int y0, uint8_t r, uint16_t c),
    fill(uint16_t c),
    updateDisplay(void),
    swapBuffers(boolean),
    dumpMatrix(void);
  uint8_t
    width(void),
    height(void),
    *backBuffer(void);
  uint16_t
    Color333(uint8_t r, uint8_t g, uint8_t b),
    Color444(uint8_t r, uint8_t g, uint8_t b),
    Color888(uint8_t r, uint8_t g, uint8_t b),
    Color888(uint8_t r, uint8_t g, uint8_t b, boolean gflag),
    ColorHSV(long hue, uint8_t sat, uint8_t val, boolean gflag);

  // Printing
  void
    attachInterrupt(void (*func)(void)),
    setCursor(int x, int y),
    setTextSize(uint8_t s),
    setTextColor(uint16_t c),
    drawChar(int x, int y, char c, uint16_t color, uint8_t size);
#if ARDUINO >= 100
  size_t write(uint8_t c);
#else
  void   write(uint8_t c);
#endif

 private:

  uint8_t  *matrixbuff[2];
  uint8_t  nRows, nPlanes, textsize, backindex;
  int      cursor_x, cursor_y;
  uint16_t textcolor;
  boolean  swapflag;

  // Init/alloc code common to both constructors:
  void init(uint8_t rows, uint8_t a, uint8_t b, uint8_t c,
    uint8_t sclk, uint8_t latch, uint8_t oe, boolean dbuf);

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

