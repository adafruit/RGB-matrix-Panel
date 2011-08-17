// Controller for RGB matrix panels that use 1:8 refresh mode

#include <WProgram.h>
#include <util/delay.h>
#include "pins_arduino.h"
#include "wiring_private.h"

#define RGBMATRIX_DATAPORT PORTD
#define RGBMATRIX_DATADDR  DDRD
#define RGBMATRIX_DATASHIFT 2   // that is, shift all data up to use the top 6 bits

#define RGBMATRIX_CLOCKPIN 0
#define RGBMATRIX_CLOCKPORT PORTB
#define RGBMATRIX_CLOCKDDR DDRB

#define PWMBITS 3      // 3 = 3 bit color per LED, 9-bit total pixel color
#define PWMMAX ((1 << PWMBITS) - 1)

#define WIDTH 32       // RGB LEDs
#define HEIGHT 16      //  " 

#if (PWMBITS == 3) || (PWMBITS == 4) 
#define NUMBYTES (WIDTH * HEIGHT / 2) * 3  // use 1.5 bytes (12 bits) per pixel
#endif 


#define swap(a, b) { uint16_t t = a; a = b; b = t; }

class RGBmatrixPanel  : public Print {
 public:
  RGBmatrixPanel(uint8_t a, uint8_t b, uint8_t c, uint8_t latch, uint8_t oe);
  void begin();
  uint16_t Color333(uint8_t r, uint8_t g, uint8_t b);
  uint16_t Color444(uint8_t r, uint8_t g, uint8_t b);
  uint16_t Color888(uint8_t r, uint8_t g, uint8_t b);
  void drawPixel(uint8_t x, uint8_t y, uint16_t c);
  void drawLine(int8_t x0, int8_t y0, int8_t x1, int8_t y1, uint16_t c);
  void drawRect(uint8_t x, uint8_t y, uint8_t w, uint8_t h, uint16_t color); 
  void fillRect(uint8_t x, uint8_t y, uint8_t w, uint8_t h, uint16_t color);
  void drawCircle(uint8_t x0, uint8_t y0, uint8_t r, uint16_t color);
  void fillCircle(uint8_t x0, uint8_t y0, uint8_t r, uint16_t color);
  void fill(uint16_t c);

  // Printing
  void setCursor(uint8_t x, uint8_t y);
  void setTextSize(uint8_t s);
  void setTextColor(uint16_t c);
  void write(uint8_t c);
  void drawChar(uint8_t x, uint8_t y, char c, uint16_t color, uint8_t size);

  void updateDisplay();
  void dumpMatrix(void);

  uint8_t width();
  uint8_t height();
 private:
  uint8_t matrixbuff[NUMBYTES];  // 768 bytes for 16x32

  uint8_t cursor_x, cursor_y, textsize;
  uint16_t textcolor;

  volatile uint8_t pwmcounter;
  volatile uint8_t scansection;

  void writeSection(uint8_t section, uint8_t *buffptr);

  uint8_t _a, _b, _c, _latch, _oe, _sclk;
  volatile uint8_t *sclkportreg, *latportreg, *oeportreg;
  volatile uint8_t *addraportreg, *addrbportreg, *addrcportreg;
  uint8_t sclkpin, latpin, oepin, addrapin, addrbpin, addrcpin;
};
