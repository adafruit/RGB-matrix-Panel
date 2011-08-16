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


class RGBmatrixPanel {
 public:
  RGBmatrixPanel(uint8_t a, uint8_t b, uint8_t c, uint8_t latch, uint8_t oe);
  void begin();
  uint16_t Color333(uint8_t r, uint8_t g, uint8_t b);
  uint16_t Color444(uint8_t r, uint8_t g, uint8_t b);
  uint16_t Color888(uint8_t r, uint8_t g, uint8_t b);
  void setPixel(uint8_t x, uint8_t y, uint16_t c);
  void updateDisplay();
  void dumpMatrix(void);

  uint8_t width();
  uint8_t height();
 private:
  uint8_t matrixbuff[NUMBYTES];  // 768 bytes for 16x32

  volatile uint8_t pwmcounter;
  volatile uint8_t scansection;

  void writeSection(uint8_t section, uint8_t *buffptr);

  uint8_t _a, _b, _c, _latch, _oe, _sclk;
  volatile uint8_t *sclkportreg, *latportreg, *oeportreg;
  volatile uint8_t *addraportreg, *addrbportreg, *addrcportreg;
  uint8_t sclkpin, latpin, oepin, addrapin, addrbpin, addrcpin;
};
