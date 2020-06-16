/*!
 * @file RGBmatrixPanel.h
 *
 * This is the documentation for Adafruit's RGB LED Matrix Panel library
 * for the Arduino platform.  It is designed to work with 16x32, 32x32 and
 * 32x64 panels.
 *
 * Adafruit invests time and resources providing this open source code,
 * please support Adafruit and open-source hardware by purchasing
 * products from Adafruit!
 *
 * Written by Limor Fried/Ladyada & Phil Burgess/PaintYourDragon for
 * Adafruit Industries.
 *
 * BSD license, all text here must be included in any redistribution.
 *
 */

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
typedef uint8_t PortType;
#elif defined(__arm__) || defined(__xtensa__)
typedef uint32_t PortType; // Formerly 'RwReg' but interfered w/CMCIS header
#endif

/*!
    @brief  Class encapsulating RGB LED matrix functionality.
*/
class RGBmatrixPanel : public Adafruit_GFX {

public:
  /*!
    @brief  Constructor for 16x32 panel.
    @param  a        Address/row-select A pin number.
    @param  b        Address/row-select B pin number.
    @param  c        Address/row-select C pin number.
    @param  clk      RGB clock pin number.
    @param  lat      RGB latch pin number.
    @param  oe       Output enable pin number.
    @param  dbuf     If true, display is double-buffered, allowing for
                     smoother animation (requires 2X RAM).
    @note   pinlist  (SAMD only) uint8_t array of 6 pin numbers corresponding
                     to upper R, G, B and lower R, G, B pins.
  */
  RGBmatrixPanel(uint8_t a, uint8_t b, uint8_t c, uint8_t clk, uint8_t lat,
                 uint8_t oe, boolean dbuf
#if defined(ARDUINO_ARCH_SAMD) || defined(ARDUINO_ARCH_ESP32)
                 ,
                 uint8_t *pinlist = NULL
#endif
  );

  /*!
    @brief  Constructor for 32x32 or 32x64 panel.
    @param  a        Address/row-select A pin number.
    @param  b        Address/row-select B pin number.
    @param  c        Address/row-select C pin number.
    @param  d        Address/row-select D pin number.
    @param  clk      RGB clock pin number.
    @param  lat      RGB latch pin number.
    @param  oe       Output enable pin number.
    @param  dbuf     If true, display is double-buffered, allowing for
                     smoother animation (requires 2X RAM).
    @param  width    Specify 32 or 64 for the two supported matrix widths
                     (default is 32).
    @note   pinlist  (SAMD only) uint8_t array of 6 pin numbers corresponding
                     to upper R, G, B and lower R, G, B pins.
  */
  RGBmatrixPanel(uint8_t a, uint8_t b, uint8_t c, uint8_t d, uint8_t clk,
                 uint8_t lat, uint8_t oe, boolean dbuf, uint8_t width = 32
#if defined(ARDUINO_ARCH_SAMD) || defined(ARDUINO_ARCH_ESP32)
                 ,
                 uint8_t *pinlist = NULL
#endif
  );

  /*!
    @brief  Start RGB matrix. Initializes timers and interrupts.
  */
  void begin(void);

  /*!
    @brief  Lowest-level pixel drawing function required by Adafruit_GFX.
            Does not have an immediate effect -- must call updateDisplay()
            after any drawing operations to refresh matrix contents.
    @param  x  Pixel column (horizontal).
    @param  y  Pixel row (vertical).
    @param  c  Pixel color (16-bit 5/6/5 color, but actual color on matrix
               will be decimated from this as it uses fewer bitplanes).
  */
  void drawPixel(int16_t x, int16_t y, uint16_t c);

  /*!
    @brief  Fill entire matrix a single color.
            Does not have an immediate effect -- must call updateDisplay()
            after any drawing operations to refresh matrix contents.
    @param  c  Color (16-bit 5/6/5 color, but actual color on matrix
               will be decimated from this as it uses fewer bitplanes).
  */
  void fillScreen(uint16_t c);

  /*!
    @brief  Refresh matrix contents following one or more drawing calls.
  */
  void updateDisplay(void);

  /*!
    @brief  If using double buffering, swap the front and back buffers.
  */
  void swapBuffers(boolean);

  /*!
    @brief  Dump display contents to the Serial Monitor, adding some
            formatting to simplify copy-and-paste of data as a PROGMEM-
            embedded image for another sketch. If using multiple dumps
            this way, you'll need to edit the output to change the 'img'
            name for each. Data can then be loaded back into the display
            using a pgm_read_byte() loop.
  */
  void dumpMatrix(void);

  /*!
    @brief   Get address of back buffer -- can then load/store data directly.
             Format is very strangely interleaved, used at the lowest level
             by the timer interrupt, and not an intuitive pixel sequence,
             so this will likely only be used with data previously stored
             from dumpMatrix().
    @return  uint8_t* pointer to back buffer.
  */
  uint8_t *backBuffer(void);

  /*!
    @brief   Promote 3-bits R,G,B (used by earlier versions of this library)
             to the '565' color format used in Adafruit_GFX. New code should
             not use this, it's provided for backward compatibility.
    @param   r  Red value, 0-7.
    @param   g  Green value, 0-7.
    @param   b  Blue value, 0-7.
    @return  16-bit '565' color as used by Adafruit_GFX, can then be passed
             to drawing functions. Actual colors issued to matrix will be
             decimated from this, since it uses fewer bitplanes.
  */
  uint16_t Color333(uint8_t r, uint8_t g, uint8_t b);

  /*!
    @brief   Promote 4-bits R,G,B (handled by the current version of this
             library) to the '565' color format used in Adafruit_GFX.
    @param   r  Red value, 0-15.
    @param   g  Green value, 0-15.
    @param   b  Blue value, 0-15.
    @return  16-bit '565' color as used by Adafruit_GFX, can then be passed
             to drawing functions. Actual colors issued to matrix will be
             decimated from this (back to 444), since it uses fewer bitplanes.
  */
  uint16_t Color444(uint8_t r, uint8_t g, uint8_t b);

  /*!
    @brief   Decimate 8-bits R,G,B (used in a lot of existing graphics code
             in other projects and languages) to the '565' color format used
             in Adafruit_GFX.
    @param   r  Red value, 0-255.
    @param   g  Green value, 0-255.
    @param   b  Blue value, 0-255.
    @return  16-bit '565' color as used by Adafruit_GFX, can then be passed
             to drawing functions. Actual colors issued to matrix will be
             further decimated from this, since it uses fewer bitplanes.
  */
  uint16_t Color888(uint8_t r, uint8_t g, uint8_t b);

  /*!
    @brief   Decimate 8-bits R,G,B (used in a lot of existing graphics code
             in other projects and languages) to the '565' color format used
             in Adafruit_GFX, applying gamma correction if requested.
    @param   r      Red value, 0-255.
    @param   g      Green value, 0-255.
    @param   b      Blue value, 0-255.
    @param   gflag  If true, run 8-bit inputs through gamma correction table.
    @return  16-bit '565' color as used by Adafruit_GFX, can then be passed
             to drawing functions. Actual colors issued to matrix will be
             further decimated from this, since it uses fewer bitplanes.
  */
  uint16_t Color888(uint8_t r, uint8_t g, uint8_t b, boolean gflag);

  /*!
    @brief   Convert hue, saturation, value (used in some existing graphics
             code in other projects and languages) to the '565' RGB color
             format used in Adafruit_GFX, with gamma correction if requested.
    @param   hue    Hue (0 to 1535).
    @param   sat    Saturation (0 (monochrome) to 255 (full color)).
    @param   val    Value (0 (darkest) to 255 (brightest)).
    @param   gflag  If true, apply gamma correction table.
    @return  16-bit '565' color as used by Adafruit_GFX, can then be passed
             to drawing functions. Actual colors issued to matrix will be
             decimated from this, since it uses fewer bitplanes.
  */
  uint16_t ColorHSV(long hue, uint8_t sat, uint8_t val, boolean gflag);

private:
  uint8_t *matrixbuff[2];     ///< Buffer pointers for double-buffering
  uint8_t nRows;              ///< Number of rows (derived from A/B/C/D pins)
  volatile uint8_t backindex; ///< Index (0-1) of back buffer
  volatile boolean swapflag;  ///< if true, swap on next vsync

  // Init/alloc code common to both constructors:
  void init(uint8_t rows, uint8_t a, uint8_t b, uint8_t c, uint8_t clk,
            uint8_t lat, uint8_t oe, boolean dbuf, uint8_t width
#if defined(ARDUINO_ARCH_SAMD) || defined(ARDUINO_ARCH_ESP32)
            ,
            uint8_t *rgbpins
#endif
  );

  uint8_t _clk;       ///< RGB clock pin number
  uint8_t _lat;       ///< RGB latch pin number
  uint8_t _oe;        ///< Output enable pin number
  uint8_t _a;         ///< Address/row-select A pin number
  uint8_t _b;         ///< Address/row-select B pin number
  uint8_t _c;         ///< Address/row-select C pin number
  uint8_t _d;         ///< Address/row-select D pin number
  PortType clkmask;   ///< RGB clock pin bitmask
  PortType latmask;   ///< RGB latch pin bitmask
  PortType oemask;    ///< Output enable pin bitmask
  PortType addramask; ///< Address/row-select A pin bitmask
  PortType addrbmask; ///< Address/row-select B pin bitmask
  PortType addrcmask; ///< Address/row-select C pin bitmask
  PortType addrdmask; ///< Address/row-select D pin bitmask
  // PORT register pointers (CLKPORT is hardcoded on AVR)
  volatile PortType *latport;   ///< RGB latch PORT register
  volatile PortType *oeport;    ///< Output enable PORT register
  volatile PortType *addraport; ///< Address/row-select A PORT register
  volatile PortType *addrbport; ///< Address/row-select B PORT register
  volatile PortType *addrcport; ///< Address/row-select C PORT register
  volatile PortType *addrdport; ///< Address/row-select D PORT register

#if defined(ARDUINO_ARCH_SAMD) || defined(ARDUINO_ARCH_ESP32)
  uint8_t rgbpins[6];           ///< Pin numbers for 2x R,G,B bits
  volatile PortType *outsetreg; ///< RGB PORT bit set register
  volatile PortType *outclrreg; ///< RGB PORT bit clear register
  PortType rgbclkmask;          ///< Mask of all RGB bits + CLK
  PortType expand[256];         ///< 6-to-32 bit converter table
#endif

  volatile uint8_t row;      ///< Row counter for interrupt handler
  volatile uint8_t plane;    ///< Bitplane counter for interrupt handler
  volatile uint8_t *buffptr; ///< Current RGB pointer for interrupt handler
};

#endif // RGBMATRIXPANEL_H
