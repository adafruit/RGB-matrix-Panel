This is an Arduino library for our 16x32 and 32x32 RGB LED matrix panels

Pick one up at http://www.adafruit.com/products/420 & http://www.adafruit.com/products/607 !


Written by Limor Fried/Ladyada & Phil Burgess/PaintYourDragon for Adafruit Industries.  
BSD license, all text above must be included in any redistribution

To download. click the ZIP (download) button, rename the uncompressed folder RGBLEDMatrix. 
Check that the RGBmatrixPanel folder contains RGBmatrixPanel.cpp and RGBmatrixPanel.h

Place the RGBmatrixPanel library folder your arduinosketchfolder/libraries/ folder. 
You may need to create the libraries subfolder if its your first library. 
Restart the IDE.

We also have a great tutorial on Arduino library installation at:
http://learn.adafruit.com/adafruit-all-about-arduino-libraries-install-use

If you need support for RGB888 (24bpp) and need to/can run on Teensy 3.1/3.2/3.5/3.6 or 
ESP32 chips (not supported by RGB-matrix-Panel), please look at
https://github.com/marcmerlin/SmartMatrix_GFX which offers a GFX compatibility layer on
top of https://github.com/pixelmatix/SmartMatrix

If you have RGBPanels of up to at least 256x256 or unsupported panels (AB or AC, or using FM6126A), and you'd 
like to use them, another driver is needed.  
You can use https://github.com/hzeller/rpi-rgb-led-matrix/ , however it requires you to use a raspberry pi, which is a problem if your current code is using Adafruit::GFX.

Thankfully, there is now a solution from Marc MERLIN:
http://marc.merlins.org/perso/arduino/post_2020-01-01_Running-FastLED_-Adafruit_GFX_-and-LEDMatrix-code-on-High-Resolution-RGBPanels-with-a-Raspberry-Pi.html explains the new solution in https://github.com/marcmerlin/ArduinoOnPc-FastLED-GFX-LEDMatrix  
This solution allows you to build arduino code so that it works on linux and uses these layers:
- https://github.com/marcmerlin/ArduinoOnPc-FastLED-GFX-LEDMatrix
- https://github.com/marcmerlin/Framebuffer_GFX is the base arduino framebuffer that also supports more 2D arduino code in addition to Adafruit::GFX
- https://github.com/marcmerlin/FastLED_RPIRGBPanel_GFX is the driver that bridges that framebuffer and the APIs it supports (FastLED, Adafruit::GFX, and LEDMatrix), with rpi-rgb-led-matrix for display
