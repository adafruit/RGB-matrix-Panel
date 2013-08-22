// THIS IS NOT ARDUINO CODE -- DON'T INCLUDE IN YOUR SKETCH.  It's a
// command-line tool that outputs an 8-bit sine table to stdout;
// redirect or copy and paste the results into header file for use
// with sketches.

#include <stdio.h>
#include <math.h>

int main(int argc, char *argv[]) {
	int i;

	(void)printf("static const int8_t PROGMEM sinetab[256] = {\n  ");

	for(i=0; i<256; i++) {
		(void)printf("%4d",
		  (int)(sin((double)i / 128.0 * M_PI) * 127.5 - 0.5));
		if(i < 255) (void)printf(((i & 7) == 7) ? ",\n  " : ",");
	}

	(void)puts("\n};");

	return 0;
}

