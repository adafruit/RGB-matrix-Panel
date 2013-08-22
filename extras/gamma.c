// THIS IS NOT ARDUINO CODE -- DON'T INCLUDE IN YOUR SKETCH.  It's a
// command-line tool that outputs a gamma correction table to stdout;
// redirect or copy and paste the results into header file for the
// RGBmatrixPanel library code.
// Optional 1 parameter: bit depth (default=4, for 16 output levels).

#include <stdio.h>
#include <math.h>

#define GAMMA 2.5

int planes = 4;

int main(int argc, char *argv[])
{
	int i, maxval;

	if(argc > 1) planes = atoi(argv[1]);

	maxval = (1 << planes) - 1;

	(void)printf(
	  "#ifndef _GAMMA_H_\n"
	  "#define _GAMMA_H_\n\n"
	  "#include <avr/pgmspace.h>\n\n"
	  "static const uint8_t PROGMEM gamma[] = {\n  ");

	for(i=0; i<256; i++) {
		(void)printf("0x%02x",(int)(pow((float)i / 255.0, GAMMA) *
		  (float)maxval + 0.5));
		if(i < 255) (void)printf(((i & 7) == 7) ? ",\n  " : ",");
	}

	(void)puts(
	  "\n};\n\n"
	  "#endif // _GAMMA_H_");

	return 0;
}
