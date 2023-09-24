#ifndef SQUARE_MAX_221_AT_16384_1024INT8_H_
#define SQUARE_MAX_221_AT_16384_1024INT8_H_

#if ARDUINO >= 100
 #include "Arduino.h"
#else
 #include "WProgram.h"
#endif
#include "mozzi_pgmspace.h"


#define SQUARE_MAX_221_AT_16384_1024_NUM_CELLS 1024
#define SQUARE_MAX_221_AT_16384_1024_SAMPLERATE 1024

CONSTTABLE_STORAGE(int8_t) SQUARE_MAX_221_AT_16384_1024_DATA [] = 
{
0, 16, 31, 46, 61, 74, 86, 97, 106, 113, 119, 123, 126, 127, 127, 126, 124, 121, 118, 115, 111, 108, 105, 102, 100, 98, 97, 97, 97, 98, 100, 101, 103, 
105, 107, 109, 111, 113, 114, 115, 115, 115, 114, 114, 113, 111, 110, 109, 107, 106, 104, 103, 103, 102, 102, 102, 103, 104, 104, 106, 107, 108, 109, 110, 111, 
112, 112, 112, 112, 112, 111, 111, 110, 109, 108, 107, 106, 105, 105, 104, 104, 104, 104, 104, 105, 106, 106, 107, 108, 109, 109, 110, 111, 111, 111, 111, 111, 
110, 110, 109, 109, 108, 107, 106, 106, 105, 105, 105, 105, 105, 105, 106, 106, 107, 107, 108, 109, 109, 110, 110, 110, 110, 110, 110, 110, 109, 109, 108, 108, 
107, 107, 106, 106, 105, 105, 105, 105, 106, 106, 106, 107, 107, 108, 109, 109, 109, 110, 110, 110, 110, 110, 110, 109, 109, 108, 108, 107, 107, 106, 106, 106, 
106, 106, 106, 106, 106, 107, 107, 107, 108, 108, 109, 109, 110, 110, 110, 110, 110, 109, 109, 109, 108, 108, 107, 107, 106, 106, 106, 106, 106, 106, 106, 106, 
107, 107, 108, 108, 108, 109, 109, 109, 110, 110, 110, 109, 109, 109, 109, 108, 108, 107, 107, 106, 106, 106, 106, 106, 106, 106, 106, 107, 107, 108, 108, 108, 
109, 109, 109, 110, 110, 110, 109, 109, 109, 108, 108, 108, 107, 107, 107, 106, 106, 106, 106, 106, 106, 106, 107, 107, 108, 108, 108, 109, 109, 109, 110, 110, 
110, 109, 109, 109, 108, 108, 108, 107, 107, 106, 106, 106, 106, 106, 106, 106, 107, 107, 107, 108, 108, 108, 109, 109, 109, 110, 110, 110, 109, 109, 109, 108, 
108, 108, 107, 107, 106, 106, 106, 106, 106, 106, 106, 106, 107, 107, 108, 108, 109, 109, 109, 109, 110, 110, 110, 109, 109, 109, 108, 108, 108, 107, 107, 106, 
106, 106, 106, 106, 106, 106, 106, 107, 107, 108, 108, 109, 109, 109, 110, 110, 110, 110, 110, 109, 109, 108, 108, 107, 107, 107, 106, 106, 106, 106, 106, 106, 
106, 106, 107, 107, 108, 108, 109, 109, 110, 110, 110, 110, 110, 110, 109, 109, 109, 108, 107, 107, 106, 106, 106, 105, 105, 105, 105, 106, 106, 107, 107, 108, 
108, 109, 109, 110, 110, 110, 110, 110, 110, 110, 109, 109, 108, 107, 107, 106, 106, 105, 105, 105, 105, 105, 105, 106, 106, 107, 108, 109, 109, 110, 110, 111, 
111, 111, 111, 111, 110, 109, 109, 108, 107, 106, 106, 105, 104, 104, 104, 104, 104, 105, 105, 106, 107, 108, 109, 110, 111, 111, 112, 112, 112, 112, 112, 111, 
110, 109, 108, 107, 106, 104, 104, 103, 102, 102, 102, 103, 103, 104, 106, 107, 109, 110, 111, 113, 114, 114, 115, 115, 115, 114, 113, 111, 109, 107, 105, 103, 
101, 100, 98, 97, 97, 97, 98, 100, 102, 105, 108, 111, 115, 118, 121, 124, 126, 127, 127, 126, 123, 119, 113, 106, 97, 86, 74, 61, 46, 31, 16, 0, 
-16, -31, -46, -61, -74, -86, -97, -106, -113, -119, -123, -126, -127, -127, -126, -124, -121, -118, -115, -111, -108, -105, -102, -100, -98, -97, -97, -97, -98, -100, -101, -103, 
-105, -107, -109, -111, -113, -114, -115, -115, -115, -114, -114, -113, -111, -110, -109, -107, -106, -104, -103, -103, -102, -102, -102, -103, -104, -104, -106, -107, -108, -109, -110, -111, 
-112, -112, -112, -112, -112, -111, -111, -110, -109, -108, -107, -106, -105, -105, -104, -104, -104, -104, -104, -105, -106, -106, -107, -108, -109, -109, -110, -111, -111, -111, -111, -111, 
-110, -110, -109, -109, -108, -107, -106, -106, -105, -105, -105, -105, -105, -105, -106, -106, -107, -107, -108, -109, -109, -110, -110, -110, -110, -110, -110, -110, -109, -109, -108, -108, 
-107, -107, -106, -106, -105, -105, -105, -105, -106, -106, -106, -107, -107, -108, -109, -109, -109, -110, -110, -110, -110, -110, -110, -109, -109, -108, -108, -107, -107, -106, -106, -106, 
-106, -106, -106, -106, -106, -107, -107, -107, -108, -108, -109, -109, -110, -110, -110, -110, -110, -109, -109, -109, -108, -108, -107, -107, -106, -106, -106, -106, -106, -106, -106, -106, 
-107, -107, -108, -108, -108, -109, -109, -109, -110, -110, -110, -109, -109, -109, -109, -108, -108, -107, -107, -106, -106, -106, -106, -106, -106, -106, -106, -107, -107, -108, -108, -108, 
-109, -109, -109, -110, -110, -110, -109, -109, -109, -108, -108, -108, -107, -107, -107, -106, -106, -106, -106, -106, -106, -106, -107, -107, -108, -108, -108, -109, -109, -109, -110, -110, 
-110, -109, -109, -109, -108, -108, -108, -107, -107, -106, -106, -106, -106, -106, -106, -106, -107, -107, -107, -108, -108, -108, -109, -109, -109, -110, -110, -110, -109, -109, -109, -108, 
-108, -108, -107, -107, -106, -106, -106, -106, -106, -106, -106, -106, -107, -107, -108, -108, -109, -109, -109, -109, -110, -110, -110, -109, -109, -109, -108, -108, -108, -107, -107, -106, 
-106, -106, -106, -106, -106, -106, -106, -107, -107, -108, -108, -109, -109, -109, -110, -110, -110, -110, -110, -109, -109, -108, -108, -107, -107, -107, -106, -106, -106, -106, -106, -106, 
-106, -106, -107, -107, -108, -108, -109, -109, -110, -110, -110, -110, -110, -110, -109, -109, -109, -108, -107, -107, -106, -106, -106, -105, -105, -105, -105, -106, -106, -107, -107, -108, 
-108, -109, -109, -110, -110, -110, -110, -110, -110, -110, -109, -109, -108, -107, -107, -106, -106, -105, -105, -105, -105, -105, -105, -106, -106, -107, -108, -109, -109, -110, -110, -111, 
-111, -111, -111, -111, -110, -109, -109, -108, -107, -106, -106, -105, -104, -104, -104, -104, -104, -105, -105, -106, -107, -108, -109, -110, -111, -111, -112, -112, -112, -112, -112, -111, 
-110, -109, -108, -107, -106, -104, -104, -103, -102, -102, -102, -103, -103, -104, -106, -107, -109, -110, -111, -113, -114, -114, -115, -115, -115, -114, -113, -111, -109, -107, -105, -103, 
-101, -100, -98, -97, -97, -97, -98, -100, -102, -105, -108, -111, -115, -118, -121, -124, -126, -127, -127, -126, -123, -119, -113, -106, -97, -86, -74, -61, -46, -31, -16, 
 };

#endif