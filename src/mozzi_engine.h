

#include <MozziGuts.h>
#include <Oscil.h> // oscillator template
#include <tables/sin2048_int8.h> // sine table for oscillator
#include <RollingAverage.h>
#include <ControlDelay.h>

#define INPUT_PIN 0 // analog control input
#define CONTROL_RATE 64

unsigned int echo_cells_1 = 32;
unsigned int echo_cells_2 = 60;
unsigned int echo_cells_3 = 127;

ControlDelay <128, int> kDelay; // 2seconds

// oscils to compare bumpy to averaged control input
Oscil <SIN2048_NUM_CELLS, AUDIO_RATE> aSin0(SIN2048_DATA);
Oscil <SIN2048_NUM_CELLS, AUDIO_RATE> aSin1(SIN2048_DATA);
Oscil <SIN2048_NUM_CELLS, AUDIO_RATE> aSin2(SIN2048_DATA);
Oscil <SIN2048_NUM_CELLS, AUDIO_RATE> aSin3(SIN2048_DATA);

// use: RollingAverage <number_type, how_many_to_average> myThing
RollingAverage <int, 32> kAverage; // how_many_to_average has to be power of 2
// int averaged;

void setupMozzi(){
  kDelay.set(echo_cells_1);
  startMozzi();
}


void updateControl(){
  int bumpy_input = mozziAnalogRead(INPUT_PIN);
  // averaged = kAverage.next(bumpy_input);
  aSin0.setFreq(averaged[0]*8);
  aSin1.setFreq(averaged[2]*12);

  aSin2.setFreq(kDelay.next(averaged[1]*8));
  aSin3.setFreq(kDelay.read(averaged[3]*8));
}


AudioOutput_t updateAudio(){
  return MonoOutput::fromAlmostNBit(12,
    3*((int)aSin0.next()+aSin1.next()+(aSin2.next()>>1)
    +(aSin3.next()>>2))
  );
}



