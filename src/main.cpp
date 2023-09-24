
#include "main.h"

#include <AT42QT2120.h>
#include <Wire.h>
#include <SimpleKalmanFilter.h>

#include "mozzi_engine.h"

TaskHandle_t Task1;

const long BAUD = 115200;
const int RESET_DELAY = 2000;
const int CALIBRATION_LOOP_DELAY = 50;

const int CHANGE_PIN = 23;

AT42QT2120 touch_sensor(Wire,CHANGE_PIN);

SimpleKalmanFilter* kalman = (SimpleKalmanFilter*)malloc(sizeof(SimpleKalmanFilter) * 12);

uint16_t values[12] = {0};
uint16_t maxValues[12] = {0};
uint16_t minValues[12] = {1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000};


float fmap(float x, float in_min, float in_max, float out_min, float out_max) {
  const float run = in_max - in_min;
  if(run == 0){
      return -1; // AVR returns -1, SAM returns 0
  }
  const float rise = out_max - out_min;
  const float delta = x - in_min;
  return (delta * rise) / run + out_min;
}

float fclamp(float x, float in_min, float in_max) {
  return (x < in_min) ? in_min : (in_max < x) ? in_max : x;
}


void loop2( void * parameter ) {
  for (;;) {
    // vTaskDelay(10);
    vTaskDelay(pdMS_TO_TICKS(20));
    
    for (int i = 0; i<12; i++) {
      values[i] = touch_sensor.getKeySignal(i);
   
      if (millis()>3000) {
        digitalWrite(LED_BUILTIN, millis()%1000 < 500);
        if (values[i]<minValues[i]) {
          minValues[i] -=1; //values[i];
        }
        //minValues[i] += 1.0;

        if (values[i]>maxValues[i]) {
          maxValues[i] +=1; //values[i];
        }
      // maxValues[i] -= 1.0;
      }
      averaged[i] =  fclamp(fmap(kalman[i].updateEstimate(values[i]),minValues[i], maxValues[i], 0, 100.0 ), 0.0f, 100.0f);
    }

    // averaged =  fclamp(fmap(kalman[0].updateEstimate(values[0]),minValues[0], maxValues[0], 0, 100.0 ), 0.0f, 100.0f);
    for (int i = 0; i<12; i++) {
    Serial.print(">v");
    Serial.print(i);
    Serial.print(':');
    Serial.println(averaged[i]);
   }
  }
}

void setup() {

  Serial.begin(BAUD);
  touch_sensor.begin();

  touch_sensor.reset();
  delay(RESET_DELAY);

 setupMozzi();
 // 0x0 / 0x00
// 0x2 / 0x01
// 0x4 / 0x02
// 0x6 / 0x03
// 0x8 / 0x04
// 0x0A/ 0x05
// 0x0C/ 0x06
// 0x0E/ 0x07

  AT42QT2120::KeyPulseScale key_pulse_scale;
  uint8_t pulse = 0x0;
  uint8_t scale = 0x00;
  key_pulse_scale.pulse = pulse;
  key_pulse_scale.scale = scale;

  for (int i = 0; i<12; i++) {
    touch_sensor.setKeyPulseScale(i, key_pulse_scale);
  }

  touch_sensor.triggerCalibration();
  delay(CALIBRATION_LOOP_DELAY);
  while (touch_sensor.calibrating()){
   // Serial.println("calibrating...");
    delay(CALIBRATION_LOOP_DELAY);
  }

  for (int i = 0; i<12; i++) {
    kalman[i] = SimpleKalmanFilter(20.0, 20.0, 0.1);
  }
  pinMode(LED_BUILTIN, OUTPUT);
  // touch_sensor.setKeyPulseScale(10, key_pulse_scale);
  // touch_sensor.setKeyPulseScale(9, key_pulse_scale);

  //touch_sensor.setDetectionIntegrator()
  //Serial.println("waiting for touch...");


   xTaskCreatePinnedToCore(
    loop2,          // name of the task function
    "loop2",  // name of the task
    4000,           // memory assigned for the task
    NULL,           // parameter to pass if any
    1,              // priority of task, starting from 0(Highestpriority) *IMPORTANT*( if set to 1 and there is no activity in your 2nd loop, it will reset the esp32)
    &Task1,         // Reference name of taskHandle variable
    0
  );

  delay(3000);
}


void loop() { 
  audioHook(); // required here
}
