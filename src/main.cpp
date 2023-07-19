#include <AT42QT2120.h>
#include <Wire.h>
#include <SimpleKalmanFilter.h>


const long BAUD = 115200;
const int RESET_DELAY = 2000;
const int CALIBRATION_LOOP_DELAY = 50;

const int CHANGE_PIN = 23;

AT42QT2120 touch_sensor(Wire,CHANGE_PIN);
SimpleKalmanFilter simpleKalmanFilter(100.0, 100.0, 0.1);

float minSignal = 40000;
float maxSignal = 0;

long fmap(float x, float in_min, float in_max, float out_min, float out_max) {
    const float run = in_max - in_min;
    if(run == 0){
        log_e("map(): Invalid input range, min == max");
        return -1; // AVR returns -1, SAM returns 0
    }
    const float rise = out_max - out_min;
    const float delta = x - in_min;
    return (delta * rise) / run + out_min;
}

void setup() {
  Serial.begin(BAUD);

  touch_sensor.begin();

  //Serial.println("reseting...");
  touch_sensor.reset();
  delay(RESET_DELAY);

 // Serial.println("triggerCalibration");
    //  touch_sensor.setKeyDetectThreshold(11, 5);

  touch_sensor.triggerCalibration();
  delay(CALIBRATION_LOOP_DELAY);
  while (touch_sensor.calibrating()){
   // Serial.println("calibrating...");
    delay(CALIBRATION_LOOP_DELAY);
  }
  //Serial.println("finished calibrating");
  AT42QT2120::KeyPulseScale key_pulse_scale;
  uint8_t pulse = 0x8;
  uint8_t scale = 0x04;
  key_pulse_scale.pulse = pulse;
  key_pulse_scale.scale = scale;

  touch_sensor.setKeyPulseScale(11, key_pulse_scale);
  touch_sensor.setKeyPulseScale(10, key_pulse_scale);

  //touch_sensor.setDetectionIntegrator()
  //Serial.println("waiting for touch...");
  //delay(3000);
}

void loop() {
  float signal10 = touch_sensor.getKeySignal(10);
  float signal11 = touch_sensor.getKeySignal(11);

  minSignal = 9400.00; // min(minSignal, signal);
  maxSignal = 14000.00; //max(maxSignal, signal);
  // minSignal+=1;
  // maxSignal-=1;

  float smoothedSignal10 = simpleKalmanFilter.updateEstimate(signal10);
  float smoothedSignal11 = simpleKalmanFilter.updateEstimate(signal11);
  // Serial.print("getKeySignal(");
  // Serial.print(11);
  // Serial.print("): ");
  // Serial.print(minSignal);
  // Serial.print(" ");
  //   Serial.print(maxSignal);

  // // Serial.print(" ");
  // //   Serial.print(signal);
  // Serial.print(">10:");
  // Serial.print(signal10);//|xy
  // Serial.print(":");
  // Serial.println(signal11);//|xy
  // // Serial.println("|xy");

  Serial.print(">10:");
  Serial.println(signal10);//fmap(signal, minSignal, maxSignal, 0, 100.0));
  Serial.print(">11:");
  Serial.println(signal11);//fmap(signal, minSignal, maxSignal, 0, 100.0));
  
  Serial.print(">S10:");
  Serial.println(smoothedSignal10);//fmap(signal, minSignal, maxSignal, 0, 100.0));
  Serial.print(">S11:");
  Serial.println(smoothedSignal11);//fmap(signal, minSignal, maxSignal, 0, 100.0));
  delay(20);
}