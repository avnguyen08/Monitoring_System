/*
  Solid State Systems
  LED Timer
  Aaron Nguyen
  Summary: Data that takes ADS1015 Reading then produces it to 7 segment display. Adafruit Feather Board and 7 Segment Wing
  
*/
#include <Adafruit_GFX.h>
#include "Adafruit_LEDBackpack.h"
#include <Adafruit_ADS1X15.h>
Adafruit_ADS1115 ads;
Adafruit_7segment matrix = Adafruit_7segment();
float matrixVal = 0.00;
float sensorVal = 0;
float startTime;
float displayTime;
float endTime;
float r = 0;
  float max_v = 0;
// get maximum reading value
float get_max() {
  startTime = millis();
  r = 0;
  max_v = 0;
  for(int i = 0; i < 100; i++) {
     r = abs(ads.readADC_Differential_0_1());  // read from ADS1x15 0 and 1 portSerial.print("Variable_1:");
    if(max_v < r) max_v = r;
  // Serial.print("Voltage_Raw:");
  // Serial.print(r);
  // Serial.print(",");
  // Serial.print("Voltage_Peak:");
  // Serial.println(max_v);
  }
  endTime = millis();
  return max_v;

}
void setup() {
  matrix.begin(0x70);
  ads.setGain(GAIN_TWO);  // 2x gain   +/- 2.048V  1 bit = 1mV | // 4x gain   +/- 1.024V  1 bit = 0.5mV
  ads.setDataRate(RATE_ADS1115_860SPS);
  Serial.begin(115200);
  delay(500);
  //Serial.println("ADC Range: +/- 2.048V  1 bit = 1mV");
  ads.begin();
  matrix.print(8888);
  matrix.writeDisplay();
}
void loop() {
  sensorVal = get_max(); //Voltage 

  Serial.print("Time:");
  Serial.println((endTime-startTime)/1000);
  Serial.print("Hertz:");
  Serial.println(1/((endTime-startTime)/1000));
  // Serial.print("Voltage_Raw:");
  // Serial.println(sensorVal);
    matrix.print(sensorVal);
    matrix.writeDisplay();
  
}