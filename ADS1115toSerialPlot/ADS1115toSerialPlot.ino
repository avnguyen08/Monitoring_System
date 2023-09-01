/*
  Solid State Systems
  LED Timer
  Aaron Nguyen
  Summary: Data that takes ADS1015 Reading then produces it to 7 segment display. Adafruit Feather Board and 7 Segment Wing
  
*/
#include <Adafruit_GFX.h>
#include "Adafruit_LEDBackpack.h"
#include <Adafruit_ADS1X15.h>
// This is required on ESP32 to put the ISR in IRAM. Define as
// empty for other platforms. Be careful - other platforms may have
// other requirements.
#ifndef IRAM_ATTR
#define IRAM_ATTR
#endif
Adafruit_ADS1015 ads;
float sensorVal = 0;
void setup() {
  ads.setGain(GAIN_TWO);  // 2x gain   +/- 2.048V  1 bit = 1mV | // 4x gain   +/- 1.024V  1 bit = 0.5mV
  if (!ads.begin()) {
    Serial.println("Failed to initialize ADS.");
    while (1)
      ;
  }
  Serial.begin(115200);
  delay(500);
  ads.startADCReading(ADS1X15_REG_CONFIG_MUX_DIFF_0_1, /*continuous=*/true);
  ads.begin();
}
void loop() {
  sensorVal = ads.getLastConversionResults();

  Serial.print("SensorValue: ");
  Serial.println(sensorVal);
  
}