/*
  Solid State Systems
  LED Timer
  David Nguyen
  4/12/2023 3.9 MOhm pull-down resistor between ground and positive lead (A0 Pin on the ADC)
  Parts: Adafruit Feather ESP32-S3, ADS1015 ADC [Mouser No: 485-1083, Manufactoror: Adafruit] - A0 Pin (Positive Lead) & A1 Pin (Negative Lead), 4x 7-Segment LED RED HT16K33 FeatherWing (Address used 0x70), 1x Button, 3.9 MOhm between A0 (Positive Lead) and Ground.
  These displays use I2C to communicate, 2 pins are required to 
  interface. There are multiple selectable I2C addresses. For backpacks
  with 2 Address Select pins: 0x70, 0x71, 0x72 or 0x73. For backpacks
  with 3 Address Select pins: 0x70 thru 0x77
  Button Displays seconds when pressed
  Window Filter used. Dropping negative reads and the last 4 values outputted due to debouncing when removing the positive lead.

*/
#include <Adafruit_GFX.h>
#include "Adafruit_LEDBackpack.h"
#include <Adafruit_ADS1X15.h>
#include <ArduinoQueue.h>
Adafruit_ADS1015 ads1015;
Adafruit_7segment matrix = Adafruit_7segment();

bool stateEmpty = true;  // Returns true if the queue is empty, false otherwise
bool stateFull = false;  // Returns true if the queue is full, false otherwise
float startTime;
float endTime;
float displayTime = 0;
int16_t sensorVal;
int16_t matrixVal = 0;
const int SMOOTHING_WINDOW_SIZE = 10;  // 10 samples
int _samples[SMOOTHING_WINDOW_SIZE];   // the readings from the analog input
int _curReadIndex = 0;                 // the index of the current reading
int _sampleTotal = 0;                  // the running total
int _sampleAvg = 2;                    // the average
int sens = 2;                          // input sensitivity
float multiplier = 1.0;                //Adjusts for pull down resistor changing the voltage value
int amp = 20;                          //the multiplier to convert mV to Amps
bool startOn = 0;
const int SecondButton = 5;
const int ConfigButton1 = 6;
const int ConfigButton2 = 9;
int ConfigState1 = 1;
int ConfigState2 = 1;
const int prev_ith_val = 4;                    //nth last value
ArduinoQueue<int16_t> intQueue(prev_ith_val);  //Queue that is used to take the nth last value from the average value readings

void setup() {
  pinMode(SecondButton, INPUT_PULLUP);
  pinMode(ConfigButton1, INPUT_PULLDOWN);
  pinMode(ConfigButton2, INPUT_PULLDOWN);
  matrix.begin(0x70);
  ads1015.setGain(GAIN_TWO);  // 2x gain   +/- 2.048V  1 bit = 1mV | // 4x gain   +/- 1.024V  1 bit = 0.5mV
  Serial.begin(115200);
  delay(500);
  //Serial.println("ADC Range: +/- 2.048V  1 bit = 1mV");
  ads1015.begin();
  matrix.print(8888);
  matrix.writeDisplay();
  delay(3000);
  //D6 and D9 input pins are used respectively. 0b00 means 2, 0b01 means 10, 0b10 means 20, 0b11 means 40.
  ConfigState1 = digitalRead(ConfigButton1);
  ConfigState2 = digitalRead(ConfigButton2);
  if (ConfigState1 == 0 && ConfigState2 == 0)
    sens = 2;
  else if (ConfigState1 == 0 && ConfigState2 == 1)
    sens = 40;
  else if (ConfigState1 == 1 && ConfigState2 == 0)
    sens = 80;
  else if (ConfigState1 == 1 && ConfigState2 == 1)
    sens = 200;
  else
    sens = 60;

  // Serial.print("Sensitivity is: ");
  // Serial.println(sens);
  // Serial.print("Config1 is:  ");
  // Serial.println(ConfigState1);
  // Serial.print("Config2 is:  ");
  // Serial.println(ConfigState2);
}
void loop() {
  int pushButtonState = digitalRead(SecondButton);
  startOn = 1;
  sensorVal = ads1015.readADC_Differential_0_1();
  while (sensorVal > sens) {
    // read the sensor value
    sensorVal = ads1015.readADC_Differential_0_1();
    if (sensorVal > 0) {
      // subtract the last reading from our sliding window
      _sampleTotal = _sampleTotal - _samples[_curReadIndex];

      // add in current reading to our sliding window
      _samples[_curReadIndex] = sensorVal;

      // add the reading to the total
      _sampleTotal = _sampleTotal + _samples[_curReadIndex];

      // calculate the average:
      _sampleAvg = _sampleTotal / SMOOTHING_WINDOW_SIZE;

      stateEmpty = intQueue.isEmpty();  // Returns true if the queue is empty, false otherwise
      stateFull = intQueue.isFull();    // Returns true if the queue is full, false otherwise

      while (!stateFull) {
        intQueue.enqueue(_sampleAvg);
        stateFull = intQueue.isFull();
      }
      intQueue.dequeue();

      intQueue.enqueue(_sampleAvg);
      intQueue.enqueue(_sampleAvg);
      // advance to the next position in the array
      _curReadIndex = _curReadIndex + 1;

      // if we're at the end of the array...
      if (_curReadIndex >= SMOOTHING_WINDOW_SIZE) {
        // ...wrap around to the beginning:
        _curReadIndex = 0;
      }
    }
    if (startOn) {
      startTime = millis();
      startOn = 0;
    }

    endTime = millis();
    displayTime = (endTime - startTime) / 1000;
    // matrix.print(displayTime, 2);
    // matrix.writeDisplay();
    
    // Serial.print("displayTime:");
    // Serial.println(displayTime);

    // Serial.print("ADC_Raw:");
    // Serial.print(sensorVal);
    // Serial.print(", ");
    // Serial.print("WindowFilter:");
    // Serial.println(_sampleAvg);
  }
  if (pushButtonState == LOW) {    //Check to see if button is depressed
    matrix.print(displayTime, 2);  //display time with 2 decimal places
    matrix.writeDisplay();
    Serial.print("displayTimeButton:");
    Serial.println(displayTime);
  } else {
    matrixVal = intQueue.getHead();  //get the nth average filter value
    if ((matrixVal * multiplier * amp) > 9999) {
      matrix.println("EROR");
      matrix.writeDisplay();
    } else {
      matrix.print(matrixVal * multiplier * amp, 0);
      matrix.writeDisplay();
      Serial.print("matrixVal:");
      Serial.println(matrixVal);
    }
  }
}
