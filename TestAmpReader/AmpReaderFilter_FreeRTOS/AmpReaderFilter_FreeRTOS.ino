/*
  Solid State Systems
  LED Timer
  David Nguyen
  3/22/2023 3.9 MOhm pull-down resistor between ground and positive lead (A0 Pin on the ADC)
  Parts: Adafruit Feather ESP32-S3, ADS1015 ADC [Mouser No: 485-1083, Manufactoror: Adafruit] - A0 Pin (Positive Lead) & A1 Pin (Negative Lead), 4x 7-Segment LED RED HT16K33 FeatherWing (Address used 0x70), 1x Button, 3.9 MOhm between A0 (Positive Lead) and Ground.
  These displays use I2C to communicate, 2 pins are required to 
  interface. There are multiple selectable I2C addresses. For backpacks
  with 2 Address Select pins: 0x70, 0x71, 0x72 or 0x73. For backpacks
  with 3 Address Select pins: 0x70 thru 0x77
  Button Displays seconds when pressed
  Window Filter used. Dropping negative reads and the last 4 values outputted due to debouncing when removing the positive lead.

  4/12/2023 Update: Updated code to include different input threshhold configurations. There are 4 different modes that can be changed by changing pins D6 and D9 on the feather board. 0b00 means 2, 0b01 means 40, 0b10 means 80, 0b11 means 200.
  Error  with different power methods. Only method to produce successful results is through the usb ports on the main computer. Tried laptop, powerbank, and powersupply, all failed. Suspected to be a grounding issue.
  4/17/2023 Update: Replaced ADS1015, increased code filtering, and connected ground to earth ground. These seemed to solve the problem.   
*/
#include <Adafruit_GFX.h>
#include "Adafruit_LEDBackpack.h"
#include <Adafruit_ADS1X15.h>
#include <CircularBuffer.h>
#include <ArduinoQueue.h>
//Debugging mode statements. Helps differentiate serial lines from serial lines only used for debugging.
#define DEBUG 1
//if in debugging mode then make each of these debug functions the same as a serial print.
#if DEBUG
#define debug(x) Serial.print(x)
#define debugln(x) Serial.println(x)
//all debug lines are equal to nothing
#else
#define debug(x)
#define debugln(x)
#endif

#define D10 10  //LED Display for Amp Display
#define D11 11  //LED Display for Timer Display
Adafruit_ADS1015 ads;
// Adafruit_7segment matrix = Adafruit_7segment();
float sensorVal; //sensor value
const int SENS_SIZE = 10;
CircularBuffer<int, SENS_SIZE> sensBuffer; //Buffer that detects the amount of 
void read_adc(void* parameter){
  while(1){
    sensorVal = ads.readADC_Differential_0_1();
    sensBuffer.push(sensorVal);
  }
}

void setup() {
  matrix.begin(0x70);
  ads.setGain(GAIN_TWO);  // 2x gain   +/- 2.048V  1 bit = 1mV | // 4x gain   +/- 1.024V  1 bit = 0.5mV
  Serial.begin(115200);
  delay(500);
  // ADC Reading
  xTaskCreatePinnedToCore(  // Use xTaskCreate() in vanilla FreeRTOS
    read_adc,            // Function to be called
    "read adc",             // Name of task
    1024,                   // Stack size (bytes in ESP32, words in FreeRTOS)
    NULL,                   // Parameter to pass to function
    1,                      // Task priority (0 to configMAX_PRIORITIES - 1)
    NULL,                   // Task handle
    1);               // Run on one core (ESP32 only)
}
void loop() {
  // int pushButtonState = digitalRead(SecondButton);
  // startOn = 1;
  // sensorVal = ads.readADC_Differential_0_1();
  // sensBuffer.push(sensorVal);
  // while (sensBuffer[0] > sens && sensBuffer[1] > sens && sensBuffer[2] > sens && sensBuffer[3] > sens && sensBuffer[4] > sens && sensBuffer[5] > sens && sensBuffer[6] > sens
  //        && sensBuffer[7] > sens && sensBuffer[8] > sens && sensBuffer[9] > sens) {
  //   flag = 0;
  //   flag2 = 1;

  //   digitalWrite(D10, 0);  //turn off led 10 (Amps)
  //   digitalWrite(D11, 0);  //turn off led 11 (Time)
  //   for (int i = 0; i < SENS_SIZE; ++i) {
  //     aveBuffer.push(sensBuffer[i]);
  //   }
  //   // read the sensor value

  //   sensorVal = ads.readADC_Differential_0_1();
  //   if (sensorVal > 2) {
  //     // subtract the last reading from our sliding window
  //     _sampleTotal = _sampleTotal - _samples[_curReadIndex];

  //     // add in current reading to our sliding window
  //     _samples[_curReadIndex] = sensorVal;

  //     // add the reading to the total
  //     _sampleTotal = _sampleTotal + _samples[_curReadIndex];

  //     // calculate the average:
  //     _sampleAvg = _sampleTotal / SMOOTHING_WINDOW_SIZE;

  //     // stateEmpty = aveBuffer.isEmpty();  // Returns true if the queue is empty, false otherwise
  //     // stateFull = aveBuffer.isFull();    // Returns true if the queue is full, false otherwise

  //     // while (!stateFull) {
  //     //   aveBuffer.push(_sampleAvg);
  //     //   stateFull = aveBuffer.isFull();
  //     // }
  //     aveBuffer.push(_sampleAvg);
  //     // advance to the next position in the array
  //     _curReadIndex = _curReadIndex + 1;

  //     // if we're at the end of the array...
  //     if (_curReadIndex >= SMOOTHING_WINDOW_SIZE) {
  //       // ...wrap around to the beginning:
  //       _curReadIndex = 0;
  //     }
  //   }
  //   if (startOn) {
  //     startTime = millis();
  //     startOn = 0;
  //   }

  //   sensorVal = ads.readADC_Differential_0_1();
  //   sensBuffer.push(sensorVal);
  //   endTime = millis();
  //   displayTime = (endTime - startTime) / 1000;
  //   matrix.print(displayTime, 2);
  //   matrix.writeDisplay();

  //   // sensorVal = ads.readADC_Differential_0_1();
  //   // matrix.print(1234, 0);
  //   // matrix.writeDisplay();

  //   Serial.print("Display Time:");
  //   Serial.println(displayTime);
  //   // Serial.print(", ");
  //   // Serial.print("WindowFilter:");
  //   // Serial.println(_sampleAvg);
  // }
  // if (startOn_dft || !flag) {
  //   startTime_dft = millis();
  //   startOn_dft = 0;
  // }

  // flag = 1;
  // for (int i = 0; i < aveBuffer.size(); ++i) {
  //   if (i < aveBuffer.size() / 10) {
  //     total_ave += aveBuffer[i];
  //   }
  //   if (i == aveBuffer.size() - 1) {
  //     matrixVal = total_ave / (aveBuffer.size());
  //     total_ave = 0;
  //   }
  // }
  // //Error LED Print
  // if ((matrixVal * multiplier * amp) > 9999) {
  //   matrix.println("EROR");
  //   matrix.writeDisplay();
  //   //Display Time LED Print
  // } else if (flag2) {
  //   if (((int)(millis() - startTime_dft) / 1000) % (int)(AmpTime + TimerTime) > (AmpTime - 1)) {
  //     matrix.print(displayTime, 2);
  //     matrix.writeDisplay();
  //     digitalWrite(D10, 0);    //turn off led 10 (Amps)
  //     digitalWrite(D11, 123);  //turn on led 11 (Time)
  //     debugln("Displaying TIme");
  //     if (((int)(millis() - startTime_dft) / 1000) % (int)(AmpTime + TimerTime) < (AmpTime - 1))
  //       startOn_dft = 1;
  //     //Amp LED Print
  //   } else {
  //     matrix.print(matrixVal * multiplier * amp, 0);
  //     matrix.writeDisplay();
  //     digitalWrite(D11, 0);    //Turn off LED 11 (Time)
  //     digitalWrite(D10, 123);  //Turn on LED 10 (Amps)
  //     debugln("Displaying Amps");
  //   }
  // }
  // //serial.print debugging values
  // // debug("sensorVal:");
  // // debug(sensorVal);
  // // debug(", ");
  // // debug("MatrixVal:");
  // // debug(matrixVal);
  // // debug(", ");
  // // debug("DisplayVal:");
  // // debug(matrixVal * multiplier * amp);
  // // debug(", ");
  // // debug("WindowFilter:");
  // // debugln(_sampleAvg);
}
