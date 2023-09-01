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
  4/24/2023 Update: Added potentiometer to adjust time, added leds to indicate which one is displaying, added pin d5 jumper pins to do absolute or regular sensor readings, 
  4/29/2023 Update: Took out abs or regular sensor reading options, default to absolute value. debug lines used to represent serial.print() so i can turn on and off all serial.prints used for debugging
*/
#include <Adafruit_GFX.h>
#include "Adafruit_LEDBackpack.h"
#include <Adafruit_ADS1X15.h>
#include <CircularBuffer.h>
#include <ArduinoQueue.h>
//Debugging mode statements. Helps differentiate serial lines from serial lines only used for debugging.
#define DEBUG 0
//if in debugging mode then make each of these debug functions the same as a serial print.
#if DEBUG
#define debug(x) Serial.print(x)
#define debugln(x) Serial.println(x)
//all debug lines are equal to nothing
#else
#define debug(x)
#define debugln(x)
#endif
#define TIME_POT A4  //Potentiometer input to calibrate display time for Time
#define AMP_POT A5   //Potentiometer input to calibrate display time for AMP
#define AMP_LED 11   //LED flash for Amp Display
#define TIME_LED 12  //LED flash for Timer Display
Adafruit_ADS1015 ads;
Adafruit_7segment matrix = Adafruit_7segment();
float startTime;
float endTime;
float latency = 0;    //80 ms latency
float startTime_dft;  //the reference time used when switching between amps and millis on the 7 segment display.
float displayTime = 0;
int16_t sensorVal = 0;
float matrixVal = 0.00;
float r = 0;                           //get_max function: absolute value voltage data stored
float max_v = 0;                       //get_max function: maximum absolute voltage data stored
const int SMOOTHING_WINDOW_SIZE = 10;  // 10 samples
int _samples[SMOOTHING_WINDOW_SIZE];   // the readings from the analog input
int _curReadIndex = 0;                 // the index of the current reading
int _sampleTotal = 0;                  // the running total
int _sampleAvg = 2;                    // the average
int sens = 8;                         // input sensitivity
float multiplier = 1;                  //Adjusts for pull down resistor changing the voltage value
int amp = 20;                          //the multiplier to convert mV to Amps
// bool startOn = 0;                      //timer start for displayTime
bool startOn_dft = 0;     //timer start for another time
bool flag = 0;            //indicator of when to stop default display
bool flag2 = 0;           //indicator of when first reading is taken
bool flag3 = 0;           //indicator of when to run the timer clock in get_max()
bool LastmsMode = false;  //Dictates the mode of the program. Mode that captures the average of the last 60ms of the voltage reading or a mode that captures the average of the entire voltage reading
float total_ave = 0;
// const int SecondButton = 5;
// const int ConfigButton1 = 6;
// const int ConfigButton2 = 9;
// int ConfigState1 = 1;
// int ConfigState2 = 1;
float AmpTime = 3;
float TimerTime = 3;
const int AVE_SIZE = 50;  //size of array of average values
const int SENS_SIZE = 10;
CircularBuffer<int16_t, AVE_SIZE> aveBuffer;  //Queue that is used to take the nth last value from the average value readings
CircularBuffer<int, SENS_SIZE> sensBuffer;

float get_max() {
  r = 0;
  max_v = 0;
  debugln("Entering max function");
  for (int i = 0; i < 100; i++) {
    r = abs(ads.readADC_Differential_0_1());  // read from ADS1x15 0 and 1 portSerial.print("Variable_1:");
    if (max_v < r) max_v = r;
    if (flag3 == true) {
      endTime = millis();
      displayTime = ((endTime - startTime) / 1000) + latency;
      debug("displayTime: ");
      debugln(displayTime);
      matrix.print(displayTime, 2);
      matrix.writeDisplay();
    }
    Serial.print("Voltage_Raw:");
    Serial.println(r);
    Serial.print(",");
    Serial.print("Voltage_Peak:");
    Serial.println(max_v);
  }

  flag3 = false;
  debug("Returning max_v");
  debugln(max_v);
  return max_v;
}

void setup() {
  // pinMode(ConfigButton1, INPUT_PULLDOWN);
  // pinMode(ConfigButton2, INPUT_PULLDOWN);
  //initialization for DIP switch multiplexer (not implemented yet)
  pinMode(5, INPUT_PULLUP);                   //D5 multiplexer input
  pinMode(6, INPUT_PULLUP);                   //D6 multiplexer input
  pinMode(9, INPUT_PULLUP);                   //D9 multiplexer input
  pinMode(10, INPUT_PULLUP);                  //D10 multiplexer input
  pinMode(AMP_LED, OUTPUT);                   //LED Output
  pinMode(TIME_LED, OUTPUT);                  //LED Output
  AmpTime = analogRead(AMP_POT);              // potentiometer output read on respective analog pin
  TimerTime = analogRead(TIME_POT);           // potentiometer output read on respective analog pin
  AmpTime = map(AmpTime, 4095, 0, 1, 9);      //maps Amptime Display to potentiometer set level.
  TimerTime = map(TimerTime, 4095, 0, 1, 9);  //maps Timertime Display to potentiometer set level
  for (int i = 0; i < SENS_SIZE; ++i) {
    sensBuffer.push(0);
  }
  matrix.begin(0x70);
  ads.setGain(GAIN_TWO);                  // 2x gain   +/- 2.048V  1 bit = 1mV | // 4x gain   +/- 1.024V  1 bit = 0.5mV
  ads.setDataRate(RATE_ADS1015_3300SPS);  //sets the samples per seconds, different for ads1015 and ads1115 so caution.
  Serial.begin(115200);
  delay(500);
  // Serial.println("ADC Range: +/- 2.048V  1 bit = 1mV");
  ads.begin();
  matrix.print(8888);
  matrix.writeDisplay();
  delay(1000);
  //D6 and D9 input pins are used respectively. 0b00 means 2, 0b01 means 40, 0b10 means 80, 0b11 means 200.
  // ConfigState1 = digitalRead(ConfigButton1);
  // ConfigState2 = digitalRead(ConfigButton2);
  // if (ConfigState1 == 0 && ConfigState2 == 0)
  //   sens = 200;
  // else if (ConfigState1 == 0 && ConfigState2 == 1)
  //   sens = 40;
  // else if (ConfigState1 == 1 && ConfigState2 == 0)
  //   sens = 80;
  // else if (ConfigState1 == 1 && ConfigState2 == 1)
  //   sens = 200;
  // else
  //   sens = 2;

  // Serial.println("Set up completed successfully");
}
void loop() {
  startTime = millis();
  sensorVal = get_max();
  sensBuffer.push(sensorVal);
  while (sensBuffer[0] > sens && sensBuffer[1] > sens && sensBuffer[2] > sens && sensBuffer[3] > sens && sensBuffer[4] > sens && sensBuffer[5] > sens && sensBuffer[6] > sens
         && sensBuffer[7] > sens && sensBuffer[8] > sens && sensBuffer[9] > sens) {
    debugln("Entering while loop");
    flag = 0;
    flag2 = 1;

    digitalWrite(AMP_LED, 0);   //turn off AMP LED
    digitalWrite(TIME_LED, 0);  //turn off TIME LED
                                // read the sensor value

    flag3 = 1;
    sensorVal = get_max();
    if (sensorVal > sens) {
      matrixVal = sensorVal;
      aveBuffer.push(matrixVal);
  // Serial.print("MatrixVal1: ");
  // Serial.println(matrixVal);
    }

    sensBuffer.push(sensorVal);
    debug("SensorVal in while loop: ");
    debugln(sensorVal);
  }

  debugln("got out of while loop");
  // Serial.print("MatrixVal2: ");
  // Serial.println(matrixVal);
  if (startOn_dft || !flag) {
    startTime_dft = millis();
    startOn_dft = 0;
  }

  flag = 1;
  //Error LED Print
  if ((aveBuffer[1] * multiplier * amp) > 9999) {
    matrix.println("EROR");
    matrix.writeDisplay();
    //Display Time LED Print
  } else if (flag2) {
    if (((int)(millis() - startTime_dft) / 1000) % (int)(AmpTime + TimerTime) > (AmpTime - 1)) {
      matrix.print(displayTime, 2);
      matrix.writeDisplay();
      digitalWrite(AMP_LED, 0);     //turn off AMP LED
      digitalWrite(TIME_LED, 123);  //turn on TIME LED
      if (((int)(millis() - startTime_dft) / 1000) % (int)(AmpTime + TimerTime) < (AmpTime - 1))
        startOn_dft = 1;
      //Amp LED Print
    } else {
      matrix.print(aveBuffer[1] * multiplier * amp, 0);
      matrix.writeDisplay();
      digitalWrite(TIME_LED, 0);   //Turn off TIME LED
      digitalWrite(AMP_LED, 123);  //Turn on AMP LED
    }
  }
}
