/*
  Solid State Systems
  LED Timer
  David Nguyen
  5/4/2023 
  Summary: Solid State Systems program using the Adafruit Feather Esp32-s3 to making a custom ammeter/timer for client. Uses ADC and reads differential
  voltage across shunt to detect the amperage. Uses moving average filter, buffer for false readings, potentiometer for scaling, 
  DIP Switch Configuration
  "D5: Off, D6: Off" for 1000/25 shunt
  "D5: Off, D6: On" for 1000/50 shunt
  "D5: On, D6: Off" for 1000/100 shunt
  "D5: On, D6: On" for 1000/100 shunt
  3.9 MOhm pull-down resistor between ground and all four ADS1x15 Pins (A0,A1,A2,A3). 
  Parts: Adafruit Feather ESP32-S3, ADS1015 ADC [Mouser No: 485-1083, Manufactoror: Adafruit] - A0 Pin (Positive Lead) & A1 Pin (Negative Lead), 4x 7-Segment LED RED HT16K33 FeatherWing (Address used 0x70), 2x Potentiometer, 4x 3.9 Ohms Resistor.
  Displays use I2C to communicate. SDA and SCL pins used to
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
// This is required on ESP32 to put the ISR in IRAM. Define as
// empty for other platforms. Be careful - other platforms may have
// other requirements.
#ifndef IRAM_ATTR
#define IRAM_ATTR
#endif
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
float latency = 0.08;  //80 ms latency
float startTime_dft;   //the reference time used when switching between amps and millis on the 7 segment display.
float displayTime = 0;
int16_t sensorVal = 0;
float matrixVal = 0.00;
const int truncation = 100;              // amount of samples truncated at the end part of the array storing sensor values.
const int SMOOTHING_WINDOW_SIZE = 260;  // amount of samples stored
int _curReadIndex = 0;                  // the index of the current reading
int _sampleTotal = 0;                   // the running total
int _sampleAvg = 2;                     // the average
int sens = 2;                           // input sensitivity
int num = 0;                            // used for while loop iteration
float multiplier = 1.41*1.18;                   //Adjusts for pull down resistor changing the voltage value
int amp = 20;                           //the multiplier to convert mV to Amps
// bool startOn = 0;                      //timer start for displayTime
bool startOn_dft = 0;  //timer start for another time
bool flag = 0;         //indicator of when to stop default display
bool flag2 = 0;        //indicator of when first reading is taken
float total_ave = 0;
int sens_count = 0;       //total count of sensor readings
float count_per_sec = 0;  //counts per second
float AmpTime = 3;
float TimerTime = 3;
int mtp[4];     //array for storing multiplexer DIP SWITCH values. Used to configure start up settings like what the multiplier value will be
int count = 0;  //counter for sps
float start = 0;
const int AVE_SIZE = 50;                                  //size of array of average values
const int SENS_SIZE = 5;                                  //size of array that checks for consistent voltage above threshold sensitivity
CircularBuffer<int16_t, AVE_SIZE> aveBuffer;              //Queue that is used to take the nth last value from the average value readings
CircularBuffer<int, SENS_SIZE> sensBuffer;                //Buffer that checks filters out rare bad readings from good readings
CircularBuffer<int, SMOOTHING_WINDOW_SIZE> windowBuffer;  //Buffer that checks filters out rare bad readings from good readings

void IRAM_ATTR CONFIG_INTERRUPT() {
  reconfigure();
}
void reconfigure() {
  //read DIP switch to set configurations during setup
  mtp[0] = digitalRead(5);
  mtp[1] = digitalRead(6);
  mtp[2] = digitalRead(9);
  mtp[3] = digitalRead(10);
  if (mtp[0] == 1 && mtp[1] == 1) { amp = 40; }  //D5 Off, D6 Off
  else if (mtp[0] == 1 && mtp[1] == 0) {
    amp = 20;
  }                                                   //D5 Off, D6 On
  else if (mtp[0] == 0 && mtp[1] == 1) { amp = 10; }  //D5 On, D6 Off
  else if (mtp[0] == 0 && mtp[1] == 0) {
    amp = 1;
  }                   //D5 On, D6 On
  else { amp = 10; }  //D5 On, D6 On
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
  for (int i = 0; i < 6; ++i) {
    sensBuffer.push(20);
  }
  matrix.begin(0x70);
  ads.setGain(GAIN_TWO);  // 2x gain   +/- 2.048V  1 bit = 1mV | // 4x gain   +/- 1.024V  1 bit = 0.5mV
  if (!ads.begin()) {
    Serial.println("Failed to initialize ADS.");
    while (1)
      ;
  }
  Serial.begin(115200);
  // reconfigure();
  // Set up the GPIO pin for the external interrupt
  reconfigure();
  attachInterrupt(digitalPinToInterrupt(5), CONFIG_INTERRUPT, CHANGE);
  attachInterrupt(digitalPinToInterrupt(6), CONFIG_INTERRUPT, CHANGE);
  ads.startADCReading(ADS1X15_REG_CONFIG_MUX_DIFF_0_1, /*continuous=*/true);

  delay(500);
  Serial.println("ADC Range: +/- 2.048V  1 bit = 1mV");
  ads.begin();
  matrix.print(1.001, 3);  //Version number. 1st digit DC or AC (1 DC, 2 AC). 2nd digit hardware version updates. 3rd and 4th are for software version updates
  matrix.writeDisplay();

  delay(1500);
  matrix.print(8888);
  matrix.writeDisplay();
  delay(1000);

  Serial.println("Set up completed successfully");
}
void loop() {
  // sensorVal = 0;
  sensorVal = abs(ads.getLastConversionResults());
    Serial.println(sensorVal);
  count++;
  while (count > 3000) {
    float time = millis() - start;
    Serial.print("Samples per second: ");
    Serial.println(count / (time / 1000));
    count = 0;
    start = millis();
  }
  sensBuffer.push(sensorVal);
  sens_count = 0;
  count_per_sec = 0;
  startTime = millis();
  while (sensBuffer[0] > sens && sensBuffer[1] > sens && sensBuffer[2] > sens && sensBuffer[3] > sens && sensBuffer[4]) {
    flag = 0;
    flag2 = 1;
    digitalWrite(AMP_LED, 0);   //turn off AMP LED
    digitalWrite(TIME_LED, 0);  //turn off TIME LED
    for (int i = 0; i < SENS_SIZE; ++i) {
      windowBuffer.push(sensBuffer[i]);
    }
    // read the sensor value

    sensorVal = abs(ads.getLastConversionResults());
    if (sensorVal > sens) {
      windowBuffer.push(sensorVal);
    }

    sensBuffer.push(sensorVal);
    endTime = millis();
    displayTime = ((endTime - startTime) / 1000) + latency;
    matrix.print(displayTime, 3);
    matrix.writeDisplay();
    Serial.println(sensorVal);
  }
  if (startOn_dft || !flag) {
    startTime_dft = millis();
    startOn_dft = 0;
  }

  flag = 1;
  //average
  if (windowBuffer.isFull()) {
    for(int i = 0; i < windowBuffer.size() - truncation; ++i)
    {
      total_ave += windowBuffer[i];
      if (i == windowBuffer.size() - truncation - 1) {
        matrixVal = total_ave / (windowBuffer.size()-truncation);
        total_ave = 0;
        windowBuffer.clear();
      }
    }
  }
  //Error LED Print
  if ((matrixVal * multiplier * amp) > 9999) {
    matrix.println("EROR");
    matrix.writeDisplay();
    //Display Time LED Print
  } else if (flag2) {
    if (((int)(millis() - startTime_dft) / 1000) % (int)(AmpTime + TimerTime) > (AmpTime - 1)) {
      matrix.print(displayTime, 3);
      matrix.writeDisplay();
      digitalWrite(AMP_LED, 0);   //turn off AMP LED
      digitalWrite(TIME_LED, 5);  //turn on TIME LED
      if (((int)(millis() - startTime_dft) / 1000) % (int)(AmpTime + TimerTime) < (AmpTime - 1))
        startOn_dft = 1;
      //Amp LED Print
    } else {
      matrix.print(matrixVal * multiplier * amp, 0);
      matrix.writeDisplay();
      digitalWrite(TIME_LED, 0);  //Turn off TIME LED
      digitalWrite(AMP_LED, 5);   //Turn on AMP LED
    }
  }
}
