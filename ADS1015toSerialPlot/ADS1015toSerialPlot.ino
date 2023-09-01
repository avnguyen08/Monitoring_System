
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
  sensorVal = ads.getLastConversionResults();

  Serial.print("SensorValue:");
  Serial.println(sensorVal);
  
}