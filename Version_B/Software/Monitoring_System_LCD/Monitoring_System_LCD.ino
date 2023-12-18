/*
* Company: Solid State Systems
* Author: Aaron Nguyen
* Project: Monitoring System Version C (LCD)

* Summary: Solid State Systems program using the LilyGo T-Display s3 to making a custom ammeter/timer for client. Uses ADC and reads differential
* voltage across shunt to detect the amperage. Uses software peak detector to obtain correct voltage.

* DIP Switch Configuration
* "D5: Off, D6: Off" for 1000/25 shunt
* "D5: Off, D6: On" for 1000/50 shunt
* "D5: On, D6: Off" for 1000/100 shunt
* "D5: On, D6: On" for 1000/100 shunt
* 
* Displays use I2C to communicate. SDA and SCL pins used to interface. 
* Dropping negative reads and the last 4 values outputted due to debouncing when removing the positive lead.
*
* 6/25/2023 Update: ADS 1115 gets around 1450 samples per second in the code while loop. This means for .18 seconds, there will be 261 samples. for 1 seconds, there will be 1450 samples
* 6/26/2023 Update: adding time correcting function: corrects time if while loop isn't exited as fast as it should be.  changed display to display 4 different config types for debugging. plotted test machine output and made formula
* to predict next amp values since theirs is not linear
*/
#include <Adafruit_ADS1X15.h>
#include <CircularBuffer.h>
#include "SPI.h"
#include "TFT_eSPI.h"
#include <TFT_eWidget.h>  // Widget library
#include "MyFont.h"
#include "MyriadProBold58.h"
#include "MyriadProBold70.h"
#include "MyriadProBold25.h"
#include <PNGdec.h>      //PNG decoder library
#include "Artboard_1.h"  //Contains SSS bit map array for image
// This is required on ESP32 to put the ISR in IRAM. Define as
// empty for other platforms. Be careful - other platforms may have
// other requirements.
#ifndef IRAM_ATTR
#define IRAM_ATTR
#endif
#define I2C_SDA 12
#define I2C_SCL 13
#define BACKG_CLR TFT_BLUE   //BACKGROUND COLOR
#define AMP_FONT TFT_BLUE    //BACKGROUND COLOR
#define MAX_IMAGE_WDITH 320  // Solid State Systems logo pixel width
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
// Easily remembered name for the font
#define MYFONT25 &MyriadProBold25pt7b
#define MYFONT58 &MyriadProBold58pt7b
#define MYFONT70 &MyriadProBold70pt7b

#define AMPFONT MYFONT58
#define TIMEFONT MYFONT25
#define AMPFONT70 MYFONT70
// Use hardware SPI
TFT_eSPI tft = TFT_eSPI();
TFT_eSprite spr = TFT_eSprite(&tft);  // Sprite for meter reading
int counter = 0;
char str[20];
//---------------------------------------------------------------------------------------------------------------------------------------------------------------------------
//coordinates for various TFT display features
float ampX = 160;
float ampY = 10;
float timeX = 320;
float timeY = 170;
float peakX = 0;
float peakY = 170;
//--------------------------------------------------------------------------------------------------------------------------------------------------------------------------
//SSS Png Boot up

PNG png;  // PNG decoder inatance
int16_t xpos = 0;
int16_t ypos = 0;
Adafruit_ADS1015 ads;
char string[50];
float startTime;
float endTime;
float displayTime = 0.11;
float sensorVal = 0;
float ampPeak = 0.00;
float voltPeak = 0;
const int SMOOTHING_WINDOW_SIZE = 1450;  // amount of samples stored. Number chosen for 2 seconds worth of samples. Sampling rate: 1450 samples/sec
float multiplier = 1;                    //Adjusts for pull down resistor changing the voltage value
float max_num = 0;                       // max number storage in void loop
int amp = 20;                            //the multiplier to convert mV to Amps
bool startOn_dft = 0;                    //timer start for another time
bool refresh = 0;                        //Refresh status
int mtp[4];                              //array for storing multiplexer DIP SWITCH values. Used to configure start up settings like what the multiplier value will be
int loop_counter = 0;
const int AVE_SIZE = 50;                                    //size of array of average values
const int SENS_SIZE = 10;                                   //size of array that checks for consistent voltage above threshold sensitivity
CircularBuffer<float, SENS_SIZE> sensBuffer;                //Buffer that checks filters out rare bad readings from good readings
CircularBuffer<float, SMOOTHING_WINDOW_SIZE> windowBuffer;  //Buffer that checks filters out rare bad readings from good readings
CircularBuffer<float, SMOOTHING_WINDOW_SIZE> *windowBufferptr = &windowBuffer;
//VVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVV
//---------------------------------DEBUG variables -----------------------------
int sensorCount = 0;
const int front_trunc = 0;     // amount of samples truncated at the beginning part of the array storing sensor values.
const int back_trunc = 0;      // amount of samples truncated at the end part of the array storing sensor values.
const int back_sort_trunc = 20;  // amount of samples truncated at the end part of the sorted array
int sens = 5;                   // input sensitivity
int exit_sens = 2;
float latency = 0.0;            //0 ms latency (customizable)
float counting_rate = 0;
char *charptr;
//Function used in the qsort function argument. Customize it based on what the size of the data type and if you want ascending sort or descending sort. (float and ascending sort is used here)
int comp(const void *elem1, const void *elem2) {
  float f = *((float *)elem1);
  float s = *((float *)elem2);
  if (f > s) return 1;
  if (f < s) return -1;
  return 0;
}
//VVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVV
//===================================================================================
//                                      pngDraw
//===================================================================================
// This next function will be called during decoding of the png file to
// render each image line to the TFT.  If you use a different TFT library
// you will need to adapt this function to suit.
// Callback function to draw pixels to the display
void pngDraw(PNGDRAW *pDraw) {
  uint16_t lineBuffer[MAX_IMAGE_WDITH];
  png.getLineAsRGB565(pDraw, lineBuffer, PNG_RGB565_BIG_ENDIAN, 0xffffffff);
  tft.pushImage(xpos, ypos + pDraw->y, pDraw->iWidth, 1, lineBuffer);
}
//====================================================================================

//=========================================v==========================================
//-------------------TFT DISPLAY-------------------------
//=========================================v==========================================
void amp_display(const GFXfont *font) {

  //AMP Value Display

  tft.setTextColor(TFT_RED);
  tft.setFreeFont(font);       // Select the font
  tft.setTextDatum(TC_DATUM);  //Adjusts reference point of text generation
  sprintf(string, "%.0f", ampPeak);
  tft.drawString(string, ampX, ampY);  // Print the test text in the custom font
}
void time_display(const GFXfont *font) {

  //Time Value Display

  tft.setTextColor(TFT_WHITE);
  tft.setFreeFont(font);       // Select the font
  tft.setTextDatum(BR_DATUM);  //Adjusts reference point of text generation
  sprintf(string, "%.2f%s", correct_time(windowBufferptr, voltPeak, counting_rate, displayTime), "s");
  if (correct_time(windowBufferptr, voltPeak, counting_rate, displayTime) > 0) {
    tft.drawString(string, timeX, timeY);
  } else {

    tft.drawString("N/A", timeX, timeY);
  }
}
void peak_display(const GFXfont *font) {

  //Peak Value Display
  tft.setTextColor(TFT_WHITE);
  tft.setFreeFont(font);       // Select the font
  tft.setTextDatum(BL_DATUM);  //Adjusts reference point of text generation
  sprintf(string, "%.0f%s", voltPeak, "mv");
  tft.drawString(string, peakX, peakY);
}
// CircularBuffer<float, SMOOTHING_WINDOW_SIZE> *Buffer
void debug_display() {

  const int Afront_trunc = 0;      // amount of samples truncated at the beginning part of the array storing sensor values.
  const int Aback_trunc = 0;       // amount of samples truncated at the end part of the array storing sensor values.
  const int Aback_sort_trunc = 0;  // amount of samples truncated at the end part of the sorted array

  const int Bfront_trunc = 0;       // amount of samples truncated at the beginning part of the array storing sensor values.
  const int Bback_trunc = 0;        // amount of samples truncated at the end part of the array storing sensor values.
  const int Bback_sort_trunc = 20;  // amount of samples truncated at the end part of the sorted array

  const int Cfront_trunc = 0;      // amount of samples truncated at the beginning part of the array storing sensor values.
  const int Cback_trunc = 20;      // amount of samples truncated at the end part of the array storing sensor values.
  const int Cback_sort_trunc = 5;  // amount of samples truncated at the end part of the sorted array

  const int Dfront_trunc = 10;     // amount of samples truncated at the beginning part of the array storing sensor values.
  const int Dback_trunc = 20;      // amount of samples truncated at the end part of the array storing sensor values.
  const int Dback_sort_trunc = 5;  // amount of samples truncated at the end part of the sorted array

  //Debug A: No Front Truncation. No Back Truncation. No Sort Back Truncation
  voltPeak = (peak_detector(windowBufferptr, Afront_trunc, Aback_trunc, Aback_sort_trunc));
  ampPeak = mVtoAmp(voltPeak);
  tft.setTextColor(TFT_WHITE);
  tft.setTextFont(2);  // Select the font
  sprintf(string, "Front:%d | Back:%d | Sort_Cut:%d", Afront_trunc, Aback_trunc, Aback_sort_trunc);
  tft.println(string);
  tft.setTextFont(4);  // Select the font
  sprintf(string, "%.0fmV | %.2fs | %.0f Amp", voltPeak, correct_time(windowBufferptr, voltPeak, counting_rate, displayTime), ampPeak);
  tft.println(string);
  //Debug B: No Front Truncation. No Back Truncation. Yes Sort Back Truncation
  voltPeak = (peak_detector(windowBufferptr, Bfront_trunc, Bback_trunc, Bback_sort_trunc));
  ampPeak = mVtoAmp(voltPeak);
  tft.setTextColor(TFT_RED);
  tft.setTextFont(2);  // Select the font
  sprintf(string, "Front:%d | Back:%d | Sort_Cut:%d", Bfront_trunc, Bback_trunc, Bback_sort_trunc);
  tft.println(string);
  tft.setTextFont(4);  // Select the font
  sprintf(string, "%.0fmV | %.2fs | %.0f Amp", voltPeak, displayTime, ampPeak);
  tft.println(string);
  //Debug C: No Front Truncation. Yes Back Truncation. Yes Sort Back Truncation
  voltPeak = (peak_detector(windowBufferptr, Cfront_trunc, Cback_trunc, Cback_sort_trunc));
  ampPeak = mVtoAmp(voltPeak);
  tft.setTextColor(TFT_BLUE);
  tft.setTextFont(2);  // Select the font
  sprintf(string, "Front:%d | Back:%d | Sort_Cut:%d", Cfront_trunc, Cback_trunc, Cback_sort_trunc);
  tft.println(string);
  tft.setTextFont(4);  // Select the font
  sprintf(string, "%.0fmV | %.2fs | %.0f Amp", voltPeak, displayTime, ampPeak);
  tft.println(string);
  //Debug D: Yes Front Truncation. Yes Back Truncation. Yes Sort Back Truncation
  voltPeak = (peak_detector(windowBufferptr, Dfront_trunc, Dback_trunc, Dback_sort_trunc));
  ampPeak = mVtoAmp(voltPeak);
  tft.setTextColor(TFT_GREEN);
  tft.setTextFont(2);  // Select the font
  sprintf(string, "Front:%d | Back:%d | Sort_Cut:%d", Dfront_trunc, Dback_trunc, Dback_sort_trunc);
  tft.println(string);
  tft.setTextFont(4);  // Select the font
  sprintf(string, "%.0fmV | %.2fs | %.0f Amp", voltPeak, correct_time(windowBufferptr, voltPeak, counting_rate, displayTime), ampPeak);
  tft.println(string);

  windowBuffer.clear();
}
//function to convert mV to amps
float mVtoAmp(float x) {
  float amp_convert = 0;
  amp_convert = -0.000653 + 0.00324*x + 0.00000841*x*x;  //2nd polynomial formula
  return amp_convert * 1000;
}

//function that corrects display time from being too long due to while loop not exiting correctly
//Takes an array pointer as an argument, peak value, and sampling rate
//Reverse iterates through the array until peak value is reached, counts how many elements it iterated through then subtracts that from display time
//=========================================v==========================================
float correct_time(CircularBuffer<float, SMOOTHING_WINDOW_SIZE> *arr, float peak_value, float samp_rate, float time) {
  int iterate_count = 0;
  for (int i = ((*arr).size() - 1); i > 1; --i) {
    //if value is greater than 90% of peak value, break out of for loop
    if ((*arr)[i] > (.9*peak_value)) {
      break;
    }
    iterate_count++;
  }
  return (time - iterate_count / samp_rate);
}
//Prints values inside array
void print_buffer_array(CircularBuffer<float, SMOOTHING_WINDOW_SIZE> *arr) {
  for (int i = 0; i < (*arr).size(); ++i) {
    sprintf(string, "Array Value[%d] = %.0f", i, (*arr)[i]);
    Serial.println(string);
  }
}

//=========================================v==========================================
//-------------------DIP SWITCH RECONFIGURATION-------------------------
//=========================================v==========================================
//interrupt that will call reconfigure function
void IRAM_ATTR CONFIG_INTERRUPT() {
  reconfigure();
}
//read the dip switches and change the multiplier of the voltage to amps based on the dip switch readings
void reconfigure() {
  mtp[0] = digitalRead(1);
  mtp[1] = digitalRead(2);
  mtp[2] = digitalRead(3);
  mtp[3] = digitalRead(10);
  if (mtp[0] == 1 && mtp[1] == 1) { amp = 4; }  //D5 Off, D6 Off
  else if (mtp[0] == 1 && mtp[1] == 0) {
    amp = 20;
  }                                                   //D5 Off, D6 On
  else if (mtp[0] == 0 && mtp[1] == 1) { amp = 20; }  //D5 On, D6 Off
  else if (mtp[0] == 0 && mtp[1] == 0) {
    amp = 20;
  }                   //D5 On, D6 On
  else { amp = 20; }  //D5 On, D6 On
}
//=========================================v==========================================


//=========================================v==========================================
//-------------------AMP OUTPUT CALCULATION-------------------------
//=========================================v==========================================
//function that takes in circuilar buffer of graph points and returns the peak of the buffer
float peak_detector(CircularBuffer<float, SMOOTHING_WINDOW_SIZE> *Buffer, int front, int back, int sort_cut) {
  float max = 0;
  CircularBuffer<float, SMOOTHING_WINDOW_SIZE> newBuffer;
  CircularBuffer<float, SMOOTHING_WINDOW_SIZE> *newBufferptr = &newBuffer;
  for (int i = front; i < (((*Buffer).size()) - back); ++i) {
    newBuffer.push((*Buffer)[i]);
  }

  qsort(newBufferptr, (*newBufferptr).size(), sizeof(float), comp);  //qsort has weird bug where last entry in array is not sorted
  max = newBuffer[newBuffer.size() - 2 - sort_cut];                  // - 1 for index starting at 0 and -1 for qsort bug mentioned in above comment
  (*newBufferptr).clear();
  debug("max: ");
  debugln(max);

  return max;
}

//=========================================v==========================================

//=========================================v==========================================
//-------------------VOID SETUP-------------------------
//=========================================v==========================================
void setup() {
  // pinMode(15, OUTPUT);        // to boot with battery...
  // digitalWrite(15, 1);        // and/or power from 5v rail instead of USB
  Serial.begin(250000);
  delay(500);
  Serial.println("Testing");
  delay(500);
  pinMode(1, INPUT_PULLUP);   //D5 multiplexer input
  pinMode(2, INPUT_PULLUP);   //D6 multiplexer input
  pinMode(3, INPUT_PULLUP);   //D9 multiplexer input
  pinMode(10, INPUT_PULLUP);  //D10 multiplexer input
  tft.begin();
  Wire.begin(I2C_SDA, I2C_SCL);
  tft.setRotation(1);
  tft.fillScreen(TFT_BLACK);  // Clear screen
  tft.setTextColor(TFT_WHITE);
  for (int i = 0; i < SENS_SIZE; ++i) {
    sensBuffer.push(0);
  }
  // for (int i = 0; i < 6; ++i) {
  //   sensBuffer.push(20);
  // }
  if (!ads.begin()) {
    Serial.println("Failed to initialize ADS.");
    while (1)
      ;
  }
  ads.setGain(GAIN_TWO);  // 2x gain   +/- 2.048V  1 bit = 1mV | // 4x gain   +/- 1.024V  1 bit = 0.5mV
  // reconfigure();
  // Set up the GPIO pin for the external interrupt
  reconfigure();
  attachInterrupt(digitalPinToInterrupt(5), CONFIG_INTERRUPT, CHANGE);
  attachInterrupt(digitalPinToInterrupt(6), CONFIG_INTERRUPT, CHANGE);
  ads.startADCReading(ADS1X15_REG_CONFIG_MUX_DIFF_0_1, /*continuous=*/true);

  delay(500);
  Serial.println("ADC Range: +/- 1.024V  1 bit = 0.5mV");
  ads.begin();
  Serial.println("1.102");  //Version number. 1st digit DC or AC (1 DC, 2 AC). 2nd digit hardware version updates. 3rd and 4th are for software version updates


  tft.setTextDatum(MC_DATUM);
  int16_t rc = png.openFLASH((uint8_t *)Artboard_1, sizeof(Artboard_1), pngDraw);
  if (rc == PNG_SUCCESS) {
    // Serial.println("Successfully png file");
    // Serial.printf("image specs: (%d x %d), %d bpp, pixel type: %d\n", png.getWidth(), png.getHeight(), png.getBpp(), png.getPixelType());
    tft.startWrite();
    uint32_t dt = millis();
    rc = png.decode(NULL, 0);
    Serial.print(millis() - dt);
    Serial.println("ms");
    tft.endWrite();
    // png.close(); // not needed for memory->memory decode
  }
  tft.setTextColor(TFT_RED);
  tft.setFreeFont(&FreeMonoBold12pt7b);  // Select the font
  tft.setTextDatum(BR_DATUM);            //Adjusts reference point of text generation
  tft.drawString("D1.102", 320, 170);    // Print the test text in the custom font
  delay(2000);
  Serial.println("Set up completed successfully");
}
//=========================================v==========================================

//=========================================v==========================================
//-------------------VOID SETUP-------------------------
//=========================================v==========================================
void loop() {
  sensorVal = (float)ads.getLastConversionResults();  // polling for sensor values to see when there is actually significant voltage (voltage above the threshold value)
  sensBuffer.push(sensorVal);                         //store sensor values into a circular buffer of sensor values
  // Serial.print("sensorVal:");
  // Serial.println(sensorVal);
  startTime = millis();
  if (abs(sensBuffer[0]) > sens && abs(sensBuffer[1]) > sens) {
    Serial.println("Enter Loop");
    while (1) {
      sensorVal = (float)ads.getLastConversionResults();  // polling for sensor values to see when there is actually significant voltage (voltage above the threshold value)
      sensorCount++;

      sensBuffer.push(abs(sensorVal));
      if (abs(sensorVal) > sens) {
        windowBuffer.push(abs(sensorVal));
      }
      Serial.print("sensorVal:");
      Serial.println(sensorVal);
      if (abs(sensBuffer[0]) < exit_sens && abs(sensBuffer[1]) < exit_sens && abs(sensBuffer[2]) < exit_sens && abs(sensBuffer[3]) < exit_sens && abs(sensBuffer[4]) < exit_sens && abs(sensBuffer[5]) < exit_sens && abs(sensBuffer[6]) < exit_sens && abs(sensBuffer[7]) < exit_sens) {

        endTime = millis();
        displayTime = ((endTime - startTime) / 1000) + latency;


        print_buffer_array(windowBufferptr);
        voltPeak = (peak_detector(windowBufferptr, front_trunc, back_trunc, back_sort_trunc));
        ampPeak = mVtoAmp(voltPeak);
        refresh = 1;
        Serial.println("Exited While Loop");
        Serial.print("Real Time: ");
        Serial.println(displayTime - latency);
        break;
      }
    }
    counting_rate = sensorCount / (displayTime - latency);
    Serial.print("The counting rate is: ");
    Serial.println(counting_rate);
    sensorCount = 0;
  }
  if (refresh == 1) {
    tft.fillScreen(TFT_BLACK);  // Clear screen

    //---------------------TFT DISPLAY-------------------------
    if (ampPeak > 9999) {
      amp_display(AMPFONT);
    } else {
      amp_display(AMPFONT70);
    }
  
    time_display(TIMEFONT);
    peak_display(TIMEFONT);

    refresh = 0;
  }
}
//=========================================v==========================================
