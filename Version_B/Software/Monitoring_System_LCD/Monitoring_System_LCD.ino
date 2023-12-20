/*
* Company: Solid State Systems
* Author: Aaron Nguyen
* Project: Monitoring System Version C (LCD + Hardware Filter)
*
* Description: Secured Solutions Group program using the LilyGo T-Display S3 (170x320 pixels) to make a custom ammeter/timer for client. Communicates with external ADC via I2C protocol using it to read differential
* voltage across varying shunts from 1000Amps/100mV to 1000Amps/25mV to detect the amperage. Uses software peak detector to obtain correct voltage.
*
* DIP Switch Configuration
* "D10: Off, D18: Off" for 1000/25 shunt
* "D10: Off, D18: On" for 1000/50 shunt
* "D10: On, D18: Off" for 1000/100 shunt
* "D10: On, D18: On" for 1000/200 shunt
* 
* LCD uses SPI protocol to communicate
*
* 6/25/2023 Update: ADS 1115 gets around 1450 samples per second in the code while loop. This means for .18 seconds, there will be 261 samples. for 1 seconds, there will be 1450 samples
* 6/26/2023 Update: adding time correcting function: corrects time if while loop isn't exited as fast as it should be.  changed display to display 4 different config types for debugging. plotted test machine output and made formula
* to predict next amp values since theirs is not linear
*/
#include <Adafruit_ADS1X15.h>  // ADC Library
#include <CircularBuffer.h>    // Circular Buffer library to store voltage samples
#include <SPI.h>               // SPI Communication Interface
#include <TFT_eSPI.h>          // LILYGO Specific library, taken from Adafruit GFX
#include "FontsAndImages.h"    // Contains my custom fonts and images
#include <TFT_eWidget.h>       // Widget library
#include <PNGdec.h>            //PNG decoder library

/* This is required on ESP32 to put the ISR in IRAM. Define as
empty for other platforms. Be careful - other platforms may have
 other requirements. */
#ifndef IRAM_ATTR
#define IRAM_ATTR
#endif

#define I2C_SDA 12           // I2C Data pin for ADC
#define I2C_SCL 13           // I2C Clock pin for ADC
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

TFT_eSPI tft = TFT_eSPI();  // Use hardware SPI

//coordinates for various TFT display features (T-Display S3 is 170x320 pixels)
float ampX = 160;   // display x axist pixel coordinate
float ampY = 10;    // display x axist pixel coordinate
float timeX = 320;  // display x axis pixel coordinate
float timeY = 170;  // display y axis pixel coordinate
float peakX = 0;    // display x axis pixel coordinate
float peakY = 170;  // display y axis pixel coordinate

//Variables needed for Solid State Systems compoany Logo LCD display
PNG png;           // PNG decoder instance
int16_t xpos = 0;  // x-coordinate of logo
int16_t ypos = 0;  // y-coordinate of logo

Adafruit_ADS1015 ads;
char string[50];      //string to hold display data, non-essential but useful for debugging and logging
float sensorVal = 0;  // sensorVal
bool refresh = 0;     //Refresh status
  const int SMOOTHING_WINDOW_SIZE = 2890;                     // amount of samples stored. Number chosen for 1 seconds worth of samples. Sampling rate: 1450 samples/sec
  const int SENS_SIZE = 10;                                   //size of array that checks for consistent voltage above threshold sensitivity

//Function used in the qsort function argument. Customize it based on what the size of the data type and if you want ascending sort or descending sort
int comp(const void *elem1, const void *elem2);

// Structure containing all data used to capture and modify the signal data
class SignalInfo {

private:

public:

  float startTime;
  float endTime;
  float displayTime = 0.11;        // Display time variable
  int sensorCount = 0;             // tracking amount of data counted for use in capture/sec data
  const int front_trunc = 0;       // amount of samples truncated at the beginning part of the array storing sensor values.
  const int back_trunc = 0;        // amount of samples truncated at the end part of the array storing sensor values.
  const int back_sort_trunc = 20;  // amount of samples truncated at the end part of the sorted array
  int enter_sens = 5;              // input sensitivity
  int exit_sens = 2;
  float latency = 0.0;  //0 ms latency (customizable)
  float counting_rate = 0;
  float ampPeak = 0.00;
  float voltPeak = 0;
  CircularBuffer<float, SENS_SIZE> sensBuffer;                //Buffer that checks filters out rare bad readings from good readings
  CircularBuffer<float, SMOOTHING_WINDOW_SIZE> windowBuffer;  //Buffer that checks filters out rare bad readings from good readings
  CircularBuffer<float, SMOOTHING_WINDOW_SIZE> *windowBufferptr = &windowBuffer;


  //Prints values inside window buffer
  void print_buffer_array() {
    for (int i = 0; i < windowBuffer.size(); ++i) {
      sprintf(string, "Array Value[%d] = %.0f", i, windowBuffer[i]);
      Serial.println(string);
    }
  }

  /* Returns boolean of whether sensor values are below exit threshold or not
 Used to determine when signal has ended */
  bool SignalEnd() {
    if (abs(sensBuffer[0]) < exit_sens && abs(sensBuffer[1]) < exit_sens && abs(sensBuffer[2]) < exit_sens && abs(sensBuffer[3]) < exit_sens && abs(sensBuffer[4]) < exit_sens && abs(sensBuffer[5]) < exit_sens && abs(sensBuffer[6]) < exit_sens && abs(sensBuffer[7]) < exit_sens) {
      return true;
    }
    return false;
  }

  /* Returns boolean of whether sensor values are above enter threshold or not
 Used to determine when signal has started */
  bool SignalBegin() {
    if (abs(sensBuffer[0]) > enter_sens && abs(sensBuffer[1]) > enter_sens) {
      return true;
    }
    return false;
  }

/* Corrects display time from being too long due to while loop not exiting correctly
 Takes an array pointer as an argument, peak value, and sampling rate
 Reverse iterates through the array until peak value is reached, counts how many elements
 it iterated through then subtracts that from display time */
//=========================================v==========================================
float correct_time() {
  int iterate_count = 0;
  for (int i = (windowBuffer.size() - 1); i > 1; --i) {
    //if value is greater than 90% of peak value, break out of for loop
    if (windowBuffer[i] > (.9 * voltPeak)) {
      break;
    }
    iterate_count++;
  }
  return (displayTime - iterate_count / counting_rate);
}

//Takes in circular buffer of graph points and returns the peak of the buffer
float peak_detector() {
  float max = 0;
  CircularBuffer<float, SMOOTHING_WINDOW_SIZE> newBuffer;
  CircularBuffer<float, SMOOTHING_WINDOW_SIZE> *newBufferptr = &newBuffer;
  for (int i = front_trunc; i < ((windowBuffer.size()) - back_trunc); ++i) {
    newBuffer.push(windowBuffer[i]);
  }

  qsort(newBufferptr, (*newBufferptr).size(), sizeof(float), comp);  //qsort has strange bug where last entry in array is not sorted
  max = newBuffer[newBuffer.size() - 2 - back_sort_trunc];                  // - 1 for index starting at 0 and -1 for qsort bug mentioned in above comment
  (*newBufferptr).clear();
  debug("max: ");
  debugln(max);
  return max;
}
} wave;




//Draws png for company logo
void pngDraw(PNGDRAW *pDraw);

//LCD Text Display functions
void amp_display(const GFXfont *font);
void time_display(const GFXfont *font);
void peak_display(const GFXfont *font);


//interrupt that will call reconfigure function
void IRAM_ATTR CONFIG_INTERRUPT();
//read the dip switches and change the multiplier of the voltage to amps based on the dip switch readings
void reconfigure();


// Core definitions (Used for dual-core ESP32)
static const BaseType_t pro_cpu = 0;  // Core 0
static const BaseType_t app_cpu = 1;  // Core 1

// Pins
static const int pin_1 = 38;  // LED pin

// Globals
static SemaphoreHandle_t bin_sem;

//*****************************************************************************
// Tasks

// Task in Core 0
void doTask0(void *parameters) {
  while (1) {

    sensorVal = (float)ads.getLastConversionResults();  // polling for sensor values to see when there is actually significant voltage (voltage above the threshold value)
    wave.sensBuffer.push(sensorVal);                    //store sensor values into a circular buffer of sensor values
    wave.startTime = millis();
    if (wave.SignalBegin()) {
      Serial.println("Enter Loop");
      while (1) {
        sensorVal = (float)ads.getLastConversionResults();  // polling for sensor values to see when there is actually significant voltage (voltage above the threshold value)
        wave.sensorCount++;

        Serial.print("Counting sensors in while loop: ");
        Serial.println(wave.sensorCount);
        wave.sensBuffer.push(abs(sensorVal));
        if (abs(sensorVal) > wave.enter_sens) {
          wave.windowBuffer.push(abs(sensorVal));
        }
        Serial.print("sensorVal:");
        Serial.println(sensorVal);
        // if newest ADC readings below a certain sensitivity then end capture
        if (wave.SignalEnd()) {

          wave.endTime = millis();
          wave.displayTime = ((wave.endTime - wave.startTime) / 1000) + wave.latency;



          wave.print_buffer_array();
          wave.voltPeak = wave.peak_detector();
          wave.ampPeak = mVtoAmp(wave.voltPeak);
          refresh = 1;
          wave.windowBuffer.clear();  // Clears buffer so it doesn't have left over values
          Serial.println("Exited While Loop");
          Serial.print("Real Time: ");
          Serial.println(wave.displayTime - wave.latency);
          break;
        }
      }
      wave.counting_rate = wave.sensorCount / (wave.displayTime - wave.latency);
      Serial.print("The counting rate is: ");
      Serial.println(wave.counting_rate);
      wave.sensorCount = 0;
    }
  }
}

// Task in Core 1
void doTask1(void *parameters) {

  // Do forever
  while (1) {
    vTaskDelay(500 / portTICK_PERIOD_MS);
    if (refresh == 1) {
      tft.fillScreen(TFT_BLACK);  // Clear screen

      // if amp display more than 4 digits then change font size to smaller size
      if (wave.ampPeak > 9999) {
        amp_display(AMPFONT);  // displays amp in top center of LCD
      } else {
        amp_display(AMPFONT70);  // displays amps in top center of LCD
      }

      time_display(TIMEFONT);  //displays time in bottom right of LCD
      peak_display(TIMEFONT);  // displays voltage in bottom left of LCD

      refresh = 0;
    }
  }
}

//*****************************************************************************
// Main (runs as its own task with priority 1 on core 1)

void setup() {


  pinMode(15, OUTPUT);  // to boot with battery...
  digitalWrite(15, 1);  // and/or power from 5v rail instead of USB
  Serial.begin(250000);
  delay(500);
  Serial.println("Testing");
  delay(500);
  pinMode(10, INPUT_PULLUP);     //D10 multiplexer input
  pinMode(18, INPUT_PULLUP);     //D18 multiplexer input
  pinMode(44, INPUT_PULLUP);     //D44 multiplexer input
  pinMode(43, INPUT_PULLUP);     //D43 multiplexer input
  tft.begin();                   // initializes communication LCD screen
  tft.setRotation(3);            // Sets orientation of display [1: 0 degrees] [2: 90 degrees] [3: 180 degrees] [4: 270 degrees]
  tft.fillScreen(TFT_BLACK);     // Clear screen
  tft.setTextColor(TFT_WHITE);   // Sets text color to white
  Wire.begin(I2C_SDA, I2C_SCL);  // Initializes communication with ADC
  for (int i = 0; i < SENS_SIZE; ++i) {
    wave.sensBuffer.push(0);
  }
  if (!ads.begin()) {
    while (1) {
      Serial.println("Failed to initialize ADS.");
      delay(3000);
    }
  }
  ads.setGain(GAIN_TWO);  // 2x gain   +/- 2.048V  1 bit = 1mV | // 4x gain   +/- 1.024V  1 bit = 0.5mV

  // Set up the GPIO pin for the external interrupt
  reconfigure();
  attachInterrupt(digitalPinToInterrupt(10), CONFIG_INTERRUPT, CHANGE);
  attachInterrupt(digitalPinToInterrupt(18), CONFIG_INTERRUPT, CHANGE);
  attachInterrupt(digitalPinToInterrupt(44), CONFIG_INTERRUPT, CHANGE);
  attachInterrupt(digitalPinToInterrupt(43), CONFIG_INTERRUPT, CHANGE);
  ads.startADCReading(ADS1X15_REG_CONFIG_MUX_DIFF_0_1, /*continuous=*/true);
  delay(500);
  Serial.println("ADC Range: +/- 1.024V  1 bit = 0.5mV");
  ads.begin();
  Serial.println("1.102");  //Version number. 1st digit DC or AC (1 DC, 2 AC). 2nd digit hardware version updates. 3rd and 4th are for software version updates


  tft.setTextDatum(MC_DATUM);
  int16_t rc = png.openFLASH((uint8_t *)Artboard_1, sizeof(Artboard_1), pngDraw);
  if (rc == PNG_SUCCESS) {
    tft.startWrite();
    uint32_t dt = millis();
    rc = png.decode(NULL, 0);
    Serial.print(millis() - dt);
    Serial.println("ms");
    tft.endWrite();
  }
  tft.setTextColor(TFT_RED);
  tft.setFreeFont(&FreeMonoBold12pt7b);  // Select the font
  tft.setTextDatum(BR_DATUM);            //Adjusts reference point of text generation
  tft.drawString("D1.102", 320, 170);    // Print the test text in the custom font
  delay(2000);
  bin_sem = xSemaphoreCreateBinary();

  // Start Task 0 (in Core 0)
  xTaskCreatePinnedToCore(doTask0,
                          "Task 0",
                          102400,
                          NULL,
                          1,
                          NULL,
                          pro_cpu);

  // Start Task 1 (in Core 1)
  xTaskCreatePinnedToCore(doTask1,
                          "Task 1",
                          102400,
                          NULL,
                          1,
                          NULL,
                          app_cpu);

  Serial.println("Set up completed successfully");

  vTaskDelay(10000 / portTICK_PERIOD_MS);
  // Delete "setup and loop" task
  vTaskDelete(NULL);
}

void loop() {
  vTaskDelete(NULL);
}

//Function used in the qsort function argument. Customize it based on what the size of the data type and if you want ascending sort or descending sort. (float and ascending sort is used here)
int comp(const void *elem1, const void *elem2) {
  float f = *((float *)elem1);
  float s = *((float *)elem2);
  if (f > s) return 1;
  if (f < s) return -1;
  return 0;
}


/*
 PNG function taken from example libraries
 This next function will be called during decoding of the png file to
 render each image line to the TFT.  If you use a different TFT library
 you will need to adapt this function to suit.
 Callback function to draw pixels to the display */
void pngDraw(PNGDRAW *pDraw) {
  uint16_t lineBuffer[MAX_IMAGE_WDITH];
  png.getLineAsRGB565(pDraw, lineBuffer, PNG_RGB565_BIG_ENDIAN, 0xffffffff);
  tft.pushImage(xpos, ypos + pDraw->y, pDraw->iWidth, 1, lineBuffer);
}

// Function that displays current at top center of LCD
void amp_display(const GFXfont *font) {

  tft.setTextColor(TFT_RED);           //Sets color of text to red
  tft.setFreeFont(font);               // Selects the font
  tft.setTextDatum(TC_DATUM);          // Adjusts reference point of text generation to top center
  sprintf(string, "%.0f", wave.ampPeak);    // stores peak amp value into string to be printed to LCD
  tft.drawString(string, ampX, ampY);  // Print the test text in the custom font
}

// Function that displays time in bottom right corner of LCD
void time_display(const GFXfont *font) {

  tft.setTextColor(TFT_WHITE);                                                                               // Sets color of text to white
  tft.setFreeFont(font);                                                                                     // Selects the font
  tft.setTextDatum(BR_DATUM);                                                                                //Adjusts reference point of text generation to bottom right
  sprintf(string, "%.2f%s", wave.correct_time(), "s");  // stores corrected time value into string to be printed to LCD

  //Check that corrected time is above 0 seconds. If time negative then print N/A instead
  if (wave.correct_time() > 0) {
    tft.drawString(string, timeX, timeY);  // prints string to LCD screen
  } else {
    tft.drawString("N/A", timeX, timeY);  // prints string to LCD screen
  }
}

// Function that displays the peak voltage in bottom left corner of LCD
void peak_display(const GFXfont *font) {

  tft.setTextColor(TFT_WHITE);                // Sets color of text to white
  tft.setFreeFont(font);                      // Select the font
  tft.setTextDatum(BL_DATUM);                 // Adjusts reference point of text generation to bottom left
  sprintf(string, "%.0f%s", wave.voltPeak, "mv");  // stores voltage peak into string to be printed to LCD
  tft.drawString(string, peakX, peakY);       // prints string to LCD screen
}

// Function that converts mV to Amps using a formula made from excel plots of legacy build
float mVtoAmp(float x) {
  float amp_convert = 0;
  amp_convert = -0.000653 + 0.00324 * x + 0.00000841 * x * x;  //2nd polynomial formula
  return amp_convert * 1000;
}



//===================================================================================
//-------------------DIP SWITCH RECONFIGURATION-------------------------
//===================================================================================
// interrupt that will call reconfigure function
void IRAM_ATTR CONFIG_INTERRUPT() {
  reconfigure();
}
// read the dip switches and change the multiplier of the voltage to amps based on the dip switch readings
void reconfigure() {
  int mtp[4];    //array for storing multiplexer DIP SWITCH values. Used to configure start up settings like what the multiplier value will be
  int amp = 20;  //the multiplier to convert mV to Amps
  mtp[0] = digitalRead(10);
  mtp[1] = digitalRead(18);
  mtp[2] = digitalRead(44);
  mtp[3] = digitalRead(43);
  if (mtp[0] == 1 && mtp[1] == 1) { amp = 4; }  //D10 Off, D18 Off
  else if (mtp[0] == 1 && mtp[1] == 0) {
    amp = 20;
  }                                                   // D10 Off, D18 On
  else if (mtp[0] == 0 && mtp[1] == 1) { amp = 20; }  //D10 On, D18 Off
  else if (mtp[0] == 0 && mtp[1] == 0) {
    amp = 20;
  }                   //D10 On, D18 On
  else { amp = 20; }  //D10 On, D18 On
}



