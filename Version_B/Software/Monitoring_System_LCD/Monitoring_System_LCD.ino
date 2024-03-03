/*
* Company: Solid State Systems
* Author: Aaron Nguyen
* Project: Monitoring System Version C (LCD + Hardware Filter)
*
* Description: Secured Solutions Group program using the LilyGo T-Display S3 (170x320 pixels) to make a custom ammeter/timer for client. Communicates with external ADC via I2C protocol using it to read differential
* voltage across varying shunts from 1000Amps/100mV to 1000Amps/25mV to detect the amperage. Uses software average and peak detector to obtain correct voltage. 
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
#include <esp_task_wdt.h>

/* This is required on ESP32 to put the ISR in IRAM. Define as
empty for other platforms. Be careful - other platforms may have
 other requirements. */
#ifndef IRAM_ATTR
#define IRAM_ATTR
#endif

//3 seconds WDT
#define WDT_TIMEOUT 10

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
float ampX = 160;   // display x axis pixel coordinate
float ampY = 10;    // display x axis pixel coordinate
float timeX = 320;  // display x axis pixel coordinate
float timeY = 170;  // display y axis pixel coordinate
float voltX = 0;    // display x axis pixel coordinate
float voltY = 170;  // display y axis pixel coordinate

//Variables needed for Solid State Systems company Logo LCD display
PNG png;           // PNG decoder instance
int16_t xpos = 0;  // x-coordinate of logo
int16_t ypos = 0;  // y-coordinate of logo

Adafruit_ADS1015 ads;
char task0_string[50];               //string to hold display data, non-essential but useful for debugging and logging
char task1_string[50];               //string to hold display data, non-essential but useful for debugging and logging
const int WAVEFORM_MAX_SIZE = 2890;  // amount of samples stored. Number chosen for 1 seconds worth of samples. Sampling rate: 1450 samples/sec
const int SENS_SIZE = 10;            //size of array that checks for consistent voltage above threshold sensitivity
bool hardware_filter = 1;            // changes hardware filter; 0 is off and 1 is on
bool old_hardware_filter = 1;        // keeps track of previous hardware filter value in order to detect change

//Function used in the qsort function argument. Customize it based on what the size of the data type and if you want ascending sort or descending sort
int comp(const void *elem1, const void *elem2);

// Structure containing all data used to capture and modify the signal data
class SignalInfo {

private:
  int shunt = 0;               // indicates what shunt is being used
  float sensorVal = 0;         // sensorVal
  float startTime = 0;         // marked start time of waveform
  float endTime = 1;           // marked end time of waveform
  uint sample_count = 0;       // holds the count of samples
  float waveform_time = 0.11;  // The waveform's true length of time in seconds
  float displayTime = 0.11;    // Display time variable in seconds
  int enter_sens = 2;          // input sensitivity
  int exit_sens = 2;
  float latency = 0.0;  //0 ms latency (customizable)
  float counting_rate = 0;
  float ampPeak = 0.00;
  float voltPeak = 0;
  const float front_trunc = 0.4;       // 40% amount of samples truncated at the beginning part of the array storing sensor values.
  const float back_trunc = 0.2;        // 20% amount of samples truncated at the end part of the array storing sensor values.
  const float back_sort_trunc = .001;  // No HW Filter: .1% amount of samples truncated at the end part of the sorted array
  const float hw_front_trunc = 0.4;    // HW Filter: 20% amount of samples truncated at the beginning part of the array storing sensor values.
  const float hw_back_trunc = 0.4;     // HW Filter: 20% amount of samples truncated at the end part of the array storing sensor values
public:

  bool complete_flag;                                 // flag that says waveform has been captured
  CircularBuffer<float, SENS_SIZE> sensBuffer;        //Buffer that checks filters out rare bad readings from good readings
  CircularBuffer<float, WAVEFORM_MAX_SIZE> Waveform;  //Buffer that checks filters out rare bad readings from good readings
  CircularBuffer<float, WAVEFORM_MAX_SIZE> *Waveformptr = &Waveform;



  //checks if valid waveform is exists (Essentially differentiates between a waveform or noise)
  bool waveform_exist() {

    esp_task_wdt_reset();
    sensBuffer.push((float)ads.getLastConversionResults());  // polling for sensor values to see when there is actually significant voltage (voltage above the threshold value)
    // checks if last two values are from a valid waveform
    for (int i = SENS_SIZE - 2; i < SENS_SIZE; ++i) {
      if (abs(sensBuffer[i]) <= enter_sens) {
        return false;  // False, waveform does not exist, only low voltage detected
      }
    }
    return true;  // True, waveform exists, significant voltage detected
  }

  //capture waveform
  void waveform_capture() {
    esp_task_wdt_reset();
    Waveform.clear();  // Clears buffer so it doesn't have left over values
    sample_count = 0;
    startTime = millis();  //marks the beginning time of the waveform
      // if newest ADC readings below a certain sensitivity then end capture
    while (!(waveform_ended())) {
      if (old_hardware_filter != hardware_filter) {
        old_hardware_filter = hardware_filter;
        if (hardware_filter == 1) {
          //turn on hardware filter
          ads.startADCReading(ADS1X15_REG_CONFIG_MUX_DIFF_0_1, /*continuous=*/true);  // puts ADC into continous mode hardware filter
        } else {
          //turn off hardware filter
          ads.startADCReading(ADS1X15_REG_CONFIG_MUX_DIFF_2_3, /*continuous=*/true);  // puts ADC into continous mode hardware filter
        }
      }
      sensorVal = (float)ads.getLastConversionResults();  // polling for sensor values to see when there is actually significant voltage (voltage above the threshold value)
      sample_count++;
      debug("SensorValue: ");
      debugln(sensorVal);
      Waveform.push(sensorVal);
      sensBuffer.push(sensorVal);
    }
    // If samples are really low then return and
    if (sample_count < 200) {
      debugln("Low Samples");
      return;
    }
    endTime = millis();  //signals end of timer
    set_volt_peak();     //sets the peak voltage of the waveform
    set_amp_peak();      // sets the peak amp value of the waveform
    print_wave_form();   //prints the waveform

    waveform_time = ((endTime - startTime) / 1000);
    displayTime = correct_time();  // Corrects time and outputs it to display
    displayTime += latency;        //accounts for latency issues in tracking


    samps_per_sec();
    debug("voltPeak: ");
    debugln(voltPeak);
    debug("Samples per second is: ");
    debugln(counting_rate);
    debug("The size is: ");
    debugln(Waveform.size());
    debug("The seconds is: ");
    debugln(waveform_time);
    complete_flag = 1;
  }

  //checks if waveform ended (Sensor only picks up negligible voltages)
  bool waveform_ended() {
    // checks if last 8 values have been below the exit sensitivity threshold
    for (int i = sensBuffer.size() - 8; i < sensBuffer.size(); ++i) {
      if (abs(sensBuffer[i]) >= exit_sens) {
        return false;  // False, waveform has not ended
      }
    }
    return true;  // True, Waveform has ended
  }
  //sets the type of shunt being used
  void shunt_type(int type) {
    shunt = type;
  }
  float samps_per_sec() {
    counting_rate = sample_count / (waveform_time);
    return counting_rate;
  }
  //returns display time
  float timerDisplay() {
    return displayTime;
  }
  //Prints values inside wwaveform
  void print_wave_form() {
    for (int i = 0; i < Waveform.size(); ++i) {
      sprintf(task0_string, "Array Value[%d] = %.0f", i, Waveform[i]);
      Serial.println(task0_string);
    }
  }


  /* Corrects display time from being too long due to while loop not exiting correctly
 Takes an array pointer as an argument, peak value, and sampling rate
 Reverse iterates through the array until peak value is reached, counts how many elements
 it iterated through then subtracts that from display time */
  //=========================================v==========================================
  float correct_time() {
    int iterate_count = 0;
    for (int i = (Waveform.size() - 1); i >= 0; --i) {
      //if value is greater than 90% of peak value, break out of for loop
      if (Waveform[i] > (.9 * voltPeak)) {
        break;
      }
      iterate_count++;
    }
    return (waveform_time - (iterate_count / counting_rate));
  }

  //Takes in circular buffer of graph points and returns the peak of the buffer
  float set_volt_peak() {
    int wave_size = Waveform.size();  //size of waveform

    //Variables deciding how much of the waveform to modify based on how long waveform is
    int wave_start = .4 * wave_size;       //Cuts the front of waveform by changing the index the sort begins
    int wave_end = .2 * wave_size;         //Cuts the back of waveform by ending for loop early
    int wave_sort_end = .001 * wave_size;  //Cuts any peak values from minor noise

    // Truncate 40% off front and 20% off back. Sort. Cut .1% off peak value and return peak value
    CircularBuffer<float, WAVEFORM_MAX_SIZE> Sorted_Waveform;

    for (int i = wave_start; i < (wave_size - wave_end); ++i) {
      Sorted_Waveform.push(Waveform[i]);
    }

    qsort(&Sorted_Waveform, Sorted_Waveform.size() + 1, sizeof(float), comp);  //qsort has strange bug where last entry in array is not sorted
    voltPeak = Sorted_Waveform[Sorted_Waveform.size() - 2 - wave_sort_end];    // - 1 for index starting at 0 and -1 for qsort bug mentioned in above comment
    for (int i = 0; i < Sorted_Waveform.size(); ++i) {
      sprintf(task0_string, "Sorted Value[%d] = %.0f", i, Sorted_Waveform[i]);
      Serial.println(task0_string);
    }
    Sorted_Waveform.clear();
    return voltPeak;
  }

  /*returns the peak amps for the waveform depending on what shunt is attached 
  [Shunt 1: 1000/25 shunt] [Shunt 2: 1000/50 shunt] [Shunt 3: 1000/100 shunt] */
  float set_amp_peak() {
    switch (shunt) {
      case 0:
        ampPeak = 10 * volt_peak();
        return ampPeak;
        break;
      case 1:
        ampPeak = 20 * volt_peak();
        return ampPeak;
        break;
      case 2:
        ampPeak = 40 * volt_peak();
        return ampPeak;
        break;
      default:
        ampPeak = 40 * volt_peak();
        return ampPeak;
    }
  } /*returns the peak amps for the waveform */
  float amp_peak() {
    return ampPeak;
  }

  /*returns the peak amps for the waveform */
  float volt_peak() {
    return voltPeak;
  }
  /* Function that converts peak mv to Amps using a formula made from excel plots of legacy build. 
    Old build had similar amp readings but not quite the same. used a formula to adjust for its error*/
  float Amp_peak_legacy() {
    return (1000 * (-0.000653 + 0.00324 * voltPeak + 0.00000841 * voltPeak * voltPeak));  //2nd polynomial formula.
  }
} wave;

//Draws png for company logo
void pngDraw(PNGDRAW *pDraw);

//LCD Text Display functions
void amp_display(const GFXfont *font);
void time_display(const GFXfont *font);
void volt_display(const GFXfont *font);


//INTERRUPTS
void IRAM_ATTR CONFIG_INTERRUPT();  // interrupt that will call reconfigure function
void IRAM_ATTR ADS_READING();       // Changes between hardware filter and no hardware filter

//read the dip switches and change the multiplier of the voltage to amps based on the dip switch readings
void reconfigure();


// Core definitions (Used for dual-core ESP32)
static const BaseType_t pro_cpu = 0;  // Core 0
static const BaseType_t app_cpu = 1;  // Core 1

// Pins
static const int pin_1 = 38;  // LED pin

// Globals
static SemaphoreHandle_t bin_sem;

// Tasks

// Task in Core 0
void doTask0(void *parameters) {

  // Subscribe this task to TWDT, then check if it is subscribed
  ESP_ERROR_CHECK(esp_task_wdt_add(NULL));
  ESP_ERROR_CHECK(esp_task_wdt_status(NULL));

  while (1) {

    if (old_hardware_filter != hardware_filter) {
      old_hardware_filter = hardware_filter;
      if (hardware_filter == 1) {
        //turn on hardware filter
        ads.startADCReading(ADS1X15_REG_CONFIG_MUX_DIFF_0_1, /*continuous=*/true);  // puts ADC into continous mode hardware filter
      } else {
        //turn off hardware filter
        ads.startADCReading(ADS1X15_REG_CONFIG_MUX_DIFF_2_3, /*continuous=*/true);  // puts ADC into continous mode hardware filter
      }
    }
    if (wave.waveform_exist()) {
      wave.waveform_capture();
    }
  }
}

// Task in Core 1
void doTask1(void *parameters) {

  // Subscribe this task to TWDT, then check if it is subscribed
  ESP_ERROR_CHECK(esp_task_wdt_add(NULL));
  ESP_ERROR_CHECK(esp_task_wdt_status(NULL));

  // Do forever
  while (1) {
    vTaskDelay(500 / portTICK_PERIOD_MS);
    esp_task_wdt_reset();
    if (wave.complete_flag == 1) {
      tft.fillScreen(TFT_BLACK);  // Clear screen


      // if amp display more than 4 digits then change font size to smaller size
      if (wave.amp_peak() > 9999) {
        amp_display(AMPFONT);  // displays amp in top center of LCD
      } else {
        amp_display(AMPFONT70);  // displays amps in top center of LCD
      }

      time_display(TIMEFONT);  //displays time in bottom right of LCD
      volt_display(TIMEFONT);  // displays voltage in bottom left of LCD

      wave.complete_flag = 0;
    }
  }
}

//*****************************************************************************
// Main (runs as its own task with priority 1 on core 1)

void setup() {


  pinMode(15, OUTPUT);  // to boot with battery...
  digitalWrite(15, 1);  // and/or power from 5v rail instead of USB


#if !CONFIG_ESP_TASK_WDT_INIT
    // If the TWDT was not initialized automatically on startup, manually intialize it now
  esp_task_wdt_init(WDT_TIMEOUT, true);  //enable panic so ESP32 restarts
  printf("TWDT initialized\n");
#endif  // CONFIG_ESP_TASK_WDT_INIT

  Serial.begin(250000);
  vTaskDelay(500 / portTICK_PERIOD_MS);
  Serial.println("Testing");
  vTaskDelay(500 / portTICK_PERIOD_MS);
  pinMode(10, INPUT_PULLUP);     //D10 multiplexer input
  pinMode(11, INPUT_PULLUP);     //D18 multiplexer input
  pinMode(43, INPUT_PULLUP);     //D44 multiplexer input
  pinMode(44, INPUT_PULLUP);     //D43 multiplexer input
  pinMode(17, INPUT_PULLUP);     //D14 Onboard Button for Hardware Filter Config
  pinMode(18, INPUT_PULLUP);     //D14 Onboard Button for Hardware Filter Config
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
      vTaskDelay(3000 / portTICK_PERIOD_MS);
    }
  }
  ads.setGain(GAIN_TWO);  // 2x gain   +/- 2.048V  1 bit = 1mV | // 4x gain   +/- 1.024V  1 bit = 0.5mV

    //turn on hardware filter
    ads.startADCReading(ADS1X15_REG_CONFIG_MUX_DIFF_0_1, /*continuous=*/true);  // puts ADC into continous mode hardware filter
  // Set up the GPIO pin for the external interrupt
  reconfigure();
  attachInterrupt(digitalPinToInterrupt(10), CONFIG_INTERRUPT, CHANGE);
  attachInterrupt(digitalPinToInterrupt(11), CONFIG_INTERRUPT, CHANGE);
  attachInterrupt(digitalPinToInterrupt(43), CONFIG_INTERRUPT, CHANGE);
  attachInterrupt(digitalPinToInterrupt(44), CONFIG_INTERRUPT, CHANGE);

  //                                                                ADS1015  ADS1115
  //                                                                -------  -------
  // ads.setGain(GAIN_TWOTHIRDS);  // 2/3x gain +/- 6.144V  1 bit = 3mV      0.1875mV (default)
  // ads.setGain(GAIN_ONE);        // 1x gain   +/- 4.096V  1 bit = 2mV      0.125mV
  // ads.setGain(GAIN_TWO);        // 2x gain   +/- 2.048V  1 bit = 1mV      0.0625mV
  // ads.setGain(GAIN_FOUR);       // 4x gain   +/- 1.024V  1 bit = 0.5mV    0.03125mV
  // ads.setGain(GAIN_EIGHT);      // 8x gain   +/- 0.512V  1 bit = 0.25mV   0.015625mV
  // ads.setGain(GAIN_SIXTEEN);    // 16x gain  +/- 0.256V  1 bit = 0.125mV  0.0078125mV


  vTaskDelay(500 / portTICK_PERIOD_MS);
  Serial.println("ADC Range: +/- 2.048V  1 bit = 1mV");
  ads.begin();
  Serial.println("C1.104");  //Version number. 1st digit DC or AC (1 DC, 2 AC). 2nd digit hardware version updates. 3rd and 4th are for software version updates

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
  tft.drawString("C1.104", 320, 170);    // Print the version number in the bottom right
  vTaskDelay(1000 / portTICK_PERIOD_MS);
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

  tft.setTextColor(TFT_RED);                       //Sets color of text to red
  tft.setFreeFont(font);                           // Selects the font
  tft.setTextDatum(TC_DATUM);                      // Adjusts reference point of text generation to top center
  sprintf(task1_string, "%.0f", wave.amp_peak());  // stores peak amp value into string to be printed to LCD
  tft.drawString(task1_string, ampX, ampY);        // Print the test text in the custom font
}

// Function that displays time in bottom right corner of LCD
void time_display(const GFXfont *font) {

  tft.setTextColor(TFT_WHITE);                                // Sets color of text to white
  tft.setFreeFont(font);                                      // Selects the font
  tft.setTextDatum(BR_DATUM);                                 //Adjusts reference point of text generation to bottom right
  sprintf(task1_string, "%.2f%s", wave.timerDisplay(), "s");  // stores corrected time value into string to be printed to LCD

  //Check that corrected time is above 0 seconds. If time negative then print N/A instead
  if (wave.timerDisplay() > 0) {
    tft.drawString(task1_string, timeX, timeY);  // prints string to LCD screen
  } else {
    tft.drawString("N/A", timeX, timeY);  // prints string to LCD screen
  }
}

// Function that displays the voltage in bottom left corner of LCD
void volt_display(const GFXfont *font) {
  float volt_peak = wave.volt_peak();
  tft.setTextColor(TFT_WHITE);                       // Sets color of text to white
  tft.setFreeFont(font);                             // Select the font
  tft.setTextDatum(BL_DATUM);                        // Adjusts reference point of text generation to bottom left
  sprintf(task1_string, "%.0f%s", volt_peak, "mv");  // stores voltage peak into string to be printed to LCD
  if (volt_peak > 0) {
    tft.drawString(task1_string, voltX, voltY);  // prints string to LCD screen
  } else {
    tft.drawString("N/A", voltX, voltY);  // prints string to LCD screen
  }
}





//===================================================================================
//-------------------DIP SWITCH RECONFIGURATION-------------------------
//===================================================================================
// interrupt that will call reconfigure function
void IRAM_ATTR CONFIG_INTERRUPT() {
  reconfigure();
}
void IRAM_ATTR ADS_READING() {
  hardware_filter = !hardware_filter;
}
// read the dip switches and change the multiplier of the voltage to amps based on the dip switch readings
void reconfigure() {
  int mtp[4];  //array for storing multiplexer DIP SWITCH values. Used to configure start up settings like what the multiplier value will be
  mtp[0] = digitalRead(44);
  mtp[1] = digitalRead(43);
  mtp[2] = digitalRead(11);
  mtp[3] = digitalRead(10);
  if (mtp[0] == 1 && mtp[1] == 1) {
    wave.shunt_type(0);  //D44 Off, D43 Off
  } else if (mtp[0] == 1 && mtp[1] == 0) {
    wave.shunt_type(1);  // D44 Off, D43 On
  } else if (mtp[0] == 0 && mtp[1] == 1) {
    wave.shunt_type(2);  //D44 On, D43 Off

  } else if (mtp[0] == 0 && mtp[1] == 0) {
    wave.shunt_type(3);  //D44 On, D43 On
  } else {
    wave.shunt_type(3);  //D44 On, D43 On
  }
  hardware_filter = 1;  // make hardware filter reading always active
}
