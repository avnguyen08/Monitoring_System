/*
  Example for TFT_eSPI library

  This example shows the use of a Adafruit_GFX custom font with a
  character code range of 32 - 255, this means accented characters
  (amongst others) are available.

  The custom font file is attached to this sketch as a header file. The
  font data has been created following the instructions here:
  https://www.youtube.com/watch?v=L8MmTISmwZ8

  Note that online converters for Adafruit_GFX compatible fonts are
  available but these typically only use characters in the range 32-127,
  and thus do not include the accented characters. These online converters
  can however still be used with this sketch but the example characters
  used must be changed.

  The Arduino IDE uses UTF8 encoding for these characters. The TFT_eSPI
  library also expects characters in the range 128 to 255 to be UTF-8
  encoded. See link here for details:

  https://playground.arduino.cc/Code/UTF-8

  To summarise, UTF-8 characters are encoded as more than 1 byte so care must
  be taken:

     char c = 'µ';          // Wrong
     char bad[4] = "5µA";   // Wrong
     char good[] = "5µA";   // Good
     String okay = "5µA";   // Good

  Created by Bodmer 08/02/19

  Make sure LOAD_GFXFF is defined in the used User_Setup file
  within the library folder.

  #########################################################################
  ###### DON'T FORGET TO UPDATE THE User_Setup.h FILE IN THE LIBRARY ######
  ######   TO SELECT YOUR DISPLAY TYPE, PINS USED AND ENABLE FONTS   ######
  #########################################################################
*/

#define TEST_TEXT "ßäöü ñâàå"   // Text that will be printed on screen in the font
//#define TEST_TEXT "Hello"     // Text that will be printed on screen in the font

#include "SPI.h"
#include "TFT_eSPI.h"

// The custom font file attached to this sketch must be included
#include "MyFont.h"

// Stock font and GFXFF reference handle
#define GFXFF 1

// Easily remembered name for the font
#define MYFONT32 &myFont32pt8b

// Use hardware SPI
TFT_eSPI tft = TFT_eSPI();
int counter = 0;
char str[20];
float time_ccc = 0;
void setup(void) {

  Serial.begin(250000);
  
  tft.begin();

  tft.setRotation(1);
  tft.fillScreen(TFT_BLACK);  // Clear screen

}

void loop() {

  // >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
  // Show custom fonts
  // >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>

  // Where font sizes increase the screen is not cleared as the larger fonts overwrite
  // the smaller one with the background colour.

  // We can set the text datum to be Top, Middle, Bottom vertically and Left, Centre
  // and Right horizontally. These are the text datums that can be used:
  // TL_DATUM = Top left (default)
  // TC_DATUM = Top centre
  // TR_DATUM = Top right
  // ML_DATUM = Middle left
  // MC_DATUM = Middle centre <<< This is used below
  // MR_DATUM = Middle right
  // BL_DATUM = Bottom left
  // BC_DATUM = Bottom centre
  // BR_DATUM = Bottom right
  // L_BASELINE = Left character baseline (Line the 'A' character would sit on)
  // C_BASELINE = Centre character baseline
  // R_BASELINE = Right character baseline

  //Serial.println();
  // Set text datum to middle centre (MC_DATUM)
  tft.setTextDatum(MC_DATUM);

  // Set text colour to white with black background
  // Unlike the stock Adafruit_GFX library, the TFT_eSPI library DOES optionally draw
  // the background colour for the custom and Free Fonts when using drawString()
  //tft.setTextColor(TFT_WHITE);               // or white characters, no background

  tft.setFreeFont(MYFONT32);                   // Select the font
  // Setting textDatum does nothing when using tft.print
  // tft.fillScreen(TFT_BLACK);  // Clear screen

  tft.setCursor(0,60);       // To be compatible with Adafruit_GFX the cursor datum is always bottom left
  tft.print("Amp:");         // Using tft.print means text background is NEVER rendered
 counter++;
sprintf(str, "%.2f", time_ccc);
   time_ccc = (float)millis()/1000;
           tft.fillRoundRect(160,0, 160, 70, 0, TFT_BLACK );
  tft.print(12);
  delay(100);
  
  tft.fillRoundRect(160,80, 160, 70, 0, TFT_BLACK );
  Serial.println(str);

  
  tft.setCursor(0,140);       // To be compatible with Adafruit_GFX the cursor datum is always bottom left
  tft.print("Time:");         // Using tft.print means text background is NEVER rendered
  tft.print(24);
  // Reset text padding to zero (default)
  tft.setTextPadding(0);
}
