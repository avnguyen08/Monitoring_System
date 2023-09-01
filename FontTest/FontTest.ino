/*
 Display all the fonts.

 This sketch uses the GLCD (font 1) and fonts 2, 4, 6, 7, 8
 
 #########################################################################
 ###### DON'T FORGET TO UPDATE THE User_Setup.h FILE IN THE LIBRARY ######
 ######       TO SELECT THE FONTS AND PINS YOU USE, SEE ABOVE       ######
 #########################################################################

 */

// New background colour
#define TFT_BROWN 0x38E0

// Pause in milliseconds between screens, change to 0 to time font rendering
#define WAIT 5000
#include <TFT_eSPI.h> // Graphics and font library for ILI9341 driver chip
#include <SPI.h>
#include "Free_Fonts.h" // Include the header file attached to this sketch

TFT_eSPI tft = TFT_eSPI();  // Invoke library, pins defined in User_Setup.h

unsigned long targetTime = 0; // Used for testing draw times
int xpos = 0;
int ypos = 0;
void setup(void) {
  tft.begin();
  tft.setRotation(1);
  tft.setCursor(xpos, ypos);    // Set cursor near top left corner of screen

  Serial.begin(115200);
  // First we test them with a background colour set
  tft.setFreeFont(FSB24);       // Select Free Serif 24 point font
  tft.print("Serif Bold 24pt"); // Print the font name onto the TFT screen}
  delay(1000);
}

void loop() {

}

