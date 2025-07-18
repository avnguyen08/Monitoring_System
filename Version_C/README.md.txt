#Monitoring System Version C (LCD)

### Summary
- Tracks varying levels of high amperage bursts and accurately detects its amount of time and amperage
- Displays both values simultaneously on LCD screen for clients to see.
- Bill of Materials can be found in local csv file


### Hardware Details
- LILYGO T-Display S3 contains the microcontroller and the LCD screen
- ADS1015 is the ADC 
- 3.9 MOhm pull-down resistors attached to ADC leads
- A0 and A1 leads pass through the hardware filter. A2 and A3 leads go out to socket connector

### Hardware Filter
- The hardware filter helps filter machines with heavy noise to produce readable voltage levels.
- Two potentiometers are used to calibrate filter



