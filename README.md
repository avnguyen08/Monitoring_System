# Current Monitoring System 

Various current monitoring software products made on the ESP32 using varied electronic components including: LCDs, external ADC, potentiometers, capacitors, resistors, plug-and-play connectors, and 7 segment displays

### Version A (7-Segment Display)
ESP32 Feather Board paired with a Featherwing 7-Segment Display to display the amount of amps passing through a material and the amount of time the amps passed through for. The external ADC can handle voltage bursts as low as 2mv at 180 ms 
 - Components: ESP32 Feather, external ADC, 7-segment display featherwing, LEDs, potentiometers, capacitors, resistors, socket connectors, pin head connectors, 5V power adapter.
  - Description:
    Cycles between displaying either the current or time based on adjustable potentiometers on back of PCB. 
    External pull down resistors and decoupling cacpacitors used for signal integrity.
    Dip switch in back to provide customizability with regard to different shunts and machines. 
    Machines operate at thousands of current and use a shunt to have readable levels, ranging from 2mv to 4V. 
    Pin headers and socket headers for easy installation into varied machines.

### Version B (LCD Display)
- Components: LilyGo T Display S3, external ADC, capacitors, resistors, socket connectors, pin head connectors, 5V power adapter.
- Description: 
  Communication with LCD using SPI. Communication with external ADC using I2C. ESP32 simulatenously displays the amount of amps passing through a material and the amount of time the amps passed through for. Decoupling Capacitors and pull down resistors used for signal integrety. Pin headers and 
  socket headers for easy installation into varied machines.

### Version C (LCD Display + Hardware Filter)
 - Components: LilyGo T Display S3, external ADC, capacitors, resistors, socket connectors, pin head connectors, 5V power adapter, op amps, potentiometers.
 - Description: Communication with LCD using SPI, Communication with external ADC using I2C. ESP32 simulatenously displays the amount of amps passing through a material and the amount of time the amps passed through for. Decoupling Capacitors and pull down resistors used for signal integrety. Pin headers and socket headers for easy installation into varied machines. Integrated legacy hardware filter used to help clean up severely disorted signals.
