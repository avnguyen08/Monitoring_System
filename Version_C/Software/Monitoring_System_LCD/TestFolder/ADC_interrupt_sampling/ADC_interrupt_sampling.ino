

#include <Adafruit_ADS1X15.h>  // ADC Library

#define I2C_SDA 12  // I2C Data pin for ADC
#define I2C_SCL 13  // I2C Clock pin for ADC
Adafruit_ADS1015 ads;
// Pin connected to the ALERT/RDY signal for new sample notification.
constexpr int READY_PIN = 1;

// This is required on ESP32 to put the ISR in IRAM. Define as
// empty for other platforms. Be careful - other platforms may have
// other requirements.
#ifndef IRAM_ATTR
#define IRAM_ATTR
#endif

volatile bool new_data = false;
void IRAM_ATTR NewDataReadyISR() {
  new_data = true;
}

float counter = 0;
float timeStart = 0;
float timeEnd = 0;
bool flag = 0;
float timeFinal = 0;
void setup() {
  // put your setup code here, to run once:
  Serial.begin(250000);
  Wire.begin(I2C_SDA, I2C_SCL);  // Initializes communication with ADC
  if (!ads.begin()) {
    while (1) {
      Serial.println("Failed to initialize ADS.");
      delay(3000);
    }
  }
  pinMode(READY_PIN, INPUT);
  // We get a falling edge every time a new sample is ready.
  attachInterrupt(digitalPinToInterrupt(READY_PIN), NewDataReadyISR, FALLING);
  ads.startADCReading(ADS1X15_REG_CONFIG_MUX_DIFF_0_1, /*continuous=*/true);
  Serial.println("Set up complete");
  ads.setDataRate(RATE_ADS1015_3300SPS);
  timeStart = millis();
}

void loop() {
  while (counter < 10000) {
    // If we don't have new data, skip this iteration.
    if (!new_data) {
      continue;
    }
    float sensorVal = ads.getLastConversionResults();
    new_data = false;
    ++counter;
  }
  if (flag == 0){
  timeEnd = millis();
  timeFinal = (timeEnd - timeStart) / 1000;
  Serial.print("Final time is: ");
  Serial.println(timeFinal);
  Serial.print("Samples per second is: ");
  Serial.println(counter / timeFinal);
}  flag = 1;
}