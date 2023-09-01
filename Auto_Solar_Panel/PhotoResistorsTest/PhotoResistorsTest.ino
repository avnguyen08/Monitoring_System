/*
SUMMARY: Auto adjusting solar panel. Uses esp32 wroom. Example code taken from Great Scott Solar panel auto adjusting video. 
Uses pins d19, d21, d32, d35 as analog read. 
Uses D2 and D4 as output to configure solar panels.


*/
//DEBUG VARIABLES
char string[50];

//value of photoresistors
int topleft;
int topright;
int downleft;
int downright;
int waittime = 10;

int PINA = 25;
int PINB = 33;
int PINC = 32;
int PIND = 35;
// Recommended PWM GPIO pins on the ESP32 include 2,4,12-19,21-23,25-27,32-33 
int servoPinTop = 2;
int servoPinBot = 4;
void setup() {
  Serial.begin(19200);

    pinMode(PINA, INPUT);
    pinMode(PINB, INPUT);
    pinMode(PINC, INPUT);
    pinMode(PIND, INPUT);
    Serial.println("Hello World");
}

void loop() {
  topleft = analogRead(PINA);
  topright = analogRead(PINB);
  downleft = analogRead(PINC);
  downright = analogRead(PIND);
    sprintf(string, "Pin %d: %d", PINA, topleft);
    Serial.println(string);
    sprintf(string, "Pin %d: %d", PINB, topright);
    Serial.println(string);
    sprintf(string, "Pin %d: %d", PINC, downleft);
    Serial.println(string);
    sprintf(string, "Pin %d: %d \n", PIND, downright);
    Serial.println(string);
    delay(2000);
}
