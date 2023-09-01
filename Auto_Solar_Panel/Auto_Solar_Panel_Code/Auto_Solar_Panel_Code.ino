/*
SUMMARY: Auto adjusting solar panel. Uses esp32 wroom. Example code taken from Great Scott Solar panel auto adjusting video. 
Uses pins d19, d21, d32, d35 as analog read. 
Uses D2 and D4 as output to configure solar panels.


*/
#include <ESP32Servo.h>
//DEBUG VARIABLES
char string[50];

//value of photoresistors
int topleft;
int topright;
int downleft;
int downright;
int change = 1;
int waittime = 10;
int light_sens = 500;


int PINA = 25;
int PINB = 33;
int PINC = 32;
int PIND = 35;
Servo myservoBot;  // create servo object to control bottom servo
Servo myservoTop;  // create servo object to control top servo
// 16 servo objects can be created on the ESP32
int minbotval = 20;
int maxbotval = 250;
int mintopval = 45;
int maxtopval = 225;
int posTop = 90;  // variable to store the servo position
int posBot = 90;  // variable to store the servo position
// Recommended PWM GPIO pins on the ESP32 include 2,4,12-19,21-23,25-27,32-33
int servoPinTop = 2;
int servoPinBot = 4;
void setup() {
  Serial.begin(19200);
  pinMode(PINA, INPUT);
  pinMode(PINB, INPUT);
  pinMode(PINC, INPUT);
  pinMode(PIND, INPUT);
  // Allow allocation of all timers
  ESP32PWM::allocateTimer(0);
  ESP32PWM::allocateTimer(1);
  ESP32PWM::allocateTimer(2);
  ESP32PWM::allocateTimer(3);
  myservoBot.setPeriodHertz(70);              // standard 50 hz servo
  myservoTop.setPeriodHertz(70);              // standard 50 hz servo
  myservoBot.attach(servoPinBot, 500, 2450);  // attaches the servo on pin 4 to the servo object
  myservoTop.attach(servoPinTop, 500, 2400);  // attaches the servo on pin 2 to the servo object
  Serial.println("Hello World");
  delay(5000);
  // using default min/max of 1000us and 2000us
  // different servos may require different min/max settings
  // for an accurate 0 to 180 sweep
}

void loop() {
  topleft = analogRead(PINA);
  topright = analogRead(PINB);
  downleft = analogRead(PINC);
  downright = analogRead(PIND);

//light shines on top right side more than top left.  Turn counterclockwise
  if (((topleft - topright) > light_sens) && (posTop < maxtopval)) {
    posTop = posTop + change;
    delay(waittime);
  }
  //light shines on down right side more than down left. Turn counterclockwise
  if (((downleft - downright) > light_sens) && (posTop < maxtopval)) {
    posTop = posTop + change;
    delay(waittime);
  }
  //light shines on top left side more than top right. Turn clockwise 
  if (((topright-topleft) > light_sens) && (posTop > mintopval)) {
    posTop = posTop - change;
    delay(waittime);
  }
  //light shines on down left side more than down right. Turn clockwise 
  if (((downright-downleft) > light_sens) && (posTop > mintopval)) {
    posTop = posTop - change;
    delay(waittime);
  }
  //light shines on downleft side more than top left side. Turn clockwise
  if (((topleft - downleft) > light_sens) && (posBot > minbotval)) {
    posBot = posBot - change;
    delay(waittime);
  }
  //light shines on downright side more than top right side. Turn clockwise
  if (((topright - downright) > light_sens) && (posBot > minbotval)) {
    posBot = posBot - change;
    delay(waittime);
  }
  //light shines on top left side more than down left side. Turn counterclockwise
  if (((downleft- topleft) > light_sens)  && (posBot < maxbotval)) {
    posBot = posBot + change;
    delay(waittime);
  }
  //light shines on top right side more than down right side. Turn counterclockwise
  if (((downright - topright) > light_sens) && (posBot < maxbotval)) {
    posBot = posBot + change;
    delay(waittime);
  }
  sprintf(string, "Before Map. Top = %d, Bot = %d", posTop, posBot);
  Serial.println(string);


  myservoTop.write(map(posTop, 0, 270, 0, 180));  // tell servo to go to position in variable 'pos'
  myservoBot.write(map(posBot, 0, 270, 0, 180));  // tell servo to go to position in variable 'pos'
  // sprintf(string, "Top Servo Position: %d", posTop);
  // Serial.println(string);
  // sprintf(string, "Bot Servo Position: %d", posBot);
  // Serial.println(string);
}
