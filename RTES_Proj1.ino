//LIBRARIES

#include "ESP8266WiFi.h"
#include <Wire.h>
#include <Adafruit_MotorShield.h>
#include <NewPingESP8266.h>

//DECLARATIONS

Adafruit_MotorShield AFMS = Adafruit_MotorShield(); 
Adafruit_DCMotor *motorR = AFMS.getMotor(3);
Adafruit_DCMotor *motorL = AFMS.getMotor(4);

long range;
long prev_range = -10000;
long minimum = 0;
long rangeQueue;

// WiFi Code
#ifndef APSSID
#define APSSID "nondeterministic"
#define APPSK  "polynomial time"
#endif

/* Set these to your desired credentials. */
const char *ssid = APSSID;
const char *password = APPSK;

#define TRIGGER_PIN  14
#define ECHO_AHEAD   13
#define ECHO_LEFT    16

//Uses only two sensors because right sensor broke. 
#define MAX_DISTANCE_AHEAD 80
#define MAX_DISTANCE_LEFT  80

NewPingESP8266 sonarAhead(TRIGGER_PIN, ECHO_AHEAD, MAX_DISTANCE_AHEAD);
NewPingESP8266 sonarLeft(TRIGGER_PIN, ECHO_LEFT, MAX_DISTANCE_LEFT);

int rightTries = 0;//Keeps track of left and right 
int leftTries = 0;


//FUNCTIONS

void turnRight(int spd, int seconds) {
  motorR->setSpeed(spd);
  motorL->setSpeed(spd);
  motorR->run(FORWARD);
  motorL->run(BACKWARD);
  delay(seconds);
  motorL->run(RELEASE);
  motorR->run(RELEASE);
}

void turnLeft(int spd, int seconds) {
  motorL->setSpeed(spd);
  motorR->setSpeed(spd);
  motorL->run(FORWARD);
  motorR->run(BACKWARD);
  delay(seconds);
  motorL->run(RELEASE);
  motorR->run(RELEASE);
}

void straight(int spd, int seconds) {
  motorL->setSpeed(spd);
  motorR->setSpeed(spd);
  motorL->run(FORWARD);
  motorR->run(FORWARD);
  delay(seconds);
  motorL->run(RELEASE);
  motorR->run(RELEASE);
  
}

void back(int spd, int seconds) {
  motorL->setSpeed(spd);
  motorR->setSpeed(spd);
  motorL->run(BACKWARD);
  motorR->run(BACKWARD);
  delay(seconds);
  motorL->run(RELEASE);
  motorR->run(RELEASE);
}

void turnAround(int spd) {
  motorL->setSpeed(spd);
  motorR->setSpeed(spd);
  motorL->run(FORWARD);
  motorR->run(BACKWARD);
  delay(800);
  motorL->run(RELEASE);
  motorR->run(RELEASE);
}

void left90(){
  turnLeft(80,800);
}

void left45(){
  turnLeft(80,400);
}

void right90(){
  turnRight(80,800);
}

void right45(){
  turnRight(80,400);
}


bool pingaSearch() { //returns 1 if obstruction is within threshold
  return (sonarAhead.ping_cm() || sonarLeft.ping_cm());
}

//Turns depending on 2 sensor input returns octagon increment mod 8 for angle tracking
//turns to face clear space
int avoidObstacle() {
  int leftDist = sonarLeft.ping_cm();
  int rightDist = 0;
  int aheadDist = sonarAhead.ping_cm();

  Serial.println(leftDist);
  Serial.println(aheadDist);
  if( aheadDist == 0 && leftDist == 0 ) return 0;
  if( aheadDist < 80 && leftDist < 80 ) return 0; //Buffers glitches
  if( leftDist == 0 && aheadDist < 20) {//clear left, turn 45deg
    left45();
    return 1;
  } else if ((leftDist > aheadDist)){
    left90();
    return avoidObstacle() + 2;
  } else if (leftDist > 30 && rightDist > 30){
    right90();
    return 6 + avoidObstacle();//recursive because right sensor broke
  } else {
    return 0; //doesn't change angle and returns 0 if no obstacles detected
  }
}

void setup()
{
  AFMS.begin();
  Serial.begin(115200);
  Serial.println();
  
  WiFi.mode(WIFI_STA);
  //WiFi.disconnect();
  WiFi.begin(ssid, password);
  //WiFi.begin(ssid);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  
  //Test motor functions
  right90();
  left90();
  left45();
  turnAround(80);
}

void loop() {
  // Takes an average of signal strength
  // 500ms is best for most stable readings (still pretty garbage though)
  range = 0;
  for (int i = 0; i < 5; i++) {
    range += WiFi.RSSI(); 
    delay(500);
  }
  range = range/5;
  
  if (range < -35) { // This makes it stop when its close enough to beacon (change to whatever or take out if needed)
  Serial.print("Range: ");
  Serial.println(range);
  Serial.print("Prev Range: ");
  Serial.println(prev_range);
  Serial.print("Left Tries: ");
  Serial.println(leftTries);


    if ((range-3) > prev_range ) {//if we get a significant increase, keep going!
      Serial.println("Clear ahead");
      if((sonarAhead.ping_cm() == 0)) straight(80, 3200);
      //if there is significant progress, default to center
      leftTries = 0;
      

      
    } else if ((range-1) >= prev_range ) {//if we are making marginal progress back up turn left
      Serial.println("marginal progress");
        back(50, 500);
        if(sonarLeft.ping_cm() == 0 && sonarAhead.ping_cm() == 0) {
          left45();
          leftTries++;//increment the left turns taken
          avoidObstacle();
        }
        
        //tries to break a loop because it made over a 1/4 turn 
        //so it goes right 90deg to correct itself and tries to go straight.
         
          if (leftTries >= 2){ 
            right45();
            right90();
            leftTries = 0;
            leftTries += avoidObstacle();
            //scans for obstacle and increments octagon
            
            if(!sonarAhead.ping_cm()) straight(80, 1600);  
                     
          } else if(leftTries > 5){ 
            //after turning left for 270deg it will turn around to cancel out 180deg
            right90();
            avoidObstacle();
            
            if(sonarLeft.ping_cm() == 0 && sonarAhead.ping_cm() == 0) straight(80,1600);
            leftTries = 0 
          } else {
            //default to going forward
            leftTries += avoidObstacle();
            
            if(!sonarAhead.ping_cm()) straight(80,  1600);
            leftTries %= 8;
            leftTries++;
          }
    } else if((range+4) < prev_range) {
        Serial.println("Going out of range");
        //If signal drops we turn around depending on where we are on the octagon.
        right90();
        right90();
        leftTries += avoidObstacle();
        leftTries %= 8;
        
        if(sonarAhead.ping_cm() == 0) straight(80, 1600);
        leftTries = 0;
        
    } else if((range - 1) <= prev_range) {
      Serial.println("Keep turning");
      //Defaults to going turning left and going straight, 
      //this is to compensate for fluctuations in RSSI
        
        back(50, 500);
        if(sonarLeft.ping_cm() == 0) {
          left45();
          leftTries += avoidObstacle();
          leftTries %= 8;
          
          if(sonarLeft.ping_cm() == 0 && sonarAhead.ping_cm() == 0) straight(80, 800);
        }
    } else {
      //Catches missed cases, defaults to left. 
        Serial.println("Going left");
        back(50, 1000);
        if(sonarLeft.ping_cm() == 0) left45();
        avoidObstacle();
        
        if(sonarLeft.ping_cm() == 0 && sonarAhead.ping_cm() == 0) straight(80,1600);
        leftTries++;
    }
  prev_range = range;
  avoidObstacle(); //Catches itself if it ends up facing a wall
  }
}
