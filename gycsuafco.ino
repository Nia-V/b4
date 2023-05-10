#include "Display4.h"
#include "Motorclass.h"
#include "Remote.h"
#include "master.h"
#include "UltrasonicSensor.h"

Car myCar;




int D1 = 24;
int D2 = 27;
int D3 = 26;
int D4 = 25;
int latch = 23;
int clock = 22;
int data = 10;


const uint8_t trigPin = 13;
const uint8_t echoPin = 12;
const unsigned int maxDistance = 400;  // Maximum distance supported by the sensor
int angle = 90;
int steve = 0;
int increment = 15;
int time;
int count;
UltrasonicSensor mySensor(trigPin, echoPin, maxDistance);

SevSeg my7(D1, D2, D3, D4, latch, clock, data);


void setup() {
  Wire.begin();
    IrReceiver.begin(9, ENABLE_LED_FEEDBACK);
my7.init();
  Serial.begin(9600);  // begin serial comunication
  mySensor.init();
}

void loop() {
  my7.display(1, 2, 3, 4);
  RemRun();
  if (button == ">>|") {  //if r is equal to 1
    myCar.Right(50);      //Move right at speed 100
  }
  if (button == "VOL+") {  //if u is equal to 1
    myCar.Forwards(50);    //Move forward at speed 100
  }
  if (button == "|<<") {  //if l is equal to 1
    myCar.Left(50);       //Move left at speed 100
  }
  if (button == "VOL-") {  //if d is equal to 1
    myCar.Backwards(50);   //Move backward at speed 100
  }
  if (button == "^^") {
    Serial.println("ri");

  }
  if (button == "vv") {
    Serial.println("le");

  }
  if (button == "ON/OFF") {
    myCar.STOP();

  }
  if(button == "EQ"){
 if (mySensor.isDistanceLessThanOrEqual(30)) {
    steve = 1;
  mySensor.myCar2.STOP();
    if (count%2==0){
    angle -= increment;  // Increment the angle
    }else{
      angle += increment;} 
 mySensor.myServo2.write(angle);
    if (angle >= 150 || angle <= 30) {
      increment = -increment;  // Reverse the direction at the limits
    }
  }
  if (!mySensor.isDistanceLessThanOrEqual(30)) {
    if (steve == 0) {
        mySensor.myCar2.Forwards(50);
mySensor.myServo2.write(90);
      angle = 90;
    }
    if (steve == 1 && angle >= 95) {
time = map(angle, 95, 150, 10, 800);
      mySensor.myCar2.Left(75);
      delay(time);
      steve = 0;
          count++;
      
    }
    if (steve == 1 && angle <= 85) {
time = map(angle, 85, 30, 10, 800);
      mySensor.myCar2.Right(75);
      delay(time);
      steve = 0;
          count++;
    }
  }
}
}

