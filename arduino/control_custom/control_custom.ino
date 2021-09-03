#include <Servo.h>
#include "Braccio.h"

Servo base;
Servo shoulder;
Servo elbow;
Servo wrist_rot;
Servo wrist_ver;
Servo gripper;


#define SOFT_START_CONTROL_PIN 12

void _softwarePWM(int high_time, int low_time){
  digitalWrite(SOFT_START_CONTROL_PIN,HIGH);
  delayMicroseconds(high_time);
  digitalWrite(SOFT_START_CONTROL_PIN,LOW);
  delayMicroseconds(low_time); 
}

/*
* This function, used only with the Braccio Shield V4 and greater,
* turn ON the Braccio softly and save it from brokes.
* The SOFT_START_CONTROL_PIN is used as a software PWM
* @param soft_start_level: the minimum value is -70, default value is 0 (SOFT_START_DEFAULT)
*/
void softStart(int soft_start_level){      
  long int tmp=millis();
  while(millis()-tmp < 2000)
    _softwarePWM(80+soft_start_level, 450 - soft_start_level);   //the sum should be 530usec  

  while(millis()-tmp < 6000)
    _softwarePWM(75 + soft_start_level, 430 - soft_start_level); //the sum should be 505usec

  digitalWrite(SOFT_START_CONTROL_PIN,HIGH);
}


void setup() {
  Serial.begin(57600);
//  Braccio.begin(SOFT_START_DISABLED);
  pinMode(SOFT_START_CONTROL_PIN,OUTPUT);
  digitalWrite(SOFT_START_CONTROL_PIN,LOW);  
  base.attach(11);
  shoulder.attach(10);
  elbow.attach(9);
  wrist_rot.attach(6);
  wrist_ver.attach(5);
  gripper.attach(3);
}

byte b,s,e,wv,wr,g;

void loop() {
  while (Serial.available() < 8);
  if(Serial.read()==255){
    int instruction = Serial.read();
    if(instruction=='G'){
      b=Serial.read();
      s=Serial.read();
      e=Serial.read();
      wv=Serial.read();
      wr=Serial.read();
      g=Serial.read();
  //    Braccio.ServoMovement(20, b,s,e,wv,wr,g);
      base.write(b);
      shoulder.write(s);
      elbow.write(e);
      wrist_ver.write(wv);
      wrist_rot.write(wr);
      gripper.write(g);
    }
    else if(instruction=='S'){
      base.write(b);
      shoulder.write(s);
      elbow.write(e);
      wrist_ver.write(wv);
      wrist_rot.write(wr);
      gripper.write(g);
      softStart(0);
    }

//    Serial.write(b);
//    Serial.write(s);
//    Serial.write(e);
//    Serial.write('\n');
  }
}
