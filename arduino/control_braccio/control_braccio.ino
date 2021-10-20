#include <Servo.h>
#include "Braccio.h"

void setup() {
  Serial.begin(57600);
  Braccio.begin();
}

byte b,s,e,wv,wr,g;

void loop() {
  while (Serial.available() < 2);
  if(Serial.read()==255){
    int instruction = Serial.read();
    if(instruction=='G'){
      while (Serial.available() < 6);
      b=Serial.read();
      s=Serial.read();
      e=Serial.read();
      wv=Serial.read();
      wr=Serial.read();
      g=Serial.read();
      Braccio.ServoMovement(1, b,s,e,wv,wr,g);
    }
  }
}
