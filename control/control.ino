#include <Servo.h>
#include "Braccio.h"

void setup() {
  Serial.begin(57600);
  Braccio.begin();
}

byte b,s,e,wv,wr,g;

void loop() {
  
  while (Serial.available() < 7);
  if(Serial.read()==255){
    b=Serial.read();
    s=Serial.read();
    e=Serial.read();
    wv=Serial.read();
    wr=Serial.read();
    g=Serial.read();
    Braccio.ServoMovement(20, b,s,e,wv,wr,g);
//    Serial.write(b);
//    Serial.write(s);
//    Serial.write(e);
//    Serial.write('\n');
  }
}
