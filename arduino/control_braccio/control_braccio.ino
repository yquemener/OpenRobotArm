#include <Servo.h>
#include "Braccio.h"
#include "InverseK.h"

// InverseK links
Link base, upperarm, forearm, hand;

const int PUMP_PIN=4;
const int VALVE_PIN=8;

// Mechanical calibration of links
const int A0_CENTER = 0;
const int A1_CENTER = -3;
const int A2_CENTER = 8;
const int A3_CENTER = 4;

long a0_value = 0;
long a0_count = 0;
long a1_value = 0;
long a1_count = 0;


// Quick conversion from the Braccio angle system to radians
float b2a(float b){
  return b / 180.0 * PI - HALF_PI;
}

// Quick conversion from radians to the Braccio angle system
float a2b(float a) {
  return round((a + HALF_PI) * 180 / PI);
}

void setup() {
  Serial.begin(57600);
  Braccio.begin();
  pinMode(VALVE_PIN, OUTPUT);
  a0_value=analogRead(A0);
  a0_count=1;
  a1_value=analogRead(A1);
  a1_count=1;

  // InverseK initialization  
  base.init(0, b2a(0.0), b2a(180.0));
  upperarm.init(130, b2a(0.0), b2a(162.0));
  forearm.init(125, b2a(0.0), b2a(188.0));
  hand.init(65, b2a(0.0), b2a(184.0));
  InverseK.attach(base, upperarm, forearm, hand);
}

byte b,s,e,wv,wr,g,pump,valve;



void loop() {
  while (Serial.available() < 2){
    a0_value += analogRead(A0);
    a0_count++;
    a1_value += analogRead(A1);
    a1_count++;
  }
  if(Serial.read()==255){
    int instruction = Serial.read();
    if(instruction=='G'){
      while (Serial.available() < 8);
      b=Serial.read();
      s=Serial.read();
      e=Serial.read();
      wv=Serial.read();
      wr=Serial.read();
      g=Serial.read();
      pump=Serial.read();
      valve=Serial.read();
      if(valve==1){
        digitalWrite(VALVE_PIN, HIGH);
      }
      else{
        digitalWrite(VALVE_PIN, LOW);
      }
      if(pump==1){
        digitalWrite(PUMP_PIN, HIGH);
      }
      else{
        digitalWrite(PUMP_PIN, LOW);
      }
      Braccio.ServoMovement(1, b,s,e,wv,wr,g);
    }
    if(instruction=='P'){
      int x,y,z;
      delay(10);
      x=Serial.parseInt();
      y=Serial.parseInt();
      z=Serial.parseInt();
      pump=Serial.parseInt();
      valve=Serial.parseInt();
      if(valve==1){
        digitalWrite(VALVE_PIN, HIGH);
      }
      else{
        digitalWrite(VALVE_PIN, LOW);
      }
      if(pump==1){
        digitalWrite(PUMP_PIN, HIGH);
      }
      else{
        digitalWrite(PUMP_PIN, LOW);
      }
      float a0, a1, a2, a3;
      if(InverseK.solve(x, y, z, a0, a1, a2, a3, b2a(0))) {
        Braccio.ServoMovement(10, a2b(a0)+A0_CENTER, a2b(a1)+A1_CENTER, a2b(a2)+A2_CENTER, a2b(a3)+A3_CENTER, 0, 0);
      }
    }
    if(instruction=='R'){
      Serial.println(a0_value/a0_count);
      Serial.println(a1_value/a1_count);
      a0_value=analogRead(A0);
      a0_count=1;
      a1_value=analogRead(A1);
      a1_count=1;
    }
  }
}
