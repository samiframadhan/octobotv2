#include <Arduino.h>
#include <motordc.h>

void MotorDC::setup(int pinP){
    pinMode(pinA, OUTPUT);
    pinMode(pinB, OUTPUT);
    digitalWrite(pinA, LOW);
    digitalWrite(pinB, LOW);
}

void MotorDC::set_direction(int direction){
    digitalWrite(pinA, LOW);
    digitalWrite(pinB, LOW);
    if (direction == 1) {
    digitalWrite(pinA, HIGH);  
    digitalWrite(pinB, LOW); 
  } else if (direction == 2) {
    digitalWrite(pinA, HIGH);
    digitalWrite(pinB, HIGH);
  } else {
    digitalWrite(pinA, LOW);
    digitalWrite(pinB, LOW);
  }
    
}