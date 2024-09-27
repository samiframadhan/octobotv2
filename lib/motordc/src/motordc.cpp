#include <motordc.h>

MotorDC::MotorDC(/* args */):
pinA{0}, pinB{0}
{
}

MotorDC::~MotorDC()
{
}

void MotorDC::setup(int pin1, int pin2){
  pinA = pin1;
  pinB = pin2;
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