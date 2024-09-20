#include <Arduino.h>
#include <servo.h>

void ServoControl::setup(){
    // TODO: 
}

void ServoControl::set_length(int pwm){
    analogWrite(pinPWM, pwm);
}