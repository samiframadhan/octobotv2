#include <Arduino.h>
#include <servo.h>

void ServoControl::setup(){
    pinMode(pinPWM, OUTPUT);
    analogWrite(pinPWM, LOW);
}

void ServoControl::set_length(int pwm){
    analogWrite(pinPWM, pwm);
}