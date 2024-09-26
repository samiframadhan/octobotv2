#include <Arduino.h>
#include <motordc.h>

void MotorDC::setup(){
    pinMode(pinPWM, OUTPUT);
    analogWrite(pinPWM, LOW);
}

void MotorDC::set_speed(int pwm){
    analogWrite(pinPWM, pwm);
}